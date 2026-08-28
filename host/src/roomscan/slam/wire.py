"""Length-prefixed framing for the SLAM compute service (internal IPC, NOT the
device wire protocol -- deliberately independent of protocol.py/CRC).

A "message" is a dict of JSON scalars and numpy ndarrays. On the wire:
  [4-byte BE total-length][4-byte BE header-length][json header][raw array bytes]
The header is {"scalars": {...}, "arrays": {name: [dtype_str, [shape...]]}}.

Wire version 2 (E2, #223) adds three message types on top of the per-frame
`pose`/`mesh` split, all self-describing (a `type` scalar) so either side can
ignore what it does not understand:

  * `hello`  -- client -> service once per connection: the client's mapper cfg
               (was re-sent on EVERY frame before), the frame geometry, and
               the mesh format it wants. The service answers with its own
               `hello` carrying the resolved device. Frames no longer carry cfg.
  * `mesh_ack` -- client -> service: returns the single mesh CREDIT. The
               service sends a mesh only while it holds a credit, and keeps at
               most one newer mesh pending (latest-wins), so a slow consumer
               can never pile up 36 MB copies in the socket.
  * `mesh` (format "packet") -- the service ran `prepare_packet`+`pack_mesh`
               itself and ships the finished MESH bytes; the client forwards
               them without touching the GIL for prep.

`pose` messages now carry every numeric `FrameStep` field (stage timings,
inlier counts, block gauge, budget flag) plus IPC timestamps, so a remote
worker reports the same HUD numbers as a local one.

Receive/decode are zero-copy: `recv_message` reads into one preallocated
`bytearray`, and `decode_message` builds arrays with `np.frombuffer` over
that buffer, so a 36 MB mesh is copied once by the kernel and never again in
Python.
"""
from __future__ import annotations

import json
import struct
import time

import numpy as np

_LEN = struct.Struct(">I")
_ALIGN = 8
_PAD = b"\0" * _ALIGN

WIRE_VERSION = 2

# Message types.
POSE = "pose"
MESH = "mesh"
HELLO = "hello"
MESH_ACK = "mesh_ack"
FRAME = "frame"

MESH_FORMAT_ARRAYS = "arrays"      # raw vertex/triangle/colour arrays (v1)
MESH_FORMAT_PACKET = "packet"      # finished pack_mesh() bytes (v2)


def encode_message(fields: dict) -> bytes:
    scalars, arrays, blobs = {}, {}, []
    for k, v in fields.items():
        if isinstance(v, np.ndarray):
            v = np.ascontiguousarray(v)
            arrays[k] = [v.dtype.str, list(v.shape)]
            blobs.append(v.reshape(-1).view(np.uint8) if v.size else b"")
        elif isinstance(v, (bytes, bytearray, memoryview)):
            # Opaque byte blobs travel as a uint8 array -- no base64, no copy
            # beyond the socket write.
            mv = memoryview(v).cast("B") if len(v) else b""
            arrays[k] = ["|u1", [len(mv)]]
            blobs.append(mv)
        else:
            scalars[k] = v
    header = json.dumps({"scalars": scalars, "arrays": arrays}).encode("utf-8")
    # Align: pad the header so the first blob starts on an 8-byte boundary
    # (relative to the payload start), and pad every blob up to a multiple of
    # 8. `np.frombuffer` over an unaligned offset yields an ALIGNED=False
    # array, which Open3D's Tensor constructor rejects ("=f" format), so the
    # zero-copy decode below depends on this. Decode recomputes the same
    # padding from the header alone, so the framing stays self-describing.
    header += b" " * (-(4 + len(header)) % _ALIGN)
    parts = [_LEN.pack(len(header)), header]
    for b in blobs:
        parts.append(b)
        pad = -len(b) % _ALIGN
        if pad:
            parts.append(_PAD[:pad])
    return b"".join(parts)


def decode_message(buf) -> dict:
    """Decode one message. Arrays are VIEWS over `buf` (no copy) when it is a
    writable/bytes-like buffer; the caller must keep `buf` alive as long as
    the arrays are in use, and must not reuse it."""
    mv = memoryview(buf)
    (hlen,) = _LEN.unpack(mv[:4])
    header = json.loads(bytes(mv[4:4 + hlen]))
    out = dict(header["scalars"])
    off = 4 + hlen
    for name, (dtype_str, shape) in header["arrays"].items():
        dt = np.dtype(dtype_str)
        n = int(np.prod(shape)) if shape else 1
        nbytes = n * dt.itemsize
        out[name] = np.frombuffer(mv, dtype=dt, count=n, offset=off).reshape(shape)
        off += nbytes + (-nbytes % _ALIGN)
    return out


def send_message(sock, fields: dict) -> None:
    payload = encode_message(fields)
    sock.sendall(_LEN.pack(len(payload)) + payload)


def _recv_exactly(sock, n: int):
    # `np.empty`, not `bytearray(n)`: a bytearray is zero-filled on creation,
    # and that memset of a 36 MB mesh buffer runs WITH the GIL held (~5-10 ms
    # per mesh, measured as a 65 ms worst stall on the receiving loop). An
    # uninitialised buffer defers the page-faulting to `recv_into`, which
    # releases the GIL while the kernel copies into it.
    buf = np.empty(n, dtype=np.uint8)
    view = memoryview(buf)
    got = 0
    while got < n:
        k = sock.recv_into(view[got:], n - got)
        if not k:
            return None
        got += k
    return buf


def recv_message(sock) -> dict | None:
    head = _recv_exactly(sock, 4)
    if head is None:
        return None
    (total,) = _LEN.unpack(head)
    payload = _recv_exactly(sock, total)
    if payload is None:
        return None
    return decode_message(payload)


def message_type(msg: dict) -> str:
    """The message's `type`; an untagged message is a v1 `frame` (client ->
    service) or a v1 combined pose (service -> client) depending on direction,
    so callers that know the direction treat "" accordingly."""
    return str(msg.get("type") or "")


# ---- meshes ------------------------------------------------------------------

def mesh_to_arrays(mesh) -> dict:
    m = mesh.cpu()
    if "positions" in m.vertex:
        v = m.vertex["positions"].numpy().astype(np.float32, copy=False)
    else:
        v = np.zeros((0, 3), np.float32)
    if "indices" in m.triangle:
        t = m.triangle["indices"].numpy().astype(np.int32, copy=False)
    else:
        t = np.zeros((0, 3), np.int32)
    out = {"mesh_v": np.ascontiguousarray(v), "mesh_t": np.ascontiguousarray(t)}
    if "colors" in m.vertex:
        out["mesh_c"] = np.ascontiguousarray(
            m.vertex["colors"].numpy().astype(np.float32, copy=False))
    return out


def arrays_to_mesh(d: dict):
    import open3d as o3d
    o3c = o3d.core
    m = o3d.t.geometry.TriangleMesh()
    m.vertex["positions"] = o3c.Tensor(np.asarray(d["mesh_v"], np.float32))
    m.triangle["indices"] = o3c.Tensor(np.asarray(d["mesh_t"], np.int32))
    if "mesh_c" in d:
        m.vertex["colors"] = o3c.Tensor(np.asarray(d["mesh_c"], np.float32))
    return m


def mesh_message(mesh_seq, mesh, t_pub: float | None = None) -> dict:
    """A `mesh` message (format "arrays"): sent only when a new throttled mesh
    is ready AND the client holds a credit. Carries the same array payload as
    `mesh_to_arrays` plus its `mesh_seq` identity."""
    out = {"type": MESH, "mesh_seq": int(mesh_seq), "format": MESH_FORMAT_ARRAYS,
           "t_pub": time.monotonic() if t_pub is None else float(t_pub)}
    out.update(mesh_to_arrays(mesh))
    return out


def mesh_packet_message(mesh_seq, packed: bytes, *, source_vertex_count: int,
                        sent_vertex_count: int, prep_ms: float, pack_ms: float,
                        extract_ms: float = 0.0, t_pub: float | None = None) -> dict:
    """A `mesh` message (format "packet"): the finished `pack_mesh()` bytes,
    prepared in the SERVICE process, plus the prep-side counters the HUD shows
    (`mesh_prep_ms`/`mesh_pack_ms`/`mesh_payload_bytes` in the `slam` message)."""
    return {"type": MESH, "mesh_seq": int(mesh_seq), "format": MESH_FORMAT_PACKET,
            "t_pub": time.monotonic() if t_pub is None else float(t_pub),
            "source_vertex_count": int(source_vertex_count),
            "sent_vertex_count": int(sent_vertex_count),
            "prep_ms": float(prep_ms), "pack_ms": float(pack_ms),
            "extract_ms": float(extract_ms),
            "packet": packed}


def mesh_ack_message(mesh_seq) -> dict:
    return {"type": MESH_ACK, "mesh_seq": int(mesh_seq)}


# ---- handshake -----------------------------------------------------------------

def hello_message(*, cfg: dict, width: int, height: int,
                  mesh_format: str = MESH_FORMAT_ARRAYS, wall_mode: str = "split",
                  mesh_every: int | None = None) -> dict:
    """Client -> service, once per connection. `cfg` is the client's mapper
    kwargs (minus `device`, which the service owns)."""
    msg = {"type": HELLO, "version": WIRE_VERSION, "cfg": json.dumps(cfg),
           "width": int(width), "height": int(height),
           "mesh_format": str(mesh_format), "wall_mode": str(wall_mode)}
    if mesh_every is not None:
        msg["mesh_every"] = int(mesh_every)
    return msg


def hello_reply(*, device: str, icp_device: str | None = None,
                mesh_format: str = MESH_FORMAT_ARRAYS) -> dict:
    return {"type": HELLO, "version": WIRE_VERSION, "device": str(device),
            "icp_device": icp_device, "mesh_format": str(mesh_format)}


# ---- poses ---------------------------------------------------------------------

# Every numeric/bool FrameStep field beyond the five originals, in one place so
# the service (serialise) and the client (rebuild) cannot drift. `pose` itself
# travels as an array.
STEP_SCALAR_FIELDS = (
    "blocks_used", "blocks_capacity", "blocks_configured",
    "raycast_ms", "icp_ms", "integrate_ms", "inliers", "source_points",
    "t_tracking_ms", "t_icp_ms", "t_integrate_ms", "t_raycast_ms",
    "total_step_ms", "budget_breached", "t_device_us",
)


def step_fields(step) -> dict:
    """The serialisable scalars of a `FrameStep` (everything but `pose`).
    Missing attributes (an older FrameStep) are simply omitted."""
    out = {}
    for name in STEP_SCALAR_FIELDS:
        v = getattr(step, name, None)
        if v is None:
            continue
        if isinstance(v, (bool, np.bool_)):
            out[name] = bool(v)
        elif isinstance(v, (int, np.integer)):
            out[name] = int(v)
        else:
            out[name] = float(v)
    return out


def pose_message(fid, pose, fitness, rmse, tracking_lost, slam_ms,
                 tracking_lost_count, device: str | None = None,
                 step=None, **extra) -> dict:
    """A per-frame `pose` message: small + bounded, sent every frame so it is
    never delayed behind a growing mesh (Component B).

    `device` (plan item 2, 2026-08-02) is the SERVICE's own resolved Open3D
    compute device; optional so older peers still decode. `step` (E2, #223)
    adds every numeric `FrameStep` field via `step_fields`, so the remote
    worker's HUD is as complete as the local one's. `extra` scalars (IPC
    timestamps such as `t_client_submit`/`t_service_recv`/`t_service_done`)
    ride along untouched."""
    msg = {
        "type": POSE,
        "fid": int(fid),
        "pose": np.asarray(pose, np.float32),
        "fitness": float(fitness),
        "rmse": float(rmse),
        "tracking_lost": bool(tracking_lost),
        "slam_ms": float(slam_ms),
        "tracking_lost_count": int(tracking_lost_count),
    }
    if device is not None:
        msg["device"] = device
    if step is not None:
        msg.update(step_fields(step))
    for k, v in extra.items():
        if v is not None:
            msg[k] = v
    return msg
