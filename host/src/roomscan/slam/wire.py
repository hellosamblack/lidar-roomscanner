"""Length-prefixed framing for the SLAM compute service (internal IPC, NOT the
device wire protocol -- deliberately independent of protocol.py/CRC).

A "message" is a dict of JSON scalars and numpy ndarrays. On the wire:
  [4-byte BE total-length][json header][raw array bytes in header order]
The header is {"scalars": {...}, "arrays": {name: [dtype_str, [shape...]]}}.
"""
from __future__ import annotations

import json
import struct
import numpy as np

_LEN = struct.Struct(">I")


def encode_message(fields: dict) -> bytes:
    scalars, arrays, blobs = {}, {}, []
    for k, v in fields.items():
        if isinstance(v, np.ndarray):
            v = np.ascontiguousarray(v)
            arrays[k] = [v.dtype.str, list(v.shape)]
            blobs.append(v.tobytes())
        else:
            scalars[k] = v
    header = json.dumps({"scalars": scalars, "arrays": arrays}).encode("utf-8")
    body = b"".join(blobs)
    return _LEN.pack(len(header)) + header + body


def decode_message(buf) -> dict:
    mv = memoryview(buf)
    (hlen,) = _LEN.unpack(mv[:4])
    header = json.loads(bytes(mv[4:4 + hlen]))
    out = dict(header["scalars"])
    off = 4 + hlen
    for name, (dtype_str, shape) in header["arrays"].items():
        dt = np.dtype(dtype_str)
        n = int(np.prod(shape)) if shape else 1
        nbytes = n * dt.itemsize
        out[name] = np.frombuffer(bytes(mv[off:off + nbytes]), dtype=dt).reshape(shape)
        off += nbytes
    return out


def send_message(sock, fields: dict) -> None:
    payload = encode_message(fields)
    sock.sendall(_LEN.pack(len(payload)) + payload)


def _recv_exactly(sock, n: int) -> bytes | None:
    chunks, got = [], 0
    while got < n:
        chunk = sock.recv(n - got)
        if not chunk:
            return None
        chunks.append(chunk); got += len(chunk)
    return b"".join(chunks)


def recv_message(sock) -> dict | None:
    head = _recv_exactly(sock, 4)
    if head is None:
        return None
    (total,) = _LEN.unpack(head)
    payload = _recv_exactly(sock, total)
    if payload is None:
        return None
    return decode_message(payload)


def mesh_to_arrays(mesh) -> dict:
    m = mesh.cpu()
    v = m.vertex["positions"].numpy().astype(np.float32) if "positions" in m.vertex else np.zeros((0, 3), np.float32)
    t = m.triangle["indices"].numpy().astype(np.int32) if "indices" in m.triangle else np.zeros((0, 3), np.int32)
    out = {"mesh_v": np.ascontiguousarray(v), "mesh_t": np.ascontiguousarray(t)}
    if "colors" in m.vertex:
        out["mesh_c"] = np.ascontiguousarray(m.vertex["colors"].numpy().astype(np.float32))
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
