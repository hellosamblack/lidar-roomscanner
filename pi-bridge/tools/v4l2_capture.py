"""Minimal pure-stdlib V4L2 MMAP capture for the Raspberry Pi bridge (issue #226).

Why this exists at all: the Pi (Raspbian trixie, armv7l, Python 3.13.5) has no
pip, no ffmpeg, no gstreamer and no python3-v4l2. Everything the camera work
needs -- the characterization instrument first, then `roomscan-cam` -- has to
come out of the standard library, so this is `ioctl(2)` against
`/dev/videoN` with `struct`-packed uapi buffers and `mmap` for the frames.

THE LAYOUT TRAP THIS MODULE EXISTS TO SURVIVE.  `struct v4l2_buffer`'s
`timestamp` field is a `struct timeval`, and the ioctl request number encodes
`sizeof(struct v4l2_buffer)` in its size bits -- so the *number itself* differs
between a 32-bit-time_t userspace (68-byte struct) and a 64-bit-time_t one
(80-byte struct, and Debian/Raspbian 13 armhf is a 64-bit-time_t port).  The
kernel accepts BOTH -- `VIDIOC_QUERYBUF` and `VIDIOC_QUERYBUF_TIME32` are
distinct ioctl numbers precisely for this -- but only one of them matches the
struct this process actually packs.  Guessing gives `ENOTTY` at best and
silently misparsed timestamps at worst, so `V4L2Device.open()` PROBES both
layouts with a real `VIDIOC_QUERYBUF` and records which one the kernel took
(`device.time_layout`).  Never infer it from `platform.architecture()`.

Nothing here is camera-specific; `roomscan-cam` and
`pi-bridge/tools/cam_characterize.py` both import it so the two cannot drift.
"""
from __future__ import annotations

import ctypes
import errno
import fcntl
import mmap
import os
import struct
import time

# --- ioctl request-number encoding (asm-generic, which armv7l uses) ---------
_IOC_NONE, _IOC_WRITE, _IOC_READ = 0, 1, 2


def _ioc(direction: int, type_ch: str, nr: int, size: int) -> int:
    """Build an ioctl request number, signed-wrapped for `fcntl.ioctl`.

    `_IOWR` sets both direction bits, so the raw value exceeds `INT_MAX` and
    must be handed to `fcntl.ioctl` as the negative C `int` with the same bit
    pattern.
    """
    val = (direction << 30) | (size << 16) | (ord(type_ch) << 8) | nr
    return val - (1 << 32) if val >= (1 << 31) else val


def _iow(type_ch: str, nr: int, size: int) -> int:
    return _ioc(_IOC_WRITE, type_ch, nr, size)


def _ior(type_ch: str, nr: int, size: int) -> int:
    return _ioc(_IOC_READ, type_ch, nr, size)


def _iowr(type_ch: str, nr: int, size: int) -> int:
    return _ioc(_IOC_READ | _IOC_WRITE, type_ch, nr, size)


# --- uapi/linux/videodev2.h constants --------------------------------------
V4L2_BUF_TYPE_VIDEO_CAPTURE = 1
V4L2_MEMORY_MMAP = 1
V4L2_FIELD_NONE = 1

V4L2_CAP_VIDEO_CAPTURE = 0x00000001
V4L2_CAP_STREAMING = 0x04000000
V4L2_CAP_DEVICE_CAPS = 0x80000000

V4L2_BUF_FLAG_TIMESTAMP_MASK = 0x0000E000
V4L2_BUF_FLAG_TIMESTAMP_UNKNOWN = 0x00000000
V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC = 0x00002000
V4L2_BUF_FLAG_TIMESTAMP_COPY = 0x00004000
V4L2_BUF_FLAG_ERROR = 0x00000040

TIMESTAMP_DOMAIN = {
    V4L2_BUF_FLAG_TIMESTAMP_UNKNOWN: "unknown",
    V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC: "pi_monotonic",
    V4L2_BUF_FLAG_TIMESTAMP_COPY: "copy",
}


def fourcc(code: str) -> int:
    return (ord(code[0]) | (ord(code[1]) << 8)
            | (ord(code[2]) << 16) | (ord(code[3]) << 24))


def fourcc_str(value: int) -> str:
    return "".join(chr((value >> s) & 0xFF) for s in (0, 8, 16, 24))


V4L2_PIX_FMT_MJPEG = fourcc("MJPG")
V4L2_PIX_FMT_YUYV = fourcc("YUYV")

# struct v4l2_capability: driver[16] card[32] bus_info[32] version caps
# device_caps reserved[3]  == 104 bytes
_CAP_FMT = "<16s32s32sIII12s"
_CAP_SIZE = struct.calcsize(_CAP_FMT)

# struct v4l2_format: __u32 type + union{...; __u8 raw_data[200]} == 204 bytes
_FORMAT_SIZE = 204
# struct v4l2_pix_format, the first 52 bytes of that union
_PIX_FMT = "<IIIIIIIIIIIII"

# struct v4l2_requestbuffers: count type memory capabilities flags+reserved[3]
_REQBUFS_FMT = "<IIIII"
_REQBUFS_SIZE = struct.calcsize(_REQBUFS_FMT)

# struct v4l2_streamparm: __u32 type + union{...; __u8 raw_data[200]}
_STREAMPARM_SIZE = 204

VIDIOC_QUERYCAP = _ior("V", 0, _CAP_SIZE)
VIDIOC_G_FMT = _iowr("V", 4, _FORMAT_SIZE)
VIDIOC_S_FMT = _iowr("V", 5, _FORMAT_SIZE)
VIDIOC_REQBUFS = _iowr("V", 8, _REQBUFS_SIZE)
VIDIOC_STREAMON = _iow("V", 18, 4)
VIDIOC_STREAMOFF = _iow("V", 19, 4)
VIDIOC_G_PARM = _iowr("V", 21, _STREAMPARM_SIZE)
VIDIOC_S_PARM = _iowr("V", 22, _STREAMPARM_SIZE)


class _BufferLayout:
    """One candidate `struct v4l2_buffer` layout plus its ioctl numbers."""

    def __init__(self, name: str, fmt: str, ts_offset: int, ts_fmt: str):
        self.name = name
        self.fmt = fmt
        self.size = struct.calcsize(fmt)
        self.ts_offset = ts_offset
        self.ts_fmt = ts_fmt
        self.QUERYBUF = _iowr("V", 9, self.size)
        self.QBUF = _iowr("V", 15, self.size)
        self.DQBUF = _iowr("V", 17, self.size)

    def pack(self, index: int, buf_type: int, memory: int) -> bytearray:
        raw = bytearray(self.size)
        struct.pack_into("<II", raw, 0, index, buf_type)
        struct.pack_into("<I", raw, self._memory_offset, memory)
        return raw

    def unpack(self, raw: bytes) -> dict:
        vals = struct.unpack(self.fmt, bytes(raw))
        out = dict(zip(self.field_names, vals))
        sec, usec = struct.unpack_from(self.ts_fmt, raw, self.ts_offset)
        out["timestamp_ns"] = sec * 1_000_000_000 + usec * 1000
        return out


# Field order of struct v4l2_buffer, shared by both layouts.  `_pad` entries
# are the alignment holes the 64-bit-time layout introduces.
_BUF_FIELDS_32 = ("index", "type", "bytesused", "flags", "field",
                  "_ts_sec", "_ts_usec",
                  "_tc_type", "_tc_flags", "_tc_a", "_tc_b",
                  "sequence", "memory", "m", "length", "reserved2", "request_fd")
_BUF_FIELDS_64 = ("index", "type", "bytesused", "flags", "field", "_pad",
                  "_ts_sec", "_ts_usec",
                  "_tc_type", "_tc_flags", "_tc_a", "_tc_b",
                  "sequence", "memory", "m", "length", "reserved2", "request_fd")

# 32-bit time_t: timeval is 2x __u32 at offset 20; struct is 68 bytes.
_LAYOUT_TIME32 = _BufferLayout("time32", "<5I" "2I" "4I" "6I", 20, "<II")
_LAYOUT_TIME32.field_names = _BUF_FIELDS_32
_LAYOUT_TIME32._memory_offset = 48

# 64-bit time_t: 8-byte alignment pads to offset 24, timeval is 2x __s64,
# struct is 80 bytes.
_LAYOUT_TIME64 = _BufferLayout("time64", "<5II" "2q" "4I" "6I", 24, "<qq")
_LAYOUT_TIME64.field_names = _BUF_FIELDS_64
_LAYOUT_TIME64._memory_offset = 60

assert _LAYOUT_TIME32.size == 68, _LAYOUT_TIME32.size
assert _LAYOUT_TIME64.size == 80, _LAYOUT_TIME64.size

BUFFER_LAYOUTS = (_LAYOUT_TIME64, _LAYOUT_TIME32)


class V4L2Error(OSError):
    pass


def enumerate_capture_devices() -> list[dict]:
    """Every `/dev/video*` with its USB identity, capability bits and by-path.

    Returns the raw evidence for device selection rather than a choice, so a
    caller that finds nothing can print the candidates it rejected.
    """
    found = []
    for name in sorted(os.listdir("/dev")):
        if not name.startswith("video") or not name[5:].isdigit():
            continue
        path = "/dev/" + name
        entry = {"path": path, "index": int(name[5:]), "by_path": [],
                 "usb_vid_pid": None, "capture": None, "streaming": None,
                 "card": None, "driver": None, "bus_info": None, "error": None}
        for link_dir in ("/dev/v4l/by-path", "/dev/v4l/by-id"):
            try:
                for link in sorted(os.listdir(link_dir)):
                    full = os.path.join(link_dir, link)
                    if os.path.realpath(full) == path:
                        entry["by_path"].append(full)
            except OSError:
                pass
        entry["usb_vid_pid"] = _sysfs_usb_id(name)
        try:
            fd = os.open(path, os.O_RDWR | os.O_NONBLOCK)
        except OSError as exc:
            entry["error"] = f"open: {exc.strerror}"
            found.append(entry)
            continue
        try:
            raw = bytearray(_CAP_SIZE)
            fcntl.ioctl(fd, VIDIOC_QUERYCAP, raw, True)
            driver, card, bus, _ver, caps, dev_caps, _res = struct.unpack(_CAP_FMT, bytes(raw))
            effective = dev_caps if caps & V4L2_CAP_DEVICE_CAPS else caps
            entry["driver"] = driver.split(b"\0", 1)[0].decode(errors="replace")
            entry["card"] = card.split(b"\0", 1)[0].decode(errors="replace")
            entry["bus_info"] = bus.split(b"\0", 1)[0].decode(errors="replace")
            entry["capture"] = bool(effective & V4L2_CAP_VIDEO_CAPTURE)
            entry["streaming"] = bool(effective & V4L2_CAP_STREAMING)
        except OSError as exc:
            entry["error"] = f"querycap: {exc.strerror}"
        finally:
            os.close(fd)
        found.append(entry)
    return found


def _sysfs_usb_id(devname: str) -> str | None:
    """`046d:0825` for a UVC node, walking up the sysfs parents to the USB device."""
    try:
        node = os.path.realpath(f"/sys/class/video4linux/{devname}/device")
    except OSError:
        return None
    for _ in range(8):
        vid = os.path.join(node, "idVendor")
        pid = os.path.join(node, "idProduct")
        if os.path.exists(vid) and os.path.exists(pid):
            try:
                with open(vid) as fh:
                    v = fh.read().strip()
                with open(pid) as fh:
                    p = fh.read().strip()
                return f"{v}:{p}"
            except OSError:
                return None
        parent = os.path.dirname(node)
        if parent == node or parent == "/sys":
            return None
        node = parent
    return None


def select_device(vid_pid: str = "046d:0825", override: str | None = None) -> dict:
    """Resolve the camera by identity, never by "first /dev/video*".

    Order is the spec's: explicit override, then USB VID:PID + Video
    Capture/Streaming capability + a `video-index0` by-path, then a diagnostic
    listing of every candidate that was rejected and why.
    """
    candidates = enumerate_capture_devices()
    report = {"selected": None, "reason": None, "candidates": candidates,
              "vid_pid": vid_pid, "override": override}
    if override:
        for cand in candidates:
            if cand["path"] == override or override in cand["by_path"]:
                report["selected"] = override
                report["reason"] = "configured override"
                return report
        report["reason"] = f"configured override {override!r} matched no /dev/video* node"
        return report

    matching = [c for c in candidates
                if c["usb_vid_pid"] == vid_pid and c["capture"] and c["streaming"]]
    if not matching:
        report["reason"] = (f"no /dev/video* node has USB id {vid_pid} with both "
                            "Video Capture and Streaming capability")
        return report
    for cand in matching:
        # Prefer /dev/v4l/by-path over /dev/v4l/by-id, deliberately and not by
        # alphabetical accident: the spec asks for the stable path of the
        # PHYSICAL PORT, so that a replug into the same port keeps working
        # while a second identical camera elsewhere on the hub cannot silently
        # take over.  by-id (which carries the unit serial) is the fallback.
        for prefix, why in (("/dev/v4l/by-path/", "by-path"), ("/dev/v4l/by-id/", "by-id")):
            index0 = sorted(p for p in cand["by_path"]
                            if p.startswith(prefix) and p.endswith("-video-index0"))
            if index0:
                report["selected"] = index0[0]
                report["reason"] = f"{vid_pid} capture node via stable {why} video-index0 link"
                return report
    report["selected"] = sorted(matching, key=lambda c: c["index"])[0]["path"]
    report["reason"] = f"{vid_pid} capture node (no video-index0 by-path link present)"
    return report


class V4L2Device:
    """One open capture node: set format, mmap buffers, dequeue frames.

    Deliberately not a context manager on the class level only -- `close()` is
    idempotent and also called from `__exit__`, because a half-opened device
    (fd open, buffers unmapped) is a state the daemon's reopen loop hits.
    """

    def __init__(self, path: str):
        self.path = path
        self.fd = -1
        self.time_layout = None
        self.buffers: list[mmap.mmap] = []
        self.buffer_lengths: list[int] = []
        self.negotiated: dict = {}

    # -- lifecycle ---------------------------------------------------------
    def open(self) -> None:
        self.fd = os.open(self.path, os.O_RDWR)

    def close(self) -> None:
        for buf in self.buffers:
            try:
                buf.close()
            except (BufferError, ValueError):
                pass
        self.buffers = []
        self.buffer_lengths = []
        if self.fd >= 0:
            try:
                os.close(self.fd)
            except OSError:
                pass
            self.fd = -1

    def __enter__(self):
        if self.fd < 0:
            self.open()
        return self

    def __exit__(self, *exc):
        try:
            self.stream_off()
        except OSError:
            pass
        self.close()
        return False

    # -- format ------------------------------------------------------------
    def query_cap(self) -> dict:
        raw = bytearray(_CAP_SIZE)
        fcntl.ioctl(self.fd, VIDIOC_QUERYCAP, raw, True)
        driver, card, bus, version, caps, dev_caps, _res = struct.unpack(_CAP_FMT, bytes(raw))
        return {
            "driver": driver.split(b"\0", 1)[0].decode(errors="replace"),
            "card": card.split(b"\0", 1)[0].decode(errors="replace"),
            "bus_info": bus.split(b"\0", 1)[0].decode(errors="replace"),
            "version": f"{(version >> 16) & 0xFF}.{(version >> 8) & 0xFF}.{version & 0xFF}",
            "capabilities": caps,
            "device_caps": dev_caps,
        }

    def set_format(self, width: int, height: int, pixelformat: int) -> dict:
        """S_FMT then read back what the driver actually chose.

        The return value is the READBACK, not the request -- a UVC driver
        silently substitutes a nearby size, and reporting the request as the
        mode is how a device ends up advertising 720p while serving 480p.
        """
        raw = bytearray(_FORMAT_SIZE)
        struct.pack_into("<I", raw, 0, V4L2_BUF_TYPE_VIDEO_CAPTURE)
        struct.pack_into(_PIX_FMT, raw, 4,
                         width, height, pixelformat, V4L2_FIELD_NONE,
                         0, 0, 0, 0, 0, 0, 0, 0, 0)
        fcntl.ioctl(self.fd, VIDIOC_S_FMT, raw, True)
        return self.get_format(raw)

    def get_format(self, raw: bytearray | None = None) -> dict:
        if raw is None:
            raw = bytearray(_FORMAT_SIZE)
            struct.pack_into("<I", raw, 0, V4L2_BUF_TYPE_VIDEO_CAPTURE)
            fcntl.ioctl(self.fd, VIDIOC_G_FMT, raw, True)
        vals = struct.unpack_from(_PIX_FMT, raw, 4)
        out = {"width": vals[0], "height": vals[1], "pixelformat": vals[2],
               "fourcc": fourcc_str(vals[2]), "field": vals[3],
               "bytesperline": vals[4], "sizeimage": vals[5],
               "colorspace": vals[6]}
        self.negotiated.update(out)
        return out

    def set_interval(self, fps: int) -> dict:
        raw = bytearray(_STREAMPARM_SIZE)
        struct.pack_into("<I", raw, 0, V4L2_BUF_TYPE_VIDEO_CAPTURE)
        # struct v4l2_captureparm: capability capturemode timeperframe{num,den}
        struct.pack_into("<IIII", raw, 4, 0, 0, 1, fps)
        fcntl.ioctl(self.fd, VIDIOC_S_PARM, raw, True)
        return self.get_interval()

    def get_interval(self) -> dict:
        raw = bytearray(_STREAMPARM_SIZE)
        struct.pack_into("<I", raw, 0, V4L2_BUF_TYPE_VIDEO_CAPTURE)
        fcntl.ioctl(self.fd, VIDIOC_G_PARM, raw, True)
        capability, capturemode, num, den = struct.unpack_from("<IIII", raw, 4)
        out = {"capability": capability, "capturemode": capturemode,
               "interval_numerator": num, "interval_denominator": den,
               "fps": (den / num) if num else None}
        self.negotiated.update(out)
        return out

    # -- buffers -----------------------------------------------------------
    def request_buffers(self, count: int = 4) -> int:
        raw = bytearray(_REQBUFS_SIZE)
        struct.pack_into(_REQBUFS_FMT, raw, 0,
                         count, V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_MMAP, 0, 0)
        fcntl.ioctl(self.fd, VIDIOC_REQBUFS, raw, True)
        granted = struct.unpack_from("<I", raw, 0)[0]
        if granted < 2:
            raise V4L2Error(errno.ENOMEM, f"driver granted only {granted} buffers")
        return granted

    def _probe_layout(self) -> _BufferLayout:
        """Ask the kernel which `struct v4l2_buffer` size it will accept.

        A wrong guess is `ENOTTY` on the ioctl number, which is exactly the
        signal we want -- so try each candidate and keep the one that works.
        """
        errors = []
        for layout in BUFFER_LAYOUTS:
            raw = layout.pack(0, V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_MMAP)
            try:
                fcntl.ioctl(self.fd, layout.QUERYBUF, raw, True)
            except OSError as exc:
                errors.append(f"{layout.name}({layout.size}B): {exc.strerror}")
                continue
            fields = layout.unpack(raw)
            if fields["length"] == 0:
                errors.append(f"{layout.name}({layout.size}B): accepted but length==0")
                continue
            return layout
        raise V4L2Error(errno.ENOTTY,
                        "no struct v4l2_buffer layout accepted: " + "; ".join(errors))

    def map_buffers(self, count: int) -> None:
        self.time_layout = self._probe_layout()
        layout = self.time_layout
        for index in range(count):
            raw = layout.pack(index, V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_MMAP)
            fcntl.ioctl(self.fd, layout.QUERYBUF, raw, True)
            fields = layout.unpack(raw)
            length, offset = fields["length"], fields["m"]
            mapped = mmap.mmap(self.fd, length, mmap.MAP_SHARED,
                               mmap.PROT_READ | mmap.PROT_WRITE, offset=offset)
            self.buffers.append(mapped)
            self.buffer_lengths.append(length)

    def queue_all(self) -> None:
        for index in range(len(self.buffers)):
            self.queue(index)

    def queue(self, index: int) -> None:
        layout = self.time_layout
        raw = layout.pack(index, V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_MMAP)
        fcntl.ioctl(self.fd, layout.QBUF, raw, True)

    def stream_on(self) -> None:
        fcntl.ioctl(self.fd, VIDIOC_STREAMON,
                    struct.pack("<I", V4L2_BUF_TYPE_VIDEO_CAPTURE))

    def stream_off(self) -> None:
        if self.fd < 0:
            return
        fcntl.ioctl(self.fd, VIDIOC_STREAMOFF,
                    struct.pack("<I", V4L2_BUF_TYPE_VIDEO_CAPTURE))

    def start(self, width: int, height: int, pixelformat: int, fps: int,
              nbuffers: int = 4) -> dict:
        """Configure + map + stream on, returning the ACTUAL negotiated mode."""
        fmt = self.set_format(width, height, pixelformat)
        parm = self.set_interval(fps)
        granted = self.request_buffers(nbuffers)
        self.map_buffers(granted)
        self.queue_all()
        self.stream_on()
        return {**fmt, **parm, "buffers": granted,
                "buffer_layout": self.time_layout.name,
                "requested": {"width": width, "height": height,
                              "fourcc": fourcc_str(pixelformat), "fps": fps}}

    def dequeue(self, timeout_s: float = 2.0) -> dict:
        """Block for one buffer and copy its exact `bytesused` bytes out.

        The copy is deliberate: the mmap slot is requeued before this returns,
        so nothing downstream can hold a V4L2 buffer hostage.  `bytes` is
        immutable, which is what makes the latest-slot fanout safe.
        """
        import select
        layout = self.time_layout
        ready, _, _ = select.select([self.fd], [], [], timeout_s)
        if not ready:
            raise TimeoutError(f"no buffer within {timeout_s}s")
        raw = bytearray(layout.size)
        struct.pack_into("<II", raw, 0, 0, V4L2_BUF_TYPE_VIDEO_CAPTURE)
        struct.pack_into("<I", raw, layout._memory_offset, V4L2_MEMORY_MMAP)
        fcntl.ioctl(self.fd, layout.DQBUF, raw, True)
        fields = layout.unpack(raw)
        index = fields["index"]
        nbytes = fields["bytesused"]
        dequeue_mono_ns = time.monotonic_ns()
        payload = self.buffers[index][:nbytes]
        self.queue(index)
        ts_flag = fields["flags"] & V4L2_BUF_FLAG_TIMESTAMP_MASK
        return {
            "data": bytes(payload),
            "bytesused": nbytes,
            "sequence": fields["sequence"],
            "flags": fields["flags"],
            "timestamp_flags": ts_flag,
            "timestamp_domain": TIMESTAMP_DOMAIN.get(ts_flag, f"reserved:{ts_flag:#x}"),
            "timestamp_valid": ts_flag == V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC,
            "capture_mono_ns": fields["timestamp_ns"],
            "dequeue_mono_ns": dequeue_mono_ns,
            "buffer_error": bool(fields["flags"] & V4L2_BUF_FLAG_ERROR),
        }


def looks_like_jpeg(data: bytes) -> bool:
    """SOI at the head, EOI at the tail -- the cheap structural check.

    UVC pads some frames, so the EOI is searched in the last few bytes rather
    than required to be the final two.
    """
    if len(data) < 4 or data[0:2] != b"\xff\xd8":
        return False
    return b"\xff\xd9" in data[-16:]


__all__ = [
    "V4L2Device", "V4L2Error", "select_device", "enumerate_capture_devices",
    "fourcc", "fourcc_str", "looks_like_jpeg",
    "V4L2_PIX_FMT_MJPEG", "V4L2_PIX_FMT_YUYV", "TIMESTAMP_DOMAIN",
]
