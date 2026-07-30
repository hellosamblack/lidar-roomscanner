"""Wire protocol v1 — see docs/protocol.md. Keep in lockstep via protocol-change skill."""
from __future__ import annotations

import struct
import zlib
from dataclasses import dataclass
from enum import IntEnum

import numpy as np

MAGIC = b"RSCN"
VERSION = 1
HEADER_SIZE = 32
FLAG_DROPPED = 0x01

_HEADER = struct.Struct("<4sBBBBIQHHII")  # magic, ver, type, stream, flags, seq, t_us, w, h, plen, reserved
assert _HEADER.size == HEADER_SIZE


class FrameType(IntEnum):
    DATA = 1
    EVENT = 2
    COMMAND = 3
    ACK = 4


class StreamId(IntEnum):
    DEPTH_ZF32 = 0
    DEPTH_ZAPC = 1
    AMBIENT = 2
    AMPLITUDE = 3
    CONFIDENCE = 4
    REFLECTANCE = 5
    STATUS = 6
    RAW_3DMD = 7
    CALIB = 8
    IMU_QUAT = 9
    ENV = 10
    IMU_RAW = 11
    IMU_CAL = 12
    IMU_SYNC = 13


class EventCode(IntEnum):
    SENSOR_INIT_FAIL = 1
    TRIGGER_TIMEOUT = 2
    DMA_TIMEOUT = 3
    SENSOR_ERROR_STATUS = 4
    TX_OVERFLOW = 5


class CommandCode(IntEnum):
    PING = 1
    SEND_CALIB = 2
    SET_USECASE = 3
    SET_FRAME_PERIOD_US = 4
    SET_EXPOSURE_MS = 5
    REINIT = 6
    SET_STANDBY = 7


class StandbyLevel(IntEnum):
    """SET_STANDBY param / ACK applied. ACTIVE = streaming, SOFT = FSM standby
    (VCSEL idle, instant resume), HARD = XSHUT power-down (full re-bring-up to wake)."""
    ACTIVE = 0
    SOFT = 1
    HARD = 2


class ResultCode(IntEnum):
    OK = 0
    UNKNOWN_CMD = 1
    BAD_PARAM = 2
    REJECTED_BINNING = 3
    SENSOR_ERROR = 4
    BUSY = 5


DEPTH_NO_RETURN_MM = 12000.0  # empirical no-return sentinel in DEPTH_ZF32 payloads (Task 8)
RAW_3DMD_SIZE_BIN2 = 14842  # size in bytes at binning=2 (54×42 zones)
CALIB_SIZE = 2332  # VL53L9_CALIB_DATA_SIZE per-device calibration blob
IMU_QUAT_SIZE = 16  # 4x float32 [w, x, y, z], LSM body frame
ENV_SIZE = 20       # pressure f32 (Pa) + mag 3xf32 (µT) + temp f32 (°C)

# --- stream 11 (IMU_RAW): verbatim LSM6DSV16X FIFO words ---------------------
IMU_RAW_REC_SIZE = 8  # tag byte + 6 data bytes + 1 reserved zero


class ImuFifoTag(IntEnum):
    """LSM6DSV16X FIFO tag ids (AN5763 rev 4, FIFO_DATA_OUT_TAG.TAG_SENSOR).

    Only the tags stream 11 carries: the SFLP game-rotation word (0x13) rides
    stream 9 already, and the sensor-hub words (0x0E-0x10) ride stream 10.
    """
    GY_NC = 0x01
    XL_NC = 0x02
    TIMESTAMP = 0x04
    SFLP_GBIAS = 0x16
    SFLP_GRAVITY = 0x17


# Sensitivities. NB the gyro/accel ones follow the FULL SCALE the firmware selects in
# firmware/scanner-stream/Src/rs_lsm.c (RS_LSM_GY_FS = ±500 dps, RS_LSM_XL_FS = ±4 g); change
# those knobs and these must change with them. The two SFLP vectors are fixed-scale regardless
# of the XL/GY full scale (gravity is always ±2 g, gbias always ±125 dps).
IMU_RAW_GY_MDPS_PER_LSB = 17.5     # ±500 dps
IMU_RAW_XL_MG_PER_LSB = 0.122      # ±4 g
IMU_RAW_GRAVITY_MG_PER_LSB = 0.061    # SFLP gravity vector, fixed ±2 g
IMU_RAW_GBIAS_MDPS_PER_LSB = 4.375    # SFLP gyro-bias vector, fixed ±125 dps
IMU_RAW_TICK_US = 21.7             # LSM timestamp counter LSB, nominal (DS13510 rev 4)

# --- stream 12 (IMU_CAL): the LSM's own clock trim -----------------------------------
# IMU_RAW_TICK_US above is only correct for a part whose internal oscillator landed exactly
# on nominal. Real parts are trimmed, and the trim is readable: INTERNAL_FREQ_FINE (register
# 0x4F) gives the deviation in 0.13% steps, and AN5763 6.4 turns it into the true period
#
#     t_tick = 1 / (46080 * (1 + 0.0013 * freq_fine))   [seconds]
#
# Measured on this rig 2026-07-28: the host clock ran 1.029790x the LSM's, i.e. ~29790 ppm
# of pure scale error, because the host was integrating gyro against the nominal tick. That
# turns a 90 deg pan into ~87.3 deg — far worse than any quantisation floor.
IMU_CAL_SIZE = 4                   # int8 freq_fine, uint8 valid, uint16 reserved
IMU_TICK_BASE_HZ = 46080.0         # nominal timestamp-counter frequency (AN5763 6.4)
IMU_TICK_FREQ_FINE_STEP = 0.0013   # 0.13% per FREQ_FINE LSB

# --- stream 13 (IMU_SYNC): the frame's FRAME_READY edge on the LSM's own clock ---------
# The header's t_us is that edge on the MCU's TIM2 clock. Placing it on the LSM clock used to
# mean guessing from stream 11's FIFO words, which are samples taken before a drain that runs
# 1-2 ms later in the firmware's loop -- measured host-side at 1082 µs RMS of residual against
# a windowed linear clock fit, and demonstrably load-dependent (a frame that also carries CALIB
# drains 673 µs later, t = -3.9 over 5331 static frames). Stream 13 carries the answer instead:
# the LSM TIMESTAMP register read at the edge, with the delay and the read's own duration
# alongside so the residual uncertainty is stated rather than assumed. See BUG-031.
#
# It also answers the question that turns out to dominate the error budget: WHEN the stream-9
# quaternion sent alongside a frame is actually valid. That quat is the mean of a whole FIFO
# batch (firmware `RS_LSM_SFLP_AVERAGE`, shipped for BUG-027's 2.8x noise cut), so it carries the
# batch's MIDPOINT orientation, which on this rig sits +7.8 ms AFTER the frame-ready edge (the
# drain is +24.3 ms after it). ~0.3 deg at 38.5 deg/s — an order of magnitude above the skew
# above, and it LEADS the depth frame rather than lagging it.
IMU_SYNC_SIZE = 22   # u32 ticks, u32 latch_us, u32 drain_us, u32 quat_mid_ticks, u16 read_us, u16 quat_n, u8 valid, u8 rsv


class ProtocolError(Exception):
    pass


@dataclass(frozen=True)
class FrameHeader:
    frame_type: int
    stream_id: int
    flags: int
    seq: int
    t_us: int
    width: int
    height: int
    payload_len: int

    @classmethod
    def unpack(cls, buf: bytes) -> "FrameHeader":
        magic, ver, ftype, stream, flags, seq, t_us, w, h, plen, _res = _HEADER.unpack(buf)
        if magic != MAGIC:
            raise ProtocolError(f"bad magic {magic!r}")
        if ver != VERSION:
            raise ProtocolError(f"unsupported version {ver}")
        return cls(ftype, stream, flags, seq, t_us, w, h, plen)

    def pack(self) -> bytes:
        return _HEADER.pack(MAGIC, VERSION, self.frame_type, self.stream_id, self.flags,
                            self.seq, self.t_us, self.width, self.height, self.payload_len, 0)


@dataclass(frozen=True)
class Frame:
    header: FrameHeader
    payload: bytes


def pack_frame(header: FrameHeader, payload: bytes) -> bytes:
    if len(payload) != header.payload_len:
        raise ProtocolError(f"payload length {len(payload)} != header {header.payload_len}")
    body = header.pack() + payload
    return body + zlib.crc32(body).to_bytes(4, "little")


def parse_event(payload: bytes) -> tuple[int, int, str]:
    """Decode a frame_type=EVENT payload -> (code, detail, message)."""
    if len(payload) < 8:
        raise ProtocolError(f"event payload too short: {len(payload)} bytes")
    code, detail = struct.unpack_from("<II", payload, 0)
    return code, detail, payload[8:].decode("ascii", "replace")


def pack_command(cmd: int, param: int, token: int) -> bytes:
    """Pack a COMMAND frame: cmd (u32) + param (u32) LE, with header seq=token.

    Returns the full wire frame (header + payload + CRC).
    """
    payload = struct.pack("<II", cmd, param)
    header = FrameHeader(
        frame_type=FrameType.COMMAND,
        stream_id=0,
        flags=0,
        seq=token,
        t_us=0,
        width=0,
        height=0,
        payload_len=len(payload),
    )
    return pack_frame(header, payload)


def parse_ack(payload: bytes) -> tuple[int, int, int]:
    """Decode a frame_type=ACK payload -> (cmd, result, applied).

    ACK payloads are exactly 12 bytes; any other length is malformed (unlike
    EVENT's legitimate variable message tail) and raises ProtocolError.
    """
    if len(payload) != 12:
        raise ProtocolError(f"ACK payload must be exactly 12 bytes, got {len(payload)}")
    cmd, result, applied = struct.unpack("<III", payload)
    return cmd, result, applied


def decode_imu_quat(payload: bytes) -> tuple[float, float, float, float]:
    """Decode a stream 9 IMU_QUAT payload -> (w, x, y, z) unit quaternion."""
    if len(payload) != IMU_QUAT_SIZE:
        raise ProtocolError(f"IMU_QUAT payload must be {IMU_QUAT_SIZE} bytes, got {len(payload)}")
    w, x, y, z = struct.unpack("<4f", payload)
    return w, x, y, z


@dataclass(frozen=True)
class ImuRawBatch:
    """One stream-11 payload, demuxed by FIFO tag. Every array is one row per FIFO word,
    in FIFO (chronological) order; the matching `*_cnt` array holds that word's TAG_CNT
    (the 2-bit sample-time slot counter), which is what groups a gyro word with the accel
    and timestamp words of the same sample time."""
    gyro_dps: "np.ndarray"        # (N, 3) float64, deg/s
    gyro_cnt: "np.ndarray"        # (N,) uint8
    accel_g: "np.ndarray"         # (N, 3) float64, g
    accel_cnt: "np.ndarray"
    gravity_g: "np.ndarray"       # (N, 3) float64, g — SFLP gravity vector
    gravity_cnt: "np.ndarray"
    gbias_dps: "np.ndarray"       # (N, 3) float64, deg/s — SFLP gyro bias estimate
    gbias_cnt: "np.ndarray"
    timestamp_ticks: "np.ndarray"  # (N,) uint32, LSM timestamp counter (`tick_us` per LSB)
    timestamp_cnt: "np.ndarray"
    n_records: int                # total words in the payload, including unknown tags
    tick_us: float = IMU_RAW_TICK_US  # timestamp LSB actually applied (see decode_imu_raw)

    @property
    def timestamp_us(self) -> "np.ndarray":
        """Timestamp words in microseconds on the LSM's own clock."""
        return self.timestamp_ticks.astype(np.float64) * self.tick_us


def decode_imu_raw(payload: bytes, tick_us: float = IMU_RAW_TICK_US) -> ImuRawBatch:
    """Decode a stream 11 IMU_RAW payload (N × 8-byte verbatim FIFO records).

    Record layout: byte 0 = the FIFO_DATA_OUT_TAG register byte, `TAG_SENSOR << 3 |
    TAG_CNT << 1` (bit 0 is the register's not_used0 and is always 0 — the ST driver
    discards it); bytes 1-6 = the FIFO data bytes untouched; byte 7 = reserved zero.

    `tick_us` is the timestamp-counter LSB to scale the TIMESTAMP words by. It defaults to
    the nominal 21.7 µs, which is what a recording made before stream 12 existed can offer;
    pass `imu_tick_us(freq_fine)` from a stream-12 frame to get the part's real tick.
    """
    if len(payload) == 0 or len(payload) % IMU_RAW_REC_SIZE != 0:
        raise ProtocolError(
            f"IMU_RAW payload must be a non-zero multiple of {IMU_RAW_REC_SIZE} bytes, "
            f"got {len(payload)}")
    rec = np.frombuffer(payload, dtype=np.uint8).reshape(-1, IMU_RAW_REC_SIZE)
    tag = rec[:, 0] >> 3
    cnt = (rec[:, 0] >> 1) & 0x03
    data = rec[:, 1:7]

    def vec3(tag_id: int, scale: float) -> tuple["np.ndarray", "np.ndarray"]:
        sel = tag == tag_id
        words = np.ascontiguousarray(data[sel]).view("<i2").reshape(-1, 3)
        return words.astype(np.float64) * scale, cnt[sel]

    gyro, gyro_cnt = vec3(ImuFifoTag.GY_NC, IMU_RAW_GY_MDPS_PER_LSB / 1000.0)
    accel, accel_cnt = vec3(ImuFifoTag.XL_NC, IMU_RAW_XL_MG_PER_LSB / 1000.0)
    gravity, gravity_cnt = vec3(ImuFifoTag.SFLP_GRAVITY, IMU_RAW_GRAVITY_MG_PER_LSB / 1000.0)
    gbias, gbias_cnt = vec3(ImuFifoTag.SFLP_GBIAS, IMU_RAW_GBIAS_MDPS_PER_LSB / 1000.0)

    ts_sel = tag == ImuFifoTag.TIMESTAMP
    # timestamp word: u32 tick in data[0:4], BDR metadata in data[4:6] (not decoded)
    ticks = np.ascontiguousarray(data[ts_sel][:, :4]).view("<u4").reshape(-1)

    return ImuRawBatch(gyro, gyro_cnt, accel, accel_cnt, gravity, gravity_cnt,
                       gbias, gbias_cnt, ticks, cnt[ts_sel], rec.shape[0], tick_us)


def imu_tick_us(freq_fine: int) -> float:
    """LSM timestamp-counter LSB in µs for a given INTERNAL_FREQ_FINE (AN5763 6.4)."""
    return 1e6 / (IMU_TICK_BASE_HZ * (1.0 + IMU_TICK_FREQ_FINE_STEP * freq_fine))


@dataclass(frozen=True)
class ImuClockCal:
    """One stream-12 payload: the LSM6DSV16X's factory oscillator trim."""
    freq_fine: int    # INTERNAL_FREQ_FINE (register 0x4F), signed, 0.13% per LSB
    valid: bool       # False => the device could not read the register; use the nominal tick

    @property
    def tick_us(self) -> float:
        """True timestamp-counter LSB, falling back to nominal when `valid` is False."""
        return imu_tick_us(self.freq_fine) if self.valid else IMU_RAW_TICK_US


def decode_imu_cal(payload: bytes) -> ImuClockCal:
    """Decode a stream 12 IMU_CAL payload -> ImuClockCal.

    Layout: int8 freq_fine, uint8 valid, uint16 reserved (0). The device only emits this
    stream at all once it has read the register successfully, so `valid` is 1 in practice;
    it exists so the field can never be silently mistaken for a legitimate freq_fine of 0.
    """
    if len(payload) != IMU_CAL_SIZE:
        raise ProtocolError(f"IMU_CAL payload must be {IMU_CAL_SIZE} bytes, got {len(payload)}")
    freq_fine, valid, _reserved = struct.unpack("<bBH", payload)
    return ImuClockCal(freq_fine, bool(valid))


@dataclass(frozen=True)
class ImuSync:
    """One stream-13 payload: this ToF frame's FRAME_READY edge on the LSM clock.

    `lsm_ticks` is in the same units as stream 11's TIMESTAMP words, so scale it with the
    stream-12 tick (`ImuClockCal.tick_us`), not the nominal 21.7 µs.
    """
    lsm_ticks: int          # LSM TIMESTAMP counter latched just after FRAME_READY
    latch_delay_us: int     # MCU µs from the frame's t_us to the middle of that read
    drain_delay_us: int     # MCU µs from the frame's t_us to the FIFO drain (diagnostic)
    quat_mid_ticks: int     # LSM tick the paired stream-9 quaternion is valid AT (batch midpoint)
    read_us: int            # duration of the latch read — the uncertainty on lsm_ticks
    quat_n: int             # SFLP samples averaged into that quaternion (0 = none this drain)
    valid: bool

    def frame_ready_ticks(self, tick_us: float) -> float:
        """The FRAME_READY edge itself, in LSM ticks: the latch minus its own delay."""
        return self.lsm_ticks - self.latch_delay_us / tick_us

    def quat_offset_us(self, tick_us: float) -> float | None:
        """How far the paired quaternion's instant sits AFTER the frame-ready edge, in µs.

        Positive means the orientation LEADS the depth frame, which is the case here
        (+7.8 ms measured): the averaged batch straddles the edge and is centred well
        past it. A host that wants the orientation at the frame instant must propagate
        it BACKWARD by this much — the opposite direction to "the quat is stale".
        Returns None when this drain produced no quaternion.
        """
        if not self.quat_n or not self.quat_mid_ticks:
            return None
        return (self.quat_mid_ticks - self.frame_ready_ticks(tick_us)) * tick_us


def decode_imu_sync(payload: bytes) -> ImuSync:
    """Decode a stream 13 IMU_SYNC payload -> ImuSync.

    Layout: u32 lsm_ticks, u32 latch_delay_us, u32 drain_delay_us, u32 quat_mid_ticks,
    u16 read_us, u16 quat_n, u8 valid, u8 reserved (0). The device only emits the stream
    when the register read succeeded, so `valid` is 1 in practice; absence of the stream
    means "unknown", exactly like stream 12.
    """
    if len(payload) != IMU_SYNC_SIZE:
        raise ProtocolError(f"IMU_SYNC payload must be {IMU_SYNC_SIZE} bytes, got {len(payload)}")
    (ticks, latch_us, drain_us, quat_mid, read_us, quat_n,
     valid, _reserved) = struct.unpack("<IIIIHHBB", payload)
    return ImuSync(ticks, latch_us, drain_us, quat_mid, read_us, quat_n, bool(valid))


def decode_env(payload: bytes) -> tuple[float, tuple[float, float, float], float]:
    """Decode a stream 10 ENV payload -> (pressure_pa, (mx, my, mz) µT, temp_c)."""
    if len(payload) != ENV_SIZE:
        raise ProtocolError(f"ENV payload must be {ENV_SIZE} bytes, got {len(payload)}")
    pressure, mx, my, mz, temp = struct.unpack("<5f", payload)
    return pressure, (mx, my, mz), temp
