"""Rerun multimodal observability sidecar (Phase 6 / Issue #136).

Diagnostic sidecar providing time-aligned multimodal Rerun recordings:
depth, point clouds, transforms, IMU/environment sensors, and map meshes.
Fail-open, off-hot-path with a bounded worker queue, explicit device timestamps,
and pluggable writer backends for testing and live recording.
"""
from __future__ import annotations

import json
import logging
import queue
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol, runtime_checkable

import numpy as np

from .deproject import Deprojector
from .protocol import FrameHeader
from .sensors import (
    T_CV_TO_BODY,
    T_WORLD_TO_CV,
    display_rotation,
    quat_to_matrix,
)

logger = logging.getLogger("roomscan.rerun")


@runtime_checkable
class RerunSink(Protocol):
    """Pluggable writer interface for Rerun telemetry sinks."""

    def set_time_seconds(self, timeline: str, time_s: float) -> None: ...
    def set_time_sequence(self, timeline: str, seq: int) -> None: ...
    def log(self, entity_path: str, entity_data: Any, static: bool = False) -> None: ...
    def save(self, path: str) -> None: ...
    def close(self) -> None: ...


@dataclass(frozen=True)
class RerunEntry:
    entity_path: str
    data: Any
    device_time_s: float
    frame_seq: int
    static: bool = False


class MockWriter:
    """In-memory recording sink for testing without requiring rerun-sdk or GUI."""

    def __init__(self) -> None:
        self.entries: list[RerunEntry] = []
        self.static_entries: list[tuple[str, Any]] = []
        self.timelines_time: dict[str, float] = {}
        self.timelines_seq: dict[str, int] = {}
        self.current_time_s: float = 0.0
        self.current_seq: int = 0
        self.saved_path: str | None = None
        self.closed: bool = False

    def set_time_seconds(self, timeline: str, time_s: float) -> None:
        self.timelines_time[timeline] = time_s
        if timeline == "device_time":
            self.current_time_s = time_s

    def set_time_sequence(self, timeline: str, seq: int) -> None:
        self.timelines_seq[timeline] = seq
        if timeline == "frame_seq":
            self.current_seq = seq

    def log(self, entity_path: str, entity_data: Any, static: bool = False) -> None:
        if static:
            self.static_entries.append((entity_path, entity_data))
        else:
            self.entries.append(
                RerunEntry(
                    entity_path=entity_path,
                    data=entity_data,
                    device_time_s=self.current_time_s,
                    frame_seq=self.current_seq,
                    static=False,
                )
            )

    def save(self, path: str) -> None:
        self.saved_path = path

    def close(self) -> None:
        self.closed = True

    def get_entries(self, entity_path_prefix: str = "") -> list[RerunEntry]:
        return [e for e in self.entries if e.entity_path.startswith(entity_path_prefix)]


class NativeRerunWriter:
    """Live rerun-sdk bridge with lazy import and archetype mapping."""

    def __init__(self, recording_id: str = "roomscanner", output_path: str | None = None, spawn: bool = False) -> None:
        import rerun as rr
        self._rr = rr
        self._rr.init(recording_id, recording_id=recording_id, spawn=spawn)
        if output_path:
            Path(output_path).parent.mkdir(parents=True, exist_ok=True)
            self._rr.save(output_path)

    def set_time_seconds(self, timeline: str, time_s: float) -> None:
        self._rr.set_time_seconds(timeline, time_s)

    def set_time_sequence(self, timeline: str, seq: int) -> None:
        self._rr.set_time_sequence(timeline, seq)

    def log(self, entity_path: str, entity_data: Any, static: bool = False) -> None:
        rr_data = self._to_rerun_archetype(entity_path, entity_data)
        if rr_data is not None:
            self._rr.log(entity_path, rr_data, static=static)

    def _to_rerun_archetype(self, entity_path: str, data: Any) -> Any:
        rr = self._rr
        if isinstance(data, (rr.Archetype, rr.ComponentBatch)):
            return data
        if entity_path.endswith("/depth") and isinstance(data, np.ndarray):
            return rr.DepthImage(data)
        if (entity_path.endswith("/reflectance") or entity_path.endswith("/confidence") or entity_path.endswith("/ambient")) and isinstance(data, np.ndarray):
            return rr.Image(data)
        if entity_path.endswith("/points") and isinstance(data, np.ndarray):
            return rr.Points3D(data)
        if isinstance(data, (int, float, np.floating, np.integer)):
            return rr.Scalar(float(data))
        if isinstance(data, str):
            return rr.TextLog(data)
        if isinstance(data, dict):
            if "vertices" in data:
                return rr.Mesh3D(
                    vertex_positions=data["vertices"],
                    triangle_indices=data.get("triangles"),
                    vertex_colors=data.get("vertex_colors"),
                )
            if "matrix" in data:
                mat = np.asarray(data["matrix"])
                return rr.Transform3D(mat3x3=mat[:3, :3], translation=data.get("translation"))
            if "quaternion" in data:
                q = data["quaternion"]  # [w, x, y, z]
                return rr.Transform3D(
                    quaternion=rr.Quaternion(xyzw=[q[1], q[2], q[3], q[0]]),
                    translation=data.get("translation"),
                )
        return data

    def save(self, path: str) -> None:
        self._rr.save(path)

    def close(self) -> None:
        pass


@dataclass
class RerunSidecarConfig:
    enabled: bool = False
    max_queue_size: int = 100
    output_dir: str = "results/rerun"
    recording_name: str | None = None
    rate_limit_hz: float = 30.0
    max_duration_s: float | None = None
    max_bytes: int | None = None
    tof_hz: float | None = None
    mesh_hz: float | None = None
    mesh_vertex_budget: int = 65536


@dataclass
class _FrameItem:
    t_us: int
    seq: int
    outputs: dict[str, np.ndarray]
    points: np.ndarray | None = None


@dataclass
class _ImuItem:
    t_us: int
    seq: int
    quat: tuple[float, float, float, float] | None = None
    env: Any | None = None
    raw_imu: Any | None = None
    heading_deg: float | None = None


@dataclass
class _PoseItem:
    t_us: int
    seq: int
    quat: tuple[float, float, float, float] | None = None
    fused_quat: tuple[float, float, float, float] | None = None
    display_quat: tuple[float, float, float, float] | None = None
    translation: tuple[float, float, float] | None = None


@dataclass
class _MeshItem:
    t_us: int
    seq: int
    vertices: np.ndarray
    triangles: np.ndarray | None = None
    vertex_colors: np.ndarray | None = None


@dataclass
class _HostStreamItem:
    t_us: int
    seq: int
    metrics: dict[str, Any]


@dataclass
class _SlamItem:
    t_us: int
    seq: int
    metrics: dict[str, Any]


@dataclass
class _EventItem:
    t_us: int
    seq: int
    message: str
    code: int | None = None


class RerunSidecar:
    """Multimodal observability sidecar logging time-aligned scanner telemetry."""

    def __init__(
        self,
        config: RerunSidecarConfig | None = None,
        writer: RerunSink | None = None,
        deprojector: Deprojector | None = None,
    ) -> None:
        self.config = config if config is not None else RerunSidecarConfig()
        self._custom_writer = writer
        self._writer: RerunSink | None = writer
        self._deprojector = deprojector if deprojector is not None else Deprojector(54, 42)
        self._queue: queue.Queue = queue.Queue(maxsize=self.config.max_queue_size)
        self._worker_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._active = False
        self._samples_logged = 0
        self._queue_drops = 0
        self._fault_count = 0
        self._output_path: Path | None = None
        self._manifest_path: Path | None = None
        self._start_t_us: int | None = None
        self._last_t_us: int | None = None
        self._lock = threading.Lock()

    @property
    def active(self) -> bool:
        with self._lock:
            return self._active

    def start(self) -> None:
        """Start the sidecar worker thread and initialize sink."""
        with self._lock:
            if self._active:
                return
            self._stop_event.clear()
            self._samples_logged = 0
            self._queue_drops = 0
            self._fault_count = 0
            self._start_t_us = None
            self._last_t_us = None

            if self._custom_writer is not None:
                self._writer = self._custom_writer
            else:
                try:
                    out_dir = Path(self.config.output_dir)
                    out_dir.mkdir(parents=True, exist_ok=True)
                    rec_name = self.config.recording_name or f"live_{int(time.time())}"
                    self._output_path = out_dir / f"{rec_name}.rrd"
                    self._manifest_path = out_dir / f"{rec_name}.rerun.json"
                    self._writer = NativeRerunWriter(
                        recording_id=rec_name,
                        output_path=str(self._output_path),
                    )
                except Exception as exc:
                    self._fault_count += 1
                    logger.warning("Failed to initialize NativeRerunWriter (fail open): %s", exc)
                    self._writer = MockWriter()

            self._active = True
            self._log_static_frames()
            self._worker_thread = threading.Thread(
                target=self._worker_loop,
                name="rerun_sidecar_worker",
                daemon=True,
            )
            self._worker_thread.start()

    def _log_static_frames(self) -> None:
        """Log static coordinate frame declarations at recording start."""
        if self._writer is None:
            return
        try:
            self._writer.log(
                "world",
                {"axes": "X=North, Y=West, Z=Up", "type": "coordinate_system", "t_world_to_cv": T_WORLD_TO_CV.tolist()},
                static=True,
            )
            self._writer.log(
                "world/scanner",
                {"transform": "T_CV_TO_BODY", "matrix": T_CV_TO_BODY.tolist()},
                static=True,
            )
        except Exception as exc:
            self._fault_count += 1
            logger.warning("Failed to log static frames: %s", exc)

    def log_frame(
        self,
        header: FrameHeader,
        outputs: dict[str, np.ndarray],
        points: np.ndarray | None = None,
    ) -> None:
        """Submit a decoded/transformed frame snapshot (non-blocking, fail-open)."""
        try:
            if not self._active:
                return
            outputs_copy = {
                k: np.copy(v) if isinstance(v, np.ndarray) else v
                for k, v in outputs.items()
            }
            points_copy = np.copy(points) if points is not None else None
            item = _FrameItem(
                t_us=header.t_us,
                seq=header.seq,
                outputs=outputs_copy,
                points=points_copy,
            )
            self._queue.put_nowait(item)
        except queue.Full:
            self._queue_drops += 1
        except Exception as exc:
            self._fault_count += 1
            logger.warning("log_frame failed: %s", exc)

    def log_imu(
        self,
        t_us: int,
        seq: int,
        quat: tuple[float, float, float, float] | None = None,
        env: Any | None = None,
        raw_imu: Any | None = None,
        heading_deg: float | None = None,
    ) -> None:
        """Submit IMU / environment state (non-blocking, fail-open)."""
        try:
            if not self._active:
                return
            item = _ImuItem(
                t_us=t_us,
                seq=seq,
                quat=quat,
                env=env,
                raw_imu=raw_imu,
                heading_deg=heading_deg,
            )
            self._queue.put_nowait(item)
        except queue.Full:
            self._queue_drops += 1
        except Exception as exc:
            self._fault_count += 1
            logger.warning("log_imu failed: %s", exc)

    def log_pose(
        self,
        t_us: int,
        seq: int,
        quat: tuple[float, float, float, float] | None = None,
        fused_quat: tuple[float, float, float, float] | None = None,
        display_quat: tuple[float, float, float, float] | None = None,
        translation: tuple[float, float, float] | None = None,
    ) -> None:
        """Submit poses in coordinate convention (non-blocking, fail-open)."""
        try:
            if not self._active:
                return
            item = _PoseItem(
                t_us=t_us,
                seq=seq,
                quat=quat,
                fused_quat=fused_quat,
                display_quat=display_quat,
                translation=translation,
            )
            self._queue.put_nowait(item)
        except queue.Full:
            self._queue_drops += 1
        except Exception as exc:
            self._fault_count += 1
            logger.warning("log_pose failed: %s", exc)

    def log_mesh(
        self,
        t_us: int,
        seq: int,
        vertices: np.ndarray,
        triangles: np.ndarray | None = None,
        vertex_colors: np.ndarray | None = None,
    ) -> None:
        """Submit map mesh snapshot (non-blocking, fail-open)."""
        try:
            if not self._active:
                return
            v_copy = np.copy(vertices)
            t_copy = np.copy(triangles) if triangles is not None else None
            c_copy = np.copy(vertex_colors) if vertex_colors is not None else None
            item = _MeshItem(
                t_us=t_us,
                seq=seq,
                vertices=v_copy,
                triangles=t_copy,
                vertex_colors=c_copy,
            )
            self._queue.put_nowait(item)
        except queue.Full:
            self._queue_drops += 1
        except Exception as exc:
            self._fault_count += 1
            logger.warning("log_mesh failed: %s", exc)

    def log_host_stream(self, t_us: int, seq: int, metrics: dict[str, Any]) -> None:
        """Submit host stream scalars (non-blocking, fail-open)."""
        try:
            if not self._active:
                return
            item = _HostStreamItem(t_us=t_us, seq=seq, metrics=dict(metrics))
            self._queue.put_nowait(item)
        except queue.Full:
            self._queue_drops += 1
        except Exception as exc:
            self._fault_count += 1
            logger.warning("log_host_stream failed: %s", exc)

    def log_slam(self, t_us: int, seq: int, metrics: dict[str, Any]) -> None:
        """Submit SLAM metrics (non-blocking, fail-open)."""
        try:
            if not self._active:
                return
            item = _SlamItem(t_us=t_us, seq=seq, metrics=dict(metrics))
            self._queue.put_nowait(item)
        except queue.Full:
            self._queue_drops += 1
        except Exception as exc:
            self._fault_count += 1
            logger.warning("log_slam failed: %s", exc)

    def log_event(self, t_us: int, seq: int, message: str, code: int | None = None) -> None:
        """Submit event message (non-blocking, fail-open)."""
        try:
            if not self._active:
                return
            item = _EventItem(t_us=t_us, seq=seq, message=message, code=code)
            self._queue.put_nowait(item)
        except queue.Full:
            self._queue_drops += 1
        except Exception as exc:
            self._fault_count += 1
            logger.warning("log_event failed: %s", exc)

    def _worker_loop(self) -> None:
        while not self._stop_event.is_set() or not self._queue.empty():
            try:
                item = self._queue.get(timeout=0.05)
            except queue.Empty:
                continue

            if item is None:
                self._queue.task_done()
                break

            try:
                self._process_item(item)
                self._samples_logged += 1
            except Exception as exc:
                self._fault_count += 1
                logger.warning("RerunSidecar worker processing fault: %s", exc)
            finally:
                self._queue.task_done()

    def _process_item(self, item: Any) -> None:
        if self._writer is None:
            return

        t_us = getattr(item, "t_us", 0)
        seq = getattr(item, "seq", 0)
        device_time_s = t_us / 1e6
        self._writer.set_time_seconds("device_time", device_time_s)
        self._writer.set_time_sequence("frame_seq", seq)

        if self._start_t_us is None:
            self._start_t_us = t_us
        self._last_t_us = t_us

        if self.config.max_duration_s is not None and self._start_t_us is not None:
            duration_s = (t_us - self._start_t_us) / 1e6
            if duration_s > self.config.max_duration_s:
                return

        if isinstance(item, _FrameItem):
            outputs = item.outputs
            if "depth" in outputs:
                depth = outputs["depth"]
                self._writer.log("world/scanner/tof/depth", depth)
                if item.points is not None:
                    self._writer.log("world/scanner/tof/points", item.points)
                elif self._deprojector is not None:
                    try:
                        pts = self._deprojector(depth)
                        self._writer.log("world/scanner/tof/points", pts)
                    except Exception:
                        pass
            if "reflectance" in outputs:
                self._writer.log("world/scanner/tof/reflectance", outputs["reflectance"])
            if "confidence" in outputs:
                self._writer.log("world/scanner/tof/confidence", outputs["confidence"])
            if "ambient" in outputs:
                self._writer.log("world/scanner/tof/ambient", outputs["ambient"])

        elif isinstance(item, _ImuItem):
            if item.quat is not None:
                self._writer.log("world/scanner/pose/sflp", {"quaternion": item.quat})
            if item.env is not None:
                env = item.env
                if hasattr(env, "pressure_pa"):
                    self._writer.log("world/scanner/environment/pressure", env.pressure_pa)
                if hasattr(env, "temp_c"):
                    self._writer.log("world/scanner/environment/temperature", env.temp_c)
                if hasattr(env, "mag_ut"):
                    self._writer.log("world/scanner/environment/magnetic_field", list(env.mag_ut))
            if item.heading_deg is not None:
                self._writer.log("world/scanner/environment/heading", item.heading_deg)
            if item.raw_imu is not None:
                self._writer.log("world/scanner/imu/raw", item.raw_imu)

        elif isinstance(item, _PoseItem):
            if item.quat is not None:
                rot = quat_to_matrix(*item.quat)
                self._writer.log(
                    "world/scanner/pose/raw_sflp",
                    {
                        "quaternion": item.quat,
                        "matrix": rot.tolist(),
                        "translation": item.translation or (0.0, 0.0, 0.0),
                    },
                )
            if item.fused_quat is not None:
                fused_rot = quat_to_matrix(*item.fused_quat)
                self._writer.log(
                    "world/scanner/pose/fused",
                    {
                        "quaternion": item.fused_quat,
                        "matrix": fused_rot.tolist(),
                        "translation": item.translation or (0.0, 0.0, 0.0),
                    },
                )
            if item.quat is not None:
                disp_rot = display_rotation(item.quat)
                if disp_rot is not None:
                    self._writer.log(
                        "world/scanner/pose/display",
                        {
                            "matrix": disp_rot.tolist(),
                            "translation": item.translation or (0.0, 0.0, 0.0),
                        },
                    )
            primary_quat = item.fused_quat or item.quat
            if primary_quat is not None:
                rot = quat_to_matrix(*primary_quat)
                self._writer.log(
                    "world/scanner",
                    {
                        "quaternion": primary_quat,
                        "matrix": rot.tolist(),
                        "translation": item.translation or (0.0, 0.0, 0.0),
                    },
                )

        elif isinstance(item, _MeshItem):
            verts = item.vertices
            triangles = item.triangles
            colors = item.vertex_colors
            if self.config.mesh_vertex_budget and len(verts) > self.config.mesh_vertex_budget:
                verts = verts[: self.config.mesh_vertex_budget]
                if colors is not None:
                    colors = colors[: self.config.mesh_vertex_budget]
                triangles = None
            self._writer.log(
                "world/map/mesh",
                {
                    "vertices": verts,
                    "triangles": triangles,
                    "vertex_colors": colors,
                },
            )

        elif isinstance(item, _HostStreamItem):
            for k, v in item.metrics.items():
                self._writer.log(f"host/stream/{k}", v)

        elif isinstance(item, _SlamItem):
            for k, v in item.metrics.items():
                self._writer.log(f"host/slam/{k}", v)

        elif isinstance(item, _EventItem):
            self._writer.log(
                "events",
                {
                    "message": item.message,
                    "code": item.code,
                },
            )

    def stop(self, reason: str = "stopped") -> None:
        """Stop worker, close sink, and write provenance manifest."""
        with self._lock:
            if not self._active:
                return
            self._stop_event.set()
            try:
                self._queue.put_nowait(None)
            except queue.Full:
                pass

        if self._worker_thread is not None and self._worker_thread.is_alive():
            self._worker_thread.join(timeout=2.0)

        with self._lock:
            if self._writer is not None:
                try:
                    self._writer.close()
                except Exception as exc:
                    self._fault_count += 1
                    logger.warning("Error closing writer: %s", exc)

            if self._manifest_path is not None:
                try:
                    manifest_data = self._generate_manifest(reason=reason)
                    self._manifest_path.write_text(json.dumps(manifest_data, indent=2))
                except Exception as exc:
                    self._fault_count += 1
                    logger.warning("Error writing manifest: %s", exc)

            self._active = False

    def status(self) -> dict[str, Any]:
        """Return health status and counters."""
        with self._lock:
            return {
                "active": self._active,
                "samples_logged": self._samples_logged,
                "queue_drops": self._queue_drops,
                "fault_count": self._fault_count,
                "output_path": str(self._output_path) if self._output_path else None,
                "manifest_path": str(self._manifest_path) if self._manifest_path else None,
                "queue_size": self._queue.qsize(),
            }

    def _generate_manifest(self, reason: str = "stopped") -> dict[str, Any]:
        time_range = None
        if self._start_t_us is not None and self._last_t_us is not None:
            time_range = {
                "start_device_time_s": self._start_t_us / 1e6,
                "end_device_time_s": self._last_t_us / 1e6,
                "duration_s": (self._last_t_us - self._start_t_us) / 1e6,
            }
        return {
            "schema_version": 1,
            "created_at_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "recording_name": self.config.recording_name,
            "output_path": str(self._output_path) if self._output_path else None,
            "stop_reason": reason,
            "samples_logged": self._samples_logged,
            "queue_drops": self._queue_drops,
            "fault_count": self._fault_count,
            "device_time_range": time_range,
        }
