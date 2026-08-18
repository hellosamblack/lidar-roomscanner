"""Server-side raster rendering for GPU-less thin clients (`/ws-thin`, #194).

A thin client (the CrowPanel ESP32-P4 prop) has no GPU, no 3D pipeline and no
image codec: it can display raw pixels and send small JSON commands, nothing
more. So this module does the 3D work on the server and ships finished
480x480 RGB565 frames. See `docs/thin-client.md` for the wire contract and
`docs/superpowers/specs/2026-08-17-thin-client-render-design.md` for the design.

Two hard Filament constraints shape everything here. Both **abort the process**
(`utils::PreconditionPanic`, `terminate called`) rather than raising something
catchable, so the code must make them unreachable rather than handle them:

1. **One `OffscreenRenderer` per process.** Constructing a second one aborts.
   `ThinRenderer` is therefore a hard process-wide singleton.
2. **Every Filament call must run on the thread that created the renderer**
   (`JobSystem::getState(): This thread has not been adopted.`). So the
   singleton owns a dedicated render thread that both creates the renderer and
   services every job; nothing else ever touches Open3D's rendering stack.

Measured on the GPU-less dev host (llvmpipe, EGL headless, Open3D 0.19): 17 ms
per 480x480 render, 27 ms per tick including geometry churn and RGB565, and a
`tick_share` of 1.005 vs a 1.000 baseline -- Filament renders on its own
threads and releases the GIL, so a 10 fps feed costs the asyncio event loop
nothing measurable (BUG-063 methodology).
"""

from __future__ import annotations

import logging
import queue
import struct
import threading
from concurrent.futures import Future
from dataclasses import dataclass, field

import numpy as np

log = logging.getLogger(__name__)

# --- wire contract (duplicated verbatim in the CrowPanelProp companion spec) ---
THIN_TAG = 1
THIN_WIDTH = 480
THIN_HEIGHT = 480
THIN_HEADER = struct.Struct("<IHH")  # u32 tag, u16 width, u16 height

THIN_MODES = ("point_cloud", "slam", "ir")
DEFAULT_MODE = "point_cloud"

# --- camera limits ---
PITCH_LIMIT_DEG = 89.0
ZOOM_MIN = 0.25
ZOOM_MAX = 8.0

_BACKGROUND = (0.03, 0.04, 0.06, 1.0)  # matches the web UI's dark instrument look

#: Vertex budget for a thin-client SLAM render. A Detailed mesh can arrive with
#: >1 M vertices (issue #190: meshes ship far past their own 150 k budget), and
#: uploading that to Filament on llvmpipe takes tens of SECONDS -- measured
#: 1 frame in 40 s, then none in 90 s. Because every thin client shares one
#: render thread, that does not just stall the client asking for `slam`, it
#: starves the other one too. So the thin path decimates to something it can
#: actually draw at cadence rather than inheriting an upstream defect.
THIN_MESH_VERT_BUDGET = 120_000


# --------------------------------------------------------------------------
# Step 1 -- pure functions
# --------------------------------------------------------------------------


def rgba_to_rgb565(img: np.ndarray) -> bytes:
    """Convert an (H, W, 3|4) uint8 image to little-endian RGB565, row-major.

    Vectorized: the per-pixel Python loop this replaces would cost ~0.2 s per
    480x480 frame, twice the whole render budget. Any alpha channel is dropped
    -- `THIN_FRAME` has no alpha.
    """
    arr = np.asarray(img)
    if arr.ndim != 3 or arr.shape[2] not in (3, 4):
        raise ValueError(f"expected (H, W, 3|4) image, got shape {arr.shape}")
    if arr.dtype != np.uint8:
        # A float image is conventionally 0.0-1.0; clipping it to 0-255 and
        # casting would collapse the whole picture to black without erroring.
        if np.issubdtype(arr.dtype, np.floating) and float(np.nanmax(arr, initial=0.0)) <= 1.0:
            arr = arr * 255.0
        arr = np.clip(np.nan_to_num(arr), 0, 255).astype(np.uint8)
    r = arr[:, :, 0].astype(np.uint16)
    g = arr[:, :, 1].astype(np.uint16)
    b = arr[:, :, 2].astype(np.uint16)
    packed = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
    return np.ascontiguousarray(packed, dtype="<u2").tobytes()


def pack_thin_frame(pixels_rgb565: bytes, width: int = THIN_WIDTH,
                    height: int = THIN_HEIGHT) -> bytes:
    """THIN_FRAME binary (tag 1): u32 tag=1 - u16 width - u16 height -
    u8[width*height*2] RGB565, all little-endian, row-major."""
    expected = width * height * 2
    if len(pixels_rgb565) != expected:
        raise ValueError(
            f"pixel payload is {len(pixels_rgb565)} bytes, expected {expected} "
            f"for {width}x{height} RGB565")
    return THIN_HEADER.pack(THIN_TAG, width, height) + pixels_rgb565


def extract_ir_grid(rgb_or_refl: np.ndarray | None) -> list[int] | None:
    """Downsample a 2D reflectance or 3D RGB image into an 8x8 grid of 64 integers [0..255].

    Used by `thin_telemetry` to power the CrowPanel sidebar IR thumbnail widget.
    Returns None if `rgb_or_refl` is None or empty.
    """
    if rgb_or_refl is None:
        return None
    arr = np.asarray(rgb_or_refl)
    if arr.size == 0:
        return None
    if arr.ndim == 3:
        # RGB / RGBA to luminance
        gray = 0.299 * arr[:, :, 0] + 0.587 * arr[:, :, 1] + 0.114 * arr[:, :, 2]
    elif arr.ndim == 2:
        gray = arr.astype(np.float64)
        gmin = float(np.nanmin(gray))
        gmax = float(np.nanmax(gray))
        if gmax > gmin:
            gray = (gray - gmin) / (gmax - gmin) * 255.0
        else:
            gray = np.zeros_like(gray)
    else:
        return None

    h, w = gray.shape[:2]
    if h == 0 or w == 0:
        return None

    row_splits = np.array_split(np.arange(h), 8)
    col_splits = np.array_split(np.arange(w), 8)
    grid: list[int] = []
    for r_idx in range(8):
        rows = row_splits[r_idx]
        for c_idx in range(8):
            cols = col_splits[c_idx]
            block = gray[rows[:, None], cols]
            val = int(np.clip(np.round(np.mean(block)), 0, 255))
            grid.append(val)
    return grid


# --------------------------------------------------------------------------
# Step 2 -- per-connection camera state
# --------------------------------------------------------------------------


@dataclass
class ThinCamera:
    """One thin client's orbit camera. Per-connection, never shared.

    `UiState` is explicitly server-held and shared by all tabs; camera state is
    the opposite -- two thin clients must be able to look in different
    directions -- so this lives on the per-connection flow object instead.
    """

    yaw: float = 30.0
    pitch: float = 20.0
    zoom: float = 1.0
    mode: str = DEFAULT_MODE

    def apply_orbit(self, dyaw: float = 0.0, dpitch: float = 0.0,
                    dzoom: float = 0.0) -> None:
        """Accumulate relative deltas, clamped. Non-finite deltas are ignored
        rather than poisoning the camera with NaN for the rest of the session."""
        for name, delta in (("yaw", dyaw), ("pitch", dpitch), ("zoom", dzoom)):
            if delta and not np.isfinite(delta):
                log.warning("thin_orbit: ignoring non-finite d%s=%r", name, delta)
                return
        self.yaw = float((self.yaw + float(dyaw)) % 360.0)
        self.pitch = float(np.clip(self.pitch + float(dpitch),
                                   -PITCH_LIMIT_DEG, PITCH_LIMIT_DEG))
        self.zoom = float(np.clip(self.zoom + float(dzoom), ZOOM_MIN, ZOOM_MAX))

    def set_mode(self, mode: str) -> bool:
        """Switch render mode. Returns False (and changes nothing) for junk."""
        if mode not in THIN_MODES:
            log.warning("thin_mode: ignoring unknown mode %r", mode)
            return False
        self.mode = mode
        return True

    def eye(self, center: np.ndarray, radius: float) -> np.ndarray:
        """Eye position orbiting `center` at a distance set by `radius`/zoom."""
        radius = max(float(radius), 1e-3)
        dist = radius * 2.5 / max(self.zoom, ZOOM_MIN)
        yaw = np.radians(self.yaw)
        pitch = np.radians(self.pitch)
        offset = np.array([
            dist * np.cos(pitch) * np.sin(yaw),
            dist * np.sin(pitch),
            dist * np.cos(pitch) * np.cos(yaw),
        ], dtype=np.float32)
        return np.asarray(center, dtype=np.float32) + offset


# --------------------------------------------------------------------------
# Step 2 -- renderer-agnostic scene description
# --------------------------------------------------------------------------


@dataclass
class ThinScene:
    """What to draw, in a form that outlives any one rendering backend.

    `kind` is `"points"`, `"mesh"` or `"image"`. Image scenes never touch
    Filament -- they are a numpy upscale -- so the IR mode keeps working on a
    host where the offscreen context is unavailable.
    """

    kind: str
    points: np.ndarray | None = None
    colors: np.ndarray | None = None
    triangles: np.ndarray | None = None
    image: np.ndarray | None = None
    #: generation of the source that produced this scene (#101 barrier)
    generation: object = None
    meta: dict = field(default_factory=dict)

    @property
    def point_count(self) -> int:
        if self.points is None:
            return 0
        return int(len(self.points))

    def bounds(self) -> tuple[np.ndarray, float]:
        """(center, radius) of the geometry; a unit ball if there is none."""
        if self.points is None or len(self.points) == 0:
            return np.zeros(3, dtype=np.float32), 1.0
        pts = np.asarray(self.points, dtype=np.float32)
        lo = pts.min(axis=0)
        hi = pts.max(axis=0)
        center = (lo + hi) * 0.5
        radius = float(np.linalg.norm(hi - lo) * 0.5)
        return center, max(radius, 1e-3)


def points_scene(points, colors, *, generation=None) -> ThinScene:
    return ThinScene(kind="points",
                     points=np.asarray(points, dtype=np.float32),
                     colors=np.asarray(colors, dtype=np.float32),
                     generation=generation)


def image_scene(rgb, *, generation=None) -> ThinScene:
    return ThinScene(kind="image", image=np.asarray(rgb), generation=generation)


def mesh_scene(vertices, colors, triangles, *, generation=None) -> ThinScene:
    return ThinScene(kind="mesh",
                     points=np.asarray(vertices, dtype=np.float32),
                     colors=np.asarray(colors, dtype=np.float32),
                     triangles=np.asarray(triangles, dtype=np.uint32),
                     generation=generation)


def unpack_mesh_scene(mesh_bytes: bytes, *, generation=None,
                      vert_budget: int = THIN_MESH_VERT_BUDGET) -> ThinScene | None:
    """Rebuild renderable geometry from a packed MESH frame (`web.pack_mesh`).

    The SLAM path only retains the *packed* bytes (`state.latest_mesh`), so the
    thin renderer unpacks them. That costs real work on a 150 k-vertex mesh, so
    callers must cache by object identity and unpack only when the bytes
    change -- never per tick. Wall and non-wall submeshes are merged: the thin
    client has no wall toggle. Returns None if the payload is empty or short.
    """
    header = struct.Struct("<IIIIIIIII")
    if mesh_bytes is None or len(mesh_bytes) < header.size:
        return None
    (tag, mesh_seq, flags, nw_nv, nw_nt,
     w_nv, w_nt, f_np, f_nl) = header.unpack_from(mesh_bytes, 0)
    off = header.size

    def take(count: int, per: int, dtype: str):
        nonlocal off
        nbytes = count * per * np.dtype(dtype).itemsize
        if off + nbytes > len(mesh_bytes):
            raise ValueError("truncated MESH payload")
        arr = np.frombuffer(mesh_bytes, dtype=dtype, count=count * per, offset=off)
        off += nbytes
        return arr.reshape(count, per)

    try:
        nw_v = take(nw_nv, 3, "<f4")
        nw_c = take(nw_nv, 3, "<f4")
        nw_t = take(nw_nt, 3, "<u4")
        w_v = take(w_nv, 3, "<f4")
        w_c = take(w_nv, 3, "<f4")
        w_t = take(w_nt, 3, "<u4")
    except ValueError as exc:
        log.warning("thin: %s (seq=%s)", exc, mesh_seq)
        return None

    verts = np.concatenate([nw_v, w_v]) if w_nv else nw_v
    colors = np.concatenate([nw_c, w_c]) if w_nv else nw_c
    tris = np.concatenate([nw_t, w_t + nw_nv]) if w_nt else nw_t
    if len(verts) == 0 or len(tris) == 0:
        return None

    decimated_from = None
    if len(verts) > vert_budget:
        # Keep every Nth triangle, then drop the vertices no surviving triangle
        # references and reindex. Subsampling triangles (not vertices) is what
        # keeps the result a valid mesh.
        decimated_from = int(len(verts))
        # Derive the step from the TRIANGLE count, not the vertex count: each
        # kept triangle can pull in up to 3 distinct vertices, so budgeting
        # `vert_budget // 3` triangles is what actually guarantees the bound.
        max_tris = max(1, vert_budget // 3)
        step = max(1, int(np.ceil(len(tris) / max_tris)))
        tris = tris[::step]
        used, tris = np.unique(tris.reshape(-1), return_inverse=True)
        tris = tris.reshape(-1, 3).astype(np.uint32)
        verts = verts[used]
        colors = colors[used]
        if len(verts) == 0 or len(tris) == 0:
            return None

    scene = mesh_scene(verts, colors, tris, generation=generation)
    scene.meta["mesh_seq"] = int(mesh_seq)
    scene.meta["flags"] = int(flags)
    if decimated_from is not None:
        scene.meta["decimated_from_verts"] = decimated_from
        log.info("thin: decimated SLAM mesh %d -> %d verts for rendering",
                 decimated_from, len(verts))
    return scene


# --------------------------------------------------------------------------
# Step 2 -- the renderer
# --------------------------------------------------------------------------


class ThinRenderUnavailable(RuntimeError):
    """The offscreen rendering context could not be created on this host.

    `/ws-thin` turns this into a JSON error plus a clean close. It must never
    reach `roomscan.web`'s startup path or affect `/ws` / `/ws-mesh` clients.
    """


def _letterbox(rgb: np.ndarray, width: int, height: int) -> np.ndarray:
    """Nearest-neighbour upscale of a small image into a centred letterbox.

    Nearest, not bilinear, on purpose: an 18x18-ish IR frame blown up 25x
    should read as honest sensor zones, not an interpolated smear.
    """
    src = np.asarray(rgb)
    if src.ndim == 2:
        src = np.repeat(src[:, :, None], 3, axis=2)
    src = np.ascontiguousarray(src[:, :, :3]).astype(np.uint8, copy=False)
    sh, sw = src.shape[:2]
    if sh == 0 or sw == 0:
        return np.zeros((height, width, 3), dtype=np.uint8)
    if sh > height or sw > width:
        # Decimate rather than crop: cropping would silently drop the right and
        # bottom edges of the frame and look like a correct picture.
        step = int(np.ceil(max(sh / height, sw / width)))
        src = src[::step, ::step]
        sh, sw = src.shape[:2]
    scale = max(1, min(height // sh, width // sw))
    big = np.repeat(np.repeat(src, scale, axis=0), scale, axis=1)
    out = np.zeros((height, width, 3), dtype=np.uint8)
    bh, bw = big.shape[:2]
    bh, bw = min(bh, height), min(bw, width)
    y0 = (height - bh) // 2
    x0 = (width - bw) // 2
    out[y0:y0 + bh, x0:x0 + bw] = big[:bh, :bw]
    return out


class ThinRenderer:
    """Process-wide singleton owning the one offscreen context and its thread.

    Never construct this directly -- use `ThinRenderer.instance()`. A second
    `OffscreenRenderer` aborts the process, so the singleton is the safety
    mechanism, not a convenience.
    """

    _lock = threading.Lock()
    _instance: "ThinRenderer | None" = None

    def __init__(self, width: int = THIN_WIDTH, height: int = THIN_HEIGHT):
        self.width = int(width)
        self.height = int(height)
        self._jobs: queue.Queue = queue.Queue()
        self._ready = threading.Event()
        self._init_error: BaseException | None = None
        self._closed = False
        self._thread = threading.Thread(
            target=self._run, name="thin-render", daemon=True)
        self._thread.start()

    # -- lifecycle ---------------------------------------------------------

    @classmethod
    def instance(cls, *, factory=None) -> "ThinRenderer":
        """The singleton, created on first use.

        `factory` exists for tests: it replaces the whole renderer object, so a
        test can exercise `/ws-thin` without an offscreen context at all.
        """
        with cls._lock:
            if cls._instance is None:
                cls._instance = (factory or cls)()
            return cls._instance

    @classmethod
    def instance_if_created(cls) -> "ThinRenderer | None":
        """The singleton **only if something already built it**.

        Shutdown must use this, never `instance()`: closing via `instance()`
        would construct a thread, an EGL context and a Filament engine purely
        in order to destroy them, on every shutdown of every process that never
        served a thin client -- about 4 s, and an attempt at a GL context on
        hosts that have none.
        """
        with cls._lock:
            return cls._instance

    @classmethod
    def reset_singleton(cls) -> None:
        """Drop the cached instance. Tests only -- the real process keeps one
        renderer for its whole life (a replacement would abort Filament)."""
        with cls._lock:
            cls._instance = None

    def ensure_available(self, timeout: float = 30.0) -> None:
        """Block until the context is up, or raise `ThinRenderUnavailable`.

        The failure is cached by the thread, so every later connection gets the
        same answer immediately instead of retrying a 4-second context
        creation per connect.
        """
        if not self._ready.wait(timeout):
            raise ThinRenderUnavailable(
                f"offscreen renderer did not initialise within {timeout:.0f}s")
        if self._init_error is not None:
            raise ThinRenderUnavailable(
                f"offscreen renderer unavailable: {self._init_error}")

    @property
    def available(self) -> bool:
        return self._ready.is_set() and self._init_error is None

    def close(self, timeout: float = 5.0) -> None:
        """Shut the render thread down and destroy the context **on its owning
        thread**.

        Not optional hygiene: letting the interpreter garbage-collect a live
        `OffscreenRenderer` from the main thread ends the process with
        `pure virtual method called` / `terminate called`, turning a clean
        `roomscan-web` shutdown into an abort. `_run` drops its reference before
        exiting, and `atexit` calls this if the lifespan never did.
        """
        if self._closed:
            return
        self._closed = True
        self._jobs.put(None)
        if self._thread.is_alive():
            self._thread.join(timeout)

    # -- submission --------------------------------------------------------

    def submit(self, scene: ThinScene, camera: ThinCamera, *,
               pack: bool = True) -> Future:
        """Queue a render. The future resolves to packed `THIN_FRAME` bytes
        (`pack=True`) or an (H, W, 3) uint8 array.

        Image scenes are resolved inline -- they are a numpy upscale, need no
        rendering context, and would only add a thread hop.
        """
        fut: Future = Future()
        if self._closed:
            fut.set_exception(ThinRenderUnavailable("renderer is closed"))
            return fut
        if scene is not None and scene.kind == "image":
            try:
                fut.set_result(self._finish(
                    _letterbox(scene.image, self.width, self.height), pack))
            except BaseException as exc:  # noqa: BLE001 - reported to the caller
                fut.set_exception(exc)
            return fut
        self._jobs.put((scene, camera, pack, fut))
        return fut

    def render(self, scene: ThinScene, camera: ThinCamera, *,
               pack: bool = True, timeout: float = 30.0):
        """Synchronous convenience wrapper. Do not call from the event loop."""
        return self.submit(scene, camera, pack=pack).result(timeout)

    def _finish(self, rgb: np.ndarray, pack: bool):
        if not pack:
            return rgb
        return pack_thin_frame(rgb_to_bytes(rgb), self.width, self.height)

    # -- the owning thread -------------------------------------------------

    def _run(self) -> None:
        renderer = None
        o3d = None
        try:
            import open3d as o3d  # noqa: PLC0415 - deferred: 4 s + a GL context
            renderer = o3d.visualization.rendering.OffscreenRenderer(
                self.width, self.height)
            renderer.scene.set_background(list(_BACKGROUND))
        except BaseException as exc:  # noqa: BLE001 - any failure is "unavailable"
            self._init_error = exc
            log.warning("thin renderer unavailable: %s: %s",
                        type(exc).__name__, exc)
        finally:
            self._ready.set()

        if self._init_error is not None:
            self._drain()
            return

        import atexit  # noqa: PLC0415 - only meaningful once a context exists
        atexit.register(self.close)

        current_key = None
        while True:
            job = self._jobs.get()
            if job is None:
                # Destroy the context here, on the thread that created it.
                renderer = None
                self._fail_pending(ThinRenderUnavailable("renderer is closed"))
                return
            scene, camera, pack, fut = job
            if not fut.set_running_or_notify_cancel():
                continue
            try:
                current_key = self._draw(o3d, renderer, scene, camera, current_key)
                rgb = np.asarray(renderer.render_to_image())
                fut.set_result(self._finish(rgb, pack))
            except BaseException as exc:  # noqa: BLE001 - reported to the caller
                current_key = None
                fut.set_exception(exc)

    def _drain(self) -> None:
        """Serve every later submission the cached init failure, so a host
        without a rendering context fails fast instead of hanging a client."""
        exc = ThinRenderUnavailable(
            f"offscreen renderer unavailable: {self._init_error}")
        while True:
            job = self._jobs.get()
            if job is None:
                self._fail_pending(exc)
                return
            _scene, _camera, _pack, fut = job
            fut.set_exception(exc)

    def _fail_pending(self, exc: BaseException) -> None:
        """Resolve anything still queued at shutdown -- a caller awaiting a
        future that will never be serviced would hang until its own timeout."""
        while True:
            try:
                job = self._jobs.get_nowait()
            except queue.Empty:
                return
            if job is None:
                continue
            _scene, _camera, _pack, fut = job
            if not fut.done():
                fut.set_exception(exc)

    def _draw(self, o3d, renderer, scene: ThinScene, camera: ThinCamera,
              current_key):
        """Sync the Filament scene to `scene` and aim the camera. Render-thread
        only. Returns the geometry key now resident in the scene.

        Geometry is re-uploaded only when its identity changes: at 10 fps a
        static SLAM mesh would otherwise be torn down and rebuilt 10x/s for
        nothing. The camera is re-aimed every tick regardless -- that is the
        cheap part, and it is what the client's orbit commands move.
        """
        rendering = o3d.visualization.rendering
        key = (scene.kind, id(scene.points), id(scene.colors), id(scene.triangles))
        if key != current_key:
            renderer.scene.clear_geometry()
            mat = rendering.MaterialRecord()
            if scene.kind == "mesh":
                # Unlit when the mesh carries vertex colors -- which SLAM
                # meshes always do. An `OffscreenRenderer` scene has NO light
                # by default, so `defaultLit` renders near-black: the room's
                # shape was there but the frame was unreadable until this
                # changed. Unlit also matches how the point-cloud mode draws.
                has_colors = (scene.colors is not None
                              and len(scene.colors) == len(scene.points))
                mat.shader = "defaultUnlit" if has_colors else "defaultLit"
                geom = o3d.geometry.TriangleMesh()
                geom.vertices = o3d.utility.Vector3dVector(
                    np.asarray(scene.points, dtype=np.float64))
                geom.triangles = o3d.utility.Vector3iVector(
                    np.asarray(scene.triangles, dtype=np.int32))
                if scene.colors is not None and len(scene.colors) == len(scene.points):
                    geom.vertex_colors = o3d.utility.Vector3dVector(
                        np.clip(np.asarray(scene.colors, dtype=np.float64), 0.0, 1.0))
                geom.compute_vertex_normals()
            else:
                mat.shader = "defaultUnlit"
                mat.point_size = 4.0
                geom = o3d.geometry.PointCloud()
                geom.points = o3d.utility.Vector3dVector(
                    np.asarray(scene.points, dtype=np.float64))
                if scene.colors is not None and len(scene.colors) == len(scene.points):
                    geom.colors = o3d.utility.Vector3dVector(
                        np.clip(np.asarray(scene.colors, dtype=np.float64), 0.0, 1.0))
            renderer.scene.add_geometry("thin", geom, mat)
            if mat.shader == "defaultLit":
                # A lit material with no light source renders black.
                renderer.scene.scene.add_sun_light(
                    [-0.5, -0.8, -0.3], [1.0, 1.0, 1.0], 75000.0)
            current_key = key

        center, radius = scene.bounds()
        eye = camera.eye(center, radius)
        renderer.setup_camera(60.0,
                              np.asarray(center, dtype=np.float32),
                              np.asarray(eye, dtype=np.float32),
                              np.array([0.0, 1.0, 0.0], dtype=np.float32))
        return current_key


def rgb_to_bytes(rgb: np.ndarray) -> bytes:
    """RGB565 bytes for an (H, W, 3|4) uint8 image."""
    return rgba_to_rgb565(rgb)
