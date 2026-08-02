// magcal3d.js — the "Shell & Steering" 3D renderer for the magnetometer
// calibration modal (owner ask, 2026-07-29; design
// docs/superpowers/specs/completed/2026-07-29-magcal-3d-feedback-design.md, Phase 1).
//
// THE PROBLEM THIS SOLVES
// -----------------------
// > "the goal is to make it clear to the user what has been done/what needs to
// >  be done so they don't feel like a crazy person just waving this thing
// >  around in the air with no idea what the goal and progress is."
//
// The 2D disc pair (magcal.js) already answers "what's missing" honestly. What
// it cannot do is show the DEVICE, and the device is what carries the "oh, THAT
// is what I have to do with my hands" moment. Hence two cameras on one scene.
//
// ONE SCENE, ONE PASS, TWO POSSIBLE CAMERAS  (merged 2026-07-31)
// --------------------------------------------------------------
// Everything lives in `bodyGroup`, in SFLP BODY axes (X = Up, Y = Right,
// Z = Forward): the 92 coverage cells, the device model, the field arrow at
// `field_dir_body`, gravity at `gravity_body`.
//
// Until 2026-07-31 this file rendered that one scene TWICE — a body-fixed
// "hero" with the full cell styling, and a small world-fixed "steering" widget
// with a ghost to fly the board into. The owner asked for one view: *"Can they
// be combined? The orientation from the bottom one, with coverage from the top
// one?"* They can, and the merge also halves the WebGL contexts, which is not
// nothing on a software renderer.
//
//   MERGED (world-fixed camera, the normal case):
//       bodyGroup.matrix = T_WORLD_TO_CV · R, camera = `steerCam`.
//       The device and its shell tumble in the operator's hand; the field
//       arrow, the ghost and the target all stand still in the room — because
//       `R · (Rᵀ·b_world) = b_world`, i.e. the Earth's field genuinely is fixed
//       there. Cells keep the HERO styling (dashed hollow rings for missing,
//       discs sized by sample count, hue = |B| deviation), not the old faint
//       steering wash, because the shell is now the only coverage map on
//       screen. The device fill stays ghosted at 0.30: the §7 block is 8×
//       deeper than the old board silhouette and would otherwise blank the
//       middle of the shell, exactly where the boresight, the target ring and
//       the geodesic live.
//
//   FALLBACK (body-fixed first-person camera):  bodyGroup.matrix = identity,
//       camera = `heroCam`. Used whenever there is no orientation —
//       `have_quat` false, no `t_world_to_cv` yet, a ToF-only session. This is
//       NOT decoration: `cell_dirs` and `field_dir_body` are already body
//       vectors, so this framing renders a completely correct coverage map with
//       zero IMU data, where the merged framing would render an empty canvas.
//       The camera sits behind the device on its boresight looking where the
//       ToF camera looks, gravity-levelled (`camera.up` tracks −g, § heroUp),
//       the same rule the live view's FPV mode applies to the cloud.
//
// THE COST WE RE-ACCEPTED, ON PURPOSE
// -----------------------------------
// This file used to record that a world-fixed hero had been REJECTED, because
// a body-fixed shell seen from a room-fixed camera has its holes orbit at hand
// speed: you can see *that* there is a gap and never *where*, nor whether it is
// shrinking. That reasoning was not wrong and is not deleted — it is
// reconsidered. The owner wants the targets to live in the room, and the
// mitigation is that the aiming instrument is no longer the shell: it is the
// **ghost** (fly the block into the wireframe) and the **geodesic leader line**
// (follow the dashes), both of which are world-fixed and therefore steady under
// exactly the motion that makes the shell swim. The shell reverts to being the
// progress readout it always was. If this proves unusable on the rig the
// fallback is the inverse merge — body-fixed camera with the ghost drawn into
// it — same closure condition either way.
//
// Consequence of a moving camera: the near/far cell split (which cells are
// BEHIND the eye and therefore drawn translucent) is no longer static. It is
// recomputed against the camera direction expressed in BODY coordinates, at
// ~10 Hz — see `updateBehind`.
//
// FRAMES — WHAT THIS FILE IS ALLOWED TO COMPUTE
// ---------------------------------------------
// Per docs/coordinate-frames.md and the "server-side math stays server-side"
// invariant: NO sign or permutation matrix is ever written here. `T_WORLD_TO_CV`
// arrives as `t_world_to_cv[9]` (row-major) on the `magcal` open report and is
// applied as a static matrix; the guidance rotation arrives as an explicit
// body `axis` + `angle_deg`. The only per-frame math is quaternion slerp.
//
// This module is PRESENTATION ONLY. It reads two server channels and draws.
// It cannot reach `display_rotation`, the point cloud, `fused_quat()`, or
// anything SLAM sees.
//
// COLOUR (dataviz skill consulted; validator numbers in the design §9)
// --------------------------------------------------------------------
//   Hue means exactly ONE thing here: |B| deviation.  Everything else is ink,
//   shape, size or motion.
//     covered vs missing  -> SHAPE  (filled disc vs dashed hollow ring + stipple)
//     sample density      -> SIZE   (disc radius ramps over 1..8 samples)
//     next target         -> double ring + pulse + geodesic leader line
//     trail recency       -> a NEUTRAL ordinal ramp (hue is spoken for)
//   That is why there is no RGB axis triad on the device model: it would put red
//   and blue on screen meaning "axis" while red and blue already mean "|B| too
//   strong / too weak".
//
// GRACEFUL DEGRADATION (§8.5) — this must never take the modal down.
//   getContext/three throws  -> 2D Lambert fallback (magcal.js, unmodified)
//   webglcontextlost         -> same; no automatic restore attempt
//   ?magcal2d=1              -> forced fallback, so both paths get screenshotted
//   ?magcalstatic=1          -> freeze the near/far split (the pre-merge
//                               behaviour), so the per-frame recompute's cost
//                               can be MEASURED rather than asserted
//   have_quat false          -> body-fixed fallback framing + the note
//
// Public surface:
//   chooseRenderer({force2d, webglOk}) -> 'webgl' | '2d'        (pure; testable)
//   decodeMagpose(ArrayBuffer)         -> pose object | null    (pure)
//   createMagcal3d({heroCanvas, note}) -> handle

import * as THREE from 'three';
import { createDeviceMesh, createDeviceGhost } from './devicemodel.js';

const D = (m, l) => { try { window.__diag && window.__diag('magcal3d: ' + m, l); } catch (e) {} };

// --- design tokens (mirrored from index.html; WebGL can't read CSS vars) -----
const INK = 0xe2e8f0;
const MUTED = 0x94a3b8;
const SURFACE = 0x16181e;
const DEV_COOL = [96, 165, 250];      // #60a5fa — |B| reads LOW
const DEV_MID = [100, 116, 139];      // #64748b — |B| correct (NEUTRAL midpoint)
const DEV_WARM = [239, 68, 68];       // #ef4444 — |B| reads HIGH
const DEV_CLAMP = 30.0;               // percent, clamped so the shipped defect saturates
const EMPTY_INK = [226, 232, 240];    // missing-cell outline (depth cue is alpha, not ink)
// Ordinal neutral trail ramp — validated `--ordinal`: monotone L, adjacent
// ΔL ≥ 0.06, light end 2.97:1 vs surface, hue spread 5°.
const TRAIL_RAMP = [[91, 100, 114], [139, 148, 163], [183, 192, 207], [226, 232, 240]];

// --- geometry constants ------------------------------------------------------
const SHELL_R = 1.0;
// The lattice's own cell angular radius is ~12° (tan ≈ 0.205), but drawing the
// marks at full cell size makes 92 tangent discs tessellate into a single noisy
// crust with no gaps between marks — verified on screen. Drawn at ~3/4 size the
// individual cells separate and the shell reads as a countable map, which is the
// entire point of the hero.
const CELL_BASE_R = 0.150;
const CELL_MIN_R = 0.32;        // fraction of CELL_BASE_R at 1 sample
const DENSITY_FULL = 8;         // samples at which a cell reads "well visited"
const TRAIL_LEN = 90;           // 3 s at 30 Hz
const RENDER_LAG_MS = 33;       // one pose interval: interpolate, never extrapolate

// Hero framing. The camera is on the −boresight axis looking at the origin, so
// it sees the shell head-on from where the sensor sits. `HERO_DIST` is the
// standoff: at the optical centre the shell is a wall in every direction and you
// see one cell, so a non-zero standoff is structural, not cosmetic (the same
// point the live view's FPV framing makes). It is set so the half-height at
// `HERO_FOV` clears the |B| tick at r ≈ 1.56.
const HERO_FOV = 40;
const HERO_DIST = 4.3;
// Camera-roll smoothing. The roll is a CAMERA property, never applied to a
// mark: gravity is noisy at ±0.05°, and a shell that shivers reads as a broken
// instrument. 0.12 s is below hand-motion timescale, so steering still feels direct.
const UP_TAU_S = 0.12;
// Below this much of g perpendicular to the boresight, "level" is undefined
// (aimed at the floor or the ceiling) — hold the last roll rather than spin.
const UP_MIN = 0.12;

const FACES = [
    ['Top', [1, 0, 0]], ['Bottom', [-1, 0, 0]],
    ['Right', [0, 1, 0]], ['Left', [0, -1, 0]],
    ['Front', [0, 0, 1]], ['Back', [0, 0, -1]],
];

// =============================================================================
// Pure helpers (no THREE, no DOM) — these are the bits worth asserting on.
// =============================================================================

/** The graceful-degradation decision, isolated so it can be asserted rather
 *  than inferred from a screenshot. `force2d` is the `?magcal2d=1` escape
 *  hatch; `webglOk` is whether a context was actually obtained. */
export function chooseRenderer({ force2d = false, webglOk = true } = {}) {
    if (force2d) return '2d';
    return webglOk ? 'webgl' : '2d';
}

/** MAGPOSE (binary tag 5, 68 bytes) -> a plain object, or null if the frame is
 *  the wrong size. Layout mirrors web.py `_MAGPOSE` exactly; keep them in
 *  lockstep (docs/web-protocol.md). */
export function decodeMagpose(buffer) {
    if (!(buffer instanceof ArrayBuffer) || buffer.byteLength !== 68) return null;
    const v = new DataView(buffer);
    return {
        seq: v.getUint32(4, true),
        quat: [v.getFloat32(8, true), v.getFloat32(12, true),
               v.getFloat32(16, true), v.getFloat32(20, true)],   // w,x,y,z
        dir: [v.getFloat32(24, true), v.getFloat32(28, true), v.getFloat32(32, true)],
        gravity: [v.getFloat32(36, true), v.getFloat32(40, true), v.getFloat32(44, true)],
        fieldUt: v.getFloat32(48, true),
        devPct: v.getFloat32(52, true),
        dipDeg: v.getFloat32(56, true),
        liveCell: v.getInt16(60, true),
        filledCell: v.getInt16(62, true),
        flags: v.getUint16(64, true),
    };
}

export const POSE_COLLECTING = 1 << 0;
export const POSE_STATIONARY = 1 << 1;
export const POSE_ANOMALY = 1 << 2;
export const POSE_HAVE_QUAT = 1 << 3;
export const POSE_PROVISIONAL = 1 << 4;
export const POSE_REJECTED = 1 << 5;

function lerpRgb(a, b, t) {
    return [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t];
}

/** Signed |B| deviation percent -> [r,g,b] 0..255 on the diverging ramp.
 *  Two hues + a NEUTRAL grey midpoint, never a hue at the midpoint. */
export function devRgb(pct) {
    if (pct === null || pct === undefined || !isFinite(pct)) return [148, 163, 184];
    const t = Math.max(-1, Math.min(1, pct / DEV_CLAMP));
    return t < 0 ? lerpRgb(DEV_MID, DEV_COOL, -t) : lerpRgb(DEV_MID, DEV_WARM, t);
}

/** Sample count -> cell disc radius, as a fraction of the full cell.
 *  A cell grabbed by one stray sample must not be able to masquerade as solid
 *  coverage, so "barely visited" is a SIZE, not a shade. */
export function densityRadius(count) {
    if (!count) return 0;
    const t = Math.min(1, Math.max(0, (count - 1) / (DENSITY_FULL - 1)));
    return CELL_MIN_R + (1 - CELL_MIN_R) * t;
}

// =============================================================================
// Geometry builders
// =============================================================================

/** A hollow DASHED ring with a stipple of interior dots — the "missing" mark.
 *  Dash + stipple rather than a plain thin outline because a back-hemisphere
 *  cell is drawn dimmed, where a bare 1px dash disappears. Built once, reused by
 *  all 92 instances. Lies in the XY plane with its normal on +Z. */
function dashedRingGeometry(rIn, rOut, dashes, dutyFrac, seg) {
    const pos = [];
    const idx = [];
    const period = (Math.PI * 2) / dashes;
    let base = 0;
    for (let d = 0; d < dashes; d++) {
        const a0 = d * period;
        const a1 = a0 + period * dutyFrac;
        for (let s = 0; s <= seg; s++) {
            const a = a0 + (a1 - a0) * (s / seg);
            const c = Math.cos(a), si = Math.sin(a);
            pos.push(rIn * c, rIn * si, 0, rOut * c, rOut * si, 0);
        }
        for (let s = 0; s < seg; s++) {
            const i = base + s * 2;
            idx.push(i, i + 1, i + 2, i + 1, i + 3, i + 2);
        }
        base += (seg + 1) * 2;
    }
    // Stipple: one tiny centre pip, so a missing cell stays legible at the small
    // dimmed size the far hemisphere gets, where a bare dash goes sub-pixel.
    // Deliberately ONE, and the dash count deliberately low: 92 cells x N marks
    // is a confetti field long before it is a map — verified on screen at 8
    // dashes + 3 pips, which read as noise rather than as countable holes.
    const dot = rIn * 0.22;
    pos.push(-dot, -dot, 0, dot, -dot, 0, -dot, dot, 0, dot, dot, 0);
    idx.push(base, base + 1, base + 2, base + 1, base + 3, base + 2);
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    g.setIndex(idx);
    return g;
}

/** Text label as a canvas sprite. Six of these name the body faces at r = 1.15,
 *  using the SAME six names `magsweep.FACES` uses — so the guidance sentence and
 *  the picture name the same thing. */
function labelSprite(text, colorCss) {
    const c = document.createElement('canvas');
    c.width = 128; c.height = 48;
    const ctx = c.getContext('2d');
    ctx.font = '600 26px Inter, sans-serif';
    ctx.fillStyle = colorCss;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, 64, 24);
    const tex = new THREE.CanvasTexture(c);
    tex.colorSpace = THREE.SRGBColorSpace;
    const sp = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, transparent: true, depthTest: false }));
    sp.scale.set(0.42, 0.16, 1);
    return sp;
}

// The device model and its ghost now come from `devicemodel.js` — the real
// instrument (a 5.5 x 3 x 2.5 inch block, grey/white/blue through the depth,
// camera on the blue face), shared verbatim with the Sensors card's orientation
// gizmo so the two views cannot drift into teaching different shapes. What used
// to live here was a bare-NUCLEO board silhouette: right for the reference
// firmware's dev board, wrong for the thing in the owner's hand, and 8x too
// thin. `devicemodel.MOUNT_ROTATION` carries the "+X is the device's BOTTOM"
// correction that the old model had backwards.

function lineFrom(points, color, opacity, dashed) {
    const g = new THREE.BufferGeometry().setFromPoints(points);
    const m = dashed
        ? new THREE.LineDashedMaterial({ color, transparent: true, opacity,
                                         dashSize: 0.06, gapSize: 0.045 })
        : new THREE.LineBasicMaterial({ color, transparent: true, opacity });
    const l = new THREE.Line(g, m);
    if (dashed) l.computeLineDistances();
    return l;
}

/** Slerp on the sphere: writes the great-circle arc between two unit directions
 *  into a PRE-ALLOCATED (n+1)*3 array. Used for the geodesic leader line (the
 *  path the head dot will actually trace, so following it is self-verifying) and
 *  for the dip arc. Writes in place because both are updated per pose and this
 *  view allocates nothing per frame. */
function writeArc(out, a, b, radius, n) {
    let ax = a[0], ay = a[1], az = a[2];
    let bx = b[0], by = b[1], bz = b[2];
    const la = Math.hypot(ax, ay, az) || 1, lb = Math.hypot(bx, by, bz) || 1;
    ax /= la; ay /= la; az /= la; bx /= lb; by /= lb; bz /= lb;
    const theta = Math.acos(Math.max(-1, Math.min(1, ax * bx + ay * by + az * bz)));
    const sin = Math.sin(theta);
    for (let i = 0; i <= n; i++) {
        const t = i / n;
        let w1, w2;
        if (theta < 1e-4 || sin < 1e-6) { w1 = 1 - t; w2 = t; }
        else { w1 = Math.sin((1 - t) * theta) / sin; w2 = Math.sin(t * theta) / sin; }
        let x = ax * w1 + bx * w2, y = ay * w1 + by * w2, z = az * w1 + bz * w2;
        const l = Math.hypot(x, y, z) || 1;
        const s = radius / l;
        out[i * 3] = x * s; out[i * 3 + 1] = y * s; out[i * 3 + 2] = z * s;
    }
}

/** A Line backed by a fixed-size position buffer, so updates are writes rather
 *  than geometry rebuilds. */
function polyline(nPoints, color, opacity, dashed) {
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(nPoints * 3), 3));
    const m = dashed
        ? new THREE.LineDashedMaterial({ color, transparent: true, opacity,
                                         dashSize: 0.055, gapSize: 0.04 })
        : new THREE.LineBasicMaterial({ color, transparent: true, opacity });
    const l = new THREE.Line(g, m);
    l.frustumCulled = false;
    return l;
}

// =============================================================================
// The renderer
// =============================================================================

export function createMagcal3d(opts) {
    const heroCanvas = opts.heroCanvas;
    // The "no orientation — connect the IMU" line. It used to belong to the
    // Steering widget; the merged view still needs it, because the fallback
    // framing is a real coverage map but it CANNOT show the ghost, and a
    // missing ghost with no explanation reads as a broken feature.
    const note = opts.note || null;
    const force2d = opts.force2d === true;
    // Debug escape: freeze the near/far split at its body-fixed value, i.e. the
    // pre-merge behaviour. Exists so the per-frame recompute below can be
    // measured against a real baseline in the same build (docs/web-ui-testing.md).
    const staticBehind = opts.staticBehind === true;

    const reduceMotion = !!(window.matchMedia
        && window.matchMedia('(prefers-reduced-motion: reduce)').matches);

    const stats = { renderer: '2d', frames: 0, cells: 0, covered: 0, ghosted: 0,
                    upDeg: 0, lastPoseMs: 0, poseHz: 0, reason: null,
                    // Merged-view instrumentation: how often the near/far split
                    // is recomputed and what one recompute+refresh costs, so
                    // "throttled to 10 Hz is cheap" is a measurement.
                    framing: 'body', behindRecomputes: 0, refreshMs: 0,
                    staticBehind };

    let heroRenderer = null;
    if (chooseRenderer({ force2d, webglOk: true }) === '2d') {
        stats.reason = 'forced by ?magcal2d=1';
        D('renderer=2d (' + stats.reason + ')');
        return degraded(stats);
    }
    try {
        heroRenderer = new THREE.WebGLRenderer({ canvas: heroCanvas, antialias: true, alpha: true });
    } catch (e) {
        stats.reason = 'WebGL unavailable: ' + (e && e.message);
    }
    if (chooseRenderer({ force2d, webglOk: heroRenderer !== null }) === '2d') {
        D('renderer=2d (' + stats.reason + ')', 'error');
        return degraded(stats);
    }
    stats.renderer = 'webgl';

    // ---- scene ------------------------------------------------------------
    const scene = new THREE.Scene();
    const bodyGroup = new THREE.Group();
    bodyGroup.matrixAutoUpdate = false;
    scene.add(bodyGroup);
    const ghostGroup = new THREE.Group();
    ghostGroup.matrixAutoUpdate = false;
    ghostGroup.visible = false;
    scene.add(ghostGroup);

    // Layers: 0 = shared, 1 = face labels, 2 = the ghost + wrist arrow. Both
    // cameras enable 1; only the world-fixed one enables 2, because a ghost
    // drawn under the body-fixed fallback framing would sit exactly on top of
    // the device and mean nothing (no orientation ⇒ no ΔR to fly toward).
    const setLayer = (obj, n) => obj.traverse((o) => o.layers.set(n));

    // ---- the shell: four meshes, because translucency is per-MATERIAL ------
    // A cell is one of {covered, missing} × {in front of the camera, behind it},
    // and `InstancedMesh`'s per-instance channel is colour, not alpha — so the
    // near/far split has to be a material split, i.e. a mesh split. Every cell
    // has an instance slot in all four; the three that don't apply are parked at
    // scale 0. Cheap (4 draw calls). The split used to be STATIC because the
    // only camera was fixed in body axes; the merged view's camera is fixed in
    // the ROOM, so it is recomputed — see `updateBehind`, and note that the
    // recompute is a rewrite of 4 x 92 instance matrices, not a rebuild.
    //
    // Translucency goes on the cells BEHIND the camera, not the far ones. The
    // usual depth cue fades the far side, but here the near cap covers the
    // entire silhouette — fading the far side would hide precisely the
    // hemisphere you are steering into. Ghosting the near cap instead lets you
    // see the aim hemisphere through it, and a hole behind you still reads.
    const N = 92;
    const cellMat = (opacity) => new THREE.MeshBasicMaterial({
        transparent: true, opacity, side: THREE.DoubleSide, depthWrite: false });
    const discGeom = new THREE.CircleGeometry(1, 20);
    const ringGeom = dashedRingGeometry(0.62, 1.0, 4, 0.62, 3);
    // One styling, in both framings: the full coverage map. There used to be a
    // second, faint "steering" wash for the small world-fixed widget, where the
    // shell was a backdrop and the ghost carried the message. After the merge
    // there is no other coverage map on screen, so the wash would simply be a
    // coverage readout nobody can read (owner ask, 2026-07-31).
    const CELL_MESHES = [
        { filled: true,  ghost: false, geom: discGeom, opacity: 0.90 },
        { filled: true,  ghost: true,  geom: discGeom, opacity: 0.26 },
        { filled: false, ghost: false, geom: ringGeom, opacity: 0.95 },
        { filled: false, ghost: true,  geom: ringGeom, opacity: 0.30 },
    ];
    for (const c of CELL_MESHES) {
        c.mat = cellMat(c.opacity);
        c.mesh = new THREE.InstancedMesh(c.geom, c.mat, N);
        c.mesh.frustumCulled = false;
        c.mesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
        // Back-to-front: the ghosted near cap composites OVER the solid far side.
        c.mesh.renderOrder = c.ghost ? 2 : 1;
        bodyGroup.add(c.mesh);
    }

    // The block, ghosted: 0.30 in BOTH framings now. The old steering pass drew
    // it solid at 0.85 because its shell was a wash and the mechanic was landing
    // a solid body inside a wireframe; with the full coverage map behind it, a
    // block 8x deeper than the old board would blank the middle of the shell.
    const device = createDeviceMesh(THREE, { fillOpacity: 0.30 });
    bodyGroup.add(device);

    const ghost = createDeviceGhost(THREE, { color: INK });
    setLayer(ghost, 2);
    ghostGroup.add(ghost);

    // Face labels, on layer 1 so BOTH framings show them: they are sprites, so
    // the text stays screen-aligned and readable however the body tumbles, and
    // "Top is over there now" is exactly what the guidance sentence ("roll ≈70°,
    // Top → Back") needs you to be able to see. Front/Back are still skipped:
    // under the body-fixed fallback framing the boresight projects to a point
    // and both would land stacked on the device at screen centre, naming
    // something the picture already says.
    const labels = new THREE.Group();
    for (const [name, dir] of FACES) {
        if (Math.abs(dir[2]) > 0.9) continue;
        const s = labelSprite(name, '#94a3b8');
        s.position.set(dir[0] * 1.18, dir[1] * 1.18, dir[2] * 1.18);
        labels.add(s);
    }
    setLayer(labels, 1);
    bodyGroup.add(labels);

    // Comet trail: the last 3 s of the body-frame field direction.
    // The covered cells already ARE the long-term history (that is what a cell
    // IS), so the polyline never needs to be long — its job is the DERIVATIVE
    // (am I moving, which way, how fast), the cells' is the INTEGRAL.
    const trailPos = new Float32Array(TRAIL_LEN * 3);
    const trailCol = new Float32Array(TRAIL_LEN * 3);
    const trailGeom = new THREE.BufferGeometry();
    trailGeom.setAttribute('position', new THREE.BufferAttribute(trailPos, 3));
    trailGeom.setAttribute('color', new THREE.BufferAttribute(trailCol, 3));
    trailGeom.setDrawRange(0, 0);
    const trail = new THREE.Line(trailGeom, new THREE.LineBasicMaterial({
        vertexColors: true, transparent: true, opacity: 0.95 }));
    trail.frustumCulled = false;
    bodyGroup.add(trail);
    const trailPts = [];   // ring of [x,y,z], newest last

    // "You are here": head dot + a stem to the device centre.
    const head = new THREE.Mesh(new THREE.SphereGeometry(0.045, 12, 10),
        new THREE.MeshBasicMaterial({ color: INK }));
    bodyGroup.add(head);
    const stem = lineFrom([new THREE.Vector3(), new THREE.Vector3(0, 0, 1)], INK, 0.30, false);
    bodyGroup.add(stem);

    // Field arrow B (ink, solid) + gravity arrow g (muted, dashed).
    const bLine = lineFrom([new THREE.Vector3(), new THREE.Vector3(0, 0, 1)], INK, 0.95, false);
    const bTip = new THREE.Mesh(new THREE.ConeGeometry(0.045, 0.13, 12),
        new THREE.MeshBasicMaterial({ color: INK }));
    const gLine = lineFrom([new THREE.Vector3(), new THREE.Vector3(0, 0, 1)], MUTED, 0.7, true);
    const gTip = new THREE.Mesh(new THREE.ConeGeometry(0.035, 0.10, 10),
        new THREE.MeshBasicMaterial({ color: MUTED, transparent: true, opacity: 0.7 }));
    bodyGroup.add(bLine, bTip, gLine, gTip);

    // The live |B| magnitude tick at the arrow head: drawn OUTWARD when |B|
    // reads high, INWARD when low, length ∝ |dev%|. Redundantly encoded
    // (direction + length + colour), so |B| error is legible continuously and
    // not only at fit time.
    const devTick = lineFrom([new THREE.Vector3(), new THREE.Vector3(0, 0, 1)], 0xffffff, 1.0, false);
    devTick.material.linewidth = 2;
    bodyGroup.add(devTick);

    // The dip arc B∠g. For a CORRECT calibration this angle is a constant of
    // the location (90° + magnetic dip): both vectors are world-fixed, so their
    // mutual angle cannot depend on attitude. Immune to scale error — it catches
    // faults a self-consistent-but-wrong-magnitude calibration sails past.
    const DIP_SEG = 20;
    const dipArc = polyline(DIP_SEG + 1, MUTED, 0.55, false);
    bodyGroup.add(dipArc);

    // The geodesic from HERE to the next target, dashed.
    const GEO_SEG = 32;
    const geodesic = polyline(GEO_SEG + 1, INK, 0.6, true);
    geodesic.visible = false;
    bodyGroup.add(geodesic);

    // Next target: a DOUBLE RING (+ pulse), never a colour — hue is spoken for.
    const targetGroup = new THREE.Group();
    const ringMat = new THREE.MeshBasicMaterial({ color: INK, transparent: true,
                                                  opacity: 0.95, side: THREE.DoubleSide });
    const ringOuter = new THREE.Mesh(new THREE.RingGeometry(0.235, 0.265, 28), ringMat);
    const ringInner = new THREE.Mesh(new THREE.RingGeometry(0.155, 0.180, 28), ringMat);
    targetGroup.add(ringOuter, ringInner);
    targetGroup.visible = false;
    bodyGroup.add(targetGroup);

    let wristArrow = null;    // curved arrow around the rotation axis (ghost layer)

    // ---- cameras ----------------------------------------------------------
    // FALLBACK (body-fixed, first-person). Stands off behind the device on the
    // boresight (body −Z) and looks along it, so the shell is seen from where
    // the ToF camera is. Position FIXED in body axes; the only thing that moves
    // is `up`, which tracks gravity so the view never rolls with the board (see
    // updateHeroUp). Used when there is no orientation — the framing needs no
    // quaternion at all, which is exactly why it survived the merge.
    const heroCam = new THREE.PerspectiveCamera(HERO_FOV, 1, 0.1, 30);
    heroCam.up.set(1, 0, 0);                       // body Up, until gravity says otherwise
    heroCam.position.set(0, 0, -HERO_DIST);
    heroCam.lookAt(0, 0, 0);
    heroCam.layers.enable(1);

    // MERGED (world-fixed room camera), up = (0,-1,0) matching scene.js's
    // Open3D CV world, so "up on screen" is up in the room.
    const steerCam = new THREE.PerspectiveCamera(42, 1, 0.1, 30);
    steerCam.up.set(0, -1, 0);
    steerCam.position.set(1.5, -1.15, -2.3).setLength(3.4);
    steerCam.lookAt(0, 0, 0);
    steerCam.layers.enable(1);
    steerCam.layers.enable(2);

    // ---- state ------------------------------------------------------------
    const IDENT = new THREE.Matrix4();
    let tWorldToCv = null;       // THREE.Matrix4 from the server's 9 numbers
    let cellDirs = null;         // [92][3], cached from the `open` report
    let counts = null;
    let devPcts = null;
    let behind = null;           // near/far split, recomputed by updateBehind
    let report = null;
    let haveQuat = false;
    let running = false;
    let rafId = 0;

    const poseA = { t: 0, q: new THREE.Quaternion(), dir: new THREE.Vector3(0, 0, 1),
                    grav: new THREE.Vector3(), ok: false };
    const poseB = { t: 0, q: new THREE.Quaternion(), dir: new THREE.Vector3(0, 0, 1),
                    grav: new THREE.Vector3(), ok: false };
    const liveQ = new THREE.Quaternion();
    const liveDir = new THREE.Vector3(0, 0, 1);
    const liveGrav = new THREE.Vector3();
    const heroUp = new THREE.Vector3(1, 0, 0);
    let upSettled = false;       // first gravity sample snaps; later ones ease
    let lastFrameMs = 0;
    let lastDevPct = NaN;
    let poseCount = 0;
    let poseWindowStart = 0;
    let lastDiag = 0;

    // Scratch — this view allocates nothing per frame.
    const _m4 = new THREE.Matrix4();
    const _m4b = new THREE.Matrix4();
    const _q = new THREE.Quaternion();
    const _v = new THREE.Vector3();
    const _pos = new THREE.Vector3();
    const _tmp = new THREE.Vector3();
    const _gv = new THREE.Vector3();
    const _dn = new THREE.Vector3();
    const _dirArr = [0, 0, 0];
    const _gArr = [0, 0, 0];
    const _upTarget = new THREE.Vector3();
    const _qa = new THREE.Quaternion();
    const _qb = new THREE.Quaternion();
    const _zero = new THREE.Vector3();
    const _yAxis = new THREE.Vector3(0, 1, 0);
    const _up = new THREE.Vector3(0, 0, 1);
    const _scale = new THREE.Vector3();

    function placeTangent(target, dir, radius, scale) {
        _v.set(dir[0], dir[1], dir[2]).normalize();
        _q.setFromUnitVectors(_up, _v);
        _scale.set(scale, scale, scale);
        _pos.copy(_v).multiplyScalar(radius);
        target.compose(_pos, _q, _scale);
    }

    // ---- the 5 Hz truth channel ------------------------------------------
    function setReport(msg) {
        report = msg;
        if (Array.isArray(msg.cell_dirs)) {
            cellDirs = msg.cell_dirs;
            stats.cells = cellDirs.length;
            // Allocate the near/far split ONLY when the lattice size changes
            // (i.e. once). Re-seeding it on every report — this arrives at 5 Hz —
            // both forced a full reclassification 5x/s regardless of the 10 Hz
            // throttle AND left `refreshCells()` below drawing one frame with
            // every cell marked "in front", a visible 5 Hz flicker of the near
            // cap. `updateBehind` owns the contents from here on.
            if (!behind || behind.length !== cellDirs.length) {
                behind = new Array(cellDirs.length).fill(false);
                behindDirty = true;
            }
        }
        if (Array.isArray(msg.t_world_to_cv) && msg.t_world_to_cv.length === 9) {
            const t = msg.t_world_to_cv;
            // THREE's Matrix4.set is ROW-major, matching the server's row-major
            // flatten. No sign or permutation convention is written here.
            tWorldToCv = new THREE.Matrix4().set(
                t[0], t[1], t[2], 0, t[3], t[4], t[5], 0, t[6], t[7], t[8], 0, 0, 0, 0, 1);
        }
        if (Array.isArray(msg.cell_counts)) counts = msg.cell_counts;
        if (Array.isArray(msg.cell_dev_pct)) devPcts = msg.cell_dev_pct;
        refreshCells();
        refreshTarget();
    }

    function refreshCells() {
        if (!cellDirs || !counts) return;
        let covered = 0;
        for (let i = 0; i < N; i++) {
            const dir = cellDirs[i] || [0, 0, 1];
            const n = counts[i] || 0;
            const filled = n > 0;
            const ghost = behind ? behind[i] : false;
            if (filled) covered++;
            const rgb = filled ? devRgb(devPcts ? devPcts[i] : null) : EMPTY_INK;
            const scale = filled ? CELL_BASE_R * densityRadius(n) : CELL_BASE_R;
            // Hue survives the ghosting untouched — alpha is the depth cue, so
            // the |B| ramp still reads on a cell behind you.
            for (const c of CELL_MESHES) {
                const live = c.filled === filled && c.ghost === ghost;
                placeTangent(_m4, dir, SHELL_R, live ? scale : 0);
                c.mesh.setMatrixAt(i, _m4);
                if (live) c.mesh.setColorAt(i, colorOf(rgb));
            }
        }
        stats.covered = covered;
        for (const c of CELL_MESHES) {
            c.mesh.instanceMatrix.needsUpdate = true;
            if (c.mesh.instanceColor) c.mesh.instanceColor.needsUpdate = true;
        }
    }

    // ---- the near/far split, now a per-frame quantity -----------------------
    // Which cells sit BETWEEN the eye and the origin: those are ghosted, so the
    // near cap cannot hide the hemisphere being aimed into (see CELL_MESHES).
    //
    // Under the body-fixed fallback framing this is a CONSTANT — the camera
    // stands on body −Z, so the answer is `dir·(0,0,1) < 0` forever, which is
    // why it used to be computed once in `setReport`. The merged view's camera
    // is fixed in the ROOM while the shell tumbles in the hand, so that cached
    // answer goes stale within a frame or two of any motion and the wrong
    // hemisphere gets ghosted.
    //
    // The recompute rotates the camera's view direction into BODY coordinates
    // (the transpose of `bodyGroup`'s rotation, which is exactly
    // T_WORLD_TO_CV·R — no new convention is written here), re-classifies 92
    // dot products, and rewrites 4 x 92 instance matrices via refreshCells()
    // *only if a cell actually changed side*. Two guards, both load-bearing:
    // BEHIND_MIN_MS caps it at ~10 Hz, and BEHIND_MIN_COS skips it entirely
    // while the view barely moves — without them a cell straddling the terminator
    // flips at display rate and flickers. Cost is reported in
    // `window.__magcal3d` (`behindRecomputes`, `refreshMs`); `?magcalstatic=1`
    // freezes it at the pre-merge behaviour so the two can be compared.
    const BEHIND_MIN_MS = 100;
    const BEHIND_MIN_COS = Math.cos(1.5 * Math.PI / 180);
    const _viewBody = new THREE.Vector3(0, 0, 1);
    const _viewPrev = new THREE.Vector3(0, 0, 1);
    const _m3 = new THREE.Matrix3();
    let behindDirty = true;
    let lastBehindMs = 0;

    function updateBehind(now, worldFixed) {
        if (!cellDirs || !behind) return;
        if (staticBehind) {
            if (!behindDirty) return;
            worldFixed = false;
        }
        if (!behindDirty && now - lastBehindMs < BEHIND_MIN_MS) return;
        if (worldFixed) {
            _viewBody.copy(steerCam.position).negate().normalize();
            _m3.setFromMatrix4(bodyGroup.matrix).transpose();
            _viewBody.applyMatrix3(_m3);
        } else {
            _viewBody.set(0, 0, 1);
        }
        if (!behindDirty && _viewBody.dot(_viewPrev) > BEHIND_MIN_COS) {
            lastBehindMs = now;
            return;
        }
        lastBehindMs = now;
        stats.behindRecomputes++;
        let changed = behindDirty;
        let ghosted = 0;
        for (let i = 0; i < cellDirs.length; i++) {
            const d = cellDirs[i] || [0, 0, 1];
            const b = (d[0] * _viewBody.x + d[1] * _viewBody.y + d[2] * _viewBody.z) < 0;
            if (b !== behind[i]) { behind[i] = b; changed = true; }
            if (b) ghosted++;
        }
        stats.ghosted = ghosted;
        _viewPrev.copy(_viewBody);
        behindDirty = false;
        if (changed) {
            const t0 = performance.now();
            refreshCells();
            const dt = performance.now() - t0;
            // EMA, so the number in __magcal3d is a steady-state cost rather
            // than whatever the last call happened to hit.
            stats.refreshMs = +(stats.refreshMs ? stats.refreshMs * 0.85 + dt * 0.15 : dt)
                .toFixed(3);
        }
    }

    const _c = new THREE.Color();
    function colorOf(rgb) {
        _c.setRGB(rgb[0] / 255, rgb[1] / 255, rgb[2] / 255, THREE.SRGBColorSpace);
        return _c;
    }

    function disposeWrist() {
        if (!wristArrow) return;
        ghostGroup.remove(wristArrow);
        wristArrow.traverse((o) => {
            if (o.geometry) o.geometry.dispose();
            if (o.material) o.material.dispose();
        });
        wristArrow = null;
    }

    function refreshTarget() {
        const ga = report && report.guidance_axis;
        if (!ga || !Array.isArray(ga.target)) {
            targetGroup.visible = false;
            geodesic.visible = false;
            disposeWrist();
            ghostGroup.visible = false;
            return;
        }
        targetGroup.visible = true;
        targetGroup.matrixAutoUpdate = false;
        placeTangent(targetGroup.matrix, ga.target, SHELL_R * 1.008, 1);
        geodesic.visible = true;

        // The wrist arrow: an arc wrapped around the BODY rotation axis the
        // server computed, spanning the required angle. Steering-only, because
        // it only means anything next to a device you can see turning.
        disposeWrist();
        if (Array.isArray(ga.axis) && ga.angle_deg) {
            wristArrow = wristArc(ga.axis, ga.angle_deg);
            setLayer(wristArrow, 2);
            ghostGroup.add(wristArrow);
        }
    }

    /** A curved arrow around body axis `axis`, spanning `deg`. */
    function wristArc(axis, deg) {
        const n = new THREE.Vector3().fromArray(axis).normalize();
        // Any perpendicular basis; the arc's start angle is cosmetic.
        const seed = Math.abs(n.x) < 0.9 ? new THREE.Vector3(1, 0, 0) : new THREE.Vector3(0, 1, 0);
        const u = new THREE.Vector3().crossVectors(n, seed).normalize();
        const w = new THREE.Vector3().crossVectors(n, u);
        const R = 0.62;
        const total = Math.min(Math.PI * 1.9, deg * Math.PI / 180);
        const pts = [];
        const steps = 28;
        for (let i = 0; i <= steps; i++) {
            const a = total * (i / steps);
            pts.push(new THREE.Vector3()
                .addScaledVector(u, Math.cos(a) * R)
                .addScaledVector(w, Math.sin(a) * R));
        }
        const g = new THREE.Group();
        const arc = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts),
            new THREE.LineBasicMaterial({ color: INK, transparent: true, opacity: 0.9 }));
        arc.frustumCulled = false;
        g.add(arc);
        const tip = new THREE.Mesh(new THREE.ConeGeometry(0.05, 0.14, 10),
            new THREE.MeshBasicMaterial({ color: INK }));
        const last = pts[pts.length - 1];
        const prev = pts[pts.length - 2];
        tip.position.copy(last);
        tip.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0),
            new THREE.Vector3().subVectors(last, prev).normalize());
        g.add(tip);
        return g;
    }

    // ---- the 30 Hz pose channel -------------------------------------------
    function setPose(p) {
        const now = performance.now();
        poseA.t = poseB.t; poseA.q.copy(poseB.q); poseA.dir.copy(poseB.dir);
        poseA.grav.copy(poseB.grav); poseA.ok = poseB.ok;
        poseB.t = now;
        poseB.q.set(p.quat[1], p.quat[2], p.quat[3], p.quat[0]);   // (x,y,z,w) from (w,x,y,z)
        poseB.dir.set(p.dir[0], p.dir[1], p.dir[2]);
        poseB.grav.set(p.gravity[0], p.gravity[1], p.gravity[2]);
        poseB.ok = true;
        haveQuat = (p.flags & POSE_HAVE_QUAT) !== 0;
        lastDevPct = p.devPct;
        stats.lastPoseMs = now;

        // Trail: 3 s of history at r = 1.02, decaying along the neutral ordinal
        // ramp (neutral because hue means |B| deviation and nothing else).
        trailPts.push([p.dir[0], p.dir[1], p.dir[2]]);
        while (trailPts.length > TRAIL_LEN) trailPts.shift();
        const m = reduceMotion ? Math.min(1, trailPts.length) : trailPts.length;
        for (let i = 0; i < m; i++) {
            const src = trailPts[trailPts.length - m + i];
            const len = Math.hypot(src[0], src[1], src[2]) || 1;
            const s = (SHELL_R * 1.02) / len;
            trailPos[i * 3] = src[0] * s;
            trailPos[i * 3 + 1] = src[1] * s;
            trailPos[i * 3 + 2] = src[2] * s;
            const t = m > 1 ? i / (m - 1) : 1;
            const seg = Math.min(TRAIL_RAMP.length - 2, Math.floor(t * (TRAIL_RAMP.length - 1)));
            const f = t * (TRAIL_RAMP.length - 1) - seg;
            const rgb = lerpRgb(TRAIL_RAMP[seg], TRAIL_RAMP[seg + 1], f);
            trailCol[i * 3] = rgb[0] / 255;
            trailCol[i * 3 + 1] = rgb[1] / 255;
            trailCol[i * 3 + 2] = rgb[2] / 255;
        }
        trailGeom.attributes.position.needsUpdate = true;
        trailGeom.attributes.color.needsUpdate = true;
        trailGeom.setDrawRange(0, m);

        poseCount++;
        if (!poseWindowStart) poseWindowStart = now;

        if (!window.__gotMagpose) { window.__gotMagpose = true; D('first MAGPOSE'); }
    }

    function setSegment(line, a, b) {
        const arr = line.geometry.attributes.position.array;
        arr[0] = a.x; arr[1] = a.y; arr[2] = a.z;
        arr[3] = b.x; arr[4] = b.y; arr[5] = b.z;
        line.geometry.attributes.position.needsUpdate = true;
        if (line.computeLineDistances) line.computeLineDistances();
    }

    // ---- render loop ------------------------------------------------------
    // Decouple render from data: rAF at display rate, SLERP between the last two
    // received poses at one pose-interval of presentation latency. Rendering 144
    // fps against 30 Hz data without interpolation shows visible 4-frame
    // stair-step stutter. INTERPOLATE, NEVER EXTRAPOLATE — extrapolating a hand
    // tumble overshoots on direction reversals, and this view's credibility
    // rests on it never showing motion that did not happen.
    function interpolate(now) {
        if (!poseB.ok) return;
        if (!poseA.ok || poseB.t <= poseA.t) {
            liveQ.copy(poseB.q); liveDir.copy(poseB.dir); liveGrav.copy(poseB.grav);
            return;
        }
        const target = now - RENDER_LAG_MS;
        const a = Math.max(0, Math.min(1, (target - poseA.t) / (poseB.t - poseA.t)));
        liveQ.copy(poseA.q).slerp(poseB.q, a);
        liveDir.copy(poseA.dir).lerp(poseB.dir, a).normalize();
        liveGrav.copy(poseA.grav).lerp(poseB.grav, a);
    }

    /** Point `camera.up` at −g, so screen-down is room-down whatever the board
     *  is doing. Only the component perpendicular to the boresight can be shown
     *  (the parallel one points into the screen), which is the same projection
     *  `web.boresight_view_frame` does for the live FPV cloud — this is that
     *  rule, in body axes, on a camera instead of on points.
     *
     *  Eased, not snapped, and eased on the CAMERA only: no mark ever moves by
     *  a smoothed number. Returns nothing; mutates heroUp/heroCam. */
    function updateHeroUp(now, gUnit) {
        const dt = lastFrameMs ? Math.min(0.25, (now - lastFrameMs) / 1000) : 0;
        lastFrameMs = now;
        if (gUnit) {
            _upTarget.set(-gUnit.x, -gUnit.y, 0);    // ⊥ boresight == body XY
            if (_upTarget.length() >= UP_MIN) {
                _upTarget.normalize();
                const a = (upSettled && dt > 0) ? 1 - Math.exp(-dt / UP_TAU_S)
                                                : (upSettled ? 0 : 1);
                if (a > 0) {
                    // Rotate toward the target rather than lerping through it:
                    // a straight lerp between near-opposite vectors passes
                    // through zero, and normalizing that explodes.
                    _qa.setFromUnitVectors(heroUp, _upTarget);
                    _qb.identity().slerp(_qa, a);
                    heroUp.applyQuaternion(_qb).normalize();
                }
                upSettled = true;
            }
        }
        heroCam.up.copy(heroUp);
        heroCam.lookAt(0, 0, 0);
    }

    // Sizes the one canvas and keeps BOTH cameras' aspect in step. Both,
    // deliberately: the merged view swaps camera the instant orientation
    // arrives or disappears, and a camera whose aspect was last set at a
    // different canvas size renders a stretched shell on that first frame.
    function sizeTo(renderer, canvas) {
        const w = canvas.clientWidth || canvas.width;
        const h = canvas.clientHeight || canvas.height;
        if (!w || !h) return false;
        if (canvas.width !== Math.round(w * devicePixelRatio)
            || canvas.height !== Math.round(h * devicePixelRatio)) {
            renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
            renderer.setSize(w, h, false);
            for (const cam of [heroCam, steerCam]) {
                cam.aspect = w / h;
                cam.updateProjectionMatrix();
            }
        }
        return true;
    }

    function frame() {
        rafId = requestAnimationFrame(frame);
        const now = performance.now();
        interpolate(now);

        // Live marks, all in body axes.
        const d = _dn;
        if (liveDir.lengthSq() > 1e-9) d.copy(liveDir).normalize(); else d.set(0, 0, 1);

        // Gravity — SFLP gravity FIFO when the device streams it, else the
        // quat-derived down vector; the server already picked. Zero = unknown,
        // and then the hero simply doesn't roll (§ have_quat false still renders).
        const gn = liveGrav.length();
        const haveG = gn > 1e-3;
        gLine.visible = gTip.visible = dipArc.visible = haveG;
        if (haveG) {
            _gv.copy(liveGrav).divideScalar(gn);
            _tmp.copy(_gv).multiplyScalar(0.82);
            setSegment(gLine, _zero, _tmp);
            gTip.position.copy(_gv).multiplyScalar(0.88);
            gTip.quaternion.setFromUnitVectors(_yAxis, _gv);
            _dirArr[0] = d.x; _dirArr[1] = d.y; _dirArr[2] = d.z;
            _gArr[0] = _gv.x; _gArr[1] = _gv.y; _gArr[2] = _gv.z;
            writeArc(dipArc.geometry.attributes.position.array, _dirArr, _gArr, 0.5, DIP_SEG);
            dipArc.geometry.attributes.position.needsUpdate = true;
        }
        updateHeroUp(now, haveG ? _gv : null);

        head.position.copy(d).multiplyScalar(SHELL_R * 1.02);
        setSegment(stem, _zero, head.position);
        _tmp.copy(d).multiplyScalar(SHELL_R * 1.20);
        setSegment(bLine, _zero, _tmp);
        bTip.position.copy(d).multiplyScalar(SHELL_R * 1.27);
        bTip.quaternion.setFromUnitVectors(_yAxis, d);

        const ga = report && report.guidance_axis;
        if (geodesic.visible && ga && Array.isArray(ga.target)) {
            _dirArr[0] = d.x; _dirArr[1] = d.y; _dirArr[2] = d.z;
            writeArc(geodesic.geometry.attributes.position.array,
                     _dirArr, ga.target, SHELL_R * 1.03, GEO_SEG);
            geodesic.geometry.attributes.position.needsUpdate = true;
            geodesic.computeLineDistances();
        }

        // |B| magnitude tick: outward when high, inward when low.
        if (isFinite(lastDevPct) && Math.abs(lastDevPct) > 0.05) {
            const t = Math.max(-1, Math.min(1, lastDevPct / DEV_CLAMP));
            const base = SHELL_R * 1.30;
            _tmp.copy(d).multiplyScalar(base);
            _v.copy(d).multiplyScalar(base + t * 0.26);
            setSegment(devTick, _tmp, _v);
            const rgb = devRgb(lastDevPct);
            devTick.material.color.setRGB(rgb[0] / 255, rgb[1] / 255, rgb[2] / 255, THREE.SRGBColorSpace);
            devTick.visible = true;
        } else {
            devTick.visible = false;
        }

        if (targetGroup.visible && ga && Array.isArray(ga.target)) {
            const pulse = reduceMotion ? 1 : 1 + 0.09 * Math.sin(now / 260);
            placeTangent(targetGroup.matrix, ga.target, SHELL_R * 1.008, pulse);
        }

        // ---- ONE pass, one of two framings ---------------------------------
        // World-fixed whenever there is orientation to be world-fixed WITH;
        // body-fixed otherwise. The gate is the same one the old steering pass
        // used, which is the point: everything it could not do without a
        // quaternion (ghost, wrist arrow, room-stable field arrow) is exactly
        // what the fallback drops, and the coverage map — the thing the modal
        // exists for — survives either way.
        const worldFixed = !!(tWorldToCv && haveQuat);
        stats.framing = worldFixed ? 'world' : 'body';

        if (worldFixed) {
            // bodyGroup = T_WORLD_TO_CV · R: the device tumbles and the field
            // arrow stands still, because R · (Rᵀ·b_world) = b_world.
            _m4.makeRotationFromQuaternion(liveQ);
            bodyGroup.matrix.multiplyMatrices(tWorldToCv, _m4);
            // The ghost sits at R·ΔR — the minimal-effort attitude that puts the
            // field in the target cell. ΔR comes from the SERVER's axis+angle;
            // no rotation convention is derived here.
            if (ga && Array.isArray(ga.axis) && ga.angle_deg !== null && wristArrow) {
                _v.set(ga.axis[0], ga.axis[1], ga.axis[2]).normalize();
                _q.setFromAxisAngle(_v, ga.angle_deg * Math.PI / 180);
                _m4b.makeRotationFromQuaternion(_q);
                ghostGroup.matrix.multiplyMatrices(bodyGroup.matrix, _m4b);
                ghostGroup.visible = true;
            } else {
                ghostGroup.visible = false;
            }
            if (note) note.classList.add('hidden');
        } else {
            // No orientation (ToF-only session, or before the first stream-9
            // sample). `cell_dirs` and `field_dir_body` are body vectors, so the
            // coverage map is fully correct here — only the ghost is missing,
            // and the note says why.
            bodyGroup.matrix.copy(IDENT);
            ghostGroup.visible = false;
            if (note) note.classList.remove('hidden');
        }

        // Must run AFTER bodyGroup.matrix is written for this frame and BEFORE
        // the render: it reads that matrix to get the eye direction in body axes.
        updateBehind(now, worldFixed);

        if (sizeTo(heroRenderer, heroCanvas)) {
            bodyGroup.updateMatrixWorld(true);
            if (ghostGroup.visible) ghostGroup.updateMatrixWorld(true);
            heroRenderer.render(scene, worldFixed ? steerCam : heroCam);
            stats.frames++;
        }

        // 1 Hz diag line: a machine-checkable assertion that the WebGL path AND
        // the pose channel are live, not merely that a canvas exists.
        if (now - lastDiag >= 1000) {
            const dt = (now - (poseWindowStart || now)) / 1000;
            stats.poseHz = dt > 0 ? +(poseCount / dt).toFixed(1) : 0;
            // Screen-up as a body angle from +X (body Up): 0 = board upright,
            // ±90 = held on its side. This is the fallback camera's gravity
            // levelling, reported so it can be asserted rather than eyeballed.
            stats.upDeg = +(Math.atan2(heroUp.y, heroUp.x) * 180 / Math.PI).toFixed(1);
            poseCount = 0; poseWindowStart = now; lastDiag = now;
            D(`renderer=${stats.renderer} framing=${stats.framing} cells=${stats.cells} `
              + `covered=${stats.covered} ghosted=${stats.ghosted} up_deg=${stats.upDeg} `
              + `frames=${stats.frames} pose_hz=${stats.poseHz} have_quat=${haveQuat} `
              + `behind_recomputes=${stats.behindRecomputes} refresh_ms=${stats.refreshMs}`);
        }
    }

    // ---- context loss -----------------------------------------------------
    let lost = false;
    const onLost = (ev) => {
        ev.preventDefault();
        lost = true;
        stats.renderer = '2d';
        stats.reason = 'webglcontextlost';
        D('WebGL context lost — falling back to the 2D coverage map', 'error');
        stop();
        if (opts.onDegrade) opts.onDegrade(stats.reason);
    };
    heroCanvas.addEventListener('webglcontextlost', onLost, false);

    function start() {
        if (running || lost) return;
        running = true;
        poseWindowStart = 0; poseCount = 0;
        rafId = requestAnimationFrame(frame);
    }
    function stop() {
        running = false;
        if (rafId) cancelAnimationFrame(rafId);
        rafId = 0;
    }
    function dispose() {
        stop();
        try { heroRenderer.dispose(); } catch (e) {}
    }

    D('3D renderer ready (WebGL ×1, merged view'
      + (staticBehind ? ', ?magcalstatic=1: near/far split frozen' : '') + ')');
    const api = { ok: true, renderer: 'webgl', reason: null, setReport, setPose,
                  start, stop, dispose, stats: () => stats };
    window.__magcal3d = stats;
    return api;
}

/** The no-WebGL handle: same shape, every draw call a no-op, so magcal.js needs
 *  no branches beyond "show the 2D map instead". */
function degraded(stats) {
    window.__magcal3d = stats;
    return {
        ok: false, renderer: '2d', reason: stats.reason,
        setReport() {}, setPose() {}, start() {}, stop() {}, dispose() {},
        stats: () => stats,
    };
}
