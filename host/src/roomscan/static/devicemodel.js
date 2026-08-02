// devicemodel.js — ONE definition of the scanner's physical shape, drawn by two
// different renderers (owner ask, 2026-07-31).
//
// WHY THIS MODULE EXISTS
// ---------------------
// Until now there were two "devices" in the UI and neither was the device:
//   * magcal3d.js drew a 0.62 x 0.34 x 0.035 PCB silhouette — a bare NUCLEO,
//     which is what the *reference firmware* runs on, not what the owner holds;
//   * the Sensors card's orientation widget drew an RGB axis triad, i.e. no
//     device at all.
// The real instrument is a block: **5.5" tall x 3" wide x 2.5" deep**, and
// through that depth the half facing the user is dark grey, the middle quarter
// white, and the quarter facing away blue — with the camera on the blue face.
// Two renderers reading one module is the only thing that keeps them agreeing:
// a shape that differs between the gizmo and the calibration modal is worse
// than no shape, because it teaches the operator a wrong mental model.
//
// FRAMES — three of them, named once here so nothing downstream guesses
// --------------------------------------------------------------------
//   DESIGN frame — how the geometry below is authored: +X = the device's own
//       top, +Y = its right, +Z = out of the camera face (the boresight).
//       Everything in DEVICE_DIMS / DEVICE_BANDS is in this frame.
//
//   BODY frame — the SFLP body axes the rest of the app speaks (X = Up,
//       Y = Right, Z = Forward/boresight; docs/coordinate-frames.md). NOT the
//       same as DESIGN: see MOUNT_ROTATION.
//
//   CV frames — the ToF/Open3D convention (X = Right, Y = Down, Z = Forward).
//       Only `drawDeviceBox2D` touches these, because the `sensor` message's
//       `rot` is a CV->CV-world rotation. See BODY_TO_CV.
//
// Public surface:
//   DEVICE_DIMS, DEVICE_BANDS, MOUNT_ROTATION, BODY_TO_CV   (data; testable)
//   createDeviceMesh(THREE, opts)   -> THREE.Group, body-framed  (magcal3d.js)
//   createDeviceGhost(THREE, opts)  -> THREE.Group, body-framed  (magcal3d.js)
//   drawDeviceBox2D(ctx, rot, size, opts)  -> void               (sensors.js)
//
// THREE is a PARAMETER, never an import: magcal3d.js resolves `three` through
// index.html's import map, and a second import path would ship a second copy of
// the library. sensors.js must not pull three in at all — its widget is a 2D
// canvas on purpose (this host renders WebGL in software via llvmpipe, so a
// third live GL context costs measurable frame rate).

// --- dimensions ------------------------------------------------------------
// 5.5 : 3 : 2.5, scaled to keep magcal3d's existing 0.62 extent along the long
// axis so the shell/standoff framing constants there did not have to move.
// Note the depth: 0.282 is **8x** the old board's 0.035. A block that deep sits
// across the middle of the coverage shell, which is why every caller ghosts the
// fill instead of drawing it solid (see `userData.fillMat` below).
export const DEVICE_DIMS = { x: 0.62, y: 0.338, z: 0.282 };

// The SAME block in METRES — 5.5" x 3" x 2.5" at 0.0254 m/in, i.e. the physical
// truth rather than a drawing scale.
//
// Which of the two you want is decided by ONE question: does the scene it is
// going into have a metric ground truth in it already?
//   * mag-cal / the Sensors gizmo have none — the device is alone inside a
//     coverage shell whose radius is itself a made-up 1.0 — so they use
//     DEVICE_DIMS and the shell framing constants stay put;
//   * the SLAM / Detailed map is in METRES, next to real walls, so it must use
//     this one. It did not: slam.js passed no `dims`, took the shell-unit
//     default, and drew the scanner 4.44x oversize — a 62 cm slab sitting in a
//     room whose doorways are 2 m. (Reported 2026-08-01; nothing catches this
//     by eye in mag-cal, where being 4.44x a unitless shell is meaningless.)
export const DEVICE_DIMS_M = { x: 5.5 * 0.0254, y: 3.0 * 0.0254, z: 2.5 * 0.0254 };

// --- the three slabs, ordered from -Z (facing the user) to +Z (boresight) ----
// `frac` is the share of DEVICE_DIMS.z. `color` is a THREE-style hex int;
// `edge` is the line colour that keeps the band boundary legible once the fill
// is ghosted to 0.30.
//
// Colour note: hue inside the mag-cal view means |B| deviation and nothing else
// (magcal3d.js § COLOUR), and the accent blue is also that ramp's "reads LOW"
// end. Using it here is not a new collision — the ToF aperture was *already*
// drawn in exactly this accent — and the block is a solid body at the origin
// rather than a mark on the shell, so shape separates them. The alternative
// (recolouring the physical object) would break the one thing the owner asked
// for: recognising the thing in their hand.
export const DEVICE_BANDS = [
    { name: 'user',   frac: 0.50, color: 0x2b313d, edge: 0x94a3b8 },  // dark grey
    { name: 'middle', frac: 0.25, color: 0xe2e8f0, edge: 0xf1f5f9 },  // white
    { name: 'camera', frac: 0.25, color: 0x60a5fa, edge: 0xbfdbfe },  // blue
];

// The camera aperture, on the +Z face of the blue slab. Centred, because that
// is where the boresight is: body +Z through the origin is the axis the cloud
// comes out of, and an off-centre dot would quietly contradict every other mark
// in the mag-cal view that is drawn about that axis.
// Expressed as a FRACTION of the block's long axis, not an absolute, so the
// aperture stays in proportion whichever dimension set is drawn: an absolute
// 0.048 is 7.7% of the shell-unit block but 34% of the metric one, which would
// render the metric device as mostly lens. Same for the two face offsets, which
// keep the disc and its rim proud of the blue face.
export const APERTURE_R_FRAC = 0.048 / 0.62;
const APERTURE_EYE_Z_FRAC = 0.002 / 0.282;
const APERTURE_RIM_Z_FRAC = 0.001 / 0.282;
// Retained for the shell-unit callers that import it directly.
export const APERTURE_R = APERTURE_R_FRAC * 0.62;
const APERTURE_FILL = 0x0f1115;
const APERTURE_RING = 0xe2e8f0;

// --- DESIGN -> BODY --------------------------------------------------------
// A 180 deg rotation about the boresight (body/design Z).
//
// Derivation, from the owner's own reading: held normally the instrument
// reports World **pitch 0 deg, roll 180 deg**, and those readings are correct.
//   * World pitch is `sensors.tilt_from_down_deg` of the boresight -> 0 deg
//     means the boresight is horizontal, i.e. aimed across the room. It says
//     nothing about which way is up, so it does not constrain this constant.
//   * World roll is `sensors.triad_roll_deg`: the roll of body **+X** about the
//     boresight, referenced to true vertical. 180 deg therefore means body +X
//     points **down** in the room while the device is held the normal way up.
// So body +X is the device's *bottom*, not its top — which is exactly what
// magcal3d's old model got wrong when it called +X "Up, USB down" (true of a
// bare NUCLEO stood on its USB connector; false of this instrument). Mapping
// design +X -> body -X is a 180 deg turn about Z, and doing it about Z rather
// than about X or Y is what preserves the boresight: the camera face must stay
// on body +Z whatever else moves.
//
// Row-major 3x3, applied as design -> body.
export const MOUNT_ROTATION = [
    -1, 0, 0,
    0, -1, 0,
    0, 0, 1,
];

// --- BODY -> CV (sensor) ---------------------------------------------------
// The transpose of `roomscan.sensors.T_CV_TO_BODY` (X_body = -Y_cv,
// Y_body = X_cv, Z_body = Z_cv). It is written here — the only permutation this
// client owns — because `drawDeviceBox2D` is handed the `sensor` message's
// `rot`, which is `T_WORLD_TO_CV @ R @ T_CV_TO_BODY` and therefore rotates
// **CV-frame** vectors, while the geometry above is body-framed. Something has
// to bridge those, and putting it in one exported constant makes it assertable:
// `test_static_ui.py` pins it against `sensors.T_CV_TO_BODY.T`, so a server-side
// convention change fails a test instead of silently mirroring the gizmo.
export const BODY_TO_CV = [
    0, 1, 0,
    -1, 0, 0,
    0, 0, 1,
];

// --- small row-major 3x3 helpers (no dependency, so tests can call them) ----
export function mat3mul(a, b) {
    const out = new Array(9);
    for (let r = 0; r < 3; r++) {
        for (let c = 0; c < 3; c++) {
            out[r * 3 + c] = a[r * 3] * b[c] + a[r * 3 + 1] * b[3 + c] + a[r * 3 + 2] * b[6 + c];
        }
    }
    return out;
}

function mat3apply(m, x, y, z, out) {
    out[0] = m[0] * x + m[1] * y + m[2] * z;
    out[1] = m[3] * x + m[4] * y + m[5] * z;
    out[2] = m[6] * x + m[7] * y + m[8] * z;
    return out;
}

function mat4FromRowMajor(THREE, m) {
    return new THREE.Matrix4().set(
        m[0], m[1], m[2], 0,
        m[3], m[4], m[5], 0,
        m[6], m[7], m[8], 0,
        0, 0, 0, 1);
}

/** z spans of each band, in the DESIGN frame, ordered -Z -> +Z. */
export function bandSpans(dims = DEVICE_DIMS, bands = DEVICE_BANDS) {
    const out = [];
    let z = -dims.z / 2;
    for (const b of bands) {
        const span = dims.z * b.frac;
        out.push({ band: b, z0: z, z1: z + span, span, mid: z + span / 2 });
        z += span;
    }
    return out;
}

// =============================================================================
// WebGL (magcal3d.js)
// =============================================================================

/** The solid block, already rotated DESIGN -> BODY, so a caller can drop it
 *  straight into a body-framed group and never think about MOUNT_ROTATION.
 *
 *  `userData.fillMat` is a deliberate compatibility shim, not an accident: the
 *  mag-cal hero drops the fill to 0.30 with a bare
 *  `device.userData.fillMat.opacity = 0.30`, and a three-band block has three
 *  fill materials. The property below is an accessor that fans that one write
 *  out to all of them, so the existing call site keeps working AND a block this
 *  deep cannot blank the middle of the coverage shell — where the boresight,
 *  the target ring and the geodesic leader line all live. `setFillOpacity` is
 *  the same thing under a name that says what it does. */
export function createDeviceMesh(THREE, opts = {}) {
    const dims = opts.dims || DEVICE_DIMS;
    const bands = opts.bands || DEVICE_BANDS;
    const fillOpacity = opts.fillOpacity === undefined ? 0.85 : opts.fillOpacity;
    const edgeOpacity = opts.edgeOpacity === undefined ? 0.85 : opts.edgeOpacity;

    const g = new THREE.Group();
    const fillMats = [];

    for (const s of bandSpans(dims, bands)) {
        const geom = new THREE.BoxGeometry(dims.x, dims.y, s.span);
        const mat = new THREE.MeshBasicMaterial({
            color: s.band.color, transparent: true, opacity: fillOpacity, depthWrite: false });
        fillMats.push(mat);
        const mesh = new THREE.Mesh(geom, mat);
        mesh.position.z = s.mid;
        g.add(mesh);
        const edges = new THREE.LineSegments(new THREE.EdgesGeometry(geom),
            new THREE.LineBasicMaterial({
                color: s.band.edge, transparent: true, opacity: edgeOpacity }));
        edges.position.z = s.mid;
        g.add(edges);
    }

    // Aperture: a dark disc with a bright rim, standing a hair proud of the
    // blue face so it survives the fill going translucent.
    const apertureR = APERTURE_R_FRAC * dims.x;
    const zFace = dims.z / 2 + APERTURE_EYE_Z_FRAC * dims.z;
    const eye = new THREE.Mesh(new THREE.CircleGeometry(apertureR, 20),
        new THREE.MeshBasicMaterial({ color: APERTURE_FILL, transparent: true, opacity: 0.95 }));
    eye.position.z = zFace;
    g.add(eye);
    const rim = new THREE.Mesh(new THREE.RingGeometry(apertureR, apertureR * 1.22, 20),
        new THREE.MeshBasicMaterial({ color: APERTURE_RING, transparent: true, opacity: 0.9 }));
    rim.position.z = zFace + APERTURE_RIM_Z_FRAC * dims.z;
    g.add(rim);

    g.applyMatrix4(mat4FromRowMajor(THREE, MOUNT_ROTATION));

    g.userData.fillMats = fillMats;
    g.userData.setFillOpacity = (o) => { for (const m of fillMats) m.opacity = o; };
    g.userData.fillMat = {
        get opacity() { return fillMats.length ? fillMats[0].opacity : 0; },
        set opacity(v) { g.userData.setFillOpacity(v); },
    };
    return g;
}

/** Dashed wireframe twin of the block — the steering target. Outlines the whole
 *  body plus the camera slab on its own, so "which end must end up pointing
 *  where" survives being drawn in dashes. */
export function createDeviceGhost(THREE, opts = {}) {
    const dims = opts.dims || DEVICE_DIMS;
    const bands = opts.bands || DEVICE_BANDS;
    const color = opts.color === undefined ? 0xe2e8f0 : opts.color;
    const g = new THREE.Group();

    const dashed = (geom, opacity, z) => {
        const l = new THREE.LineSegments(new THREE.EdgesGeometry(geom),
            new THREE.LineDashedMaterial({ color, transparent: true, opacity,
                                           dashSize: 0.05, gapSize: 0.035 }));
        l.position.z = z;
        return l;
    };
    g.add(dashed(new THREE.BoxGeometry(dims.x, dims.y, dims.z), 0.6, 0));
    const spans = bandSpans(dims, bands);
    const lens = spans[spans.length - 1];
    g.add(dashed(new THREE.BoxGeometry(dims.x * 0.98, dims.y * 0.98, lens.span), 0.45, lens.mid));

    g.applyMatrix4(mat4FromRowMajor(THREE, MOUNT_ROTATION));
    g.traverse((o) => { if (o.computeLineDistances) o.computeLineDistances(); });
    return g;
}

// =============================================================================
// 2D canvas (sensors.js orientation gizmo)
// =============================================================================

// Design tokens mirrored from index.html (a canvas cannot read CSS vars).
const GRID_2D = 'rgba(255,255,255,0.10)';
const MUTED_2D = '#94a3b8';

// A fixed key light for the 2D block, in CV-world coordinates (X right,
// Y DOWN, Z forward/into the screen): upper-left and slightly toward the
// viewer. Shading is the only depth cue a flat orthographic box gets — without
// it three abutting slabs of the same hue read as one flat pentagon.
const LIGHT = (() => {
    const v = [-0.42, -0.72, -0.55];
    const n = Math.hypot(v[0], v[1], v[2]);
    return [v[0] / n, v[1] / n, v[2] / n];
})();
const AMBIENT = 0.52;

function shadeHex(hex, k) {
    const r = Math.round(Math.min(255, ((hex >> 16) & 0xff) * k));
    const g = Math.round(Math.min(255, ((hex >> 8) & 0xff) * k));
    const b = Math.round(Math.min(255, (hex & 0xff) * k));
    return 'rgb(' + r + ',' + g + ',' + b + ')';
}

/** Every drawable quad of the block, in the DESIGN frame. The two Z-facing
 *  faces are single-coloured (the user-facing slab's grey, the camera slab's
 *  blue); the four side faces are split into one quad per band, so the stripe
 *  through the depth reads from any angle. No interior geometry is emitted —
 *  coincident internal faces are what makes a painter's-algorithm box flicker. */
function buildQuads(dims, bands) {
    const hx = dims.x / 2, hy = dims.y / 2, hz = dims.z / 2;
    const spans = bandSpans(dims, bands);
    const quads = [];
    for (const sx of [1, -1]) {
        for (const s of spans) {
            quads.push({ n: [sx, 0, 0], band: s.band, p: [
                [sx * hx, -hy, s.z0], [sx * hx, hy, s.z0],
                [sx * hx, hy, s.z1], [sx * hx, -hy, s.z1]] });
        }
    }
    for (const sy of [1, -1]) {
        for (const s of spans) {
            quads.push({ n: [0, sy, 0], band: s.band, p: [
                [-hx, sy * hy, s.z0], [hx, sy * hy, s.z0],
                [hx, sy * hy, s.z1], [-hx, sy * hy, s.z1]] });
        }
    }
    const first = spans[0].band, last = spans[spans.length - 1].band;
    quads.push({ n: [0, 0, 1], band: last, aperture: true, p: [
        [-hx, -hy, hz], [hx, -hy, hz], [hx, hy, hz], [-hx, hy, hz]] });
    quads.push({ n: [0, 0, -1], band: first, p: [
        [-hx, -hy, -hz], [hx, -hy, -hz], [hx, hy, -hz], [-hx, hy, -hz]] });
    return quads;
}

let _quadCache = null;
function quadsFor(dims, bands) {
    if (dims === DEVICE_DIMS && bands === DEVICE_BANDS) {
        if (!_quadCache) _quadCache = buildQuads(dims, bands);
        return _quadCache;
    }
    return buildQuads(dims, bands);
}

// Half the block's body diagonal, used to pick a scale that never lets a
// corner escape the origin ring whatever the attitude.
function halfDiagonal(dims) {
    return Math.hypot(dims.x, dims.y, dims.z) / 2;
}

/** Paint the device block into a 2D context, at the attitude carried by `rot`.
 *
 *  `rot` is the 9-element row-major matrix the `sensor` message already sends
 *  (`display_rotation` = `T_WORLD_TO_CV @ R @ T_CV_TO_BODY`): it rotates
 *  CV-frame vectors into the CV world, where X is right, Y is DOWN and Z points
 *  away from the viewer — which is why the projection below is `x -> screen x,
 *  y -> screen y` with no flips, and why "visible" means the face normal has a
 *  NEGATIVE z. Same convention the axis triad this replaces used; the only
 *  addition is BODY_TO_CV, because a triad needed no frame of its own and a
 *  physical block does.
 *
 *  Orthographic and painter's-algorithm: 14 quads, back-face culled, sorted by
 *  face-centre depth. That is enough for a 96 px widget and costs nothing —
 *  the alternative (a third WebGL context on a software renderer) costs real
 *  frame rate on this host.
 *
 *  `rot` absent/malformed -> the faint origin ring and an em dash, exactly as
 *  before, so a ToF-only session degrades instead of blanking. */
export function drawDeviceBox2D(ctx, rot, size, opts = {}) {
    const dims = opts.dims || DEVICE_DIMS;
    const bands = opts.bands || DEVICE_BANDS;
    const cx = size / 2, cy = size / 2;
    const ringR = size * 0.414;

    ctx.strokeStyle = GRID_2D;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(cx, cy, ringR, 0, Math.PI * 2);
    ctx.stroke();

    if (!Array.isArray(rot) || rot.length !== 9 || !rot.every((v) => typeof v === 'number' && isFinite(v))) {
        ctx.fillStyle = MUTED_2D;
        ctx.font = '11px "JetBrains Mono", monospace';
        ctx.textAlign = 'center';
        ctx.fillText('—', cx, cy + 4);
        return;
    }

    // DESIGN -> BODY -> CV(sensor) -> CV(world).
    const m = mat3mul(rot, mat3mul(BODY_TO_CV, MOUNT_ROTATION));
    const scale = (ringR * 0.97) / halfDiagonal(dims);

    const tmp = [0, 0, 0];
    const items = [];
    for (const q of quadsFor(dims, bands)) {
        mat3apply(m, q.n[0], q.n[1], q.n[2], tmp);
        const nz = tmp[2];
        if (nz > -1e-6) continue;                      // back-facing: cull
        const lam = AMBIENT + (1 - AMBIENT)
            * Math.max(0, -(tmp[0] * LIGHT[0] + tmp[1] * LIGHT[1] + tmp[2] * LIGHT[2]));
        const pts = [];
        let depth = 0;
        for (const p of q.p) {
            mat3apply(m, p[0], p[1], p[2], tmp);
            pts.push([cx + tmp[0] * scale, cy + tmp[1] * scale]);
            depth += tmp[2];
        }
        items.push({ pts, depth: depth / q.p.length, band: q.band, lam, aperture: q.aperture,
                     axes: q.aperture ? m : null });
    }
    items.sort((a, b) => b.depth - a.depth);           // farthest first

    for (const it of items) {
        ctx.beginPath();
        ctx.moveTo(it.pts[0][0], it.pts[0][1]);
        for (let i = 1; i < it.pts.length; i++) ctx.lineTo(it.pts[i][0], it.pts[i][1]);
        ctx.closePath();
        ctx.fillStyle = shadeHex(it.band.color, it.lam);
        ctx.fill();
        ctx.strokeStyle = shadeHex(it.band.edge, it.lam * 0.85);
        ctx.lineWidth = 0.7;
        ctx.stroke();
        if (it.aperture) drawAperture2D(ctx, it.axes, cx, cy, scale, dims, it.lam);
    }
}

/** The lens, as the projected circle it actually is: sampled on the camera face
 *  and pushed through the same matrix, so it foreshortens into an ellipse with
 *  the face instead of staying a suspicious perfect circle. */
function drawAperture2D(ctx, m, cx, cy, scale, dims, lam) {
    const hz = dims.z / 2;
    const tmp = [0, 0, 0];
    const seg = 20;
    ctx.beginPath();
    for (let i = 0; i <= seg; i++) {
        const a = (i / seg) * Math.PI * 2;
        mat3apply(m, Math.cos(a) * APERTURE_R, Math.sin(a) * APERTURE_R, hz, tmp);
        const x = cx + tmp[0] * scale, y = cy + tmp[1] * scale;
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.closePath();
    ctx.fillStyle = shadeHex(APERTURE_FILL, 1);
    ctx.fill();
    ctx.strokeStyle = shadeHex(APERTURE_RING, lam);
    ctx.lineWidth = 1.1;
    ctx.stroke();
}
