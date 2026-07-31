// scene.js — Three.js scene / camera / OrbitControls / point-cloud geometry.
//
// Extracted verbatim (in behaviour) from the old monolithic app.js: same camera
// pose (0.5,0,-1.5), y-down Open3D CV up vector, MAX_POINTS, PointsMaterial.
// Subscribes to "point_cloud" and parses the tag+positions+colors layout itself
// (§6.1). Owns the requestAnimationFrame render loop, measures its own VIEW fps
// (browser paint rate) and publishes it on the hub (~1/s) — this is distinct
// from the device fps the server reports.
//
// VIEW MODE (`state.view_mode`, owner ask 2026-07-29). "world" is the orbit
// view; "fpv"/"mirror" lock the camera to the sensor's viewpoint. The frame
// change happens SERVER-side (web.py `view_rotation` ships the cloud already in
// the boresight view frame — still gravity-levelled, X-negated for mirror), so
// all this module does is park the camera at the origin, take OrbitControls
// fully out (no orbit, no pan, no zoom — it is a locked view) and hide the world
// grid. A fixed pose cannot lag the geometry the way a client-rotated camera
// would, and the level horizon comes from the server, not from `camera.up`.
//
// Public surface:
//   makeXrayMaterial(THREE, material) -> the see-through twin of a material
//   createScene(hub) -> { resetCamera, THREE, scene, camera,
//                         setPointsVisible(bool), setFollow(bool),
//                         setFollowTarget(eye, center, up),
//                         setRenderActive(bool) }
// slam.js (web Phase 4) uses the returned handle to add its mesh/trajectory
// group to the same scene and to drive the follow camera (which must coordinate
// with OrbitControls — only one may own the camera per frame).
// Hub events:  subscribes "point_cloud", "reset_camera";  emits "view_fps" (~1/s)

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const D = (m, l) => { try { window.__diag && window.__diag('scene.js: ' + m, l); } catch (e) {} };

const MAX_POINTS = 300000;                 // large buffer for later SLAM maps

// --- see-through / x-ray (owner ask, 2026-07-31) -------------------------
// "Whenever any voxel occludes another, give it transparency so we can see
// through it." The naive reading -- turn material.opacity down -- is NOT that:
// it fades every surface against the background, including the ones hiding
// nothing, so the scene gets dimmer overall while the interesting case (a near
// wall hiding the room behind it) only half improves.
//
// Instead each geometry gets a SECOND draw of the same buffers with the depth
// test inverted (`GreaterDepth`) and depth writing off. That pass shades
// exactly the fragments the normal pass rejected -- i.e. precisely the occluded
// ones -- and alpha-blends them back over their occluder at `see_through`.
// Un-occluded surfaces are byte-identical to before, because nothing failed the
// depth test in front of them. The blend that lands on an occluded pixel is
// `a*hidden + (1-a)*occluder`, which is the same result as making the occluder
// itself transparent, but only where it actually occludes something.
//
// Cost is one extra draw call per object, and only while see-through is on:
// at 0 (the default) the x-ray objects are `visible = false` and the renderer
// skips them entirely, so the historical render is untouched.
//
// Shared with slam.js, which x-rays its own mesh materials the same way.
export function makeXrayMaterial(THREE, source) {
    const x = source.clone();
    // clone() -> copy() has a fixed field list and does NOT carry an
    // onBeforeCompile override, which for the point material is the whole
    // auto-size shader patch. Re-point it at the same function: it is also the
    // material's customProgramCacheKey, so sharing the function means sharing
    // the compiled program (and the uniform objects it was handed) rather than
    // paying for a second one.
    x.onBeforeCompile = source.onBeforeCompile;
    x.transparent = true;
    x.opacity = 0;                        // set by setSeeThrough / the state echo
    x.depthWrite = false;                 // an x-ray layer must never occlude anything itself
    x.depthFunc = THREE.GreaterDepth;     // ONLY where the normal pass lost the depth test
    x.needsUpdate = true;                 // `transparent`/`depthFunc` are program+state changes
    return x;
}

// --- camera framing: one baseline, three offsets -------------------------
// Every view is described relative to ONE reference — the FPV ground truth, a
// camera sitting exactly at the sensor looking down its boresight. All-zero
// offsets reproduce that camera exactly, in every mode.
//
//   distance_m   back off along the view axis
//   height_m     lift the eye (the world is Y-DOWN, so up is NEGATIVE y)
//   rotation_deg swing the eye about the vertical axis through the aim point,
//                positive to the right; the aim point stays on the axis so the
//                subject stays framed
//
// Why any offset at all: a camera exactly at the optical centre reproduces the
// depth image's own projection — every point lands where it already was in 2D
// and the render reads as a flat picture. The offset supplies the parallax
// that makes depth legible. Mirror uses its own values but the same maths; its
// default eye is on x = 0, so it mirrors onto itself.
//
// The modes differ only in what "the axis" is: fpv/mirror ride the live
// boresight (the server has already rotated the cloud into the boresight view
// frame), world uses the fixed world forward. They coincide when the board is
// held in the reference pose, which is what makes this one baseline rather
// than three unrelated cameras. Server-side twins: web.py `_DEFAULT_VIEW_CAM`.
const CAM_LOOK_AHEAD_M = 1.0;              // aim point down the axis; also the rotation pivot
const CAM_UP = new THREE.Vector3(0, -1, 0);
const DEFAULT_VIEW_CAM = {
    world:  { distance_m: 4.2,  height_m: 2.6,  rotation_deg: 0 },   // elevated establishing shot
    fpv:    { distance_m: 0.30, height_m: 0.20, rotation_deg: 0 },   // just over the sensor's shoulder
    mirror: { distance_m: 0.30, height_m: 0.20, rotation_deg: 0 },
};

export function createScene(hub) {
    D('module loaded; THREE r' + THREE.REVISION);

    const container = document.getElementById('canvas-container');
    if (!container) { D('FATAL #canvas-container not found — scene cannot attach', 'error'); return { resetCamera() {} }; }

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0a0a0f);
    // Density retuned for the backed-off default camera: at 0.1 the cloud sat
    // ~30% washed into the background from 6 m, which defeats an establishing
    // shot. 0.05 keeps the depth cue and costs ~8% at that range.
    scene.fog = new THREE.FogExp2(0x0a0a0f, 0.05);

    const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 100);
    camera.up.set(0, -1, 0);               // Open3D CV convention, y-down
    // Position is set below by applyPose('world'), once the framing table exists.

    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.setPixelRatio(window.devicePixelRatio);
    container.appendChild(renderer.domElement);

    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;

    // Subtle ground grid. GridHelper is natively in the XZ plane, which IS the
    // horizontal plane here: the world is Open3D CV (Y-down, so gravity runs
    // along +Y) and the server ships the cloud gravity-aligned into it. It used
    // to be rotated into XY, a hangover from the pre-gravity-alignment days when
    // the cloud was raw sensor-frame -- that made it a wall facing the camera
    // rather than an earth plane.
    //
    // It also has to sit BELOW the viewer to read as ground: the sensor is the
    // origin and the default camera is at the sensor's own height, so a plane
    // through y = 0 is seen exactly edge-on and all but disappears. GRID_FLOOR_Y
    // drops it to a plausible handheld height above the floor -- a visual
    // reference only, nothing measures against it. Hidden in FPV/Mirror, where
    // the cloud is in the boresight view frame and a world-fixed plane would
    // tilt the wrong way as soon as the sensor pitched.
    const GRID_FLOOR_Y = 1.2;                  // metres BELOW the sensor (world +Y is down)
    const gridHelper = new THREE.GridHelper(10, 20, 0x333333, 0x1a1a1a);
    gridHelper.position.y = GRID_FLOOR_Y;
    scene.add(gridHelper);

    // Point cloud — position + color attributes, draw range grown per frame.
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(MAX_POINTS * 3), 3));
    geometry.setAttribute('color', new THREE.BufferAttribute(new Float32Array(MAX_POINTS * 3), 3));
    geometry.setDrawRange(0, 0);
    const material = new THREE.PointsMaterial({ size: 0.025, vertexColors: true, sizeAttenuation: true });
    const points = new THREE.Points(geometry, material);
    // Frustum culling OFF — mandatory for these live-updating buffers, not an
    // optimisation choice. Three computes `geometry.boundingSphere` lazily ONCE
    // and never invalidates it, but we rewrite the position attribute every
    // frame; the sphere it caches is the one from the first render, when the
    // buffer was still all zeros, i.e. centre (0,0,0) radius 0. World mode gets
    // away with it because the origin happens to fall inside the default
    // frustum, so the point-sphere "intersects" and the object is drawn. FPV
    // puts the camera AT the origin, which is behind the near plane — the
    // zero-radius sphere then fails the test and the entire cloud silently
    // vanishes. Recomputing per frame is the alternative and is far more
    // expensive (a 300k-vertex pass that ignores drawRange) for an object that
    // is always in front of the viewer anyway.
    points.frustumCulled = false;
    scene.add(points);

    // --- auto point size ---------------------------------------------------
    // The sensor is an angular imager: a fixed 54x42 zone grid over 55x42 deg,
    // so the world spacing between neighbouring points is ~r*dtheta (dtheta ~
    // 0.018 rad) and GROWS with range. One fixed world size therefore can't fit
    // a scene: whatever makes a far wall solid makes a near wall a blob field.
    // In auto mode each point's size is scaled by its own range, so every zone
    // covers the same solid angle and coverage is uniform at any distance --
    // then `material.size` means "size at 1 m of range" instead of metres.
    //
    // PointsMaterial has no per-vertex size, so we patch its vertex shader
    // rather than hand-rolling a ShaderMaterial (which would forfeit
    // vertexColors/fog/attenuation). `transformed` is the object-space position
    // and the server sends sensor-frame metres with the sensor at the origin
    // (Deprojector: x = z*tan(a), y = z*tan(b), z), so length(transformed) IS
    // the range. Using ray length rather than bare z also buys back part of the
    // off-axis sec(a) spreading for free.
    const pointUniforms = {
        uAutoSize: { value: 1.0 },     // 0 = fixed metres, 1 = metres per metre of range
        uMinPx: { value: 1.0 },        // device px: far points must not drop below a pixel
        uMaxPx: { value: 64.0 * window.devicePixelRatio },
    };
    material.onBeforeCompile = (shader) => {
        Object.assign(shader.uniforms, pointUniforms);
        shader.vertexShader = shader.vertexShader
            .replace('void main() {',
                'uniform float uAutoSize;\nuniform float uMinPx;\nuniform float uMaxPx;\nvoid main() {')
            .replace('gl_PointSize = size;',
                'gl_PointSize = size * mix( 1.0, length( transformed ), uAutoSize );')
            // After the USE_SIZEATTENUATION block, so this bounds the FINAL
            // on-screen size: no sub-pixel dropout at range, and no
            // screen-filling quad (or driver point-size cap) up close.
            .replace('#include <logdepthbuf_vertex>',
                'gl_PointSize = clamp( gl_PointSize, uMinPx, uMaxPx );\n\t#include <logdepthbuf_vertex>');
    };

    // Surface mesh — triangulated grid when surface mode is on.
    const meshGeom = new THREE.BufferGeometry();
    const meshMat = new THREE.MeshBasicMaterial({ vertexColors: true, side: THREE.DoubleSide });
    const surfaceMesh = new THREE.Mesh(meshGeom, meshMat);
    surfaceMesh.visible = false;
    surfaceMesh.frustumCulled = false;        // same stale-bounding-sphere trap as `points`
    scene.add(surfaceMesh);
    // Uncovered-but-valid points shown alongside the mesh.
    const uncovGeom = new THREE.BufferGeometry();
    uncovGeom.setAttribute('position', new THREE.BufferAttribute(new Float32Array(MAX_POINTS * 3), 3));
    uncovGeom.setAttribute('color', new THREE.BufferAttribute(new Float32Array(MAX_POINTS * 3), 3));
    uncovGeom.setDrawRange(0, 0);
    const uncovPoints = new THREE.Points(uncovGeom, material);
    uncovPoints.visible = false;
    uncovPoints.frustumCulled = false;        // same stale-bounding-sphere trap as `points`
    scene.add(uncovPoints);
    let surfaceOn = false;

    // See-through twins: the same geometry buffers drawn again with an inverted
    // depth test, so only the occluded fragments come back through. See
    // `makeXrayMaterial` above. Hidden (and free) until see_through > 0.
    const xrayPointMat = makeXrayMaterial(THREE, material);
    const xraySurfaceMat = makeXrayMaterial(THREE, meshMat);
    const xrayPoints = new THREE.Points(geometry, xrayPointMat);
    const xrayUncov = new THREE.Points(uncovGeom, xrayPointMat);
    const xraySurface = new THREE.Mesh(meshGeom, xraySurfaceMat);
    let seeThrough = 0;
    for (const o of [xrayPoints, xrayUncov, xraySurface]) {
        o.frustumCulled = false;              // same stale-bounding-sphere trap as `points`
        o.visible = false;
        scene.add(o);
    }
    let cloudShown = true;                    // slam.js hides the real-time cloud in SLAM mode

    // One place decides what is drawn: the render mode (points vs surface), the
    // SLAM-mode hide, and whether the x-ray pass is armed at all.
    function applyVisibility() {
        points.visible = cloudShown && !surfaceOn;
        surfaceMesh.visible = cloudShown && surfaceOn;
        uncovPoints.visible = cloudShown && surfaceOn;
        const xray = seeThrough > 0;
        xrayPoints.visible = xray && points.visible;
        xraySurface.visible = xray && surfaceMesh.visible;
        xrayUncov.visible = xray && uncovPoints.visible;
    }

    function setSeeThrough(v) {
        seeThrough = Math.min(1, Math.max(0, Number(v) || 0));
        xrayPointMat.opacity = seeThrough;
        xraySurfaceMat.opacity = seeThrough;
        applyVisibility();
    }
    applyVisibility();                        // one defined starting state

    // --- SLAM follow camera (web Phase 4) ---------------------------------
    // When follow is on, slam.js pushes an eye/center/up each frame and this
    // loop lerps the camera to it with OrbitControls disabled; when off,
    // OrbitControls owns the camera again. Smoothing mirrors the desktop's
    // _apply_follow_camera (steady when near-stationary, snappy under motion).
    let followOn = false;
    const followEye = new THREE.Vector3();
    const followCenter = new THREE.Vector3();
    const followUp = new THREE.Vector3(0, -1, 0);
    let haveFollowTarget = false;
    function setPointsVisible(v) {
        cloudShown = !!v;
        applyVisibility();
    }
    function setFollow(on) {
        followOn = !!on;
        // Hand the camera back to OrbitControls — but ONLY in world mode. slam.js
        // calls this with `false` on every `state` message, so an unconditional
        // re-enable here would quietly unlock the FPV/Mirror camera one message
        // after it was locked.
        if (!followOn && viewMode === 'world') { controls.enabled = true; }
    }
    function setFollowTarget(eye, center, up) {
        followEye.set(eye[0], eye[1], eye[2]);
        followCenter.set(center[0], center[1], center[2]);
        if (up) followUp.set(up[0], up[1], up[2]);
        haveFollowTarget = true;
    }

    // --- real-time view mode (owner ask, 2026-07-29) ------------------------
    // "world" is the orbit view; "fpv"/"mirror" lock the camera to the sensor.
    // The server does the frame change -- in FPV/Mirror it ships the cloud in
    // the boresight view frame (see web.py `view_rotation`), so the locked
    // camera here is a fixed pose and can never lag or slosh against the
    // geometry. Mirror needs nothing extra client-side: the server negates X.
    const FPV_NEAR = 0.02, WORLD_NEAR = 0.1;
    // Slow auto-orbit of the world view (owner ask, 2026-07-30). Handed to
    // OrbitControls' own `autoRotate`, which advances the AZIMUTH only —
    // elevation and distance are left exactly as they are, which is the ask.
    // Doing it here rather than by animating the persisted `rotation_deg`
    // keeps it smooth at the browser's frame rate and off the wire entirely.
    // Its speed unit is 2*PI/60 rad per second per unit, i.e. 6 deg/s.
    const AUTOROTATE_PER_DEG_S = 1 / 6;
    let orbitOn = false;
    let viewMode = 'world';
    // Oscillate mode (owner ask, 2026-07-31): "continuous" is the orbit above;
    // "oscillate" is a triangle wave about the azimuth where oscillation
    // started -- run N deg one way, reverse and run 2N deg, reverse and run
    // 2N deg, repeat, netting a swing of +-N about the start. Built entirely
    // on top of `controls.autoRotate`/`autoRotateSpeed` above: this only
    // flips the SIGN of `autoRotateSpeed` each time the accumulated offset
    // passes +-N; OrbitControls does the actual rotating.
    let orbitMode = 'continuous';
    let orbitAmplitudeDeg = 45.0;
    const viewCam = {};                     // live per-mode framing, from `state.view_cam`
    for (const m of Object.keys(DEFAULT_VIEW_CAM)) viewCam[m] = { ...DEFAULT_VIEW_CAM[m] };

    // Turn a mode's three numbers into an eye/target pair. Zero offsets give
    // eye == (0,0,0) — the FPV baseline — by construction.
    function poseFor(m) {
        const c = viewCam[m] || DEFAULT_VIEW_CAM.world;
        const target = new THREE.Vector3(0, 0, CAM_LOOK_AHEAD_M);
        const off = new THREE.Vector3(0, -c.height_m, -(CAM_LOOK_AHEAD_M + c.distance_m));
        off.applyAxisAngle(CAM_UP, c.rotation_deg * Math.PI / 180);
        return { eye: target.clone().add(off), target };
    }

    const savedPos = new THREE.Vector3();   // the orbit pose to return to
    const savedTarget = new THREE.Vector3();

    // Park the camera on a mode's computed pose. In world this also becomes the
    // new "return to" pose, so a framing change acts like a Reset Camera; in
    // fpv/mirror it IS the locked pose.
    function applyPose(m) {
        const { eye, target } = poseFor(m);
        camera.position.copy(eye);
        camera.up.set(0, -1, 0);
        controls.target.copy(target);
        camera.lookAt(controls.target);
        if (m === 'world') { savedPos.copy(eye); savedTarget.copy(target); controls.update(); }
        camera.updateProjectionMatrix();
    }
    applyPose('world');                     // initial pose + seeds savedPos/savedTarget

    function applyViewMode(m) {
        if (!(m in DEFAULT_VIEW_CAM)) return;
        if (m === viewMode) return;
        if (viewMode === 'world') {         // remember where the orbit was left
            savedPos.copy(camera.position);
            savedTarget.copy(controls.target);
        }
        viewMode = m;
        if (m === 'world') {
            camera.position.copy(savedPos);   // the user's own orbit, not the default framing
            controls.target.copy(savedTarget);
            camera.up.set(0, -1, 0);
            camera.near = WORLD_NEAR;
            controls.enableRotate = controls.enablePan = controls.enableZoom = true;
            controls.enabled = true;
            controls.update();
            camera.updateProjectionMatrix();
            gridHelper.visible = true;
        } else {
            // Off the sensor's shoulder, aimed down its boresight (CV +Z), CV
            // up = -Y. The horizon is level because the SERVER levelled the
            // geometry, not because of `camera.up` — see web.py
            // `boresight_view_frame`.
            camera.near = FPV_NEAR;         // we are near the sensor: don't clip near returns
            applyPose(m);
            // Fully locked (owner): no orbit, no pan, no zoom. `enabled = false`
            // already gates every OrbitControls handler; the three flags are set
            // too so nothing can re-enable one of them piecemeal.
            controls.enableRotate = controls.enablePan = controls.enableZoom = false;
            controls.enabled = false;
            gridHelper.visible = false;
        }
        // World only — a locked view has nothing to orbit around. (The FPV
        // branch also skips controls.update(), so it could not advance anyway;
        // this keeps the flag honest rather than relying on that.)
        controls.autoRotate = orbitOn && m === 'world';
        D('view mode -> ' + m);
    }

    // --- Oscillate steering (owner ask, 2026-07-31) -------------------------
    // Edge-triggered: `oscActive` tracks whether the wave is CURRENTLY running
    // (auto-orbit on, mode is oscillate, in World) each frame, so the start
    // azimuth is captured exactly when oscillation begins -- whether that's
    // because the mode was just switched to oscillate, auto-orbit was just
    // turned on, or World was just entered -- and never at module load
    // (`oscActive` starts false, same as `orbitOn`).
    let oscActive = false;
    let oscLastAzimuth = 0;    // previous frame's WRAPPED azimuth (rad), for the unwrap step
    let oscUnwrappedDeg = 0;   // signed, UNWRAPPED offset from the start azimuth (deg)
    // The wave's travel direction lives HERE, not in the sign of
    // `controls.autoRotateSpeed`. `state` is re-broadcast on every unrelated
    // setting change (see mergeViewCam's note below), and its handler assigns
    // autoRotateSpeed outright from `orbit_speed_deg_s` -- so storing direction
    // in that sign meant clicking a colour mid-sweep snapped the camera back to
    // forward travel and truncated the return leg. Magnitude comes from the
    // server; direction is ours.
    let oscDir = 1;

    function isOscillating() {
        return orbitOn && orbitMode === 'oscillate' && viewMode === 'world';
    }

    // `OrbitControls.getAzimuthalAngle()` wraps to (-PI, PI]. Summing RAW
    // frame-to-frame differences across that wrap misreads the step: e.g.
    // +179deg -> -179deg is really +2deg of travel, not a ~-358deg jump. Left
    // unwrapped, that phantom jump either fires a spurious reversal near the
    // wrap or, for amplitudes at/above 180deg, keeps the accumulator from ever
    // reaching +-N at all -- the wave latches going one direction forever.
    // Fixed by unwrapping each frame's OWN step into (-PI, PI] before adding
    // it to a running total that is NOT itself wrapped, so the total tracks
    // actual angular travel past the start, however many times around it goes.
    function updateOscillate() {
        const active = isOscillating();
        if (active && !oscActive) {               // rising edge: (re)start the wave here
            oscLastAzimuth = controls.getAzimuthalAngle();
            oscUnwrappedDeg = 0;
            // Start out travelling whichever way the speed slider points, so a
            // negative Orbit Speed still means "sweep left first".
            oscDir = controls.autoRotateSpeed < 0 ? -1 : 1;
        }
        oscActive = active;
        if (!oscActive) return;
        const cur = controls.getAzimuthalAngle();
        let step = cur - oscLastAzimuth;
        if (step > Math.PI) step -= 2 * Math.PI;
        else if (step < -Math.PI) step += 2 * Math.PI;
        oscLastAzimuth = cur;
        oscUnwrappedDeg += step * 180 / Math.PI;
        const n = Math.max(1e-6, orbitAmplitudeDeg);
        if (oscUnwrappedDeg >= n) oscDir = -1;
        else if (oscUnwrappedDeg <= -n) oscDir = 1;
        // Re-assert direction every frame rather than only on a crossing: an
        // unrelated `state` echo may have overwritten autoRotateSpeed since the
        // last frame, and one frame of wrong-way travel is visible.
        controls.autoRotateSpeed = Math.abs(controls.autoRotateSpeed) * oscDir;
    }

    // Merge a `state.view_cam` payload; true only if a number actually moved.
    // That check is load-bearing: `state` is re-broadcast on every unrelated
    // setting change, and reacting to those would yank a user's world-mode
    // orbit back to the default framing every time someone clicked a colour.
    function mergeViewCam(incoming) {
        let changed = false;
        for (const m of Object.keys(viewCam)) {
            const src = incoming[m];
            if (!src) continue;
            for (const k of ['distance_m', 'height_m', 'rotation_deg']) {
                if (typeof src[k] === 'number' && src[k] !== viewCam[m][k]) {
                    viewCam[m][k] = src[k];
                    changed = true;
                }
            }
        }
        return changed;
    }

    // --- point cloud ingest (§6.1: u32 tag · f32[3N] positions · f32[3N] colors) ---
    hub.on('point_cloud', (buffer) => {
        const data = new Float32Array(buffer, 4);
        let numPoints = Math.floor(data.length / 6);
        if (numPoints > MAX_POINTS) numPoints = MAX_POINTS;
        if (!window.__gotFrame) { window.__gotFrame = true; D('first point cloud: ' + numPoints + ' pts'); }

        const positions = geometry.attributes.position.array;
        const colors = geometry.attributes.color.array;
        const colorOffset = Math.floor(data.length / 6) * 3;
        const n3 = numPoints * 3;
        for (let i = 0; i < n3; i++) {
            positions[i] = data[i];
            colors[i] = data[colorOffset + i];
        }
        geometry.attributes.position.needsUpdate = true;
        geometry.attributes.color.needsUpdate = true;
        geometry.setDrawRange(0, numPoints);
    });

    // --- surface cloud ingest (tag 4: grid-ordered positions + triangles) ---
    hub.on('surface_cloud', (buffer) => {
        const view = new DataView(buffer);
        let off = 4;
        const gw = view.getUint16(off, true); off += 2;
        const gh = view.getUint16(off, true); off += 2;
        const nTris = view.getUint32(off, true); off += 4;
        const N = gw * gh;

        // Use buffer.slice for typed arrays to avoid alignment requirements.
        const pos = new Float32Array(buffer.slice(off, off + N * 12)); off += N * 12;
        const col = new Float32Array(buffer.slice(off, off + N * 12)); off += N * 12;
        const valid = new Uint8Array(buffer, off, N); off += N;
        const tris = nTris > 0 ? new Uint32Array(buffer.slice(off, off + nTris * 12)) : null;
        off += nTris * 12;
        const covered = new Uint8Array(buffer, off, N);

        if (!window.__gotFrame) { window.__gotFrame = true; D('first surface: ' + gw + 'x' + gh + ' ' + nTris + ' tris'); }

        // Mesh: set all grid positions/colors, index by triangles.
        meshGeom.setAttribute('position', new THREE.BufferAttribute(pos.slice(), 3));
        meshGeom.setAttribute('color', new THREE.BufferAttribute(col.slice(), 3));
        if (tris) {
            meshGeom.setIndex(new THREE.BufferAttribute(tris.slice(), 1));
        } else {
            meshGeom.setIndex(null);
        }
        meshGeom.computeVertexNormals();

        // Uncovered-but-valid points: copy only those not part of any triangle.
        const uncPos = uncovGeom.attributes.position.array;
        const uncCol = uncovGeom.attributes.color.array;
        let ui = 0;
        for (let i = 0; i < N; i++) {
            if (valid[i] && !covered[i]) {
                uncPos[ui * 3]     = pos[i * 3];
                uncPos[ui * 3 + 1] = pos[i * 3 + 1];
                uncPos[ui * 3 + 2] = pos[i * 3 + 2];
                uncCol[ui * 3]     = col[i * 3];
                uncCol[ui * 3 + 1] = col[i * 3 + 1];
                uncCol[ui * 3 + 2] = col[i * 3 + 2];
                ui++;
            }
        }
        uncovGeom.attributes.position.needsUpdate = true;
        uncovGeom.attributes.color.needsUpdate = true;
        uncovGeom.setDrawRange(0, ui);
    });

    // --- state echo: point size + surface visibility ---
    hub.on('state', (msg) => {
        const framingMoved = msg.view_cam ? mergeViewCam(msg.view_cam) : false;
        const modeMoved = msg.view_mode !== undefined && msg.view_mode !== viewMode;
        if (msg.view_mode !== undefined) applyViewMode(msg.view_mode);
        // A framing edit with no mode change still has to be applied — that is
        // the slider being dragged. (When the mode also changed, applyViewMode
        // already placed the camera.)
        if (framingMoved && !modeMoved) applyPose(viewMode);
        if (msg.orbit_speed_deg_s !== undefined) {
            controls.autoRotateSpeed = msg.orbit_speed_deg_s * AUTOROTATE_PER_DEG_S;
        }
        if (msg.orbit_enabled !== undefined) {
            orbitOn = !!msg.orbit_enabled;
            controls.autoRotate = orbitOn && viewMode === 'world';
        }
        if (msg.orbit_mode !== undefined) orbitMode = msg.orbit_mode;
        if (msg.orbit_amplitude_deg !== undefined) orbitAmplitudeDeg = msg.orbit_amplitude_deg;
        if (msg.point_size !== undefined) {
            material.size = msg.point_size;
            xrayPointMat.size = msg.point_size;    // the twin must stay the same shape
        }
        // Uniform-only: toggling auto never recompiles the program.
        if (msg.point_size_auto !== undefined) pointUniforms.uAutoSize.value = msg.point_size_auto ? 1.0 : 0.0;
        if (msg.see_through !== undefined) setSeeThrough(msg.see_through);
        if (msg.surface_enabled !== undefined) {
            surfaceOn = !!msg.surface_enabled;
            applyVisibility();
        }
    });

    // Back to the current mode's configured framing (which for world is the
    // establishing shot, discarding whatever orbit the user had wandered into).
    function resetCamera() { applyPose(viewMode); }
    hub.on('reset_camera', resetCamera);

    window.addEventListener('resize', () => {
        camera.aspect = window.innerWidth / window.innerHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(window.innerWidth, window.innerHeight);
        pointUniforms.uMaxPx.value = 64.0 * window.devicePixelRatio;   // may change across displays
    });

    // Render loop + VIEW-fps measurement (browser paint rate, published ~1/s).
    let framesRendered = 0;
    let lastFpsTime = performance.now();
    // The magcal modal fully occludes this scene while it is open, so rendering
    // it is pure waste (and, under software WebGL, the thing that starves the
    // modal's own view). Additive: default true, nothing else touches it.
    let renderActive = true;
    function setRenderActive(on) { renderActive = !!on; }

    // Wall-clock delta handed to controls.update(). Without it OrbitControls
    // assumes a fixed 1/60 s step for auto-rotation, so the orbit would run at
    // whatever fraction of the requested deg/s the frame rate happens to be —
    // and this box renders the live scene nearer 13 fps than 60.
    let lastFrameTime = performance.now();
    function frameDelta() {
        const now = performance.now();
        const dt = (now - lastFrameTime) / 1000;
        lastFrameTime = now;
        return Math.min(dt, 0.25);          // clamp: a backgrounded tab must not lurch
    }

    function animate() {
        requestAnimationFrame(animate);
        const dt = frameDelta();
        if (!renderActive) {
            if (viewMode === 'world') { updateOscillate(); controls.update(dt); }
            return;
        }
        if (followOn && haveFollowTarget) {
            controls.enabled = false;
            // Velocity-adaptive lerp: fast when the sensor moves, steady when still.
            const d = camera.position.distanceTo(followEye);
            const alpha = Math.min(1, Math.max(0.12, d / 0.03));
            camera.position.lerp(followEye, alpha);
            controls.target.lerp(followCenter, alpha);
            camera.up.copy(followUp);
            camera.lookAt(controls.target);
        } else if (viewMode !== 'world') {
            // Camera locked to the sensor. Deliberately NOT controls.update():
            // damping keeps easing toward the controls' own internal spherical
            // state even while `enabled` is false, which would drift the pose.
        } else {
            updateOscillate();   // may flip autoRotateSpeed's sign before this update
            controls.update(dt);
        }
        renderer.render(scene, camera);

        framesRendered++;
        const now = performance.now();
        if (now - lastFpsTime >= 1000) {
            hub.emit('view_fps', framesRendered);
            framesRendered = 0;
            lastFpsTime = now;
        }
    }
    animate();

    // `controls` exposed for diagnostics only (e.g. sampling
    // getAzimuthalAngle() from the console to verify the oscillate wave) --
    // nothing external should mutate it; the state echo above is the only
    // supported way to drive it.
    return { resetCamera, THREE, scene, camera, controls, setPointsVisible, setFollow,
             setFollowTarget, setRenderActive };
}
