// slam.js — SLAM mode: reconstructed mesh + trajectory + follow camera (web Phase 4).
//
// The 9th ES module. Unlike sensors.js/capture.js (2D canvas), this renders 3D,
// but it reuses scene.js's single WebGL context and camera (via the sceneApi
// handle app.js passes in) — no second context, cheap on the headless
// SwiftShader box. It owns a THREE.Group added to that scene: two vertex-colored
// meshes (non-wall + wall — colors are pre-shaded server-side by MeshPrep, so an
// unlit MeshBasicMaterial shows them as-is), a floor grid, a trajectory ribbon,
// and the same scanner model used by the Sensors orientation gizmo at the
// current pose.
//
// All mode/toggle/enabled state is driven FROM the server's `state` echo
// (one-way flow, §5) so multiple tabs stay in sync. DOM events become
// hub.send(...): set_mode / slam_opt / save.
//
// Hub events:  subscribes "mesh" (binary, now delivered on ws.js's dedicated
//              `/ws-mesh` socket — see ws.js), "slam", "state", "saved";
//              sends set_mode / slam_opt / save / mesh_ack (via hub.ackMesh).
//
// MESH delivery (BUG-061 Part A): the server allows at most one un-acked mesh
// in flight per client. `hub.ackMesh(seq)` must fire only AFTER the mesh is
// actually consumed (geometry swapped in, uploaded to the GPU on the next
// paint) or the credit never frees and the map display starves — hence the
// `requestAnimationFrame` after `renderMeshes()`/`applyLines()` below, not an
// immediate ack on parse. `window.__slamDiag` exposes the resulting latency
// for headless verification: `{lastSlamAt, slamAgeS, lastMeshSeq, lastMeshAt,
// acksSent}`.

import { makeXrayMaterial } from './scene.js';
import { BODY_TO_CV, DEVICE_DIMS_M, createDeviceMesh } from './devicemodel.js';

export function createSlam(hub, sceneApi) {
    const D = (m, l) => { try { window.__diag && window.__diag('slam.js: ' + m, l); } catch (e) {} };
    if (!sceneApi || !sceneApi.THREE) { D('no sceneApi — SLAM disabled', 'error'); return {}; }
    const THREE = sceneApi.THREE;
    const $ = (id) => document.getElementById(id);

    // --- scene group ------------------------------------------------------
    const group = new THREE.Group();
    group.visible = false;
    sceneApi.scene.add(group);

    // --- colormap in the shader, not on the CPU (BUG-060) -----------------
    // Mesh packets carry a shaded scalar encoded as RGB, and the View card's
    // Turbo/Gray choice re-maps it. That used to run per-vertex in JS on every
    // mesh AND on every palette change, allocating a fresh Float32Array each
    // time — O(map) main-thread work at mesh rate, for a function of one
    // uniform. Evaluating it in the vertex shader instead means a new mesh
    // uploads its raw colors once and a palette change costs a single uniform
    // write with no geometry touch at all.
    //
    // One shared uniform object and ONE shared onBeforeCompile function across
    // all four materials (both submeshes + their see-through twins): three.js
    // keys the program cache on `onBeforeCompile.toString()`, so sharing the
    // function shares the compiled program, and sharing the uniform object
    // means one assignment moves every draw.
    const colormapUniform = { value: 0.0 };          // 0 = turbo, 1 = gray
    function paletteOnBeforeCompile(shader) {
        shader.uniforms.uColormap = colormapUniform;
        shader.vertexShader = shader.vertexShader
            .replace('#include <common>', [
                '#include <common>',
                'uniform float uColormap;',
                'vec3 rs_palette(float t) {',
                '    t = clamp(t, 0.0, 1.0);',
                '    if (uColormap > 0.5) return vec3(t);',
                // Google's Turbo polynomial — the same coefficients the CPU
                // path used, so the rendered result is unchanged.
                '    float r = 0.13572138 + t * (4.61539260 + t * (-42.66032258 + t * (132.13108234 + t * (-152.94239396 + t * 59.28637943))));',
                '    float g = 0.09140261 + t * (2.19418839 + t * (4.84296658 + t * (-14.18503333 + t * (4.27729857 + t * 2.82956604))));',
                '    float b = 0.10667330 + t * (12.64194608 + t * (-60.58204836 + t * (110.36276771 + t * (-89.90310912 + t * 27.34824973))));',
                '    return clamp(vec3(r, g, b), 0.0, 1.0);',
                '}',
            ].join('\n'))
            .replace('#include <color_vertex>', [
                '#include <color_vertex>',
                'vColor = rs_palette(dot(color, vec3(0.2126, 0.7152, 0.0722)));',
            ].join('\n'));
    }
    const nonWallMat = new THREE.MeshBasicMaterial({ vertexColors: true, side: THREE.DoubleSide });
    const wallMat = new THREE.MeshBasicMaterial({ vertexColors: true, side: THREE.DoubleSide });
    nonWallMat.onBeforeCompile = wallMat.onBeforeCompile = paletteOnBeforeCompile;
    const nonWallMesh = new THREE.Mesh(new THREE.BufferGeometry(), nonWallMat);
    const wallMesh = new THREE.Mesh(new THREE.BufferGeometry(), wallMat);
    const floorLines = new THREE.LineSegments(
        new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0x2a3550 }));
    const trajLine = new THREE.Line(
        new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0x35d07f }));
    // Frustum culling OFF on every live-updating object here, same trap as
    // scene.js:156-168: three.js computes `geometry.boundingSphere` lazily
    // ONCE and never invalidates it, but these buffers are rewritten in place
    // every packet (geometry reuse, below) -- a cached sphere from an early,
    // smaller/emptier packet would silently cull a later, bigger mesh. It also
    // avoids an O(N) `computeBoundingSphere` pass per packet, which is exactly
    // the kind of main-thread cost BUG-060 was about. These objects are always
    // in front of the viewer (the SLAM map IS the scene) so nothing is lost by
    // skipping the test.
    nonWallMesh.frustumCulled = false;
    wallMesh.frustumCulled = false;
    floorLines.frustumCulled = false;
    trajLine.frustumCulled = false;
    // `createDeviceMesh` is body-framed; SLAM poses are world<-CV-camera.
    // Keep that one declared bridge (BODY_TO_CV) here, rather than inventing a
    // second scanner silhouette or silently assuming the two frames agree.
    const scanner = new THREE.Group();
    scanner.name = 'scanner-model';
    scanner.matrixAutoUpdate = false;
    let scannerHasPose = false;
    // DEVICE_DIMS_M, not the shell-unit default: this scene is in metres, beside
    // real walls, so the marker has to be the device's real 5.5x3x2.5 inches.
    scanner.add(createDeviceMesh(THREE, {
        dims: DEVICE_DIMS_M, fillOpacity: 0.92, edgeOpacity: 0.95 }));
    // The device model's own geometry is static (only `scanner.matrix` moves),
    // so it doesn't have the rewritten-buffer trap above -- but its bounding
    // sphere is still computed relative to LOCAL space and this group can sit
    // far from the world origin at a SLAM pose, so belt-and-suspenders it too.
    scanner.traverse((obj) => { obj.frustumCulled = false; });
    const bodyToCv = new THREE.Matrix4().set(
        BODY_TO_CV[0], BODY_TO_CV[1], BODY_TO_CV[2], 0,
        BODY_TO_CV[3], BODY_TO_CV[4], BODY_TO_CV[5], 0,
        BODY_TO_CV[6], BODY_TO_CV[7], BODY_TO_CV[8], 0,
        0, 0, 0, 1);
    group.add(nonWallMesh, wallMesh, floorLines, trajLine, scanner);

    // See-through twins (owner ask, 2026-07-31). Same geometry, inverted depth
    // test: the parts of the map hidden behind a nearer wall are blended back
    // over it, so you can look into a room from outside without the near wall
    // going opaque-blank. Mechanism + why not plain opacity: scene.js
    // `makeXrayMaterial`. They live in `group`, so SLAM-mode gating is free.
    const xrayNonWallMat = makeXrayMaterial(THREE, nonWallMat);
    const xrayWallMat = makeXrayMaterial(THREE, wallMat);
    const xrayNonWall = new THREE.Mesh(nonWallMesh.geometry, xrayNonWallMat);
    const xrayWall = new THREE.Mesh(wallMesh.geometry, xrayWallMat);
    xrayNonWall.frustumCulled = false;   // same stale-bounding-sphere trap as their twins
    xrayWall.frustumCulled = false;
    xrayNonWall.visible = xrayWall.visible = false;
    group.add(xrayNonWall, xrayWall);
    let seeThrough = 0;

    let state = { source: 'live', display: 'point_cloud', slam_available: true,
        slam_trajectory: true, slam_walls: 'split', slam_follow: true,
        view_colormap: 'turbo', selected_capture: null, view_mode: 'world' };
    let lastVerts = 0;
    let lastMeshes = null;
    // Server-owned job state. Keeping it here (rather than deriving a timer on
    // each tab) makes a reconnect or a second browser show the same elapsed
    // time and ETA as the tab that started the build.
    let detailedBuild = null;
    let detailedResources = null;

    // Headless-verification surface (BUG-061). Initialized eagerly so a probe
    // reading it before the first packet sees the documented shape, not
    // `undefined` field accesses.
    window.__slamDiag = { lastSlamAt: null, slamAgeS: null, lastMeshSeq: null,
        lastMeshAt: null, acksSent: 0 };

    // --- geometry reuse (BUG-061 Part A) -----------------------------------
    // The historical path did `geometry.dispose()` + `new THREE.BufferGeometry()`
    // on EVERY packet -- a full GPU buffer free-and-reallocate for the whole
    // map, every time, even when the vertex count barely moved. That is exactly
    // the kind of main-thread/driver cost BUG-060 was about. Instead, keep each
    // object's geometry instance for its whole lifetime and, when a packet's
    // arrays fit the currently allocated capacity, copy the new data into the
    // SAME typed array (`array.set` + `needsUpdate` + `setDrawRange`/index
    // update) -- a plain memcpy, no GPU realloc. Only grow (the old dispose
    // path, but sized with headroom so consecutive growing packets don't keep
    // re-triggering it) when a packet needs more room than is currently
    // allocated.
    const GROWTH_HEADROOM = 1.5;

    // `needLen` is a flat element count (e.g. 3 * vertexCount for positions,
    // already how the wire format hands us pos/col; index arrays are likewise
    // already flattened by the caller).
    function ensureAttrCapacity(geom, name, ArrayCtor, itemSize, needLen) {
        const existing = geom.getAttribute(name);
        if (existing && existing.array.length >= needLen) return existing;
        const capacity = Math.ceil(Math.max(needLen, 1) * GROWTH_HEADROOM);
        const attr = new THREE.BufferAttribute(new ArrayCtor(capacity), itemSize);
        geom.setAttribute(name, attr);
        return attr;
    }

    function ensureIndexCapacity(geom, ArrayCtor, needLen) {
        const existing = geom.index;
        if (existing && existing.array.length >= needLen) return existing;
        const capacity = Math.ceil(Math.max(needLen, 1) * GROWTH_HEADROOM);
        const attr = new THREE.BufferAttribute(new ArrayCtor(capacity), 1);
        geom.setIndex(attr);
        return attr;
    }

    // --- MESH binary ingest ----------------------------------------------
    // Layout (docs/web-protocol.md): 9×u32 header then, per submesh, f32 pos,
    // f32 col, u32 idx; floor is f32 pos + u32 line-idx. Counts up front.
    hub.on('mesh', (buffer) => {
        const dv = new DataView(buffer);
        const u = (i) => dv.getUint32(i * 4, true);
        // [0]=tag [1]=mesh_seq [2]=flags
        const meshSeq = u(1);
        const nnwv = u(3), nnwt = u(4), nwv = u(5), nwt = u(6), nfp = u(7), nfl = u(8);
        let off = 36;
        const take = (Ctor, n) => { const a = new Ctor(buffer, off, n); off += n * Ctor.BYTES_PER_ELEMENT; return a; };

        const nwPos = take(Float32Array, 3 * nnwv);
        const nwCol = take(Float32Array, 3 * nnwv);
        const nwIdx = take(Uint32Array, 3 * nnwt);
        const wPos = take(Float32Array, 3 * nwv);
        const wCol = take(Float32Array, 3 * nwv);
        const wIdx = take(Uint32Array, 3 * nwt);
        const fPos = take(Float32Array, 3 * nfp);
        const fIdx = take(Uint32Array, 2 * nfl);

        lastMeshes = { nwPos, nwCol, nwIdx, wPos, wCol, wIdx };
        renderMeshes();
        applyLines(floorLines, fPos, fIdx);
        if (state.display === 'detailed' && detailedBuild && !detailedBuild.done) {
            detailedBuild.meshShown = true;
            renderDetailedBuildStatus();
        }
        if (!window.__gotMesh) { window.__gotMesh = true; D('first mesh: ' + nnwv + ' non-wall verts'); }

        window.__slamDiag.lastMeshSeq = meshSeq;
        window.__slamDiag.lastMeshAt = Date.now() / 1000;
        // Ack only AFTER the geometry swap above has actually landed and the
        // browser has had a paint to upload it to the GPU -- an immediate ack
        // here would tell the server "consumed" before the bytes are even in
        // VRAM, which is exactly the measurement the credit scheme needs to be
        // honest. `hub.ackMesh` sends on the dedicated `/ws-mesh` socket.
        requestAnimationFrame(() => {
            hub.ackMesh(meshSeq);
            window.__slamDiag.acksSent++;
        });
    });

    function renderMeshes() {
        if (!lastMeshes) return;
        const m = lastMeshes;
        applyMesh(nonWallMesh, m.nwPos, m.nwCol, m.nwIdx);
        applyMesh(wallMesh, m.wPos, m.wCol, m.wIdx);
    }

    // The palette is a uniform now (see paletteOnBeforeCompile): a Turbo/Gray
    // change re-renders nothing, it just moves the uniform every draw reads.
    function applyColormap(name) { colormapUniform.value = name === 'gray' ? 1.0 : 0.0; }

    // The see-through twin (`xrayNonWall`/`xrayWall`) was pointed at THIS SAME
    // geometry object once, at construction time, and nothing here ever
    // replaces `mesh.geometry` with a new instance -- so the twin tracks every
    // update for free and needs no parameter or separate write.
    function applyMesh(mesh, pos, col, idx) {
        const g = mesh.geometry;
        const nV = pos.length;             // flat float count: 3 * vertexCount
        if (nV === 0) {
            g.setDrawRange(0, 0);           // nothing to draw; leave any allocated buffers as-is
            return;
        }
        // BUG-060 note this superseded still applies to the COPY itself: these
        // arrays are already correctly-typed, 4-byte-aligned views onto the
        // received frame's buffer. `array.set` is one native memcpy per
        // attribute -- cheap relative to the GPU buffer (re)allocation this
        // whole scheme exists to avoid, and unavoidable now that the
        // destination is a separately-allocated, capacity-padded buffer rather
        // than the incoming buffer itself.
        const posAttr = ensureAttrCapacity(g, 'position', Float32Array, 3, nV);
        const colAttr = ensureAttrCapacity(g, 'color', Float32Array, 3, nV);
        posAttr.array.set(pos);
        colAttr.array.set(col);
        posAttr.needsUpdate = true;
        colAttr.needsUpdate = true;

        const nI = idx.length;
        if (nI > 0) {
            const idxAttr = ensureIndexCapacity(g, Uint32Array, nI);
            idxAttr.array.set(idx);
            idxAttr.needsUpdate = true;
            g.setDrawRange(0, nI);          // indexed: count is INDICES, not vertices
        } else {
            if (g.index) g.setIndex(null);
            g.setDrawRange(0, nV / 3);       // non-indexed: count is vertices
        }
    }

    function applyLines(obj, pos, idx) {
        const g = obj.geometry;
        const nV = pos.length;
        if (nV > 0) {
            const posAttr = ensureAttrCapacity(g, 'position', Float32Array, 3, nV);
            posAttr.array.set(pos);
            posAttr.needsUpdate = true;
        }
        const nI = idx ? idx.length : 0;
        if (nI > 0) {
            const idxAttr = ensureIndexCapacity(g, Uint32Array, nI);
            idxAttr.array.set(idx);
            idxAttr.needsUpdate = true;
            g.setDrawRange(0, nI);
        } else {
            if (g.index) g.setIndex(null);
            g.setDrawRange(0, nV / 3);
        }
    }

    // --- `slam` per-frame message ----------------------------------------
    hub.on('slam', (msg) => {
        // Display-lag diagnostic (BUG-061): `msg.ts` is the server's epoch
        // time when this message was built; the gap to "now" is the whole
        // pose-path latency the credit-based mesh transport was built to keep
        // bounded. `msg.ts` is a newly-added field -- guard its absence so an
        // old server doesn't leave `slamAgeS` looking bogus.
        window.__slamDiag.lastSlamAt = Date.now() / 1000;
        if (typeof msg.ts === 'number') {
            window.__slamDiag.slamAgeS = window.__slamDiag.lastSlamAt - msg.ts;
        }

        // Trajectory ribbon from the downsampled tail (geometry reuse, as mesh).
        const tail = Array.isArray(msg.traj_tail) ? msg.traj_tail : [];
        const trajPos = new Float32Array(tail.length * 3);
        for (let i = 0; i < tail.length; i++) {
            trajPos[i * 3] = tail[i][0]; trajPos[i * 3 + 1] = tail[i][1]; trajPos[i * 3 + 2] = tail[i][2];
        }
        applyLines(trajLine, trajPos, null);

        // Scanner model at the current SLAM pose (row-major world<-CV camera).
        const p = msg.pose;
        if (Array.isArray(p) && p.length === 16) {
            const pose = new THREE.Matrix4().set(
                p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7],
                p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15]);
            scanner.matrix.copy(pose).multiply(bodyToCv);
            scanner.matrixWorldNeedsUpdate = true;
            scannerHasPose = true;
            updateScannerVisibility();
            sceneApi.setSlamPose(p);
        }

        // World + Follow ("track, keep my framing", owner choice #1): pan the
        // orbit toward the scanner by translating camera+target together,
        // preserving the user's own zoom/angle -- see scene.js `trackTarget`.
        // This deliberately does NOT use the server's `msg.follow` eye/center
        // (a fixed nose-camera shot): that framing is for FPV/Mirror, which
        // stay scanner-relative via `setSlamPose`/`applySlamPose` above and
        // are unaffected by this branch.
        if (state.display === 'slam' && state.slam_follow && state.view_mode === 'world'
                && Array.isArray(p) && p.length === 16) {
            sceneApi.trackTarget([p[3], p[7], p[11]]);
        }

        lastVerts = msg.mesh_verts || 0;
        updateHud(msg);
        updateSaveEnabled();
    });

    // --- HUD --------------------------------------------------------------
    function updateHud(m) {
        const set = (id, v) => { const el = $(id); if (el) el.textContent = v; };
        const track = $('slam-track');
        if (track) {
            track.textContent = m.tracking_lost ? 'LOST' : 'OK';
            track.classList.toggle('lost', !!m.tracking_lost);
        }
        set('slam-fitness', (m.fitness ?? 0).toFixed(2));
        set('slam-rmse', ((m.rmse ?? 0) * 1000).toFixed(1) + ' mm');
        set('slam-frames', m.frames_integrated ?? 0);
        set('slam-verts', (m.mesh_verts ?? 0).toLocaleString());
        set('slam-ms', (m.slam_ms ?? 0).toFixed(1) + ' ms');
        // GPU visibility (BUG-061 Part B): the compute device string, when the
        // server sends one. `device` may be absent/null on an older server or
        // a container-backed remote worker that doesn't report it.
        set('slam-device', m.device || '—');
        updateBlockGauge(m);
    }

    // TSDF block-grid headroom (BUG-035). The mapper samples this at ~4 Hz
    // (block_usage() is a device sync on CUDA), so it arrives null on most
    // `slam` messages early in a scan — null renders as unknown, never as 0
    // blocks, which would read as "loads of headroom" at exactly the wrong
    // moment. `capacity` is the hash grid's LIVE capacity (Open3D rehashes to
    // grow); `configured` is the [slam] block_count the operator can raise,
    // and is what the warning threshold is really about.
    function updateBlockGauge(m) {
        const val = $('res-blocks-val');
        const fill = $('res-blocks-fill');
        if (!val && !fill) return;
        const used = m.blocks_used, cap = m.blocks_capacity, cfg = m.blocks_configured;
        if (used === null || used === undefined || !cap) {
            if (val) val.textContent = 'n/a';
            if (fill) {
                fill.style.width = '0%';
                fill.classList.remove('is-warn', 'is-crit');
            }
            return;
        }
        const frac = used / cap;
        if (val) {
            val.textContent = used.toLocaleString() + ' / ' + cap.toLocaleString()
                + (cfg && cfg !== cap ? ' (cfg ' + cfg.toLocaleString() + ')' : '');
        }
        if (fill) {
            fill.style.width = (Math.max(0, Math.min(1, frac)) * 100).toFixed(0) + '%';
            fill.classList.toggle('is-warn', frac >= 0.80 && frac < 0.90);
            fill.classList.toggle('is-crit', frac >= 0.90);
        }
    }

    // --- server `state` echo drives mode + toggles + visibility ----------
    hub.on('state', (msg) => {
        state = {
            source: msg.source || 'live',
            display: msg.display || (msg.mode === 'slam' ? 'slam' : 'point_cloud'),
            selected_capture: msg.selected_capture || null,
            slam_available: msg.slam_available !== false,
            detailed: msg.detailed || null,
            slam_trajectory: msg.slam_trajectory !== false,
            slam_walls: msg.slam_walls || 'split',
            slam_follow: msg.slam_follow !== false,
            view_colormap: msg.view_colormap || 'turbo',
            view_mode: msg.view_mode || 'world',
        };
        applyColormap(state.view_colormap);
        if (msg.see_through !== undefined) {
            seeThrough = Math.min(1, Math.max(0, Number(msg.see_through) || 0));
            xrayNonWallMat.opacity = xrayWallMat.opacity = seeThrough;
        }
        applyState();
    });

    // Single source of truth for the scanner model's visibility (BUG-061 Part
    // D, owner ask): visible only in World. It used to be forced `true`
    // whenever a pose arrived, in every view mode -- correct for FPV/Mirror
    // (a scanner-relative camera has no reason to also draw the scanner in
    // frame) but wrong in those modes specifically, since the camera sits
    // AT the scanner and the model would render as a giant box wrapped around
    // the viewpoint. Called from both places a pose can update `scannerHasPose`
    // (`slam`/`detailed` handlers) and from `applyState()` on every mode/toggle
    // change, so a view-mode switch alone (no new pose) still updates it.
    function updateScannerVisibility() {
        const slamOn = state.display === 'slam' || state.display === 'detailed';
        scanner.visible = slamOn && scannerHasPose && state.view_mode === 'world';
    }

    function applyState() {
        const slamOn = state.display === 'slam' || state.display === 'detailed';
        group.visible = slamOn;
        sceneApi.setPointsVisible(!slamOn && state.display !== 'preview');
        // FPV/Mirror are explicitly scanner-relative cameras, so they always
        // follow their pose. World retains the optional follow toggle for a
        // moving map overview.
        sceneApi.setFollow(slamOn && (state.slam_follow || state.view_mode !== 'world'));
        trajLine.visible = state.slam_trajectory;
        updateScannerVisibility();
        // Point clouds are mirrored server-side. SLAM/Detailed maps stay in a
        // stable world frame, so the mirror view flips only their canvas.
        sceneApi.setViewportMirror(slamOn && state.view_mode === 'mirror');
        // Off at 0 so the renderer skips the pass entirely (the default costs
        // nothing); `group.visible` already handles the realtime case.
        xrayNonWall.visible = xrayWall.visible = seeThrough > 0;

        // Mode segmented + SLAM group / HUD visibility.
        setActive($('seg-source'), 'source', state.source);
        setActive($('seg-display'), 'display', state.display);
        for (const b of $('seg-display')?.querySelectorAll('button[data-display]') || []) {
            b.disabled = (b.dataset.display === 'preview' &&
                (state.source !== 'view' || !state.selected_capture)) ||
                (b.dataset.display === 'detailed' && state.source !== 'view') ||
                ((b.dataset.display === 'slam' || b.dataset.display === 'detailed') && !state.slam_available);
        }
        $('slam-group')?.classList.toggle('hidden', !slamOn);
        $('slam-hud')?.classList.toggle('hidden', !slamOn);
        // Resource headroom is a LIVE SLAM concern (owner ask, 2026-07-31):
        // that is the only mode that can exhaust VRAM or the TSDF block grid
        // while you are standing there holding the scanner. Replay/Detailed
        // reconstructions are repeatable, so they don't get the card.
        $('resources-card')?.classList.toggle('hidden',
            !(state.source === 'live' && state.display === 'slam'));
        renderDetailedBuildStatus();

        // Toggles reflect server truth.
        const t = $('chk-slam-traj'); if (t) t.checked = state.slam_trajectory;
        // Follow now lives in the View card (moved out of the SLAM card, owner
        // ask -- it had never been seen there), so it is visible whenever that
        // card is: disable it, with an explanatory title, whenever toggling it
        // couldn't do anything -- outside World (FPV/Mirror always follow) or
        // outside SLAM/Detailed display (nothing to follow at all).
        const f = $('chk-slam-follow');
        if (f) {
            f.checked = state.slam_follow;
            f.disabled = state.view_mode !== 'world' || !slamOn;
            if (f.parentElement) f.parentElement.title = !slamOn
                ? 'Only affects the SLAM/Detailed map display'
                : state.view_mode === 'world'
                    ? 'Camera automatically follows the device position'
                    : 'FPV and Mirror always follow the scanner; switch to World to control this';
        }
        setActive($('seg-walls'), 'walls', state.slam_walls);
        updateSaveEnabled();
    }

    function updateSaveEnabled() {
        // Live SLAM only (owner decision, 2026-07-31): a live scan is
        // unrepeatable, so its one-shot export stays. Replay SLAM is a preview
        // -- Detailed is the capture-keyed sidecar -- so the button explains
        // itself rather than silently doing nothing.
        const b = $('btn-save');
        if (!b) return;
        const live = state.source === 'live' && state.display === 'slam';
        b.disabled = !(live && lastVerts > 0);
        b.title = state.display === 'detailed'
            ? 'Detailed writes its own sidecar automatically'
            : (state.source === 'live'
                ? 'Export the reconstructed mesh as a PLY file'
                : 'Replay SLAM is a preview — use Detailed to write a saved reconstruction');
    }

    // --- saved-maps library ----------------------------------------------
    hub.on('saved', (msg) => renderSaved(Array.isArray(msg.items) ? msg.items : []));
    function renderSaved(items) {
        const el = $('saved-list');
        if (!el) return;
        if (!items.length) { el.innerHTML = '<div class="cap-status">no saved maps yet</div>'; return; }
        el.innerHTML = items.map((it) =>
            `<div class="cap-row"><a class="cap-row__name" href="/results/${encodeURIComponent(it.name)}" ` +
            `download>${escapeHtml(it.name)}</a>` +
            `<span class="cap-row__meta">${fmtBytes(it.bytes)}</span></div>`).join('');
    }

    // --- outbound controls ------------------------------------------------
    $('seg-source')?.addEventListener('click', (e) => {
        const btn = e.target.closest('button[data-source]');
        if (btn) hub.send({ type: 'set_source', source: btn.dataset.source });
    });
    $('seg-display')?.addEventListener('click', (e) => {
        const btn = e.target.closest('button[data-display]');
        if (!btn || btn.disabled) return;
        if (btn.dataset.display === 'detailed') {
            const detail = state.detailed || {};
            if (!detail.exists || detail.stale) {
                const hint = detail.stale ? 'A stale sidecar exists. Regenerate it?' : 'Build Detailed SLAM for this capture?';
                if (!window.confirm(hint)) return;
                hub.send({ type: detail.stale ? 'regenerate_detailed' : 'generate_detailed' });
            }
        }
        hub.send({ type: 'set_display', display: btn.dataset.display });
    });
    $('chk-slam-traj')?.addEventListener('change', (e) =>
        hub.send({ type: 'slam_opt', trajectory: e.target.checked }));
    $('chk-slam-follow')?.addEventListener('change', (e) =>
        hub.send({ type: 'slam_opt', follow: e.target.checked }));
    $('seg-walls')?.addEventListener('click', (e) => {
        const btn = e.target.closest('button[data-walls]');
        if (btn) hub.send({ type: 'slam_opt', walls: btn.dataset.walls });
    });
    $('btn-save')?.addEventListener('click', () => hub.send({ type: 'save' }));

    // Metrics already arrive at a modest, real-time cadence for the HUD.  The
    // Detailed overlay mirrors the host-wide values here because offline mesh
    // extraction can be the most resource-intensive stage of a build.
    hub.on('metrics', (msg) => {
        detailedResources = msg.resources || null;
        renderDetailedBuildStatus();
    });

    hub.on('detailed', (msg) => {
        const done = !!msg.done;
        const total = msg.total || 0;
        const processed = msg.processed || 0;
        const el = $('slam-frames');
        if (el && total) el.textContent = `${processed} / ${total}${done ? ' done' : ''}`;
        // A cached sidecar is ready to view, not an active build. It should not
        // resurrect an old completion banner just because the user revisits it.
        if (msg.phase === 'cached') {
            detailedBuild = null;
        } else if (msg.started || total || msg.reason || msg.phase === 'loading_cached') {
            detailedBuild = { ...(detailedBuild || {}), ...msg };
        }
        renderDetailedBuildStatus();
        // Detailed reconstruction owns an offline trajectory. Its latest pose
        // gives FPV/Mirror the same scanner-relative camera and model as SLAM.
        if (Array.isArray(msg.pose) && msg.pose.length === 16) {
            const p = msg.pose;
            const pose = new THREE.Matrix4().set(
                p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7],
                p[8], p[9], p[10], p[11], p[12], p[13], p[14], p[15]);
            scanner.matrix.copy(pose).multiply(bodyToCv);
            scanner.matrixWorldNeedsUpdate = true;
            scannerHasPose = true;
            updateScannerVisibility();
            sceneApi.setSlamPose(p);
        }
    });

    function fmtTime(seconds) {
        if (!Number.isFinite(seconds) || seconds < 0) return null;
        const s = Math.round(seconds);
        return Math.floor(s / 60) + ':' + String(s % 60).padStart(2, '0');
    }

    function renderDetailedBuildStatus() {
        const panel = $('detailed-build-status');
        if (!panel) return;
        const b = detailedBuild;
        const visible = state.display === 'detailed' && b &&
            (b.started || b.total || b.reason || b.phase === 'loading_cached') && b.phase !== 'cached';
        panel.classList.toggle('hidden', !visible);
        if (!visible) return;

        const total = Number(b.total) || 0;
        const processed = Math.max(0, Math.min(total, Number(b.processed) || 0));
        const fraction = total ? Math.max(0, Math.min(1,
            Number.isFinite(Number(b.fraction)) ? Number(b.fraction) : processed / total)) : 0;
        const pct = Math.round(fraction * 100);
        const done = !!b.done;
        const title = $('detailed-build-title');
        const progress = $('detailed-build-progress');
        const bar = $('detailed-build-bar');
        const timing = $('detailed-build-time');
        const detail = $('detailed-build-detail');
        if (title) title.textContent = b.phase === 'failed' ? 'Detailed reconstruction could not load'
            : done ? 'Detailed reconstruction saved'
            : b.phase === 'loading_cached' ? 'Loading saved detailed reconstruction'
            : b.phase === 'extracting_mesh' ? 'Extracting preview mesh'
            : processed ? 'Building detailed reconstruction' : 'Preparing detailed reconstruction';
        if (progress) progress.textContent = total ? `${processed} / ${total} · ${pct}%`
            : (b.reason || 'Preparing');
        if (bar) {
            bar.style.width = pct + '%';
            bar.setAttribute('aria-valuenow', String(pct));
        }
        renderDetailedResourceBars(detailedResources);
        const elapsed = fmtTime(Number(b.elapsed_s));
        const eta = fmtTime(Number(b.eta_s));
        if (timing) timing.textContent = b.phase === 'failed' ? 'Loading failed'
            : done
            ? `Completed in ${elapsed || '—'}`
            : `Elapsed ${elapsed || '0:00'} · ${eta ? 'ETA ' + eta : 'calculating ETA'}`;
        if (detail) {
            if (b.phase === 'failed') detail.textContent = b.reason || 'The saved reconstruction could not be loaded.';
            else if (done) detail.textContent = 'Saved reconstruction is ready to inspect or download.';
            else if (b.phase === 'loading_cached') detail.textContent = 'Reading the saved mesh and preparing it for the viewport.';
            else if (b.phase === 'extracting_mesh') detail.textContent = `Extracting a preview mesh at frame ${processed || '—'}; frame progress pauses briefly during this GPU step.`;
            else if (b.meshShown) detail.textContent = 'Rendering the latest reconstructed mesh below.';
            else if (b.mesh_every) detail.textContent = `Preparing the first mesh (updates every ${b.mesh_every} frames).`;
            else detail.textContent = 'The mesh will appear here as it is reconstructed.';
        }
    }

    function renderDetailedResourceBars(resources) {
        const set = (name, fraction, value) => {
            const fill = $('detailed-resource-' + name);
            const label = $('detailed-resource-' + name + '-value');
            if (label) label.textContent = value;
            if (!fill) return;
            if (!Number.isFinite(fraction)) {
                fill.style.width = '0%';
                fill.classList.remove('is-warn', 'is-crit');
                return;
            }
            const clamped = Math.max(0, Math.min(1, fraction));
            fill.style.width = (clamped * 100).toFixed(0) + '%';
            fill.classList.toggle('is-warn', clamped >= 0.80 && clamped < 0.90);
            fill.classList.toggle('is-crit', clamped >= 0.90);
        };
        if (!resources) {
            for (const name of ['gpu', 'cpu', 'ram', 'vram']) set(name, null, 'n/a');
            return;
        }
        const known = (value) => value !== null && value !== undefined && Number.isFinite(Number(value));
        const percent = (value) => known(value) ? Math.round(Number(value)) + '%' : 'n/a';
        const ratio = (used, total) => known(used) && Number(used) >= 0 &&
            known(total) && Number(total) > 0 ? Number(used) / Number(total) : null;
        set('gpu', known(resources.gpu_util) ? Number(resources.gpu_util) / 100 : null,
            percent(resources.gpu_util));
        set('cpu', known(resources.sys_cpu_percent) ? Number(resources.sys_cpu_percent) / 100 : null,
            percent(resources.sys_cpu_percent));
        const ram = ratio(resources.ram_used, resources.ram_total);
        set('ram', ram, ram === null ? 'n/a' : Math.round(ram * 100) + '%');
        const vram = ratio(resources.device_vram_used, resources.device_vram_total);
        set('vram', vram, vram === null ? 'n/a' : Math.round(vram * 100) + '%');
    }

    // --- helpers ----------------------------------------------------------
    function setActive(seg, attr, value) {
        if (!seg) return;
        for (const b of seg.querySelectorAll('button')) b.classList.toggle('active', b.dataset[attr] === value);
    }
    const fmtBytes = (n) => !n ? '0 B' : n < 1024 ? n + ' B'
        : n < 1048576 ? (n / 1024).toFixed(1) + ' KB' : (n / 1048576).toFixed(1) + ' MB';
    function escapeHtml(s) {
        return String(s).replace(/[&<>"']/g, (c) =>
            ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
    }

    applyState();
    return {};
}
