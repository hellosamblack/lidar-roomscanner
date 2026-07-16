// slam.js — SLAM mode: reconstructed mesh + trajectory + follow camera (web Phase 4).
//
// The 9th ES module. Unlike sensors.js/capture.js (2D canvas), this renders 3D,
// but it reuses scene.js's single WebGL context and camera (via the sceneApi
// handle app.js passes in) — no second context, cheap on the headless
// SwiftShader box. It owns a THREE.Group added to that scene: two vertex-colored
// meshes (non-wall + wall — colors are pre-shaded server-side by MeshPrep, so an
// unlit MeshBasicMaterial shows them as-is), a floor grid, a trajectory ribbon,
// and a head marker at the current pose.
//
// All mode/toggle/enabled state is driven FROM the server's `state` echo
// (one-way flow, §5) so multiple tabs stay in sync. DOM events become
// hub.send(...): set_mode / slam_opt / save.
//
// Hub events:  subscribes "mesh" (binary), "slam", "state", "saved";
//              sends set_mode / slam_opt / save.

export function createSlam(hub, sceneApi) {
    const D = (m, l) => { try { window.__diag && window.__diag('slam.js: ' + m, l); } catch (e) {} };
    if (!sceneApi || !sceneApi.THREE) { D('no sceneApi — SLAM disabled', 'error'); return {}; }
    const THREE = sceneApi.THREE;
    const $ = (id) => document.getElementById(id);

    // --- scene group ------------------------------------------------------
    const group = new THREE.Group();
    group.visible = false;
    sceneApi.scene.add(group);

    const nonWallMat = new THREE.MeshBasicMaterial({ vertexColors: true, side: THREE.DoubleSide });
    const wallMat = new THREE.MeshBasicMaterial({ vertexColors: true, side: THREE.DoubleSide });
    const nonWallMesh = new THREE.Mesh(new THREE.BufferGeometry(), nonWallMat);
    const wallMesh = new THREE.Mesh(new THREE.BufferGeometry(), wallMat);
    const floorLines = new THREE.LineSegments(
        new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0x2a3550 }));
    const trajLine = new THREE.Line(
        new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0x35d07f }));
    const head = new THREE.Mesh(
        new THREE.SphereGeometry(0.03, 12, 12),
        new THREE.MeshBasicMaterial({ color: 0x7fffd4 }));
    group.add(nonWallMesh, wallMesh, floorLines, trajLine, head);

    let state = { mode: 'realtime', slam_trajectory: true, slam_walls: 'split', slam_follow: true };
    let lastVerts = 0;

    // --- MESH binary ingest ----------------------------------------------
    // Layout (docs/web-protocol.md): 9×u32 header then, per submesh, f32 pos,
    // f32 col, u32 idx; floor is f32 pos + u32 line-idx. Counts up front.
    hub.on('mesh', (buffer) => {
        const dv = new DataView(buffer);
        const u = (i) => dv.getUint32(i * 4, true);
        // [0]=tag [1]=mesh_seq [2]=flags
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

        applyMesh(nonWallMesh, nwPos, nwCol, nwIdx);
        applyMesh(wallMesh, wPos, wCol, wIdx);
        applyLines(floorLines, fPos, fIdx);
        if (!window.__gotMesh) { window.__gotMesh = true; D('first mesh: ' + nnwv + ' non-wall verts'); }
    });

    function applyMesh(mesh, pos, col, idx) {
        const g = mesh.geometry;
        g.dispose();                                   // free the old GPU buffers
        const ng = new THREE.BufferGeometry();
        if (pos.length) {
            ng.setAttribute('position', new THREE.BufferAttribute(new Float32Array(pos), 3));
            ng.setAttribute('color', new THREE.BufferAttribute(new Float32Array(col), 3));
            if (idx.length) ng.setIndex(new THREE.BufferAttribute(new Uint32Array(idx), 1));
        }
        mesh.geometry = ng;
    }

    function applyLines(obj, pos, idx) {
        const g = obj.geometry;
        g.dispose();
        const ng = new THREE.BufferGeometry();
        if (pos.length) {
            ng.setAttribute('position', new THREE.BufferAttribute(new Float32Array(pos), 3));
            if (idx.length) ng.setIndex(new THREE.BufferAttribute(new Uint32Array(idx), 1));
        }
        obj.geometry = ng;
    }

    // --- `slam` per-frame message ----------------------------------------
    hub.on('slam', (msg) => {
        // Trajectory ribbon from the downsampled tail.
        const tail = Array.isArray(msg.traj_tail) ? msg.traj_tail : [];
        const tg = trajLine.geometry;
        tg.dispose();
        const ng = new THREE.BufferGeometry();
        if (tail.length) {
            const arr = new Float32Array(tail.length * 3);
            for (let i = 0; i < tail.length; i++) {
                arr[i * 3] = tail[i][0]; arr[i * 3 + 1] = tail[i][1]; arr[i * 3 + 2] = tail[i][2];
            }
            ng.setAttribute('position', new THREE.BufferAttribute(arr, 3));
        }
        trajLine.geometry = ng;

        // Head marker at the current pose translation (row-major col 3).
        const p = msg.pose;
        if (Array.isArray(p) && p.length === 16) head.position.set(p[3], p[7], p[11]);

        // Follow camera (server-computed eye/center/up).
        if (state.mode === 'slam' && state.slam_follow && msg.follow) {
            sceneApi.setFollowTarget(msg.follow.eye, msg.follow.center, msg.follow.up);
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
    }

    // --- server `state` echo drives mode + toggles + visibility ----------
    hub.on('state', (msg) => {
        state = {
            mode: msg.mode || 'realtime',
            slam_trajectory: msg.slam_trajectory !== false,
            slam_walls: msg.slam_walls || 'split',
            slam_follow: msg.slam_follow !== false,
        };
        applyState();
    });

    function applyState() {
        const slamOn = state.mode === 'slam';
        group.visible = slamOn;
        sceneApi.setPointsVisible(!slamOn);
        sceneApi.setFollow(slamOn && state.slam_follow);
        trajLine.visible = state.slam_trajectory;
        head.visible = slamOn;

        // Mode segmented + SLAM group / HUD visibility.
        setActive($('seg-mode'), 'mode', state.mode);
        $('slam-group')?.classList.toggle('hidden', !slamOn);
        $('slam-hud')?.classList.toggle('hidden', !slamOn);

        // Toggles reflect server truth.
        const t = $('chk-slam-traj'); if (t) t.checked = state.slam_trajectory;
        const f = $('chk-slam-follow'); if (f) f.checked = state.slam_follow;
        setActive($('seg-walls'), 'walls', state.slam_walls);
        updateSaveEnabled();
    }

    function updateSaveEnabled() {
        const b = $('btn-save');
        if (b) b.disabled = !(state.mode === 'slam' && lastVerts > 0);
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
    $('seg-mode')?.addEventListener('click', (e) => {
        const btn = e.target.closest('button[data-mode]');
        if (btn) hub.send({ type: 'set_mode', mode: btn.dataset.mode });
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
