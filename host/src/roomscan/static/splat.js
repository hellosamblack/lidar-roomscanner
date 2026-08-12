// splat.js -- the third "Splat" source: render an offline Gaussian-splat
// reconstruction navigably in the shared Three.js scene.
//
// A splat is static geometry, so this follows the slam.js integration pattern:
// pull THREE off the shared scene handle (never re-import three a second way),
// build a @mkkellogg/gaussian-splats-3d DropInViewer, and add it to the ONE
// scene/camera/renderer. DropInViewer sorts its splats in its own onBeforeRender,
// so the existing single render loop drives it for free -- no extra GL context
// (this host software-renders when headless), no second render pass.
//
// Navigation is the scene's own World-mode OrbitControls. Coordinate framing:
// the build's manifest carries a 4x4 that levels COLMAP's arbitrary frame to the
// viewer's y-down world (gravity -> +Y), decomposed here into the position/
// rotation/scale DropInViewer accepts.
import * as GaussianSplats3D from '@mkkellogg/gaussian-splats-3d';

export function createSplat(hub, sceneApi) {
    const THREE = sceneApi.THREE;
    let viewer = null;              // lazily built DropInViewer (Object3D in the scene)
    let loadedSlug = null;          // which splat is currently in the viewer
    let loading = false;
    let items = [];                 // latest `splats` list
    let state = { source: 'live', selected_splat: null };
    let framedSlug = null;          // last splat we auto-framed the camera on
    let sourceVideos = [];          // latest `source_videos` list (mp4s + their builds)
    let defaults = {};              // SplatPreset defaults, to seed the build form
    let seededForm = false;
    let build = null;               // latest `splat_build` progress, or null when idle
    let doneTimer = null;           // auto-clears a completed banner
    let resources = null;           // latest metrics.resources, for the build bars
    let seeThrough = 0;             // shared See-Through strength (0..1), from state

    // Controls that don't apply while viewing a static splat -- greyed + inert in
    // Splat source (the display selector is meaningless, and device / colour /
    // surface settings drive the live point cloud, not a splat). See-Through and the
    // Camera & Pose subgroup stay live (they navigate/reveal the splat).
    const GATE_SELECTORS = ['#seg-display', '[data-card-id="device"]',
        '[data-subgroup-id="view-color"]', '[data-subgroup-id="view-surface"]'];
    function gateSplatMode(isSplat) {
        for (const sel of GATE_SELECTORS) {
            document.querySelector(sel)?.classList.toggle('mode-off', isSplat);
        }
    }

    // Apply the shared See-Through strength to the splat: lower the scene's global
    // opacity so near gaussians turn translucent and you see into the room. Uses the
    // vendor's per-scene `opacity` (-> `sceneOpacity` uniform -> vColor.a); wrapped
    // because it reaches vendor internals and must never break the viewer.
    function applySeeThrough() {
        if (!viewer || loadedSlug === null) return;
        try {
            const scene = viewer.getSplatScene && viewer.getSplatScene(0);
            if (scene) scene.opacity = Math.max(0.08, 1 - seeThrough);
        } catch (e) { /* vendor internals changed -- ignore, keep rendering */ }
    }

    function ensureViewer() {
        if (viewer) return viewer;
        viewer = new GaussianSplats3D.DropInViewer({
            // CPU sort in a plain worker: no SharedArrayBuffer, so no COOP/COEP
            // response headers are required, and GPU sort is pointless under the
            // headless software renderer anyway.
            gpuAcceleratedSort: false,
            sharedMemoryForWorkers: false,
            sceneRevealMode: GaussianSplats3D.SceneRevealMode.Instant,
        });
        viewer.visible = false;
        sceneApi.scene.add(viewer);
        return viewer;
    }

    // Manifest 4x4 (row-major list of 4 rows) -> DropInViewer TRS.
    function decompose(transform) {
        const m = new THREE.Matrix4();
        if (Array.isArray(transform) && transform.length === 4) {
            const r = transform;
            m.set(r[0][0], r[0][1], r[0][2], r[0][3],
                  r[1][0], r[1][1], r[1][2], r[1][3],
                  r[2][0], r[2][1], r[2][2], r[2][3],
                  r[3][0], r[3][1], r[3][2], r[3][3]);
        }
        const p = new THREE.Vector3(), q = new THREE.Quaternion(), s = new THREE.Vector3();
        m.decompose(p, q, s);
        return { position: [p.x, p.y, p.z], rotation: [q.x, q.y, q.z, q.w],
                 scale: [s.x, s.y, s.z] };
    }

    function showLoading(name, pct) {
        const box = document.getElementById('splat-loading');
        const label = document.getElementById('splat-loading-label');
        const bar = document.getElementById('splat-loading-bar');
        if (box) box.classList.remove('hidden');
        if (label) label.textContent = pct >= 100 ? `Finishing “${name}”…`
            : `Loading “${name}”… ${Math.round(pct || 0)}%`;
        if (bar) bar.style.width = `${Math.min(100, pct || 0)}%`;
    }
    function hideLoading() {
        document.getElementById('splat-loading')?.classList.add('hidden');
    }

    async function load(entry) {
        if (loading || loadedSlug === entry.slug) return;
        loading = true;
        showLoading(entry.name, 0);
        hub.emit('log', { line: `[splat] loading "${entry.name}"…` });
        try {
            ensureViewer();
            if (loadedSlug !== null) {
                await viewer.removeSplatScene(0);
                loadedSlug = null;
            }
            const trs = decompose(entry.transform);
            await viewer.addSplatScene(entry.ply_url, {
                ...trs, showLoadingUI: false, progressiveLoad: false,
                splatAlphaRemovalThreshold: 5,
                // Vendor download/parse progress -> the centered overlay, so a
                // multi-hundred-MB fetch never looks like a frozen UI.
                onProgress: (pct) => showLoading(entry.name, pct),
            });
            loadedSlug = entry.slug;
            applySeeThrough();   // a freshly-loaded scene starts opaque; honour the slider
        } catch (e) {
            hub.emit('log', { line: `[splat] load failed: ${e && e.message || e}` });
        } finally {
            loading = false;
            hideLoading();
            apply();   // re-evaluate visibility / framing now the load settled
        }
    }

    // Point the World-mode camera at the splat once, when it first appears
    // (issue #108): human eye level (~6 ft / 1.83 m off the floor), centred
    // on the room's own centroid, backed off by however much THAT room's
    // size needs. The old fixed `(0, -2.5, 7)` pull-back assumed every build
    // shared the ~3-unit-radius normalisation of the one preset it was tuned
    // against -- wrong scale/position for an imported splat or a differently
    // cropped build, and not eye level at all (2.5 m is closer to a ladder).
    //
    // `SplatMesh.computeBoundingBox(true, 0)` walks the loaded scene's own
    // splat centers with its manifest transform already applied -- DropInViewer
    // itself carries no transform (see `ensureViewer`'s bare `scene.add(viewer)`),
    // so the box comes back directly in camera/controls space.
    function frameCamera() {
        if (framedSlug === loadedSlug) return;
        framedSlug = loadedSlug;
        const mesh = viewer && viewer.splatMesh;
        const box = mesh && typeof mesh.computeBoundingBox === 'function'
            ? mesh.computeBoundingBox(true, 0) : null;
        const framed = box && sceneApi.frameCameraToBBox({
            minX: box.min.x, maxX: box.max.x,
            minY: box.min.y, maxY: box.max.y,
            minZ: box.min.z, maxZ: box.max.z,
        });
        if (!framed) {
            // Degenerate box (no splats yet / a single point) or the World
            // view isn't active -- fall back to the old fixed shot so the
            // viewer is never left staring at nothing.
            const cam = sceneApi.camera, controls = sceneApi.controls;
            controls.target.set(0, 0, 0);
            cam.position.set(0, -2.5, 7);
            controls.update();
        }
    }

    function apply() {
        const inSplat = state.source === 'splat';
        const card = document.getElementById('splat-card');
        if (card) card.classList.toggle('hidden', !inSplat);
        if (viewer) viewer.visible = inSplat && loadedSlug !== null;
        gateSplatMode(inSplat);            // grey out controls that don't apply here
        renderBuildStatus();               // checks source itself; hides when not splat
        if (!inSplat) return;
        applySeeThrough();
        seedForm();
        renderSourceVideos();
        const sel = state.selected_splat
            || (items.length ? items[0].slug : null);   // default to newest
        const entry = items.find((e) => e.slug === sel);
        if (entry && entry.slug !== loadedSlug) {
            load(entry);
        } else if (loadedSlug !== null) {
            frameCamera();
        }
        renderPicker();
    }

    function renderPicker() {
        const list = document.getElementById('splat-list');
        if (!list) return;
        const empty = document.getElementById('splat-empty');
        if (empty) empty.style.display = items.length ? 'none' : '';
        const sel = state.selected_splat || (items.length ? items[0].slug : null);
        list.innerHTML = '';
        for (const e of items) {
            const btn = document.createElement('button');
            btn.className = 'splat-item' + (e.slug === sel ? ' active' : '');
            btn.dataset.slug = e.slug;
            const bits = [];
            if (e.gaussians) bits.push(`${(e.gaussians / 1e6).toFixed(2)}M`);
            const p = e.preset || {};
            if (p.depth_lambda) bits.push(`depth λ${p.depth_lambda}`);
            else if (!e.imported && p.iters) bits.push('photometry');
            const badge = e.imported ? '<span class="splat-badge splat-badge--imported">imported</span>' : '';
            btn.innerHTML = `<span class="splat-item__name"></span>` +
                `<span class="splat-item__meta">${bits.join(' · ')} ${badge}</span>`;
            btn.querySelector('.splat-item__name').textContent = e.name;
            btn.addEventListener('click', () => hub.send({ type: 'load_splat', slug: e.slug }));
            list.appendChild(btn);
        }
    }

    // ---- Capture viewer: source videos + their builds --------------------------
    const COARSE_KNOBS = ['fps', 'max_frames', 'long_edge', 'matcher', 'iters',
        'max_gaussians', 'sh_degree', 'depth_lambda'];

    function seedForm() {
        if (seededForm) return;
        for (const el of document.querySelectorAll('#splat-settings [data-param]')) {
            const d = defaults[el.dataset.param];
            if (d !== undefined && d !== null) el.placeholder = String(d);
        }
        seededForm = true;
    }

    function readForm() {
        const params = {};
        for (const el of document.querySelectorAll('#splat-settings [data-param]')) {
            const v = (el.value || '').trim();
            if (v === '') continue;                      // blank -> preset default
            params[el.dataset.param] = el.type === 'number' ? Number(v) : v;
        }
        return params;
    }

    function mb(n) { return n ? `${(n / 1e6).toFixed(0)} MB` : ''; }

    function startBuild(video, suggestedName, force) {
        const name = window.prompt(
            force ? `Regenerate splat name for ${video}:` : `Name this splat (from ${video}):`,
            suggestedName || '');
        if (!name || !name.trim()) return;
        const params = readForm();
        const extra = params.depth_lambda ? ` with depth prior (λ=${params.depth_lambda})` : '';
        if (!window.confirm(`Build "${name.trim()}"${extra}?\n\nCOLMAP + GPU training runs ` +
            `out-of-process and takes many minutes. Only one build runs at a time.`)) return;
        hub.send({ type: 'build_splat', video, name: name.trim(), params, force: !!force });
    }

    function renderSourceVideos() {
        const list = document.getElementById('splat-video-list');
        if (!list) return;
        const empty = document.getElementById('splat-video-empty');
        if (empty) empty.style.display = sourceVideos.length ? 'none' : '';
        list.innerHTML = '';
        for (const v of sourceVideos) {
            const row = document.createElement('div');
            row.className = 'splat-video';
            const builds = (v.splats || []).map((s) => {
                const stale = s.state === 'stale'
                    ? '<span class="splat-badge splat-badge--stale">stale</span>' : '';
                const g = s.gaussians ? `${(s.gaussians / 1e6).toFixed(2)}M` : '';
                const depth = s.depth_lambda ? `depth λ${s.depth_lambda}` : 'photometry';
                return `<div class="splat-video__build"><span data-view="${s.slug}" ` +
                    `class="splat-item__name" style="cursor:pointer">${s.name}</span>` +
                    `<span class="splat-item__meta">${g} · ${depth} ${stale}</span></div>`;
            }).join('');
            row.innerHTML =
                `<div class="splat-video__name">${v.video}</div>` +
                `<div class="splat-video__meta">${mb(v.bytes)}` +
                    `${v.has_splat ? ` · ${v.splats.length} build${v.splats.length > 1 ? 's' : ''}` : ' · no splat yet'}</div>` +
                (builds ? `<div class="splat-video__builds">${builds}</div>` : '') +
                `<div class="splat-video__actions"></div>`;
            const actions = row.querySelector('.splat-video__actions');
            const gen = document.createElement('button');
            gen.className = 'splat-btn splat-btn--primary';
            gen.textContent = v.has_splat ? 'Generate another' : 'Generate';
            gen.addEventListener('click', () => startBuild(v.video, '', false));
            actions.appendChild(gen);
            const play = document.createElement('button');
            play.className = 'splat-btn';
            play.textContent = 'Play video';
            play.addEventListener('click', () => togglePlay(row, v.video));
            actions.appendChild(play);
            // View + Regenerate per existing build.
            for (const s of (v.splats || [])) {
                const rb = document.createElement('button');
                rb.className = 'splat-btn';
                rb.textContent = `Regenerate ${s.name}`;
                rb.addEventListener('click', () => startBuild(v.video, s.name, true));
                actions.appendChild(rb);
            }
            row.querySelectorAll('[data-view]').forEach((el) =>
                el.addEventListener('click', () => hub.send({ type: 'load_splat', slug: el.dataset.view })));
            list.appendChild(row);
        }
    }

    function togglePlay(row, video) {
        const existing = row.querySelector('video');
        if (existing) { existing.remove(); return; }
        const vid = document.createElement('video');
        vid.controls = true;
        vid.style.cssText = 'width:100%;margin-top:6px;border-radius:6px';
        vid.src = `/capture_video/${encodeURIComponent(video)}`;
        row.appendChild(vid);
    }

    // ---- Build banner + resource bars (mirror slam.js's detailed banner) --------
    function fmtTime(s) {
        s = Math.max(0, Math.round(s || 0));
        return `${Math.floor(s / 60)}:${String(s % 60).padStart(2, '0')}`;
    }

    function renderBuildStatus() {
        const card = document.getElementById('splat-build-status');
        if (!card) return;
        const show = build !== null && state.source === 'splat';
        card.classList.toggle('hidden', !show);
        if (!show) return;
        const frac = build.done ? 1 : (build.fraction || 0);
        const bar = document.getElementById('splat-build-bar');
        if (bar) { bar.style.width = `${(frac * 100).toFixed(0)}%`; bar.setAttribute('aria-valuenow', (frac * 100).toFixed(0)); }
        const title = document.getElementById('splat-build-title');
        if (title) title.textContent = build.done
            ? (build.error ? 'Splat build failed' : `Built ${build.name || ''}`)
            : `Building ${build.name || 'splat'}`;
        const prog = document.getElementById('splat-build-progress');
        if (prog) prog.textContent = build.done ? (build.error ? 'failed' : 'done')
            : `${(frac * 100).toFixed(0)}% · ${build.phase || ''}`;
        const time = document.getElementById('splat-build-time');
        if (time) time.textContent = build.done
            ? `Completed in ${fmtTime(build.elapsed_s)}`
            : `Elapsed ${fmtTime(build.elapsed_s)}` +
              (build.eta_s != null ? ` · ~${fmtTime(build.eta_s)} left` : ' · calculating ETA');
        const detail = document.getElementById('splat-build-detail');
        if (detail) {
            if (build.error) detail.textContent = build.error;
            else if (build.done && build.stats) {
                const st = build.stats;
                detail.textContent = `${(st.gaussians / 1e6).toFixed(2)}M gaussians` +
                    (st.peak_vram_gib ? ` · peak VRAM ${st.peak_vram_gib} GiB` : '');
            } else detail.textContent = 'COLMAP + GPU training runs out-of-process; one build at a time.';
        }
        renderResourceBars();
    }

    function setBar(key, frac, label) {
        const fill = document.getElementById(`splat-resource-${key}`);
        const val = document.getElementById(`splat-resource-${key}-value`);
        if (fill) {
            fill.style.width = `${Math.min(100, Math.max(0, frac * 100)).toFixed(0)}%`;
            fill.classList.toggle('is-warn', frac >= 0.8 && frac < 0.9);
            fill.classList.toggle('is-crit', frac >= 0.9);
        }
        if (val) val.textContent = label;
    }

    function renderResourceBars() {
        const r = resources;
        if (!r) return;
        const cpu = (r.sys_cpu_percent != null ? r.sys_cpu_percent : r.proc_cpu_percent) || 0;
        setBar('cpu', cpu / 100, `${cpu.toFixed(0)}%`);
        if (r.ram_total) setBar('ram', r.ram_used / r.ram_total, `${(r.ram_used / 2 ** 30).toFixed(1)}G`);
        if (r.gpu_util != null) setBar('gpu', r.gpu_util / 100, `${r.gpu_util.toFixed(0)}%`);
        const vu = r.device_vram_used, vt = r.device_vram_total;
        if (vt) setBar('vram', vu / vt, `${(vu / 2 ** 30).toFixed(1)}G`);
    }

    hub.on('splats', (msg) => { items = (msg && msg.items) || []; apply(); });
    hub.on('source_videos', (msg) => {
        sourceVideos = (msg && msg.items) || [];
        if (msg && msg.defaults) { defaults = msg.defaults; seededForm = false; seedForm(); }
        renderSourceVideos();
    });
    hub.on('splat_build', (msg) => {
        if (!msg) return;
        build = msg.done ? { ...msg } : { ...(build || {}), ...msg };
        if (doneTimer) { clearTimeout(doneTimer); doneTimer = null; }
        // Auto-clear a finished banner after a beat so it doesn't linger forever.
        if (build.done) doneTimer = setTimeout(() => { build = null; renderBuildStatus(); }, 12000);
        renderBuildStatus();
    });
    hub.on('metrics', (m) => { resources = m && m.resources; if (build) renderResourceBars(); });
    hub.on('state', (s) => {
        const entering = s.source === 'splat' && state.source !== 'splat';
        state = { source: s.source, selected_splat: s.selected_splat };
        if (s.see_through !== undefined) seeThrough = s.see_through;
        if (entering) hub.send({ type: 'list_source_videos' });   // populate the viewer
        apply();
    });

    return {
        get loadedSlug() { return loadedSlug; },
        // diagnostics only (see scene.js's `controls` comment): read the live splat
        // scene opacity to verify See-Through end-to-end.
        get sceneOpacity() { try { return viewer && viewer.getSplatScene(0).opacity; } catch (e) { return null; } },
    };
}
