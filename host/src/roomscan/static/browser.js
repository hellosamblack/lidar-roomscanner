// browser.js — the View page's capture file browser (§12, owner ask 2026-07-31).
//
// The View source used to be a cramped `<div class="cap-list">` inside the right
// rail's Capture card: a name and a byte count per row, no way to see what a
// capture actually was, and no way to rename or delete one. This is the real
// library — thumbnails, metadata, preview, rename, multi-select delete.
//
// Shape, like every other module: DOM events become `hub.send(...)`, and ALL
// server-owned state is rendered FROM the `captures` / `session` / `state` echo
// (one-way flow, §5). Two things are deliberately CLIENT-LOCAL, because they are
// per-tab presentation rather than server state — the same reasoning as card
// collapse:
//
//   * which tile is previewed;
//   * which checkboxes are ticked.
//
// Sorting IS server-persisted (so the library comes back the way you left it)
// but is APPLIED here, off the already-broadcast `captures` array — changing the
// sort must not cost a directory rescan.
//
// Thumbnails arrive over HTTP (`GET /thumb/<name>?v=<mtime>`), not as a binary
// /ws tag: `<img loading="lazy">` buys viewport-paced fetching, browser disk
// caching, per-image cancellation and parallelism for free, and keeps ~34 PNGs
// from interleaving with the 30 Hz POINT_CLOUD. See web.py's `get_thumb`.
//
// Hub events:  subscribes "captures", "session", "state", "deleted";
//              sends list_captures / load_capture / set_browser /
//              rename_capture / delete_captures / generate_detailed /
//              regenerate_detailed.

export function createBrowser(hub, capture) {
    const $ = (id) => document.getElementById(id);

    const browserCard = $('browser-card');
    const previewCard = $('preview-card');
    const grid = $('cap-grid');
    const status = $('browser-status');
    const segSort = $('seg-browser-sort');
    const segView = $('seg-browser-view');
    const chkThumbs = $('chk-browser-thumbs');
    const btnRefresh = $('btn-browser-refresh');
    const btnDelete = $('btn-browser-delete');

    const previewThumb = $('preview-thumb');
    const previewName = $('preview-name');
    const previewFacts = $('preview-facts');
    const btnLoad = $('btn-preview-load');
    const btnRename = $('btn-preview-rename');
    const btnBuild = $('btn-preview-build');
    const previewView = $('capture-preview-view');
    const previewViewImage = $('capture-preview-image');
    const previewViewCaption = $('capture-preview-caption');
    const buildModal = $('build-modal');
    const buildIntro = $('build-intro');
    const buildFacts = $('build-facts');
    const buildNote = $('build-note');
    const btnBuildConfirm = $('build-confirm');
    const btnBuildCancel = $('build-cancel');
    const btnBuildClose = $('build-close');

    const delModal = $('delete-modal');
    const delIntro = $('delete-intro');
    const delList = $('delete-list');
    const delSidecars = $('chk-delete-sidecars');
    const btnDelConfirm = $('btn-delete-confirm');
    const btnDelCancel = $('btn-delete-cancel');
    const btnDelClose = $('delete-close');

    let captures = [];                 // latest server `captures` array
    let prefs = { sort: 'recent', view: 'grid', thumbs: true };
    let source = 'live';
    let display = 'point_cloud';
    let viewMode = 'world';
    let playing = null;                // session.playback.capture_name
    let selected = new Set();          // ticked names — CLIENT-LOCAL
    let previewed = null;              // previewed name — CLIENT-LOCAL
    // What the last `delete_captures` ACTUALLY did. Held across renders because
    // the server sends `deleted` and then immediately `captures`, whose render
    // would otherwise wipe the only report of a refusal off the screen a frame
    // after it appeared. Cleared by the next thing the user does.
    let deleteNote = null;
    let buildPending = null;             // { name, force }; waits for load echo

    // ---- formatting -------------------------------------------------------
    const fmtBytes = (n) => !n ? '0 B' : n < 1024 ? n + ' B'
        : n < 1048576 ? (n / 1024).toFixed(1) + ' KB'
            : n < 1073741824 ? (n / 1048576).toFixed(1) + ' MB'
                : (n / 1073741824).toFixed(2) + ' GB';
    const fmtTime = (s) => {
        s = Math.max(0, Math.floor(s || 0));
        return Math.floor(s / 60) + ':' + String(s % 60).padStart(2, '0');
    };
    const escapeHtml = (s) => String(s).replace(/[&<>"']/g, (c) =>
        ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
    // Where a reconstruction does not exist there is no distance and no area to
    // report — an em dash, never a 0, which would read as "it covered nothing".
    const num = (v, unit, digits) =>
        (typeof v === 'number' && isFinite(v)) ? v.toFixed(digits) + unit : '—';

    // The thumbnail is NOT a map. With no translation estimate every sampled
    // frame is deprojected about one origin, so it is a rotational sweep of
    // where the scanner was aimed. Saying so on the tile is the whole reason
    // this string exists in one place.
    const THUMB_NOTE = 'Orientation-only sweep of ~40 sampled frames — where the ' +
        'scanner was aimed, not a floor plan (there is no position estimate here; ' +
        'build a reconstruction for real geometry).';

    function thumbUrl(c) {
        return '/thumb/' + encodeURIComponent(c.name) + '?v=' + encodeURIComponent(c.mtime);
    }

    // ---- sorting (client-side, off the broadcast array) -------------------
    function sorted() {
        const items = captures.slice();
        if (prefs.sort === 'name') items.sort((a, b) => a.name.localeCompare(b.name));
        else if (prefs.sort === 'size') items.sort((a, b) => (b.bytes || 0) - (a.bytes || 0));
        else if (prefs.sort === 'duration') items.sort((a, b) => (b.duration_s || 0) - (a.duration_s || 0));
        else items.sort((a, b) => (b.mtime || 0) - (a.mtime || 0));
        return items;
    }

    // ---- render ------------------------------------------------------------
    function render() {
        if (!grid) return;
        grid.classList.toggle('is-list', prefs.view === 'list');
        const items = sorted();
        if (!items.length) {
            grid.innerHTML = '<div class="cap-status">no captures yet</div>';
        } else {
            grid.innerHTML = items.map((c) => {
                const cls = ['cap-tile'];
                if (c.name === previewed) cls.push('is-selected');
                if (c.name === playing) cls.push('is-playing');
                const img = prefs.thumbs
                    ? `<img class="cap-tile__thumb" loading="lazy" alt="" src="${thumbUrl(c)}">`
                    : '';
                const badge = c.slam ? '<span class="cap-tile__badge">3D</span>'
                    : (c.has_stream_9 ? '' : '<span class="cap-tile__badge">2D</span>');
                const tip = `${c.name}\n${fmtTime(c.duration_s)} · ${c.frames || 0} frames · ` +
                    `${fmtBytes(c.bytes)}` + (c.has_stream_9 ? '' : '\nNo orientation stream — point cloud only') +
                    (prefs.thumbs ? '\n\n' + THUMB_NOTE : '');
                return `<div class="${cls.join(' ')}" data-name="${escapeHtml(c.name)}" title="${escapeHtml(tip)}">` +
                    `<input class="cap-tile__check" type="checkbox" data-check="${escapeHtml(c.name)}"` +
                    `${selected.has(c.name) ? ' checked' : ''} title="Select for delete">` +
                    badge + img +
                    `<span class="cap-tile__name">${escapeHtml(c.name)}</span>` +
                    `<span class="cap-tile__meta">${fmtTime(c.duration_s)} · ${fmtBytes(c.bytes)}</span>` +
                    `</div>`;
            }).join('');
        }
        renderStatus();
        // A `.bin` the renderer can't read (a firmware image parked in
        // captures/, a truncated take) 404s its thumb. `/thumb` never 500s, so
        // the only symptom is a broken-image glyph -- and the page's global
        // error hook logs each one into the Diagnostics panel, which is noise,
        // not a fault. Collapse those to the placeholder instead.
        for (const img of grid.querySelectorAll('.cap-tile__thumb')) {
            img.addEventListener('error', (e) => {
                e.stopPropagation();
                img.classList.add('is-missing');
            }, { once: true });
        }
        if (btnDelete) btnDelete.disabled = selected.size === 0;
        setActive(segSort, 'sort', prefs.sort);
        setActive(segView, 'view', prefs.view);
        if (chkThumbs) chkThumbs.checked = !!prefs.thumbs;
        renderPreview();
        window.__relayout && window.__relayout();
    }

    function renderStatus() {
        if (!status) return;
        const total = captures.reduce((a, c) => a + (c.bytes || 0), 0);
        const base = `${captures.length} captures · ${fmtBytes(total)}` +
            (selected.size ? ` · ${selected.size} selected` : '');
        status.textContent = deleteNote ? base + ' — ' + deleteNote : base;
    }

    // These two cards are the capture PICKER, and they sit over the middle of
    // the viewport -- which is exactly where a map is drawn. Measured on the
    // rig in View + SLAM: 97% of the map's screen footprint was covered by
    // #browser-card + #preview-card, leaving only the 14 px gutter between
    // them, so a perfectly good reconstruction read as "nothing rendered".
    // Live mode never had this because both cards hide when source != 'view'.
    //
    // Collapse (rail-recoverable in one click), never `.hidden`: you still have
    // to be able to pick a different capture without leaving the map display.
    // Fires ONLY on the transition into a map display -- `state` is re-broadcast
    // on every unrelated setting change, so re-collapsing on each echo would
    // fight the user every time they re-opened a card (the state-echo trap that
    // killed the oscillate orbit's return leg).
    const MAP_DISPLAYS = new Set(['slam', 'detailed']);
    function collapseForMapDisplay(prevSource, prevDisplay) {
        const now = source === 'view' && MAP_DISPLAYS.has(display);
        const before = prevSource === 'view' && MAP_DISPLAYS.has(prevDisplay);
        if (!now || before) return;
        for (const [card, id] of [[browserCard, 'browser'], [previewCard, 'preview']]) {
            if (!card || card.classList.contains('collapsed')) continue;
            card.classList.add('collapsed');
            try { localStorage.setItem(`roomscan.card.${id}.collapsed`, '1'); } catch (err) {}
        }
        window.__relayout?.();
    }

    function renderPreview() {
        const c = captures.find((x) => x.name === previewed) || null;
        previewCard?.classList.toggle('hidden', source !== 'view' || !c);
        if (!c) {
            previewView?.classList.add('hidden');
            return;
        }
        if (previewThumb) {
            previewThumb.src = thumbUrl(c);
            previewThumb.title = THUMB_NOTE;
        }
        if (previewName) previewName.textContent = c.name;
        if (previewFacts) {
            const s = c.slam;
            const rows = [
                ['Duration', fmtTime(c.duration_s), 'Wall-clock length from the capture\'s own TIM2 frame timestamps'],
                ['Frames', String(c.frames || 0), 'Depth frames in the capture'],
                ['Size', fmtBytes(c.bytes), 'File size on disk'],
                ['Orientation', c.has_stream_9 ? 'yes' : 'no', 'Stream 9 (SFLP quaternion). Without it a capture is point-cloud-playable only — no SLAM.'],
                ['Distance', s ? num(s.path_m, ' m', 2) : '—', 'Path length of the reconstruction\'s trajectory. Only exists where a reconstruction does.'],
                ['Area', s ? num(s.area_m2, ' m²', 1) : '—', 'Floor area covered: the reconstruction\'s vertices projected onto the ground plane and counted on a 0.1 m grid. Not mesh surface area.'],
                ['Closure', s ? num(s.gap_m, ' m', 2) : '—', 'Start-to-end gap of the trajectory — the drift over a closed loop.'],
                ['Reconstruction', s ? (s.current ? 'current' : 'stale') : 'none',
                    'Whether results/<name>.slam.json matches this capture and the current preset.'],
            ];
            previewFacts.innerHTML = rows.map(([k, v, tip]) =>
                `<dt title="${escapeHtml(tip)}">${escapeHtml(k)}</dt>` +
                `<dd title="${escapeHtml(tip)}">${escapeHtml(v)}</dd>`).join('');
        }
        if (btnBuild) {
            const s = c.slam;
            btnBuild.textContent = s ? (s.current ? '⚙ Rebuild' : '⚙ Rebuild (stale)') : '⚙ Build';
        }
        const showInViewport = source === 'view' && display === 'preview';
        previewView?.classList.toggle('hidden', !showInViewport);
        if (showInViewport) {
            if (previewViewImage) previewViewImage.src = thumbUrl(c);
            // Preview is a saved 2D thumbnail, so World and FPV have the same
            // pixels. Mirror remains meaningful: it is the same left-right
            // camera flip used by the 3D map view, scoped to the image only.
            if (previewViewImage) previewViewImage.style.transform =
                viewMode === 'mirror' ? 'scaleX(-1)' : '';
            if (previewViewCaption) previewViewCaption.textContent =
                `${c.name} · ${fmtTime(c.duration_s)} · ${c.frames || 0} frames`;
        }
    }

    function setActive(seg, attr, value) {
        if (!seg) return;
        for (const b of seg.querySelectorAll('button')) {
            b.classList.toggle('active', b.dataset[attr] === value);
        }
    }

    // ---- outbound ----------------------------------------------------------
    grid?.addEventListener('click', (e) => {
        const box = e.target.closest('input[data-check]');
        if (box) {
            // Toggling a checkbox must not also open the preview.
            e.stopPropagation();
            const n = box.dataset.check;
            if (box.checked) selected.add(n); else selected.delete(n);
            deleteNote = null;
            renderStatus();
            if (btnDelete) btnDelete.disabled = selected.size === 0;
            return;
        }
        const tile = e.target.closest('.cap-tile[data-name]');
        if (!tile) return;
        previewed = tile.dataset.name;
        deleteNote = null;
        render();
    });

    segSort?.addEventListener('click', (e) => {
        const b = e.target.closest('button[data-sort]');
        if (b) hub.send({ type: 'set_browser', sort: b.dataset.sort });
    });
    segView?.addEventListener('click', (e) => {
        const b = e.target.closest('button[data-view]');
        if (b) hub.send({ type: 'set_browser', view: b.dataset.view });
    });
    chkThumbs?.addEventListener('change', () =>
        hub.send({ type: 'set_browser', thumbs: chkThumbs.checked }));
    btnRefresh?.addEventListener('click', () => hub.send({ type: 'list_captures' }));

    btnLoad?.addEventListener('click', () => {
        if (previewed) hub.send({ type: 'load_capture', name: previewed });
    });
    btnRename?.addEventListener('click', () => {
        // Reuse capture.js's dialog rather than growing a second one; `fromBrowser`
        // is what makes it send the explicit `from` field.
        if (previewed && capture && capture.openRename) {
            capture.openRename(previewed, { fromBrowser: true });
        }
    });
    btnBuild?.addEventListener('click', openBuildModal);
    btnBuildCancel?.addEventListener('click', closeBuildModal);
    btnBuildClose?.addEventListener('click', closeBuildModal);
    buildModal?.addEventListener('click', (e) => { if (e.target === buildModal) closeBuildModal(); });
    document.addEventListener('keydown', (e) => {
        if (e.key === 'Escape' && buildModal && !buildModal.classList.contains('hidden')) closeBuildModal();
    });
    btnBuildConfirm?.addEventListener('click', () => {
        if (!buildPending) return;
        const pending = buildPending;
        closeBuildModal(false);
        if (pending.name === playing) startBuild(pending);
        else hub.send({ type: 'load_capture', name: pending.name });
    });

    function openBuildModal() {
        const c = captures.find((x) => x.name === previewed);
        if (!c) return;
        const force = !!(c.slam && c.slam.exists);
        const est = c.detailed_estimate || {};
        buildPending = { name: c.name, force };
        if (buildIntro) buildIntro.textContent =
            `${force ? 'Rebuild' : 'Build'} the offline Detailed reconstruction for ${c.name}.`;
        if (buildFacts) {
            const time = est.calibrated ? `~${fmtTime(est.seconds)} (${est.seconds.toFixed(1)} s)` : 'Not benchmarked yet';
            buildFacts.innerHTML = `<dt>Frames</dt><dd>${c.frames || 0}</dd>` +
                `<dt>Estimated time</dt><dd>${time}</dd>` +
                `<dt>Compute</dt><dd>${est.cpu_warning ? 'CPU (slower)' : 'CUDA'}</dd>`;
        }
        if (buildNote) buildNote.textContent = est.calibrated
            ? 'The build runs offline and never changes the capture.'
            : `The active preset has no timing calibration. ${est.note || 'The first build establishes a baseline.'} The build runs offline and never changes the capture.`;
        if (btnBuildConfirm) btnBuildConfirm.textContent = force ? 'Rebuild' : 'Build';
        buildModal?.classList.remove('hidden');
    }

    function closeBuildModal(clear = true) {
        buildModal?.classList.add('hidden');
        if (clear) buildPending = null;
    }

    function startBuild(pending) {
        buildPending = null;
        hub.send({ type: pending.force ? 'regenerate_detailed' : 'generate_detailed' });
        hub.send({ type: 'set_display', display: 'detailed' });
    }

    // ---- delete: confirm modal --------------------------------------------
    btnDelete?.addEventListener('click', openDeleteModal);
    btnDelCancel?.addEventListener('click', closeDeleteModal);
    btnDelClose?.addEventListener('click', closeDeleteModal);
    delModal?.addEventListener('click', (e) => { if (e.target === delModal) closeDeleteModal(); });
    document.addEventListener('keydown', (e) => {
        if (e.key === 'Escape' && delModal && !delModal.classList.contains('hidden')) closeDeleteModal();
    });

    function openDeleteModal() {
        const names = [...selected];
        if (!names.length) return;
        const chosen = captures.filter((c) => selected.has(c.name));
        const bytes = chosen.reduce((a, c) => a + (c.bytes || 0), 0);
        if (delIntro) {
            delIntro.textContent =
                `Permanently delete ${names.length} capture${names.length === 1 ? '' : 's'} ` +
                `(${fmtBytes(bytes)})? This cannot be undone.`;
        }
        if (delList) {
            delList.innerHTML = chosen.map((c) =>
                `<li>${escapeHtml(c.name)} <span style="opacity:.6">${fmtBytes(c.bytes)}` +
                `${c.slam ? ' + reconstruction' : ''}</span></li>`).join('');
        }
        delModal?.classList.remove('hidden');
    }

    function closeDeleteModal() { delModal?.classList.add('hidden'); }

    btnDelConfirm?.addEventListener('click', () => {
        const names = [...selected];
        if (!names.length) { closeDeleteModal(); return; }
        hub.send({ type: 'delete_captures', names,
                   sidecars: delSidecars ? !!delSidecars.checked : true });
        closeDeleteModal();
    });

    // ---- inbound -----------------------------------------------------------
    hub.on('captures', (msg) => {
        captures = Array.isArray(msg.items) ? msg.items : [];
        // Selection clears on the next `captures` echo, never on click: the echo
        // is the server telling us what the library IS now, so anything ticked
        // that no longer exists (or was just deleted) is gone with it.
        const live = new Set(captures.map((c) => c.name));
        selected = new Set([...selected].filter((n) => live.has(n)));
        if (previewed && !live.has(previewed)) previewed = null;
        if (!previewed && playing && live.has(playing)) previewed = playing;
        render();
    });

    hub.on('session', (msg) => {
        const next = msg?.playback?.is_replay ? msg.playback.capture_name : null;
        if (next === playing) return;
        playing = next;
        if (!previewed && playing) previewed = playing;
        render();
        if (buildPending && playing === buildPending.name) startBuild(buildPending);
    });

    hub.on('state', (msg) => {
        const prevDisplay = display;
        const prevSource = source;
        source = msg.source || 'live';
        display = msg.display || 'point_cloud';
        viewMode = msg.view_mode || 'world';
        collapseForMapDisplay(prevSource, prevDisplay);
        if (msg.browser_sort) prefs.sort = msg.browser_sort;
        if (msg.browser_view) prefs.view = msg.browser_view;
        if (typeof msg.browser_thumbs === 'boolean') prefs.thumbs = msg.browser_thumbs;
        if (msg.selected_capture) previewed = msg.selected_capture;
        browserCard?.classList.toggle('hidden', source !== 'view');
        render();
    });

    // What was ACTUALLY deleted, not what was requested — every refusal carries
    // a reason, and a batch that silently skipped a file would be worse than one
    // that says so.
    hub.on('deleted', (msg) => {
        const n = (msg.deleted || []).length;
        const parts = [];
        if (n) parts.push(`deleted ${n} capture${n === 1 ? '' : 's'} (${fmtBytes(msg.bytes || 0)})`);
        for (const r of msg.refused || []) parts.push(`kept ${r.name}: ${r.reason}`);
        deleteNote = parts.length ? parts.join(' · ') : null;
        renderStatus();
    });

    render();
    return {};
}
