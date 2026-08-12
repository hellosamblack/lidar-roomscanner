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

export function createBrowser(hub, capture, scene) {
    const $ = (id) => document.getElementById(id);

    const browserCard = $('browser-card');
    // The capture-detail drawer now lives INLINE under the grid (`#browser-selected-
    // detail`), not in a separate `#preview-card`. Same fields (`#preview-name` /
    // `#preview-facts` / the load/rename/build buttons), so only this handle moved.
    const previewCard = $('browser-selected-detail');
    const grid = $('cap-grid');
    const status = $('browser-status');
    const filterInput = $('browser-filter');
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
    // #103: whether `previewed` is the user's OWN pick rather than a default the
    // client adopted for them. The drawer's buttons act on `previewed`, so a
    // vanished explicit pick must clear the drawer, never silently retarget it at
    // some other capture — pressing Build on the wrong file is worse than an
    // empty drawer. Auto-adopted defaults stay free to be replaced.
    let previewExplicit = false;
    // The name a browser-initiated rename asked for, until a `captures` echo
    // either shows it (rename went through -> follow the selection to it) or
    // shows the old name still there (server refused -> keep the old selection).
    let pendingRenameTo = null;
    // Name filter is a transient per-tab search, not server state (like the
    // ticked set): it narrows what is rendered off the already-broadcast array
    // and never triggers a directory rescan.
    let filterText = '';
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

    // ---- filter + sort (client-side, off the broadcast array) -------------
    function sorted() {
        let items = captures.slice();
        if (filterText) {
            const q = filterText.toLowerCase();
            items = items.filter((c) => c.name.toLowerCase().includes(q));
        }
        if (prefs.sort === 'name') items.sort((a, b) => a.name.localeCompare(b.name));
        else if (prefs.sort === 'size') items.sort((a, b) => (b.bytes || 0) - (a.bytes || 0));
        else if (prefs.sort === 'duration') items.sort((a, b) => (b.duration_s || 0) - (a.duration_s || 0));
        else items.sort((a, b) => (b.mtime || 0) - (a.mtime || 0));
        return items;
    }

    // Date grouping only makes sense chronologically, so it rides the default
    // "Recent" sort; the other orders (name/size/length) render one flat run.
    // `mtime` is epoch SECONDS (web.py rounds st_mtime), so ×1000 for Date.
    const startOfDay = (ms) => { const d = new Date(ms); d.setHours(0, 0, 0, 0); return d.getTime(); };
    function dateLabel(mtimeSec) {
        const ms = (mtimeSec || 0) * 1000;
        const days = Math.round((startOfDay(Date.now()) - startOfDay(ms)) / 86400000);
        if (days <= 0) return 'Today';
        if (days === 1) return 'Yesterday';
        if (days < 7) return new Date(ms).toLocaleDateString(undefined, { weekday: 'long' });
        return new Date(ms).toLocaleDateString(undefined, { year: 'numeric', month: 'short', day: 'numeric' });
    }

    function tileHtml(c) {
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
        // `aria-selected` mirrors `is-selected` so the selected tile is not
        // communicated by border colour alone. The tile keeps its implicit role:
        // it contains a checkbox, and `role="option"` may not contain
        // interactive content, so promoting it to a listbox option would trade a
        // styling-only gap for invalid ARIA.
        return `<div class="${cls.join(' ')}" data-name="${escapeHtml(c.name)}"` +
            ` aria-selected="${c.name === previewed ? 'true' : 'false'}"` +
            ` title="${escapeHtml(tip)}">` +
            `<input class="cap-tile__check" type="checkbox" data-check="${escapeHtml(c.name)}"` +
            `${selected.has(c.name) ? ' checked' : ''} title="Select for delete">` +
            badge + img +
            `<span class="cap-tile__name">${escapeHtml(c.name)}</span>` +
            `<span class="cap-tile__meta">${fmtTime(c.duration_s)} · ${fmtBytes(c.bytes)}</span>` +
            `</div>`;
    }

    // ---- render ------------------------------------------------------------
    function render() {
        if (!grid) return;
        grid.classList.toggle('is-list', prefs.view === 'list');
        const items = sorted();
        if (!items.length) {
            grid.innerHTML = `<div class="cap-status">${filterText ? 'no captures match the filter' : 'no captures yet'}</div>`;
        } else if (prefs.sort === 'recent') {
            // Date-grouped: a full-width header per day, then that day's tiles.
            let html = '';
            let curLabel = null;
            for (const c of items) {
                const label = dateLabel(c.mtime);
                if (label !== curLabel) {
                    curLabel = label;
                    html += `<div class="cap-group-header">${escapeHtml(label)}</div>`;
                }
                html += tileHtml(c);
            }
            grid.innerHTML = html;
        } else {
            grid.innerHTML = items.map(tileHtml).join('');
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
        const shown = filterText ? sorted().length : captures.length;
        const count = filterText
            ? `${shown} of ${captures.length} captures`
            : `${captures.length} captures`;
        const base = `${count} · ${fmtBytes(total)}` +
            (selected.size ? ` · ${selected.size} selected` : '');
        status.textContent = deleteNote ? base + ' — ' + deleteNote : base;
    }

    // These two cards are the capture PICKER, and they sit over the middle of
    // the viewport -- which is exactly where a map is drawn. Measured on the
    const MAP_DISPLAYS = new Set(['slam', 'detailed']);
    function collapseForMapDisplay(prevSource, prevDisplay) {
        const now = source === 'view' && MAP_DISPLAYS.has(display);
        const before = prevSource === 'view' && MAP_DISPLAYS.has(prevDisplay);
        if (!now || before) return;
        if (browserCard && !browserCard.classList.contains('collapsed')) {
            browserCard.classList.add('collapsed');
            try { localStorage.setItem('roomscan.card.browser.collapsed', '1'); } catch (err) {}
        }
        window.__relayout?.();
    }

    function renderPreview() {
        const c = captures.find((x) => x.name === previewed) || null;
        if (!c || source !== 'view') {
            previewCard?.classList.add('hidden');
            previewView?.classList.add('hidden');
            const sceneHandle = scene || window.__scene;
            sceneHandle?.setPreviewVisible?.(false);
            return;
        }

        previewCard?.classList.remove('hidden');
        if (previewName) previewName.textContent = c.name;
        if (previewFacts) {
            const s = c.slam;
            const rows = [
                ['Duration', fmtTime(c.duration_s), 'Wall-clock length from timestamps'],
                ['Frames', String(c.frames || 0), 'Depth frames in the capture'],
                ['Size', fmtBytes(c.bytes), 'File size on disk'],
                ['Orientation', c.has_stream_9 ? 'yes' : 'no', 'Stream 9 quaternion'],
                ['Distance', s ? num(s.path_m, ' m', 2) : '—', 'Trajectory length'],
                ['Area', s ? num(s.area_m2, ' m²', 1) : '—', 'Floor area covered'],
                ['Closure', s ? num(s.gap_m, ' m', 2) : '—', 'Loop closure gap'],
                ['Reconstruction', s ? (s.current ? 'current' : 'stale') : 'none', 'Detailed status'],
            ];
            previewFacts.innerHTML = rows.map(([k, v, tip]) =>
                `<dt title="${escapeHtml(tip)}">${escapeHtml(k)}</dt>` +
                `<dd title="${escapeHtml(tip)}">${escapeHtml(v)}</dd>`).join('');
        }
        if (btnBuild) {
            const s = c.slam;
            btnBuild.textContent = s ? (s.current ? '⚙ Rebuild' : '⚙ Rebuild (stale)') : '⚙ Build';
        }

        const showIn3D = source === 'view' && display === 'preview';
        const sceneHandle = scene || window.__scene;
        if (sceneHandle && sceneHandle.setPreviewVisible) {
            sceneHandle.setPreviewVisible(showIn3D);
            if (showIn3D && sceneHandle.setPreviewTexture) {
                sceneHandle.setPreviewTexture(thumbUrl(c));
            }
        }
        
        previewView?.classList.toggle('hidden', !showIn3D);
        if (showIn3D) {
            if (previewViewImage) previewViewImage.src = thumbUrl(c);
            if (previewViewImage) previewViewImage.style.transform =
                viewMode === 'mirror' ? 'scaleX(-1)' : '';
            if (previewViewCaption) previewViewCaption.textContent =
                `${c.name} · ${fmtTime(c.duration_s)} · ${c.frames || 0} frames`;
        }
    }

    // #103: selecting a capture must SURFACE its actions, not merely un-hide a
    // node that may be collapsed or scrolled out of the card. Called only from
    // the explicit tile click — never from a `state`/`captures` echo, so it can
    // never fight `collapseForMapDisplay()`, which is what auto-collapses this
    // card when the user switches to a map display.
    function surfaceSelectedDetail() {
        if (!previewCard || previewCard.classList.contains('hidden')) return;
        // An action pane inside a collapsed card is not reachable at all.
        if (browserCard?.classList.contains('collapsed')) {
            browserCard.classList.remove('collapsed');
            try { localStorage.setItem('roomscan.card.browser.collapsed', '0'); } catch (err) {}
            window.__relayout?.();
        }
        // `block: 'nearest'` scrolls the minimum needed. The drawer is a SIBLING
        // of #cap-grid, not a descendant, so this cannot disturb the capture
        // grid's own scroll position.
        previewCard.scrollIntoView({ block: 'nearest', inline: 'nearest' });
        // When the card is squeezed to its 260px floor (every other dock card
        // open, or a short viewport) the drawer is TALLER than the body's
        // scrollport, so aligning its top pushes the buttons back out of sight.
        // The issue is about reaching Load/Build, so the actions win over the
        // facts table. The Rename/Build row is the drawer's lowest element, so
        // it is the one that gets cut off, and bringing it into view carries the
        // Load button just above it along with it. Only scroll again if it is
        // actually still cut off, so the roomy case keeps the whole drawer.
        const actions = previewCard.querySelector('.btn-row');
        const scroller = previewCard.parentElement;
        if (!actions || !scroller) return;
        const a = actions.getBoundingClientRect();
        const s = scroller.getBoundingClientRect();
        if (a.bottom > s.bottom + 0.5 || a.top < s.top - 0.5) {
            actions.scrollIntoView({ block: 'nearest', inline: 'nearest' });
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
        previewExplicit = true;
        deleteNote = null;
        render();
        surfaceSelectedDetail();
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
    // Filter is client-local — re-render off the already-broadcast array, no
    // server round-trip. The input lives in the toolbar (outside `#cap-grid`),
    // so render()'s innerHTML rewrite never steals its focus mid-type.
    filterInput?.addEventListener('input', () => {
        filterText = filterInput.value.trim();
        deleteNote = null;
        render();
    });
    btnRefresh?.addEventListener('click', () => hub.send({ type: 'list_captures' }));

    btnLoad?.addEventListener('click', () => {
        if (previewed) hub.send({ type: 'load_capture', name: previewed });
    });
    btnRename?.addEventListener('click', () => {
        // Reuse capture.js's dialog rather than growing a second one; `fromBrowser`
        // is what makes it send the explicit `from` field.
        if (previewed && capture && capture.openRename) {
            capture.openRename(previewed, {
                fromBrowser: true,
                // Follow the selection to the new name once the library echoes it.
                onRenamed: (newName) => { pendingRenameTo = newName; },
            });
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
        // A rename REPLACES the name in the library, so the drawer follows the
        // same file to its new name instead of treating it as having vanished.
        if (pendingRenameTo) {
            if (live.has(pendingRenameTo)) {
                if (previewed && !live.has(previewed)) previewed = pendingRenameTo;
                pendingRenameTo = null;
            } else if (previewed && live.has(previewed)) {
                pendingRenameTo = null;      // refused; the old name is still there
            }
        }
        if (previewed && !live.has(previewed)) previewed = null;
        // Adopt the playing capture only as an INITIAL default. Once the user has
        // picked a capture, a delete that removes it clears the drawer rather than
        // pointing Load/Build at whatever happens to be playing.
        if (!previewed && !previewExplicit && playing && live.has(playing)) previewed = playing;
        render();
    });

    hub.on('session', (msg) => {
        const next = msg?.playback?.is_replay ? msg.playback.capture_name : null;
        if (next === playing) return;
        playing = next;
        if (!previewed && !previewExplicit && playing) previewed = playing;
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
        // `selected_capture` is the server's LOADED capture (web.py sets it in
        // `load_capture` and from `--replay` at startup), NOT the tile the user
        // highlighted. `state` re-broadcasts on every settings change, so echoing
        // it into `previewed` unconditionally clobbered this module's documented
        // CLIENT-LOCAL selection: the drawer snapped back to the loaded capture
        // ~3 ms after any rename, and Load/Build then addressed the wrong file.
        // Seed from it only while the user has no pick of their own.
        if (msg.selected_capture && !previewExplicit) previewed = msg.selected_capture;
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
