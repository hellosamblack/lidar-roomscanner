// capture.js — the Capture & Playback control group (Web Phase 3).
//
// Record a live session, browse the server's capture library, load one for
// replay at runtime, and drive playback transport (pause/resume, speed, loop,
// seek). Like controls.js, this turns DOM events into hub.send(...) and drives
// ALL active/enabled/visible state FROM the server's `session` / `captures`
// echo — one-way flow (§5), so every open tab stays in sync and a control
// fired mid-swap can't desync the UI.
//
// Post-recording naming: every falling edge of `recording.active` (seen once,
// via the `recPrompted` latch below) opens a small modal prefilled with the
// auto `web_<timestamp>.bin` name. Skip/Esc/backdrop-click leaves that name in
// place (the file is already on disk either way — this is never blocking).
// Save sends `rename_capture`; success/failure is read back off the next
// `session` echo (`recording.last_name`) rather than guessed locally, since
// the server owns collision/validity checks.
//
// Hub events:  subscribes "session", "captures";
//              sends record / list_captures / load_capture / go_live / transport / rename_capture.

export function createCapture(hub) {
    const $ = (id) => document.getElementById(id);

    const btnRecord = $('btn-record');
    const recStatus = $('record-status');
    const capList = $('cap-list');
    const btnRefresh = $('btn-refresh-caps');
    const transport = $('transport');
    const btnGoLive = $('btn-golive');
    const btnPlayPause = $('btn-playpause');
    const btnRestart = $('btn-restart');
    const segSpeed = $('seg-speed');
    const chkLoop = $('chk-loop');
    const seek = $('seek');
    const posStatus = $('pos-status');

    const recNameModal = $('record-name-modal');
    const recNameInput = $('record-name-input');
    const recNameError = $('record-name-error');
    const recNameSave = $('record-name-save');
    const recNameSkip = $('record-name-skip');
    const recNameClose = $('record-name-close');

    let session = null;      // latest server session snapshot
    let captures = [];       // latest capture library
    let dragging = false;    // true while the user drags the seek slider
    let prevRecActive = null; // recording.active as of the previous session message
    let recPrompted = false;  // already opened (or skipped opening) the modal for the CURRENT stopped take
    let renameTarget = null;  // the name the modal is currently offering to rename FROM
    let renamePending = null; // the sanitized name we just asked the server to rename TO, until echoed back

    // ---- formatting helpers ----
    const fmtBytes = (n) => {
        if (!n) return '0 B';
        if (n < 1024) return n + ' B';
        if (n < 1048576) return (n / 1024).toFixed(1) + ' KB';
        return (n / 1048576).toFixed(1) + ' MB';
    };
    const fmtTime = (s) => {
        s = Math.max(0, Math.floor(s || 0));
        const m = Math.floor(s / 60);
        return m + ':' + String(s % 60).padStart(2, '0');
    };

    // ---- outbound: record / library / transport ----
    btnRecord?.addEventListener('click', () => {
        const active = session?.recording?.active;
        hub.send({ type: 'record', on: !active });
    });
    btnRefresh?.addEventListener('click', () => hub.send({ type: 'list_captures' }));

    // Source rows are rebuilt from data; delegate clicks off the container.
    capList?.addEventListener('click', (e) => {
        const row = e.target.closest('.cap-row');
        if (!row) return;
        if (row.dataset.live === '1') hub.send({ type: 'go_live' });
        else if (row.dataset.name) hub.send({ type: 'load_capture', name: row.dataset.name });
    });

    btnGoLive?.addEventListener('click', () => hub.send({ type: 'go_live' }));
    btnPlayPause?.addEventListener('click', () => {
        const paused = session?.playback?.paused;
        hub.send({ type: 'transport', action: paused ? 'resume' : 'pause' });
    });
    btnRestart?.addEventListener('click', () => hub.send({ type: 'transport', action: 'restart' }));
    segSpeed?.addEventListener('click', (e) => {
        const btn = e.target.closest('button[data-fps]');
        if (btn) hub.send({ type: 'transport', action: 'speed', value: parseFloat(btn.dataset.fps) });
    });
    chkLoop?.addEventListener('change', () => hub.send({ type: 'transport', action: 'loop', value: chkLoop.checked ? 1 : 0 }));

    // ---- outbound + wiring: post-recording naming modal ----
    recNameSkip?.addEventListener('click', closeRenameModal);
    recNameClose?.addEventListener('click', closeRenameModal);
    recNameModal?.addEventListener('click', (e) => { if (e.target === recNameModal) closeRenameModal(); });
    document.addEventListener('keydown', (e) => {
        if (e.key === 'Escape' && recNameModal && !recNameModal.classList.contains('hidden')) closeRenameModal();
    });
    recNameInput?.addEventListener('keydown', (e) => { if (e.key === 'Enter') submitRename(); });
    recNameSave?.addEventListener('click', submitRename);

    function openRenameModal(currentName) {
        renameTarget = currentName;
        renamePending = null;
        if (recNameInput) {
            recNameInput.value = currentName.endsWith('.bin') ? currentName.slice(0, -4) : currentName;
        }
        if (recNameError) recNameError.textContent = '';
        if (recNameSave) recNameSave.disabled = false;
        recNameModal?.classList.remove('hidden');
        recNameInput?.focus();
        recNameInput?.select();
    }

    function closeRenameModal() {
        recNameModal?.classList.add('hidden');
        renameTarget = null;
        renamePending = null;
    }

    function submitRename() {
        const typed = (recNameInput?.value || '').trim();
        if (!typed) { closeRenameModal(); return; }           // blank -> keep the auto name
        const sanitized = typed.endsWith('.bin') ? typed : typed + '.bin';
        if (sanitized === renameTarget) { closeRenameModal(); return; }  // unchanged -> no-op
        renamePending = sanitized;
        if (recNameSave) recNameSave.disabled = true;
        if (recNameError) recNameError.textContent = '';
        hub.send({ type: 'rename_capture', name: typed });
    }

    // Seek: preview locally while dragging (don't fight server position echoes),
    // commit on release. `input` fires continuously, `change` on release.
    seek?.addEventListener('input', () => {
        dragging = true;
        updatePos(seek.value / 1000);
    });
    seek?.addEventListener('change', () => {
        hub.send({ type: 'transport', action: 'seek', value: seek.value / 1000 });
        dragging = false;
    });

    // ---- inbound: render from server state ----
    hub.on('captures', (msg) => { captures = Array.isArray(msg.items) ? msg.items : []; renderList(); });
    hub.on('session', (msg) => {
        session = msg;
        renderSession();

        const rec = session.recording || {};
        if (rec.active) {
            recPrompted = false;                        // re-arm for this take's eventual stop
        } else if (prevRecActive === true && !recPrompted && rec.last_name) {
            recPrompted = true;
            openRenameModal(rec.last_name);
        }
        prevRecActive = !!rec.active;

        if (renamePending) {
            if (rec.last_name === renamePending) {
                closeRenameModal();                      // server confirmed the rename
            } else if (rec.last_name === renameTarget) {
                // server rejected it (collision/invalid name) -- unchanged, let the user retry.
                if (recNameError) recNameError.textContent = 'That name is taken or invalid — try another.';
                if (recNameSave) recNameSave.disabled = false;
                renamePending = null;
            }
        }
    });

    function renderSession() {
        if (!session) return;
        const isReplay = !!session.playback?.is_replay;
        const rec = session.recording || {};

        // Record button: live-only, red + timer while active.
        if (btnRecord) {
            btnRecord.disabled = !session.has_live || isReplay;
            btnRecord.classList.toggle('recording', !!rec.active);
            btnRecord.innerHTML = rec.active ? '&#9632; Stop' : '&#9679; Record';
        }
        if (recStatus) {
            recStatus.classList.toggle('rec', !!rec.active);
            recStatus.textContent = rec.active
                ? `Rec ${fmtTime(rec.elapsed_s)} · ${fmtBytes(rec.bytes)}`
                : '';
        }

        // Transport visibility + state.
        if (transport) transport.classList.toggle('hidden', !isReplay);
        if (isReplay) {
            const pb = session.playback;
            if (btnGoLive) btnGoLive.disabled = !session.has_live;
            if (btnPlayPause) btnPlayPause.textContent = pb.paused ? 'Resume' : 'Pause';
            setActive(segSpeed, 'fps', String(pb.speed_fps ?? 0));
            if (chkLoop) chkLoop.checked = !!pb.loop;
            if (!dragging && typeof pb.position === 'number') {
                seek.value = Math.round(pb.position * 1000);
                updatePos(pb.position);
            }
        }
        renderList();   // active-row highlight tracks mode/capture_name
    }

    function updatePos(frac) {
        const pb = session?.playback || {};
        const total = pb.total_frames || 0;
        const idx = Math.round((frac || 0) * Math.max(0, total - 1));
        const elapsed = typeof pb.elapsed_s === 'number' ? pb.elapsed_s : (frac || 0) * (pb.duration_s || 0);
        const duration = pb.duration_s || 0;
        if (posStatus) posStatus.textContent = total ? `${fmtTime(elapsed)} / ${fmtTime(duration)} · frame ${idx} / ${total - 1}` : '—';
    }

    function renderList() {
        if (!capList) return;
        const isReplay = !!session?.playback?.is_replay;
        const current = session?.playback?.capture_name;
        const rows = [];
        // Live row (only when a device source exists).
        if (session?.has_live) {
            const active = !isReplay ? ' active' : '';
            rows.push(`<div class="cap-row${active}" data-live="1">` +
                `<span class="cap-row__name">&#9679; Live device</span>` +
                `<span class="cap-row__meta">${escapeHtml(shortLabel(session.source_label))}</span></div>`);
        }
        for (const c of captures) {
            const active = (isReplay && c.name === current) ? ' active' : '';
            const capability = c.has_stream_9 ? '' : ' · point cloud only';
            rows.push(`<div class="cap-row${active}" data-name="${escapeHtml(c.name)}">` +
                `<span class="cap-row__name">${escapeHtml(c.name)}</span>` +
                `<span class="cap-row__meta">${fmtBytes(c.bytes)} · ${fmtTime(c.duration_s || 0)}${capability}</span></div>`);
        }
        if (!rows.length) rows.push('<div class="cap-status">no captures yet</div>');
        capList.innerHTML = rows.join('');
    }

    function shortLabel(label) {
        if (!label) return '';
        // "Ethernet/UDP · 1.2.3.4:5000" -> "Ethernet/UDP"; keep it short.
        return label.split(' · ')[0];
    }

    function setActive(seg, attr, value) {
        if (!seg) return;
        for (const b of seg.querySelectorAll('button')) {
            b.classList.toggle('active', b.dataset[attr] === value);
        }
    }

    function escapeHtml(s) {
        return String(s).replace(/[&<>"']/g, (c) =>
            ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
    }

    return {};
}
