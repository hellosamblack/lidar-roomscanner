// capture.js — the Record top-bar control + the Playback card (Web Phase 3,
// split §11; Record moved out of its own sidebar card into the top bar in
// issue #118, 2026-08-19).
//
// Record a live session and drive playback transport (pause/resume, speed,
// loop, seek). Like controls.js, this turns DOM events into hub.send(...) and
// drives ALL active/enabled/visible state FROM the server's `session` /
// `captures` / `state` echo — one-way flow (§5), so every open tab stays in
// sync and a control fired mid-swap can't desync the UI.
//
// §11 (owner ask, 2026-07-31) split the old "Capture & Playback" group in two,
// because the two halves belong to different pages: Record only means anything
// on Live (`state.source === "live"`), transport only means anything in replay
// (`session.playback.is_replay`). Browsing the library moved out entirely — the
// View page's file browser (browser.js) owns it now — so `#cap-list` and
// `#btn-refresh-caps` are gone from here.
//
// Post-recording naming: every falling edge of `recording.active` (seen once,
// via the `recPrompted` latch below) opens a small modal prefilled with the
// auto `web_<timestamp>.bin` name. Skip/Esc/backdrop-click leaves that name in
// place (the file is already on disk either way — this is never blocking).
// Save sends `rename_capture`; success/failure is read back off the next
// `session` echo (`recording.last_name`) rather than guessed locally, since
// the server owns collision/validity checks.
//
// The rename modal is exported from the factory (`openRename`) so browser.js
// can reuse the exact same dialog for renaming an arbitrary capture instead of
// growing a second, drifting copy.
//
// Hub events:  subscribes "session", "state";
//              sends record / go_live / transport / rename_capture.

export function createCapture(hub) {
    const $ = (id) => document.getElementById(id);

    const btnRecord = $('btn-record');
    const recStatus = $('record-status');
    const recordControls = $('topbar-record');
    const transportCard = $('transport-card');
    const btnGoLive = $('btn-golive');
    const btnPlayPause = $('btn-playpause');
    // NOT `btn-restart` -- that id was shared with the top bar's "Restart Server"
    // button, and getElementById returns the FIRST match in document order. This
    // playback Restart was therefore dead, and its transport handler was bound to
    // Restart Server, so pressing that fired a transport restart AND POST
    // /api/restart. See BUGS.md BUG-047.
    const btnRestart = $('btn-transport-restart');
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
    let source = 'live';     // latest server `state.source` — gates the Record card
    let dragging = false;    // true while the user drags the seek slider
    let prevRecActive = null; // recording.active as of the previous session message
    let recPrompted = false;  // already opened (or skipped opening) the modal for the CURRENT stopped take
    let renameTarget = null;  // the name the modal is currently offering to rename FROM
    let renamePending = null; // the sanitized name we just asked the server to rename TO, until echoed back
    let fromBrowser = false;  // the modal was opened by browser.js against an arbitrary capture
    // #103: a browser rename is fire-and-forget (the `captures` echo carries no
    // per-request result), so the caller gets told the name we asked for and can
    // follow its own selection across the rename instead of losing track of it.
    let onRenamed = null;

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

    function openRenameModal(currentName, opts) {
        renameTarget = currentName;
        renamePending = null;
        // A post-recording rename addresses "the take that just stopped" and is
        // confirmed by `session.recording.last_name`; a browser rename names an
        // arbitrary file explicitly (`from`) and is confirmed by the `captures`
        // echo, which carries no per-request result. Keeping them apart is why
        // the wire's `from` field is optional rather than always sent.
        fromBrowser = !!(opts && opts.fromBrowser);
        onRenamed = (opts && typeof opts.onRenamed === 'function') ? opts.onRenamed : null;
        const title = document.getElementById('record-name-title');
        if (title) title.textContent = fromBrowser ? 'Rename Capture' : 'Name Recording';
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
        fromBrowser = false;
        onRenamed = null;
    }

    function submitRename() {
        const typed = (recNameInput?.value || '').trim();
        if (!typed) { closeRenameModal(); return; }           // blank -> keep the auto name
        const sanitized = typed.endsWith('.bin') ? typed : typed + '.bin';
        if (sanitized === renameTarget) { closeRenameModal(); return; }  // unchanged -> no-op
        if (fromBrowser) {
            // Explicit source: absent `from` means "the last recording", which
            // is emphatically not what the browser is asking for.
            hub.send({ type: 'rename_capture', from: renameTarget, name: typed });
            // Tell the opener the name we ASKED for, before closeRenameModal()
            // clears the handle. It is a request, not a confirmation — the caller
            // must still check the name against the next `captures` echo.
            onRenamed?.(sanitized);
            closeRenameModal();
            return;
        }
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
    // Record is a LIVE-page control; the server's `state.source` is the only
    // authority on which page we are on (one-way flow), so the top-bar
    // control's presence rides that echo rather than being inferred from
    // `session`.
    hub.on('state', (msg) => {
        source = msg.source || 'live';
        recordControls?.classList.toggle('hidden', source !== 'live');
        window.__relayout && window.__relayout();
    });
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

        // Playback card visibility + transport state.
        if (transportCard) transportCard.classList.toggle('hidden', !isReplay);
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
        window.__relayout && window.__relayout();
    }

    function updatePos(frac) {
        const pb = session?.playback || {};
        const total = pb.total_frames || 0;
        const idx = Math.round((frac || 0) * Math.max(0, total - 1));
        const elapsed = typeof pb.elapsed_s === 'number' ? pb.elapsed_s : (frac || 0) * (pb.duration_s || 0);
        const duration = pb.duration_s || 0;
        if (posStatus) posStatus.textContent = total ? `${fmtTime(elapsed)} / ${fmtTime(duration)} · frame ${idx} / ${total - 1}` : '—';
    }

    function setActive(seg, attr, value) {
        if (!seg) return;
        for (const b of seg.querySelectorAll('button')) {
            b.classList.toggle('active', b.dataset[attr] === value);
        }
    }

    // Exported so browser.js reuses THIS dialog (one modal, one set of
    // Skip/Esc/backdrop semantics) rather than growing a second copy that
    // drifts from it.
    return { openRename: (name, opts) => openRenameModal(name, opts) };
}
