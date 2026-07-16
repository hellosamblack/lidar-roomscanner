// capture.js — the Capture & Playback control group (Web Phase 3).
//
// Record a live session, browse the server's capture library, load one for
// replay at runtime, and drive playback transport (pause/resume, speed, loop,
// seek). Like controls.js, this turns DOM events into hub.send(...) and drives
// ALL active/enabled/visible state FROM the server's `session` / `captures`
// echo — one-way flow (§5), so every open tab stays in sync and a control
// fired mid-swap can't desync the UI.
//
// Hub events:  subscribes "session", "captures";
//              sends record / list_captures / load_capture / go_live / transport.

export function createCapture(hub) {
    const $ = (id) => document.getElementById(id);

    const btnRecord = $('btn-record');
    const recStatus = $('record-status');
    const capList = $('cap-list');
    const btnRefresh = $('btn-refresh-caps');
    const transport = $('transport');
    const btnPlayPause = $('btn-playpause');
    const btnRestart = $('btn-restart');
    const segSpeed = $('seg-speed');
    const chkLoop = $('chk-loop');
    const seek = $('seek');
    const posStatus = $('pos-status');

    let session = null;      // latest server session snapshot
    let captures = [];       // latest capture library
    let dragging = false;    // true while the user drags the seek slider

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
    hub.on('session', (msg) => { session = msg; renderSession(); });

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
        const total = session?.playback?.total_frames || 0;
        const idx = Math.round((frac || 0) * Math.max(0, total - 1));
        if (posStatus) posStatus.textContent = total ? `frame ${idx} / ${total - 1}` : '—';
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
            rows.push(`<div class="cap-row${active}" data-name="${escapeHtml(c.name)}">` +
                `<span class="cap-row__name">${escapeHtml(c.name)}</span>` +
                `<span class="cap-row__meta">${fmtBytes(c.bytes)}</span></div>`);
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
