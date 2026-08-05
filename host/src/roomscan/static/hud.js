// hud.js — the left-rail read-only telemetry HUD (§7.4) + connection indicator.
//
// Pure DOM text updates, no canvas. Subscribes to "metrics" (server snapshot),
// "view_fps" (scene's own browser paint rate — distinct from device fps), and
// "conn" (socket lifecycle, for the top-bar dot). Number formatting reimplements
// metrics.py's rules so the web HUD reads identically to the desktop panel:
//   fmt_hz  -> one decimal under 10 Hz, integer at/above 10, "-" for null
//   fmt_rate -> B/s, KB/s, MB/s (1024-based)
//
// Public surface:  createHud(hub) -> {}
// Hub events:  subscribes "metrics", "view_fps", "conn"

const LINK_BAR_MAX = 2 * 1024 * 1024;   // client-side visual cap: 2 MB/s (§7.4)

// Headroom colouring for the Resources card (owner ask, 2026-07-31).
//
// These are HEADROOM thresholds, not measurements of any particular workload:
// the point is "how close to the ceiling", and the ceiling is the same
// whatever the scan. 90% is where TsdfMap._check_saturation already warns
// (BUG-035 — the run that failed sat at ~97% of its configured block count),
// so the red step is pinned to that existing, measured threshold rather than
// invented; amber is one step earlier so it is a warning and not an epitaph.
const RES_WARN = 0.80;
const RES_CRIT = 0.90;

// Per-stream tooltips. Keyed on `stream_id`, NOT on `label`: metrics.py maps two
// different ids (DEPTH_ZF32=0 replay, RAW_3DMD=7 live) onto the same "ToF" label,
// so a label-keyed map could not tell a replay apart from a live capture. Ids are
// the wire protocol's own (docs/protocol.md); the labels are only presentation.
const STREAM_HELP = {
    0: 'Processed depth frames (ZF32) from a Phase-1 recording — one per rendered point cloud. About 28 per second.',
    7: 'Raw sensor frames (3DMD) straight off the ToF imager; the depth pipeline runs here on the host. About 28 per second.',
    9: 'Orientation: the IMU’s fused quaternion, averaged over the samples taken during each depth frame. One per frame, so about 30 per second.',
    10: 'Environment: barometric pressure, temperature and the magnetometer, read through the IMU’s sensor hub. One per depth frame.',
    11: 'IMU raw: the unfiltered 480 Hz accelerometer and gyroscope batch behind the fused orientation. Recorded for offline analysis; the live view does not use it.',
    12: 'IMU clock calibration — the measured error of the IMU’s own oscillator, used to convert its timestamps to real seconds. Sent rarely.',
    13: 'IMU sync: the IMU clock read at the instant the depth frame was ready, so a frame can be placed on the IMU timeline to within tens of microseconds.',
};

function fmtHz(hz) {
    if (hz === null || hz === undefined) return '-';
    return hz < 10 ? hz.toFixed(1) : hz.toFixed(0);
}

function fmtRate(n) {
    let x = Number(n) || 0;
    const units = ['B', 'KB', 'MB', 'GB'];
    for (const unit of units) {
        if (x < 1024 || unit === 'GB') {
            return unit === 'B' ? `${x.toFixed(0)} ${unit}/s` : `${x.toFixed(1)} ${unit}/s`;
        }
        x /= 1024;
    }
    return `${x.toFixed(1)} GB/s`;
}

function fmtBytes(n) {
    if (n === null || n === undefined) return '?';
    let x = Number(n);
    for (const unit of ['B', 'KB', 'MB', 'GB', 'TB']) {
        if (x < 1024 || unit === 'TB') return `${x.toFixed(1)} ${unit}`;
        x /= 1024;
    }
    return `${x.toFixed(1)} TB`;
}

// One headroom bar: text "used / total" plus a fill coloured by fraction.
// `frac === null` means UNKNOWN (no NVML, no SLAM running, no sample yet) --
// rendered as an empty bar and an explicit "n/a", never as 0%, which would
// read as a measurement of plenty of headroom.
function setBar(valEl, fillEl, text, frac) {
    if (valEl) valEl.textContent = text;
    if (!fillEl) return;
    const f = (frac === null || frac === undefined || !isFinite(frac))
        ? null : Math.max(0, Math.min(1, frac));
    fillEl.style.width = (f === null ? 0 : f * 100).toFixed(0) + '%';
    fillEl.classList.toggle('is-warn', f !== null && f >= RES_WARN && f < RES_CRIT);
    fillEl.classList.toggle('is-crit', f !== null && f >= RES_CRIT);
}

export function createHud(hub) {
    const viewFpsEl = document.getElementById('hud-view-fps');
    const deviceFpsEl = document.getElementById('hud-device-fps');
    const streamsEl = document.getElementById('hud-streams');
    const linkValueEl = document.getElementById('hud-link-value');
    const linkFillEl = document.getElementById('hud-link-fill');
    const dropsEl = document.getElementById('hud-drops');
    const gapsEl = document.getElementById('hud-gaps');
    // Connection dot/text live in the top bar; the HUD owns them as telemetry.
    const connText = document.getElementById('conn-text');
    const connDot = document.getElementById('conn-dot');

    hub.on('view_fps', (n) => { if (viewFpsEl) viewFpsEl.textContent = String(n); });

    hub.on('metrics', (msg) => {
        if (deviceFpsEl) deviceFpsEl.textContent = fmtHz(msg.render_fps);
        if (dropsEl) dropsEl.textContent = String(msg.drops ?? 0);
        if (gapsEl) gapsEl.textContent = String(msg.gaps ?? 0);

        // Per-stream rows: label · host_hz · jitter_ms (or "-").
        if (streamsEl) {
            const streams = Array.isArray(msg.streams) ? msg.streams : [];
            streamsEl.innerHTML = '';
            for (const s of streams) {
                const row = document.createElement('div');
                row.className = 'hud-stream';
                const jitter = (s.jitter_ms === null || s.jitter_ms === undefined)
                    ? '-' : Number(s.jitter_ms).toFixed(1) + ' ms';
                // What the stream is, then what the two numbers beside it mean.
                // An unknown id still gets the column legend rather than nothing.
                const help = STREAM_HELP[s.stream_id];
                row.title = (help ? help + '\n\n' : '')
                    + 'Left: arrival rate measured here. Right: jitter — how unevenly spaced the arrivals are.';
                row.innerHTML =
                    `<span class="hud-stream__label">${s.label ?? '?'}</span>` +
                    `<span class="hud-stream__hz">${fmtHz(s.host_hz)} Hz</span>` +
                    `<span class="hud-stream__jit">${jitter}</span>`;
                streamsEl.appendChild(row);
            }
        }

        // Link bandwidth bar — relative-magnitude gauge, capped at LINK_BAR_MAX.
        const bps = Number(msg.link_bytes_per_s) || 0;
        if (linkValueEl) linkValueEl.textContent = fmtRate(bps);
        if (linkFillEl) {
            const pct = Math.max(0, Math.min(1, bps / LINK_BAR_MAX)) * 100;
            linkFillEl.style.width = pct.toFixed(0) + '%';
        }

        renderResources(msg.resources || null);
    });

    // --- Resources card (owner ask, 2026-07-31) ---------------------------
    // CPU/RAM/VRAM are system/device-wide: the question is headroom, and the
    // ceiling is shared with every other process on the box. The per-process
    // numbers ride along in brackets so it is still possible to tell "the box
    // is busy" from "we are busy". Anything the server could not measure
    // arrives as null and renders "n/a" — see setBar.
    function renderResources(r) {
        const note = document.getElementById('res-note');
        if (!r) {
            setBar(document.getElementById('res-cpu-val'), document.getElementById('res-cpu-fill'), 'n/a', null);
            setBar(document.getElementById('res-ram-val'), document.getElementById('res-ram-fill'), 'n/a', null);
            setBar(document.getElementById('res-vram-val'), document.getElementById('res-vram-fill'), 'n/a', null);
            if (note) note.textContent = 'waiting for the first resource sample…';
            return;
        }
        const sysCpu = r.sys_cpu_percent;
        const procCpu = r.proc_cpu_percent;
        setBar(document.getElementById('res-cpu-val'), document.getElementById('res-cpu-fill'),
            (sysCpu === null || sysCpu === undefined ? 'n/a' : sysCpu.toFixed(0) + '%')
            + (procCpu === null || procCpu === undefined ? '' : ` (us ${procCpu.toFixed(0)}%)`),
            (sysCpu === null || sysCpu === undefined) ? null : sysCpu / 100);

        const ramUsed = r.ram_used, ramTotal = r.ram_total;
        setBar(document.getElementById('res-ram-val'), document.getElementById('res-ram-fill'),
            (ramUsed === null || ramUsed === undefined)
                ? 'n/a'
                : `${fmtBytes(ramUsed)} / ${fmtBytes(ramTotal)} (us ${fmtBytes(r.proc_rss)})`,
            (ramUsed && ramTotal) ? ramUsed / ramTotal : null);

        const vUsed = r.device_vram_used, vTotal = r.device_vram_total;
        setBar(document.getElementById('res-vram-val'), document.getElementById('res-vram-fill'),
            (vUsed === null || vUsed === undefined)
                ? 'n/a (no NVIDIA driver)'
                : `${fmtBytes(vUsed)} / ${fmtBytes(vTotal)}`,
            (vUsed && vTotal) ? vUsed / vTotal : null);

        if (note) {
            note.textContent = r.gpu_name ? String(r.gpu_name)
                : (r.device_vram_source === 'nvml' ? 'GPU present' : 'no GPU detected');
        }
    }
    renderResources(null);

    hub.on('conn', (msg) => {
        const open = msg && msg.state === 'open';
        if (connText) connText.textContent = open ? 'Live' : (msg.state === 'connecting' ? 'Connecting…' : 'Offline');
        if (connDot) connDot.classList.toggle('connected', open);
    });

    // Comm-status: server->rig link health, one chip per hop (FileHub, Scanner,
    // ST-Link). The server computes each `state`; we only paint. title= carries
    // the address + detail so the reachability story is one hover away.
    const commEl = document.getElementById('comm-status');
    hub.on('comm', (msg) => {
        if (!commEl) return;
        const targets = Array.isArray(msg.targets) ? msg.targets : [];
        commEl.innerHTML = '';
        for (const t of targets) {
            const state = ['up', 'down', 'absent', 'unknown'].includes(t.state) ? t.state : 'unknown';
            const item = document.createElement('div');
            item.className = 'comm-item comm-item--' + state;
            const addr = t.addr ? ' (' + t.addr + ')' : '';
            item.title = (t.label || t.id) + addr + (t.detail ? ' — ' + t.detail : '');
            const dot = document.createElement('span');
            dot.className = 'comm-item__dot';
            const label = document.createElement('span');
            label.className = 'comm-item__label';
            label.textContent = t.label || t.id;
            item.appendChild(dot);
            item.appendChild(label);
            commEl.appendChild(item);
        }
    });

    return {};
}
