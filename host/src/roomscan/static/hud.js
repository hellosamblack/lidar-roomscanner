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
    });

    hub.on('conn', (msg) => {
        const open = msg && msg.state === 'open';
        if (connText) connText.textContent = open ? 'Live' : (msg.state === 'connecting' ? 'Connecting…' : 'Offline');
        if (connDot) connDot.classList.toggle('connected', open);
    });

    return {};
}
