// sensors.js — the left-rail Sensors readout (streams 9/10, Web Phase 2).
//
// Subscribes to "sensor" (JSON) and paints three 2D-canvas widgets + text:
//   - orientation gizmo: orthographic projection of the server-computed display
//     rotation `rot` (T_WORLD_TO_CV @ R @ T_CV_TO_BODY, already in the scene's
//     Open3D-CV frame), so we never re-derive the load-bearing sign matrices.
//   - tilt-compensated compass: needle at `heading` (0=up=N, clockwise), matching
//     the desktop render_compass convention.
//   - pressure / temperature sparklines: min/max-autoscaled polyline over the
//     history arrays + a live value readout.
// Plus a fusion-status line. All draws are guarded so a null-field message (a
// ToF-only or pre-calibration session) renders placeholders and never throws.
//
// Everything is read-only, so this lives in the pointer-events:none left rail.
//
// Public surface:  createSensors(hub) -> {}
// Hub events:  subscribes "sensor"

const D = (m, l) => { try { window.__diag && window.__diag('sensors.js: ' + m, l); } catch (e) {} };

// Match index.html's design tokens (canvas can't read CSS vars directly).
const AXIS_COLORS = ['#ef4444', '#10b981', '#3b82f6'];   // X red, Y green, Z blue
const AXIS_LABELS = ['X', 'Y', 'Z'];
const GRID = 'rgba(255,255,255,0.10)';
const INK = '#e2e8f0';
const MUTED = '#94a3b8';
const ACCENT = '#60a5fa';

// Size a canvas's backing store for the device pixel ratio and return its 2D
// context pre-scaled to CSS pixels, so all draw code works in CSS units.
function fitCanvas(canvas, cssW, cssH) {
    const dpr = window.devicePixelRatio || 1;
    if (canvas.width !== Math.round(cssW * dpr) || canvas.height !== Math.round(cssH * dpr)) {
        canvas.width = Math.round(cssW * dpr);
        canvas.height = Math.round(cssH * dpr);
        canvas.style.width = cssW + 'px';
        canvas.style.height = cssH + 'px';
    }
    const ctx = canvas.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, cssW, cssH);
    return ctx;
}

// --- gizmo: project the rotated basis triad orthographically ---
function drawGizmo(canvas, rot) {
    const S = 96;
    const ctx = fitCanvas(canvas, S, S);
    const cx = S / 2, cy = S / 2, len = S * 0.36;

    // faint origin ring
    ctx.strokeStyle = GRID;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(cx, cy, len * 1.15, 0, Math.PI * 2);
    ctx.stroke();

    if (!Array.isArray(rot) || rot.length !== 9) {
        ctx.fillStyle = MUTED;
        ctx.font = '11px "JetBrains Mono", monospace';
        ctx.textAlign = 'center';
        ctx.fillText('—', cx, cy + 4);
        return;
    }

    // Column c basis vector = (rot[c], rot[3+c], rot[6+c]) in CV world (x=right,
    // y=down, z=forward/into-screen). Screen x=+x, y=+y (canvas +y is down too).
    // The scene camera looks along +Z, so +Z points away: draw far axes first
    // and dim them for a cheap depth cue.
    const axes = [0, 1, 2].map((c) => ({
        c,
        x: rot[c], y: rot[3 + c], z: rot[6 + c],
    }));
    axes.sort((a, b) => b.z - a.z);   // most-away (largest +z) first

    for (const a of axes) {
        const tipx = cx + a.x * len, tipy = cy + a.y * len;
        const alpha = 0.45 + 0.55 * (1 - Math.min(1, Math.max(0, (a.z + 1) / 2)));
        ctx.globalAlpha = alpha;
        ctx.strokeStyle = AXIS_COLORS[a.c];
        ctx.fillStyle = AXIS_COLORS[a.c];
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(cx, cy);
        ctx.lineTo(tipx, tipy);
        ctx.stroke();
        ctx.beginPath();
        ctx.arc(tipx, tipy, 2.5, 0, Math.PI * 2);
        ctx.fill();
        ctx.font = 'bold 10px "JetBrains Mono", monospace';
        ctx.textAlign = 'center';
        ctx.fillText(AXIS_LABELS[a.c], cx + a.x * len * 1.28, cy + a.y * len * 1.28 + 3);
    }
    ctx.globalAlpha = 1;
}

// --- compass: dial + needle (0=up=N, clockwise) ---
function drawCompass(canvas, heading) {
    const S = 96;
    const ctx = fitCanvas(canvas, S, S);
    const cx = S / 2, cy = S / 2, r = S * 0.42;

    ctx.strokeStyle = GRID;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.stroke();

    // cardinal ticks + N label
    ctx.fillStyle = MUTED;
    ctx.font = '9px "Inter", sans-serif';
    ctx.textAlign = 'center';
    const cards = [['N', 0], ['E', 90], ['S', 180], ['W', 270]];
    for (const [lbl, deg] of cards) {
        const a = deg * Math.PI / 180;
        const tx = cx + Math.sin(a) * (r - 8), ty = cy - Math.cos(a) * (r - 8);
        ctx.fillStyle = lbl === 'N' ? ACCENT : MUTED;
        ctx.fillText(lbl, tx, ty + 3);
    }

    if (heading === null || heading === undefined) return;

    // needle: tip at heading, tail opposite. 0=up, clockwise.
    const a = heading * Math.PI / 180;
    const tipx = cx + Math.sin(a) * r * 0.8, tipy = cy - Math.cos(a) * r * 0.8;
    const tailx = cx - Math.sin(a) * r * 0.32, taily = cy + Math.cos(a) * r * 0.32;
    ctx.strokeStyle = '#ef4444';
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    ctx.moveTo(tailx, taily);
    ctx.lineTo(tipx, tipy);
    ctx.stroke();
    ctx.fillStyle = '#ef4444';
    ctx.beginPath();
    ctx.arc(tipx, tipy, 3, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = INK;
    ctx.beginPath();
    ctx.arc(cx, cy, 2.5, 0, Math.PI * 2);
    ctx.fill();
}

// --- sparkline: min/max-autoscaled polyline ---
function drawSparkline(canvas, values) {
    const W = 200, H = 32;
    const ctx = fitCanvas(canvas, W, H);
    ctx.strokeStyle = GRID;
    ctx.lineWidth = 1;
    ctx.strokeRect(0.5, 0.5, W - 1, H - 1);

    if (!Array.isArray(values) || values.length < 2) return;
    let lo = Infinity, hi = -Infinity;
    for (const v of values) { if (v < lo) lo = v; if (v > hi) hi = v; }
    const span = (hi - lo) || 1;
    const pad = 3;
    ctx.strokeStyle = ACCENT;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    for (let i = 0; i < values.length; i++) {
        const x = pad + (W - 2 * pad) * (i / (values.length - 1));
        const y = pad + (H - 2 * pad) * (1 - (values[i] - lo) / span);
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.stroke();
}

export function createSensors(hub) {
    const $ = (id) => document.getElementById(id);
    const gizmo = $('sensor-gizmo');
    const compass = $('sensor-compass');
    const headingEl = $('sensor-heading');
    const fusionEl = $('sensor-fusion');
    const pressSpark = $('sensor-press-spark');
    const pressVal = $('sensor-press-val');
    const tempSpark = $('sensor-temp-spark');
    const tempVal = $('sensor-temp-val');
    if (!gizmo || !compass) { D('sensor DOM missing — skipping', 'error'); return {}; }

    // prime placeholders
    drawGizmo(gizmo, null);
    drawCompass(compass, null);
    drawSparkline(pressSpark, null);
    drawSparkline(tempSpark, null);

    hub.on('sensor', (msg) => {
        try {
            drawGizmo(gizmo, msg.rot);
            drawCompass(compass, msg.heading);
            if (headingEl) headingEl.textContent =
                (msg.heading === null || msg.heading === undefined) ? '—' : msg.heading.toFixed(1) + '°';
            if (fusionEl) fusionEl.textContent = msg.fusion || 'off';
            if (pressVal) pressVal.textContent =
                (msg.pressure_pa === null || msg.pressure_pa === undefined) ? '—' : Math.round(msg.pressure_pa) + ' Pa';
            if (tempVal) tempVal.textContent =
                (msg.temp_c === null || msg.temp_c === undefined) ? '—' : msg.temp_c.toFixed(1) + ' °C';
            drawSparkline(pressSpark, msg.pressure_hist);
            drawSparkline(tempSpark, msg.temp_hist);
            if (!window.__gotSensor) { window.__gotSensor = true; D('first sensor frame'); }
        } catch (e) {
            D('sensor draw threw: ' + (e && e.message), 'error');
        }
    });

    return {};
}
