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
//
// Card structure (decluttered 2026-07-29; pinned to World-only 2026-07-31):
// the always-visible tiers are the two widgets above, the World orientation
// readout (3 values + warnings — the decomposition picker is gone, the owner
// only ever used World), the mag-fusion state + its help line + two buttons,
// and Environment. The diagnostic readouts — World's note, yaw offset +
// buttons, full-precision raw ZYX, jitter table — sit inside the
// collapsed-by-default `#sensor-diag` <details>. Nothing here cares whether
// it is open; every element is bound the same way.
//
// Plus a fusion-status line, and (owner ask, 2026-07-28) two more readouts:
//   - Raw Orientation: `orientation_raw` — the fused quat + Euler roll/pitch/yaw
//     + heading at FULL PRECISION, pre-OrientationSmoother (same raw signal as
//     `rot`/`heading` above, just not rounded for a gizmo/compass draw). ALWAYS
//     ZYX Tait-Bryan, regardless of the World readout above.
//   - Jitter: `jitter` — server-computed rolling-window frame-to-frame noise
//     (p95 and mean as two columns of a grid, deg/frame) for roll/pitch/yaw/heading and
//     the overall orientation step. Computed server-side from full precision —
//     never re-derive this client-side from the rounded `rot`/`heading` fields.
//   - Orientation View: `orientation_view` — the World decomposition (3
//     renamable axis-label inputs, sent via `set_orientation {labels}` and
//     echoed back through `state`) plus a near-singularity warning and a
//     mag-validity/motion warning. Presentation-only: this never changes what's
//     rendered in the 3D view, only how the Sensors card reads out the SAME
//     orientation. `set_orientation {mode}` and the zyx/zxy/boresight math
//     stay on the wire for the deprecated desktop panel — see
//     docs/web-protocol.md "Orientation decomposition modes".
//   - Fusion help (owner ask, 2026-07-31): `fusion_key` -> a remedy string from
//     FUSION_HELP below, shown under the Fusion row and mirrored into its
//     title. Hidden when fusion is `active`. No protocol change — the server
//     already sent `fusion_key`, only the label was rendered before.
//   - Zero Yaw Here (owner ask, 2026-07-29): the SFLP yaw has no magnetometer
//     input, so its zero is an arbitrary power-on attitude that free-runs with
//     gyro drift. "Zero Yaw Here" sends `zero_yaw`, which captures the CURRENT
//     attitude as the new relative-yaw reference (server-side, via
//     `sensors.graft_yaw` -- see docs/web-protocol.md); "Clear" sends
//     `clear_yaw_offset` to go back to raw. Both echo through `state`'s
//     `yaw_offset_deg`. Disabled/no-op in World mode (i.e. always, now that
//     the card is pinned to it): World's yaw slot is the ABSOLUTE magnetic
//     heading, not offsettable.
// All draws/writes are guarded so a null-field message (a ToF-only or
// pre-calibration session, or before the jitter window has 2+ samples) renders
// placeholders and never throws.
//
// Everything else is read-only (pointer-events:none left rail); the label
// inputs + yaw-offset buttons are the exceptions (pointer-events:auto, see
// index.html's .sensor-label-input/.sensor-btn rules).
//
// Public surface:  createSensors(hub) -> {}
// Hub events:  subscribes "sensor", "state"; sends "set_orientation" (labels
//   only), "zero_yaw", "clear_yaw_offset"

const D = (m, l) => { try { window.__diag && window.__diag('sensors.js: ' + m, l); } catch (e) {} };

// fusion_key -> what to do about it (owner ask, 2026-07-31). Wording tracks the
// actual gates in sensors.py's YawFusion.update and their thresholds in
// config.py: gimbal margin 15 deg, motion 40 deg/s, anomaly 30% of the
// calibrated field. `active` isn't listed -- the caller hides the line then.
// The tripod note on `gated:anomaly` is not incidental: BUG-034 (by-design)
// measured the tripod alone adding 15-27 uT, which trips this gate on its own
// regardless of calibration quality.
const FUSION_HELP = {
    init: 'Waiting for the first valid magnetometer sample — hold steady a moment.',
    off: 'Yaw fusion is disabled in config — set [viewer] yaw_fusion = true.',
    'gated:no-cal': 'No magnetometer calibration loaded — run Calibrate Mag.',
    'gated:gimbal': 'Aimed within 15° of straight up/down, where yaw is undefined — tilt back toward horizontal.',
    'gated:motion': 'Turning faster than 40°/s — slow the sweep.',
    'gated:anomaly': 'Field strength is >30% off the calibrated value — move away from metal/magnets; take it off the tripod.',
};

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

// null/undefined -> em dash; else fixed-point degrees.
function fmtDeg(v, decimals) {
    return (v === null || v === undefined) ? '—' : v.toFixed(decimals) + '°';
}

// One jitter number (p95 or mean) from a {mean_deg, p95_deg, n} stat -> "0.032",
// or "—" before the window has 2+ samples (mean_deg/p95_deg null). The unit is
// carried by the "Jitter (deg/frame)" heading, not repeated on every cell — the
// old per-row "p95 0.032° · mean 0.021°" string wrapped all five rows.
function fmtJitterNum(stat, key) {
    const v = stat ? stat[key] : null;
    return (v === null || v === undefined) ? '—' : v.toFixed(3);
}

// fusion_key ("off" / "init" / "active" / "gated:...") -> the .sensor-state
// class that colours the readout: green fusing, amber gated, muted off.
function fusionClass(key) {
    if (key === 'active') return 'sensor-state is-active';
    if (typeof key === 'string' && key.startsWith('gated')) return 'sensor-state is-gated';
    if (key === 'init') return 'sensor-state';
    return 'sensor-state is-off';
}

export function createSensors(hub) {
    const $ = (id) => document.getElementById(id);
    const gizmo = $('sensor-gizmo');
    const compass = $('sensor-compass');
    const headingEl = $('sensor-heading');
    const fusionEl = $('sensor-fusion');
    const fusionHelpEl = $('sensor-fusion-help');
    const pressSpark = $('sensor-press-spark');
    const pressVal = $('sensor-press-val');
    const tempSpark = $('sensor-temp-spark');
    const tempVal = $('sensor-temp-val');
    const resetBtn = $('sensor-reset-heading');
    const rollEl = $('sensor-roll');
    const pitchEl = $('sensor-pitch');
    const yawEl = $('sensor-yaw');
    const headingRawEl = $('sensor-heading-raw');
    const quatEl = $('sensor-quat');
    // [p95 cell, mean cell] per signal — two columns of a grid, not one string.
    const jitterEls = {
        roll: [$('jitter-roll'), $('jitter-roll-mean')],
        pitch: [$('jitter-pitch'), $('jitter-pitch-mean')],
        yaw: [$('jitter-yaw'), $('jitter-yaw-mean')],
        heading: [$('jitter-heading'), $('jitter-heading-mean')],
        orientation: [$('jitter-orientation'), $('jitter-orientation-mean')],
    };
    const jitterLabelEls = {
        roll: $('jitter-label-roll'), pitch: $('jitter-label-pitch'), yaw: $('jitter-label-yaw'),
    };
    const labelInputs = [$('orient-label-0'), $('orient-label-1'), $('orient-label-2')];
    const valEls = [$('orient-val-0'), $('orient-val-1'), $('orient-val-2')];
    const singularityWarn = $('orient-singularity-warn');
    const worldWarn = $('orient-world-warn');
    const yawOffsetVal = $('orient-yaw-offset');
    const zeroYawBtn = $('orient-zero-yaw');
    const clearYawBtn = $('orient-clear-yaw-offset');
    if (!gizmo || !compass) { D('sensor DOM missing — skipping', 'error'); return {}; }

    // prime placeholders
    drawGizmo(gizmo, null);
    drawCompass(compass, null);
    drawSparkline(pressSpark, null);
    drawSparkline(tempSpark, null);

    if (resetBtn) {
        resetBtn.addEventListener('click', () => {
            hub.send({ type: 'reset_fusion' });
        });
    }

    // #sensor-mag-cal is owned by magcal.js (it enables the button and opens the
    // calibration modal); nothing to bind here.

    // --- Orientation View: renamable axis labels (World-only) ------------
    // One-way state flow (same as controls.js): a change here just SENDS
    // set_orientation {labels}; the displayed labels are driven from the
    // server's `state` echo below, not from local click state, so every
    // open tab stays in sync. There is no mode picker any more (owner ask,
    // 2026-07-31: the card is pinned to World) -- `set_orientation {mode}`
    // itself stays on the wire for the deprecated desktop panel.
    labelInputs.forEach((input, i) => {
        if (!input) return;
        const commit = () => {
            const labels = labelInputs.map((el, j) => (el ? el.value : ['Roll', 'Tilt', 'Heading'][j]));
            hub.send({ type: 'set_orientation', labels });
        };
        input.addEventListener('change', commit);   // blur / Enter
        input.addEventListener('keydown', (e) => { if (e.key === 'Enter') input.blur(); });
    });

    // --- Zero Yaw Here / Clear (owner ask, 2026-07-29) --------------------
    // One-way flow, same as the mode select above: just send the command,
    // let the `state` echo below drive the displayed offset/button state.
    if (zeroYawBtn) {
        zeroYawBtn.addEventListener('click', () => {
            hub.send({ type: 'zero_yaw' });
        });
    }
    if (clearYawBtn) {
        clearYawBtn.addEventListener('click', () => {
            hub.send({ type: 'clear_yaw_offset' });
        });
    }

    hub.on('state', (msg) => {
        if (Array.isArray(msg.orientation_labels)) {
            msg.orientation_labels.forEach((lbl, i) => {
                const input = labelInputs[i];
                if (input && document.activeElement !== input) input.value = lbl;
            });
            msg.orientation_labels.forEach((lbl, i) => {
                const keys = ['roll', 'pitch', 'yaw'];
                const el = jitterLabelEls[keys[i]];
                if (el) el.textContent = lbl;
            });
        }

        // Yaw offset readout + button state. World mode's heading is
        // ABSOLUTE (magnetic north) -- "Zero Yaw Here" is disabled there
        // rather than silently doing nothing, per the owner's requirement
        // that the exception be explicit in the UI, not just the code.
        const isWorld = msg.orientation_mode === 'world';
        const offset = msg.yaw_offset_deg;
        const hasOffset = typeof offset === 'number' && Math.abs(offset) > 1e-6;
        if (yawOffsetVal) {
            yawOffsetVal.textContent = isWorld
                ? 'n/a (absolute)'
                : (hasOffset ? offset.toFixed(2) + '°' : 'none');
        }
        if (zeroYawBtn) {
            zeroYawBtn.disabled = isWorld;
            zeroYawBtn.title = isWorld
                ? "World mode's heading is absolute magnetic north -- not offsettable"
                : 'Zero the displayed yaw at the current attitude';
        }
        if (clearYawBtn) {
            clearYawBtn.disabled = isWorld || !hasOffset;
        }
    });

    hub.on('sensor', (msg) => {
        try {
            drawGizmo(gizmo, msg.rot);
            drawCompass(compass, msg.heading);
            if (headingEl) headingEl.textContent =
                (msg.heading === null || msg.heading === undefined) ? '—' : msg.heading.toFixed(1) + '°';
            // Fusion help (owner ask, 2026-07-31): say what's wrong and how to
            // fix it, not just the state label. Hidden on `active` -- nothing
            // to say. Mirrored into the Fusion row's title too, so it reads the
            // same whether the reader looks at the row or the line under it.
            // Muted small text (.hud-note) normally; a real fault (any
            // `gated:*` key) gets the warning styling (.hud-warn) -- `init`/
            // `off` are transient/configuration states, not faults.
            const help = FUSION_HELP[msg.fusion_key] || '';
            const isFault = typeof msg.fusion_key === 'string' && msg.fusion_key.startsWith('gated');
            const fusionTitle = msg.fusion_key
                ? 'YawFusion status: ' + msg.fusion_key + (help ? ' — ' + help : '')
                : '';
            if (fusionEl) {
                fusionEl.textContent = msg.fusion || 'Off';
                fusionEl.className = fusionClass(msg.fusion_key);
                fusionEl.title = fusionTitle;
            }
            if (fusionHelpEl) {
                fusionHelpEl.textContent = help;
                fusionHelpEl.className = isFault ? 'hud-warn' : 'hud-note';
                fusionHelpEl.classList.toggle('hidden', !help);
                fusionHelpEl.title = fusionTitle;
            }
            if (pressVal) pressVal.textContent =
                (msg.pressure_pa === null || msg.pressure_pa === undefined) ? '—' : Math.round(msg.pressure_pa) + ' Pa';
            if (tempVal) tempVal.textContent =
                (msg.temp_c === null || msg.temp_c === undefined) ? '—' : msg.temp_c.toFixed(1) + ' °C';
            drawSparkline(pressSpark, msg.pressure_hist);
            drawSparkline(tempSpark, msg.temp_hist);
            if (resetBtn) resetBtn.disabled = (msg.fusion_key === 'off');

            // Raw orientation (full precision, pre-smoothing) + jitter.
            const or = msg.orientation_raw || {};
            if (rollEl) rollEl.textContent = fmtDeg(or.roll_deg, 3);
            if (pitchEl) pitchEl.textContent = fmtDeg(or.pitch_deg, 3);
            if (yawEl) yawEl.textContent = fmtDeg(or.yaw_deg, 3);
            if (headingRawEl) headingRawEl.textContent = fmtDeg(or.heading_deg, 3);
            if (quatEl) quatEl.textContent = Array.isArray(or.quat)
                ? or.quat.map((v) => v.toFixed(4)).join(', ') : '—';

            const j = msg.jitter || {};
            for (const [signal, [p95El, meanEl]] of Object.entries(jitterEls)) {
                if (p95El) p95El.textContent = fmtJitterNum(j[signal], 'p95_deg');
                if (meanEl) meanEl.textContent = fmtJitterNum(j[signal], 'mean_deg');
            }

            // Orientation View: selected-mode readout + labels + warnings.
            const ov = msg.orientation_view || {};
            const labels = Array.isArray(ov.labels) ? ov.labels : ['Roll', 'Pitch', 'Yaw'];
            const vals = [ov.roll_deg, ov.pitch_deg, ov.yaw_deg];
            valEls.forEach((el, i) => { if (el) el.textContent = fmtDeg(vals[i], 3); });
            labelInputs.forEach((input, i) => {
                if (input && document.activeElement !== input && labels[i] !== undefined) {
                    input.value = labels[i];
                }
            });
            if (singularityWarn) {
                const margin = ov.singularity_margin_deg;
                singularityWarn.classList.toggle('hidden', !ov.near_singularity);
                if (ov.near_singularity && margin !== null && margin !== undefined) {
                    singularityWarn.textContent =
                        `⚠ Near singularity — margin ${margin.toFixed(1)}°. Values unreliable.`;
                }
            }
            if (worldWarn) {
                const showWarn = ov.mode === 'world' && ov.valid === false;
                worldWarn.classList.toggle('hidden', !showWarn);
                if (showWarn) worldWarn.textContent = '⚠ ' + (ov.reason || 'World mode invalid');
            }

            if (!window.__gotSensor) { window.__gotSensor = true; D('first sensor frame'); }
        } catch (e) {
            D('sensor draw threw: ' + (e && e.message), 'error');
        }
    });

    return {};
}
