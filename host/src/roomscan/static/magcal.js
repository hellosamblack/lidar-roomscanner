// magcal.js — the Magnetometer Calibration modal (owner ask, 2026-07-29).
//
// Opened from the Sensors card's "Calibrate Mag" button. Purely additive and
// purely diagnostic: the modal is an overlay, the 3D scene keeps rendering and
// streaming underneath it, and nothing here can change the point cloud, SLAM,
// or the orientation the viewer uses. The one action with a side effect is
// Save, which is gated behind an explicit quality preview.
//
// WHY THIS EXISTS (read before changing the thresholds)
// -----------------------------------------------------
// A correctly calibrated magnetometer reports a CONSTANT |B| in every
// orientation. The calibration shipped on 2026-07-15 did not — measured across
// a tilt sweep it read 50 µT ceiling-facing and 85 µT horizontal (~1.7×),
// producing heading errors up to ~90° in exactly the attitude the scanner is
// used in. That is the signature of an incomplete tumble: dense coverage in one
// attitude family, none elsewhere. The ellipsoid fit happily interpolated a
// plausible ellipsoid through it and nothing downstream noticed for two weeks.
// So this modal does two things the old CLI didn't:
//   1. shows WHICH directions are still missing, live, while you tumble, so the
//      gap can be fixed in the moment rather than discovered a fortnight later;
//   2. measures |B| consistency of the result and makes you accept it before it
//      can overwrite the saved file.
//
// THE MAP
// -------
// Two Lambert azimuthal EQUAL-AREA discs — the sphere of body-frame field
// directions split at the device's Up axis. Equal-area matters: the whole point
// is judging how much of the sphere is missing, and any non-equal-area
// projection (or naive lat/lon binning) makes a gap's apparent size depend on
// where it sits, which is the exact judgement error that produced the bad
// calibration. Radius from the disc centre is sqrt(1 − |v_up|), which is the
// exact equal-area radial law, so the two discs together tile the sphere with
// no area distortion at all.
//
// Both discs are drawn as seen from the SAME viewpoint (looking down the −Up
// axis) so screen-right is the device's Right and screen-up is its Front in
// BOTH, with no mirrored disc to decode. The equator (rim) is shared: a point
// on one rim is the same direction as the matching point on the other.
//
// Every one of the 92 cells is drawn whether or not it has samples, so the full
// lattice IS the "ideal" target and the hollow rings ARE the missing angles.
// Filled/hollow is a shape encoding, not a colour one, so coverage survives any
// colour-vision deficiency.
//
// COLOUR
// ------
// Occupied cells are filled by mean |B| deviation under the selected
// calibration on a diverging blue↔neutral-grey↔red ramp (validated against the
// dark surface: CVD ΔE 10.4, normal-vision ΔE 19.0, contrast ≥3:1 — all pass).
// Neutral grey is the midpoint by design: "field magnitude correct" should be
// the quiet colour and error should be the loud one. Pointing the map at the
// SAVED calibration is how the defect above becomes a picture rather than a
// table.
//
// Public surface:  createMagcal(hub) -> {}
// Hub events:  subscribes "magcal"; sends {"type":"magcal","action":...}

const D = (m, l) => { try { window.__diag && window.__diag('magcal.js: ' + m, l); } catch (e) {} };

// Design tokens mirrored from index.html (canvas can't read CSS vars).
const INK = '#e2e8f0';
const MUTED = '#94a3b8';
const GRID = 'rgba(255,255,255,0.10)';
const SURFACE = '#16181e';        // the ring colour that separates overlapping marks
const EMPTY_STROKE = 'rgba(226,232,240,0.30)';

// Diverging ramp for |B| deviation. Two hues + a NEUTRAL grey midpoint (never a
// hue at the midpoint). Clamped at ±DEV_CLAMP so the shipped defect's +70%
// saturates rather than compressing everything else toward grey.
const DEV_COOL = [96, 165, 250];      // #60a5fa — |B| reads LOW
const DEV_MID = [100, 116, 139];      // #64748b — |B| correct
const DEV_WARM = [239, 68, 68];       // #ef4444 — |B| reads HIGH
const DEV_CLAMP = 30.0;               // percent

const VERDICT_COLORS = { good: '#10b981', marginal: '#f59e0b', bad: '#ef4444' };
const VERDICT_TEXT = { good: 'GOOD', marginal: 'MARGINAL', bad: 'BAD' };

// Disc geometry: the centres are spread far enough that the two discs' rim
// labels ("Right" on the left disc, "Left" on the right one) can't collide in
// the gutter between them.
const MAP_W = 468, MAP_H = 262, DISC_R = 94;
const DISC_CENTRES = [[102, 114], [366, 114]];   // [+Up hemisphere, −Up hemisphere]
const CELL_R = 8.5;

function lerpRgb(a, b, t) {
    return [Math.round(a[0] + (b[0] - a[0]) * t),
            Math.round(a[1] + (b[1] - a[1]) * t),
            Math.round(a[2] + (b[2] - a[2]) * t)];
}

// Signed deviation percent -> css colour on the diverging ramp.
function devColor(pct) {
    const t = Math.max(-1, Math.min(1, pct / DEV_CLAMP));
    const rgb = t < 0 ? lerpRgb(DEV_MID, DEV_COOL, -t) : lerpRgb(DEV_MID, DEV_WARM, t);
    return `rgb(${rgb[0]},${rgb[1]},${rgb[2]})`;
}

// Body-frame unit direction (x=Up, y=Right, z=Front) -> canvas point.
// Lambert azimuthal equal-area about the Up axis: r = sqrt(1 - |x|), which puts
// the pole at the centre and the equator on the rim of each disc with EXACTLY
// proportional area everywhere. Both discs are viewed from the same side, so
// screen-x is always the device's Right and screen-y is always its Front.
function project(v) {
    if (!Array.isArray(v) || v.length !== 3) return null;
    const upper = v[0] >= 0;
    const [cx, cy] = DISC_CENTRES[upper ? 0 : 1];
    const r = Math.sqrt(Math.max(0, 1 - Math.abs(v[0]))) * DISC_R;
    const h = Math.hypot(v[1], v[2]);
    const ay = h < 1e-9 ? 0 : v[1] / h;
    const az = h < 1e-9 ? 0 : v[2] / h;
    return { x: cx + ay * r, y: cy - az * r, upper };
}

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

function drawDiscFrame(ctx, cx, cy, title) {
    ctx.strokeStyle = GRID;
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.arc(cx, cy, DISC_R, 0, Math.PI * 2); ctx.stroke();
    // The 45°-tilt ring: r = sqrt(1 - cos45) of the rim radius. A reference
    // circle so "how far from the pole" is readable without a protractor.
    ctx.beginPath();
    ctx.arc(cx, cy, DISC_R * Math.sqrt(1 - Math.cos(Math.PI / 4)), 0, Math.PI * 2);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(cx - DISC_R, cy); ctx.lineTo(cx + DISC_R, cy);
    ctx.moveTo(cx, cy - DISC_R); ctx.lineTo(cx, cy + DISC_R);
    ctx.stroke();

    ctx.fillStyle = MUTED;
    ctx.font = '9px "Inter", sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('Front', cx, cy - DISC_R - 5);
    ctx.fillText('Back', cx, cy + DISC_R + 12);
    ctx.textAlign = 'left';
    ctx.fillText('Right', cx + DISC_R + 4, cy + 3);
    ctx.textAlign = 'right';
    ctx.fillText('Left', cx - DISC_R - 4, cy + 3);
    ctx.textAlign = 'center';
    ctx.fillStyle = INK;
    ctx.font = '10px "Inter", sans-serif';
    ctx.fillText(title, cx, MAP_H - 8);
}

// Draw the whole coverage map. `msg` is the latest `magcal` message.
function drawMap(canvas, msg) {
    const ctx = fitCanvas(canvas, MAP_W, MAP_H);
    drawDiscFrame(ctx, DISC_CENTRES[0][0], DISC_CENTRES[0][1], 'Field from ABOVE the device (+Up)');
    drawDiscFrame(ctx, DISC_CENTRES[1][0], DISC_CENTRES[1][1], 'Field from BELOW the device (−Up)');
    if (!msg || !Array.isArray(msg.cell_dirs)) return [];

    const counts = msg.cell_counts || [];
    const devs = msg.cell_dev_pct || [];
    const hits = [];
    for (let i = 0; i < msg.cell_dirs.length; i++) {
        const p = project(msg.cell_dirs[i]);
        if (!p) continue;
        hits.push({ i, x: p.x, y: p.y });
        const n = counts[i] || 0;
        ctx.beginPath();
        ctx.arc(p.x, p.y, CELL_R, 0, Math.PI * 2);
        if (n > 0) {
            const dev = devs[i];
            ctx.fillStyle = (dev === null || dev === undefined) ? 'rgba(148,163,184,0.75)' : devColor(dev);
            ctx.fill();
            // Surface-coloured ring: keeps adjacent filled cells from merging
            // into one blob at the dense pole of a disc.
            ctx.strokeStyle = SURFACE;
            ctx.lineWidth = 1.5;
            ctx.stroke();
        } else {
            // MISSING: hollow, dashed — a shape encoding, so it survives any
            // colour-vision deficiency and any map colouring mode.
            ctx.setLineDash([2, 2]);
            ctx.strokeStyle = EMPTY_STROKE;
            ctx.lineWidth = 1.2;
            ctx.stroke();
            ctx.setLineDash([]);
        }
    }

    // "You are here": the live field direction, so the user can steer into the
    // gaps instead of guessing. Drawn last, on top.
    const live = project(msg.live_dir);
    if (live) {
        ctx.strokeStyle = INK;
        ctx.lineWidth = 2;
        ctx.beginPath(); ctx.arc(live.x, live.y, CELL_R + 4, 0, Math.PI * 2); ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(live.x - CELL_R - 8, live.y); ctx.lineTo(live.x - CELL_R - 1, live.y);
        ctx.moveTo(live.x + CELL_R + 1, live.y); ctx.lineTo(live.x + CELL_R + 8, live.y);
        ctx.stroke();
    }
    return hits;
}

function drawLegend(canvas) {
    const W = MAP_W, H = 34;
    const ctx = fitCanvas(canvas, W, H);
    const barX = 8, barY = 6, barW = 190, barH = 10;
    for (let i = 0; i < barW; i++) {
        ctx.fillStyle = devColor(-DEV_CLAMP + (2 * DEV_CLAMP) * (i / (barW - 1)));
        ctx.fillRect(barX + i, barY, 1, barH);
    }
    ctx.strokeStyle = GRID; ctx.lineWidth = 1;
    ctx.strokeRect(barX + 0.5, barY + 0.5, barW - 1, barH - 1);
    ctx.fillStyle = MUTED;
    ctx.font = '9px "Inter", sans-serif';
    ctx.textAlign = 'left'; ctx.fillText('−30%', barX, barY + barH + 11);
    ctx.textAlign = 'center'; ctx.fillText('|B| correct', barX + barW / 2, barY + barH + 11);
    ctx.textAlign = 'right'; ctx.fillText('+30%', barX + barW, barY + barH + 11);

    // Key: covered vs missing — the shape encoding, spelled out.
    let x = barX + barW + 26;
    ctx.beginPath(); ctx.arc(x, barY + barH / 2, 6, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(148,163,184,0.75)'; ctx.fill();
    ctx.strokeStyle = SURFACE; ctx.lineWidth = 1.5; ctx.stroke();
    ctx.fillStyle = INK; ctx.font = '10px "Inter", sans-serif'; ctx.textAlign = 'left';
    ctx.fillText('covered', x + 11, barY + barH / 2 + 3);

    x += 76;
    ctx.beginPath(); ctx.arc(x, barY + barH / 2, 6, 0, Math.PI * 2);
    ctx.setLineDash([2, 2]); ctx.strokeStyle = EMPTY_STROKE; ctx.lineWidth = 1.2; ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = INK; ctx.fillText('missing', x + 11, barY + barH / 2 + 3);

    x += 76;
    ctx.strokeStyle = INK; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(x, barY + barH / 2, 6, 0, Math.PI * 2); ctx.stroke();
    ctx.fillStyle = INK; ctx.fillText('field now', x + 11, barY + barH / 2 + 3);
}

function num(v, digits, suffix) {
    return (v === null || v === undefined || !isFinite(v)) ? '—' : v.toFixed(digits) + (suffix || '');
}

function row(label, value, hint) {
    const div = document.createElement('div');
    div.className = 'magcal-row';
    const l = document.createElement('span');
    l.textContent = label;
    if (hint) l.title = hint;
    const v = document.createElement('span');
    v.className = 'magcal-row__val';
    v.textContent = value;
    div.append(l, v);
    return div;
}

function badge(verdict) {
    const b = document.createElement('span');
    b.className = 'magcal-badge';
    b.textContent = VERDICT_TEXT[verdict] || '—';
    b.style.color = VERDICT_COLORS[verdict] || MUTED;
    b.style.borderColor = VERDICT_COLORS[verdict] || MUTED;
    return b;
}

// Render one calibration's quality block. Components are ALWAYS shown next to
// the headline verdict — a single opaque score is what let the bad calibration
// through, so the breakdown is not optional here.
function renderQuality(el, q, emptyText) {
    el.textContent = '';
    if (!q) {
        const p = document.createElement('div');
        p.className = 'magcal-empty';
        p.textContent = emptyText;
        el.append(p);
        return;
    }
    const head = document.createElement('div');
    head.className = 'magcal-verdict';
    head.append(badge(q.verdict));
    const reason = document.createElement('span');
    reason.className = 'magcal-verdict__why';
    reason.textContent = q.reason || '';
    head.append(reason);
    el.append(head);

    const f = q.field;
    el.append(row('|B| spread (std/mean)',
        f ? num(f.std_pct, 2, '%') : '—',
        'THE headline metric: a correct calibration reports the same field magnitude in every '
        + 'orientation. good < 2%, marginal < 5%, bad ≥ 5%.'));
    el.append(row('|B| bias vs expected',
        f ? (f.bias_pct >= 0 ? '+' : '') + num(f.bias_pct, 1, '%') : '—',
        'Spread alone is not enough — a calibration can be perfectly self-consistent at completely '
        + 'the wrong magnitude. Same thresholds as spread; the field verdict is the worse of the two.'));
    el.append(row('|B| mean',
        f ? `${num(f.mean_ut, 2)} µT (expect ${num(f.expected_ut, 2)})` : '—'));
    el.append(row('|B| min / max',
        f ? `${num(f.min_ut, 1)} / ${num(f.max_ut, 1)} µT  (×${num(f.ratio, 2)})` : '—',
        'The ratio is what the 2026-07-29 tilt sweep exposed on the shipped calibration: ×1.7.'));
    el.append(row('Fit residual (RMS)', f ? num(f.residual_rms_ut, 2, ' µT') : '—',
        'RMS deviation of the calibrated samples from the sphere of radius field_ut.'));
    const c = q.coverage || {};
    el.append(row('Sphere coverage',
        c.fraction === undefined ? '—'
            : `${num(100 * c.fraction, 0, '%')}  (${c.occupied}/${c.cells}, ${c.empty} empty)`,
        'Fraction of equal-area sphere cells with at least one sample. '
        + 'good ≥ 85%, marginal ≥ 60%, bad < 60%.'));
    el.append(row('Samples', String(q.samples ?? '—'),
        'good ≥ 300, marginal ≥ 100. The ellipsoid fit needs at least 20 to be solvable.'));
}

export function createMagcal(hub) {
    const $ = (id) => document.getElementById(id);
    const modal = $('magcal-modal');
    const openBtn = $('sensor-mag-cal');
    if (!modal || !openBtn) { D('magcal DOM missing — skipping', 'error'); return {}; }

    const mapCanvas = $('magcal-map');
    const legendCanvas = $('magcal-legend');
    const tip = $('magcal-tip');
    const guidanceEl = $('magcal-guidance');
    const progressEl = $('magcal-progress');
    const progressBar = $('magcal-progress-bar');
    const statusEl = $('magcal-status');
    const currentEl = $('magcal-current');
    const candidateEl = $('magcal-candidate');
    const binningEl = $('magcal-binning');
    const pathEl = $('magcal-path');
    const btnStart = $('magcal-start');
    const btnStop = $('magcal-stop');
    const btnSave = $('magcal-save');
    const btnDiscard = $('magcal-discard');
    const btnClear = $('magcal-clear');
    const btnClose = $('magcal-close');
    const viewSeg = $('magcal-view-seg');

    let open = false;
    let last = null;
    let hits = [];

    // The button was a disabled placeholder until now.
    openBtn.disabled = false;
    openBtn.title = 'Magnetometer calibration: sweep coverage + quality';

    function setOpen(on) {
        open = on;
        modal.classList.toggle('hidden', !on);
        // The server only pays for (and only sends) reports while a tab has the
        // modal open, so the live view/SLAM path is untouched when it's closed.
        hub.send({ type: 'magcal', action: on ? 'open' : 'close' });
        if (on) { drawLegend(legendCanvas); redraw(); }
    }

    openBtn.addEventListener('click', () => setOpen(true));
    btnClose.addEventListener('click', () => setOpen(false));
    modal.addEventListener('click', (e) => { if (e.target === modal) setOpen(false); });
    document.addEventListener('keydown', (e) => { if (open && e.key === 'Escape') setOpen(false); });

    btnStart.addEventListener('click', () => hub.send({ type: 'magcal', action: 'start' }));
    btnStop.addEventListener('click', () => hub.send({ type: 'magcal', action: 'stop' }));
    btnSave.addEventListener('click', () => hub.send({ type: 'magcal', action: 'save' }));
    btnDiscard.addEventListener('click', () => hub.send({ type: 'magcal', action: 'discard' }));
    btnClear.addEventListener('click', () => hub.send({ type: 'magcal', action: 'reset' }));
    viewSeg.addEventListener('click', (e) => {
        const b = e.target.closest('button[data-cal]');
        if (b) hub.send({ type: 'magcal', action: 'view', cal: b.dataset.cal });
    });

    // Per-cell hover tooltip: the map is a cell chart, so each mark answers for
    // itself (sample count + the |B| error that coloured it) instead of leaving
    // the colour to be eyeballed against the legend.
    mapCanvas.addEventListener('mousemove', (e) => {
        if (!last) return;
        const r = mapCanvas.getBoundingClientRect();
        const mx = e.clientX - r.left, my = e.clientY - r.top;
        let best = null, bestD = (CELL_R + 3) * (CELL_R + 3);
        for (const h of hits) {
            const d = (h.x - mx) * (h.x - mx) + (h.y - my) * (h.y - my);
            if (d < bestD) { bestD = d; best = h; }
        }
        if (!best) { tip.classList.add('hidden'); return; }
        const n = (last.cell_counts || [])[best.i] || 0;
        const dev = (last.cell_dev_pct || [])[best.i];
        tip.textContent = n === 0
            ? `cell ${best.i} — MISSING (no samples)`
            : `cell ${best.i} — ${n} sample${n === 1 ? '' : 's'}`
              + (dev === null || dev === undefined ? '' : `, |B| ${dev >= 0 ? '+' : ''}${dev.toFixed(1)}%`);
        // Flip to the left of the cursor near the right edge, so the tooltip
        // never escapes the map column and lands on the metrics text.
        tip.classList.remove('hidden');
        const flip = best.x + 12 + tip.offsetWidth > MAP_W;
        tip.style.left = Math.round(flip ? best.x - 12 - tip.offsetWidth : best.x + 12) + 'px';
        tip.style.top = Math.round(best.y - 6) + 'px';
    });
    mapCanvas.addEventListener('mouseleave', () => tip.classList.add('hidden'));

    function redraw() {
        hits = drawMap(mapCanvas, last);
        if (!last) return;
        const cov = ((last.current || last.candidate || {}).coverage) || {};
        const occupied = cov.occupied !== undefined ? cov.occupied
            : (last.cell_counts || []).filter((v) => v > 0).length;
        const cells = last.cells || 0;
        const pct = cells ? (100 * occupied / cells) : 0;
        if (progressEl) {
            progressEl.textContent =
                `Coverage ${occupied} / ${cells} cells (${pct.toFixed(0)}%) · `
                + `${last.sample_count} samples · ${last.elapsed_s.toFixed(0)} s`;
        }
        if (progressBar) progressBar.style.width = pct.toFixed(1) + '%';
        if (guidanceEl) guidanceEl.textContent = last.guidance || '';
        if (binningEl) {
            const b = last.binning;
            binningEl.textContent = b === 'provisional'
                ? 'No saved calibration yet — the map is binned with a provisional hard-iron estimate from your samples.'
                : b === 'raw'
                    ? 'Not enough samples yet to bin directions reliably — keep tumbling.'
                    : `Directions binned with the ${b} calibration.`;
        }
        if (pathEl) pathEl.textContent = last.saved_path || '';

        // Both quality blocks are measured against THIS session's samples, so
        // the empty state has to distinguish "no calibration exists" from
        // "nothing measured yet" — reporting a verdict for either would be a
        // claim we haven't earned.
        renderQuality(currentEl, last.current,
            !last.has_current
                ? 'No calibration saved yet — the heading fusion is running uncalibrated.'
                : `Field ${last.current_field_ut} µT. Collect samples to measure how consistent it really is.`);
        renderQuality(candidateEl, last.candidate,
            last.fit_error ? `Fit failed: ${last.fit_error}` : 'Stop collection to fit a candidate.');

        for (const b of viewSeg.querySelectorAll('button[data-cal]')) {
            b.classList.toggle('active', b.dataset.cal === last.view);
        }
        btnStart.disabled = last.collecting;
        btnStart.textContent = last.sample_count && !last.collecting ? 'Resume' : 'Start';
        btnStop.disabled = last.sample_count < 20;
        btnStop.textContent = last.collecting ? 'Stop & Fit' : 'Fit';
        btnSave.disabled = !last.has_candidate;
        btnDiscard.disabled = !last.has_candidate;
        btnClear.disabled = last.sample_count === 0;
        if (statusEl) {
            statusEl.textContent = last.collecting
                ? 'Collecting — tumble the device slowly through every orientation.'
                : last.has_candidate
                    ? 'Preview the candidate above, then Save to apply it — or Discard and keep tumbling into the same samples.'
                    : last.sample_count
                        ? 'Paused. Resume to add more samples to the same cloud, or Stop & Fit to try a fit.'
                        : 'Press Start, then rotate the device through every orientation you can reach.';
        }
    }

    hub.on('magcal', (msg) => {
        try {
            last = msg;
            if (open) redraw();
            if (!window.__gotMagcal) { window.__gotMagcal = true; D('first magcal report'); }
        } catch (e) {
            D('magcal draw threw: ' + (e && e.message), 'error');
        }
    });

    D('modal wired');
    return {};
}
