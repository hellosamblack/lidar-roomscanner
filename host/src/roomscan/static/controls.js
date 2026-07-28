// controls.js — the right-rail interactive control panel (§8.1 right rail).
//
// Turns DOM events into hub.send(...) messages and nothing else. Active state for
// anything the server tracks (color mode, IR colormap/freeze) is driven FROM the
// server's `state` echo, never from local click state — one-way state flow (§8.3),
// so a change in one tab reflects in every open tab for free. Usecase and the
// device buttons are one-shot actions (not persistent server state) so they just
// fire a `cmd`; their result surfaces as a toast/log line via log.js.
//
// Also owns generic control-group collapse (delegated header clicks) and the IR
// card show/hide toggle (a local presentation signal relayed over the hub).
//
// Public surface:  createControls(hub) -> {}
// Hub events:  subscribes "state", "ir_shown";  emits "reset_camera", "ir_show";
//              sends cmd / set_color / set_ir via hub.send

export function createControls(hub) {
    const $ = (id) => document.getElementById(id);

    // --- Device group: one-shot commands ---
    const cmd = (name, param = 0) => hub.send({ type: 'cmd', name, param });
    $('btn-ping')?.addEventListener('click', () => cmd('ping'));
    $('btn-calib')?.addEventListener('click', () => cmd('calib'));
    $('btn-reinit')?.addEventListener('click', () => cmd('reinit'));

    // --- Usecase segmented control (action, not persistent state) ---
    const segUsecase = $('seg-usecase');
    segUsecase?.addEventListener('click', (e) => {
        const btn = e.target.closest('button[data-uc]');
        if (btn) cmd('usecase', parseInt(btn.dataset.uc, 10));
    });

    // --- View group: color mode segmented control (server-driven active) ---
    const segColor = $('seg-color');
    segColor?.addEventListener('click', (e) => {
        const btn = e.target.closest('button[data-mode]');
        if (btn) hub.send({ type: 'set_color', mode: btn.dataset.mode });
    });
    $('btn-reset-cam')?.addEventListener('click', () => hub.emit('reset_camera'));

    // View: colormap, point size, render mode, surface settings
    const segViewColormap = $('seg-view-colormap');
    const slPointSize = $('sl-point-size');
    const chkPointAuto = $('chk-point-auto');
    const pointSizeVal = $('point-size-val');
    const segRender = $('seg-render');
    const surfaceOpts = $('surface-opts');
    const segSurfaceMode = $('seg-surface-mode');
    const slSurfaceThreshold = $('sl-surface-threshold');
    const surfaceThresholdVal = $('surface-threshold-val');

    function sendView(fields) { hub.send({ type: 'set_view', ...fields }); }

    segViewColormap?.addEventListener('click', (e) => {
        const btn = e.target.closest('button[data-colormap]');
        if (btn) sendView({ colormap: btn.dataset.colormap });
    });
    slPointSize?.addEventListener('input', () => {
        sendView({ point_size: parseFloat(slPointSize.value) });
    });
    chkPointAuto?.addEventListener('change', () => {
        sendView({ point_size_auto: chkPointAuto.checked });
    });
    segRender?.addEventListener('click', (e) => {
        const btn = e.target.closest('button[data-render]');
        if (btn) sendView({ surface: btn.dataset.render === 'surface' });
    });
    segSurfaceMode?.addEventListener('click', (e) => {
        const btn = e.target.closest('button[data-smode]');
        if (btn) sendView({ surface_mode: btn.dataset.smode });
    });
    slSurfaceThreshold?.addEventListener('input', () => {
        sendView({ surface_threshold: parseFloat(slSurfaceThreshold.value) });
    });

    // --- IR Monitor group: colormap + freeze (server-driven) + card show/hide ---
    const segIrColormap = $('seg-ir-colormap');
    const chkIrFreeze = $('chk-ir-freeze');
    const chkIrShow = $('chk-ir-show');

    function sendIr(colormap, freeze) { hub.send({ type: 'set_ir', colormap, freeze }); }
    segIrColormap?.addEventListener('click', (e) => {
        const btn = e.target.closest('button[data-colormap]');
        if (btn) sendIr(btn.dataset.colormap, chkIrFreeze ? chkIrFreeze.checked : false);
    });
    chkIrFreeze?.addEventListener('change', () => {
        const active = segIrColormap && segIrColormap.querySelector('button.active');
        sendIr(active ? active.dataset.colormap : 'gray', chkIrFreeze.checked);
    });
    // Card show/hide is local presentation; relay it to ir.js over the hub.
    chkIrShow?.addEventListener('change', () => hub.emit('ir_show', chkIrShow.checked));
    hub.on('ir_shown', (v) => { if (chkIrShow) chkIrShow.checked = !!v; });   // card's own close btn

    // --- server state echo drives active segments (§7.2) ---
    hub.on('state', (msg) => {
        setActive(segColor, 'mode', msg.color_mode);
        setActive(segIrColormap, 'colormap', msg.ir_colormap);
        if (chkIrFreeze) chkIrFreeze.checked = !!msg.ir_freeze;
        // View: colormap, point size, render mode, surface
        setActive(segViewColormap, 'colormap', msg.view_colormap);
        if (slPointSize && msg.point_size !== undefined) slPointSize.value = msg.point_size;
        if (chkPointAuto && msg.point_size_auto !== undefined) chkPointAuto.checked = !!msg.point_size_auto;
        if (pointSizeVal && msg.point_size !== undefined) {
            // In auto the slider is a per-metre-of-range gain, so say so.
            const auto = chkPointAuto ? chkPointAuto.checked : false;
            pointSizeVal.textContent = msg.point_size.toFixed(3) + ' m' + (auto ? ' @1 m' : '');
        }
        setActive(segRender, 'render', msg.surface_enabled ? 'surface' : 'points');
        if (surfaceOpts) surfaceOpts.classList.toggle('hidden', !msg.surface_enabled);
        setActive(segSurfaceMode, 'smode', msg.surface_mode);
        if (slSurfaceThreshold && msg.surface_threshold_pct !== undefined) {
            slSurfaceThreshold.value = msg.surface_threshold_pct;
        }
        if (surfaceThresholdVal && msg.surface_threshold_pct !== undefined) {
            surfaceThresholdVal.textContent = msg.surface_threshold_pct.toFixed(1) + '%';
        }
    });

    function setActive(seg, attr, value) {
        if (!seg) return;
        for (const b of seg.querySelectorAll('button')) {
            b.classList.toggle('active', b.dataset[attr] === value);
        }
    }

    // --- generic control-group collapse (delegated on the right rail) ---
    const rail = $('right-rail');
    rail?.addEventListener('click', (e) => {
        const header = e.target.closest('.control-group__header');
        if (header && rail.contains(header)) {
            header.parentElement.classList.toggle('collapsed');
        }
    });

    return {};
}
