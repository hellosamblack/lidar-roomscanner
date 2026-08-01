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

    // View: view mode, colormap, point size, render mode, surface settings
    const segViewMode = $('seg-view-mode');
    const segViewColormap = $('seg-view-colormap');
    const slPointSize = $('sl-point-size');
    const chkPointAuto = $('chk-point-auto');
    const pointSizeVal = $('point-size-val');
    const slSeeThrough = $('sl-see-through');
    const seeThroughVal = $('see-through-val');
    const segRender = $('seg-render');
    const surfaceOpts = $('surface-opts');
    const segSurfaceMode = $('seg-surface-mode');
    const slSurfaceThreshold = $('sl-surface-threshold');
    const surfaceThresholdVal = $('surface-threshold-val');

    function sendView(fields) { hub.send({ type: 'set_view', ...fields }); }

    segViewMode?.addEventListener('click', (e) => {
        const btn = e.target.closest('button[data-viewmode]');
        if (btn) sendView({ view_mode: btn.dataset.viewmode });
    });

    // Camera framing — the three sliders always edit the SELECTED view mode's
    // own values (the server applies them to `ui.view_mode`), so switching mode
    // swaps the whole set. Values come back on the `state` echo like everything
    // else; nothing is applied optimistically.
    const slCamDistance = $('sl-cam-distance');
    const slCamHeight = $('sl-cam-height');
    const slCamRotation = $('sl-cam-rotation');
    const camModeVal = $('cam-mode-val');
    const camDistanceVal = $('cam-distance-val');
    const camHeightVal = $('cam-height-val');
    const camRotationVal = $('cam-rotation-val');
    const CAM_MODE_LABELS = { world: 'World', fpv: 'FPV', mirror: 'Mirror' };

    slCamDistance?.addEventListener('input', () => sendView({ cam_distance: parseFloat(slCamDistance.value) }));
    slCamHeight?.addEventListener('input', () => sendView({ cam_height: parseFloat(slCamHeight.value) }));
    slCamRotation?.addEventListener('input', () => sendView({ cam_rotation: parseFloat(slCamRotation.value) }));
    $('btn-cam-reset')?.addEventListener('click', () => sendView({ cam_reset: true }));

    // Auto-orbit (World only).
    const chkOrbit = $('chk-orbit');
    const slOrbitSpeed = $('sl-orbit-speed');
    const orbitSpeedVal = $('orbit-speed-val');
    chkOrbit?.addEventListener('change', () => sendView({ orbit: chkOrbit.checked }));
    slOrbitSpeed?.addEventListener('input', () => sendView({ orbit_speed: parseFloat(slOrbitSpeed.value) }));

    // Oscillate mode (owner ask, 2026-07-31): same World-only gate as the two
    // controls above, driven from the `state` echo below, never local clicks.
    const segOrbitMode = $('seg-orbit-mode');
    const slOrbitAmplitude = $('sl-orbit-amplitude');
    const orbitAmplitudeVal = $('orbit-amplitude-val');
    segOrbitMode?.addEventListener('click', (e) => {
        const btn = e.target.closest('button[data-orbitmode]');
        if (btn) sendView({ orbit_mode: btn.dataset.orbitmode });
    });
    slOrbitAmplitude?.addEventListener('input', () => sendView({ orbit_amplitude: parseFloat(slOrbitAmplitude.value) }));
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
    slSeeThrough?.addEventListener('input', () => {
        sendView({ see_through: parseFloat(slSeeThrough.value) });
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

    // The IR Monitor group lived here and duplicated the colormap + freeze
    // controls the IR card already carries in its own header. Removed
    // 2026-07-31; `ir.js` owns those controls and their `set_ir`/state echo.

    // --- server state echo drives active segments (§7.2) ---
    hub.on('state', (msg) => {
        setActive(segColor, 'mode', msg.color_mode);
        // View: view mode, colormap, point size, render mode, surface
        setActive(segViewMode, 'viewmode', msg.view_mode);
        // World / FPV / Mirror is shared by Point cloud, Preview, SLAM and
        // Detailed; never disable it merely because the display changed.
        // Camera framing for whichever mode is selected.
        const cam = msg.view_cam && msg.view_cam[msg.view_mode];
        if (cam) {
            if (camModeVal) camModeVal.textContent = CAM_MODE_LABELS[msg.view_mode] || msg.view_mode;
            if (slCamDistance) slCamDistance.value = cam.distance_m;
            if (slCamHeight) slCamHeight.value = cam.height_m;
            if (slCamRotation) slCamRotation.value = cam.rotation_deg;
            if (camDistanceVal) camDistanceVal.textContent = cam.distance_m.toFixed(2) + ' m';
            if (camHeightVal) camHeightVal.textContent = cam.height_m.toFixed(2) + ' m';
            if (camRotationVal) camRotationVal.textContent = Math.round(cam.rotation_deg) + '°';
        }
        if (chkOrbit && msg.orbit_enabled !== undefined) chkOrbit.checked = !!msg.orbit_enabled;
        if (msg.orbit_speed_deg_s !== undefined) {
            if (slOrbitSpeed) slOrbitSpeed.value = msg.orbit_speed_deg_s;
            if (orbitSpeedVal) orbitSpeedVal.textContent = msg.orbit_speed_deg_s.toFixed(1) + '°/s';
        }
        setActive(segOrbitMode, 'orbitmode', msg.orbit_mode);
        if (msg.orbit_amplitude_deg !== undefined) {
            if (slOrbitAmplitude) slOrbitAmplitude.value = msg.orbit_amplitude_deg;
            if (orbitAmplitudeVal) orbitAmplitudeVal.textContent = Math.round(msg.orbit_amplitude_deg) + '°';
        }
        // Orbiting only means something in World — a locked view has nothing to
        // circle, so grey the controls out rather than let them look armed.
        const worldOnly = msg.view_mode === 'world';
        if (chkOrbit) chkOrbit.disabled = !worldOnly;
        if (slOrbitSpeed) slOrbitSpeed.disabled = !worldOnly;
        if (segOrbitMode) for (const b of segOrbitMode.querySelectorAll('button')) b.disabled = !worldOnly;
        if (slOrbitAmplitude) slOrbitAmplitude.disabled = !worldOnly;
        setActive(segViewColormap, 'colormap', msg.view_colormap);
        if (slPointSize && msg.point_size !== undefined) slPointSize.value = msg.point_size;
        if (chkPointAuto && msg.point_size_auto !== undefined) chkPointAuto.checked = !!msg.point_size_auto;
        if (pointSizeVal && msg.point_size !== undefined) {
            // In auto the slider is a per-metre-of-range gain, so say so.
            const auto = chkPointAuto ? chkPointAuto.checked : false;
            pointSizeVal.textContent = msg.point_size.toFixed(3) + ' m' + (auto ? ' @1 m' : '');
        }
        if (msg.see_through !== undefined) {
            if (slSeeThrough) slSeeThrough.value = msg.see_through;
            if (seeThroughVal) {
                seeThroughVal.textContent = msg.see_through > 0
                    ? Math.round(msg.see_through * 100) + '%' : 'Off';
            }
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

    return {};
}
