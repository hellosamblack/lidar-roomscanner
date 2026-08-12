// controls.js — the right-rail interactive control panel (§8.1 right rail).
//
// Turns DOM events into hub.send(...) messages and nothing else. Active state for
// anything the server tracks (color mode, IR colormap/freeze) is driven FROM the
// server's `state` echo, never from local click state — one-way state flow (§8.3),
// so a change in one tab reflects in every open tab for free. The device buttons
// are one-shot actions (not persistent server state) so they just fire a `cmd`;
// their result surfaces as a toast/log line via log.js.
//
// Ranging profiles / manual sensor control / IMU-env poll rate (Task 10) are
// driven by their own `ranging` echo, a server-owned state independent of
// `state` (two independent pending commands, see the section below).
//
// Also owns generic control-group collapse (delegated header clicks) and the IR
// card show/hide toggle (a local presentation signal relayed over the hub).
//
// Public surface:  createControls(hub) -> {}
// Hub events:  subscribes "state", "ranging", "ir_shown";  emits "reset_camera", "ir_show";
//              sends cmd / set_profile / set_manual_params / set_imu_env_rate /
//              set_color / set_ir via hub.send

export function createControls(hub) {
    const $ = (id) => document.getElementById(id);

    // --- Device group: one-shot commands ---
    const cmd = (name, param = 0) => hub.send({ type: 'cmd', name, param });
    $('btn-ping')?.addEventListener('click', () => cmd('ping'));
    $('btn-calib')?.addEventListener('click', () => cmd('calib'));
    $('btn-reinit')?.addEventListener('click', () => cmd('reinit'));

    // --- Ranging profile / manual sensor control / IMU-env poll rate (Task 10) ---
    //
    // Two independent server-owned pending commands (RangingState / ImuEnvRateState
    // in web.py), each with its own `pending` flag -- a change while one is in
    // flight is REJECTED here (disabled controls), never queued. Every "active
    // segment" that mirrors applied DEVICE state (the profile selector, and the
    // Coupled/Explicit toggle's `coupled` half) is driven purely by the `ranging`
    // echo, one-way flow, same rule as every other segmented control in this file.
    //
    // The Manual panel's own ranging-mode/power-mode segments and the IMU/env
    // Explicit-Hz slider are the one deliberate exception: they are INPUT widgets
    // composing the next `set_manual_params`/`set_imu_env_rate` request, not
    // read-outs of applied state, so they hold a local selection between edits --
    // seeded from the server's applied config whenever nothing is pending, exactly
    // like the View card's camera-framing sliders already do.
    const PROFILE_LABELS = {
        stability: 'Stability', precision: 'Precision',
        high_framerate: 'High Frame Rate', manual: 'Manual',
    };
    // Short benefit blurb shown under the selector once a preset is applied. Mirrors
    // profiles.PRESET_DESCRIPTIONS (kept in step by test_static_ui). Presentation copy.
    const PROFILE_BLURBS = {
        stability: 'Steadiest frame rate — the most reliable tracking, with the fewest dropped ' +
            'frames, at the same close-range accuracy as longer exposures. The all-round default.',
        precision: 'Most light per frame — best on dark or distant surfaces. The longest exposure ' +
            'that still holds the full frame rate, for the lowest jitter and the most range ' +
            'headroom; runs closest to the frame-rate limit.',
        high_framerate: 'Highest frame rate for fast motion and quick sweeps — the short exposure ' +
            'also cuts motion blur. Less light per frame, so dim or dark surfaces read better on ' +
            'Stability or Precision.',
        manual: 'Full manual control of ranging mode, frame rate, exposure and power. The fps ' +
            'slider greys out the rates the current exposure cannot deliver.',
    };
    const MANUAL_DEBOUNCE_MS = 300;

    const segRangingProfile = $('seg-ranging-profile');
    const rangingManualPanel = $('ranging-manual-panel');
    const rangingAppliedVal = $('ranging-applied-val');
    const rangingRequestedStatus = $('ranging-requested-status');
    const rangingErrorEl = $('ranging-error');
    const rangingRangeVal = $('ranging-range-val');
    const rangingPowerVal = $('ranging-power-val');
    const rangingExpectedFpsVal = $('ranging-expected-fps-val');
    const rangingMeasuredVal = $('ranging-measured-val');
    const rangingI3cFill = $('ranging-i3c-fill');
    const rangingI3cCaption = $('ranging-i3c-caption');
    const rangingCdcWarning = $('ranging-cdc-warning');
    const rangingEstimateWarnings = $('ranging-estimate-warnings');

    const segManualRangingMode = $('seg-manual-ranging-mode');
    const segManualPower = $('seg-manual-power');
    const slManualFps = $('sl-manual-fps');
    const numManualFps = $('num-manual-fps');
    const manualFpsVal = $('manual-fps-val');
    const slManualExposure = $('sl-manual-exposure');
    const numManualExposure = $('num-manual-exposure');
    const manualExposureVal = $('manual-exposure-val');
    const rangingBlurb = $('ranging-blurb');
    const fpsCap = $('fps-cap');
    const fpsCapOk = $('fps-cap-ok');
    const fpsCapNo = $('fps-cap-no');
    const fpsCapNote = $('fps-cap-note');

    // Measured 1x fps ceiling per exposure (profiles._MEASURED_CEILING_FPS, on-rig
    // 2026-08-05). Kept in step with the Python table by test_static_ui. Mirrored here
    // so the fps grey-out tracks the exposure LIVE while editing -- the server's
    // est.ceiling_fps only follows the last APPLIED config, one commit behind.
    const CEILING_FPS_TABLE = [[2, 48], [4, 46], [6, 42], [8, 40], [10, 36],
        [12, 34], [14, 31], [15, 30], [16, 29]];
    function ceilingForExposure(exp) {
        const t = CEILING_FPS_TABLE;
        if (!(exp > 0)) return 100;
        if (exp <= t[0][0]) return t[0][1];
        for (let i = 0; i < t.length; i++) if (exp === t[i][0]) return t[i][1];
        for (let i = 0; i < t.length - 1; i++) {
            const [e0, c0] = t[i], [e1, c1] = t[i + 1];
            if (exp > e0 && exp < e1) {
                const f0 = 1000 / c0, f1 = 1000 / c1;
                const f = f0 + (exp - e0) / (e1 - e0) * (f1 - f0);
                return Math.floor(1000 / f + 1e-9);
            }
        }
        const [e0, c0] = t[t.length - 2], [e1, c1] = t[t.length - 1];
        const f0 = 1000 / c0, f1 = 1000 / c1, slope = (f1 - f0) / (e1 - e0);
        return Math.floor(1000 / (f1 + slope * (exp - e1)) + 1e-9);
    }
    // Paint the fps cap bar for `ceiling` and clamp the fps SLIDER out of the greyed
    // range (the number box still accepts an over-ceiling value for power users -- it
    // is applicable, just quantized, and the estimate warns). fps min is 1, max 100.
    function applyFpsCeiling(ceiling) {
        const fpsMin = 1, fpsMax = 100;
        const okFrac = Math.max(0, Math.min(1, (ceiling - fpsMin) / (fpsMax - fpsMin)));
        if (fpsCapOk) fpsCapOk.style.flex = `${okFrac}`;
        if (fpsCapNo) fpsCapNo.style.flex = `${1 - okFrac}`;
        const show = ceiling < fpsMax;
        if (fpsCap) fpsCap.hidden = !show;
        if (fpsCapNote) {
            fpsCapNote.hidden = !show;
            fpsCapNote.textContent = show ? `Max ${ceiling} fps at this exposure` : '';
        }
        if (slManualFps && Number(slManualFps.value) > ceiling) {
            slManualFps.value = ceiling;
            if (numManualFps && Number(numManualFps.value) > ceiling) numManualFps.value = ceiling;
            if (manualFpsVal) manualFpsVal.textContent = ceiling;
        }
    }
    // The ceiling to enforce is driven by the exposure currently in the box.
    function currentFpsCeiling() { return ceilingForExposure(Number(numManualExposure?.value)); }

    const segCalibrated = $('seg-calibrated');
    const calibratedNote = $('calibrated-note');

    const segImuEnvMode = $('seg-imu-env-mode');
    const slImuEnvRate = $('sl-imu-env-rate');
    const numImuEnvRate = $('num-imu-env-rate');
    const imuEnvAppliedVal = $('imu-env-applied-val');
    const imuEnvSubsampleWarning = $('imu-env-subsample-warning');
    const imuEnvError = $('imu-env-error');

    let rangingPending = false;
    let imuEnvPending = false;
    let manualRangingMode = 'ambient';
    let manualPowerMode = 'ulp';
    let imuEnvMode = 'coupled';
    let manualDebounce = null;
    let imuEnvDebounce = null;
    // A manual fps/exposure/mode edit is held locally until the device confirms it.
    // The server re-broadcasts `ranging` every ~250ms (faster than MANUAL_DEBOUNCE_MS),
    // so re-seeding the manual inputs from applied state on every echo would revert an
    // in-progress edit before it is ever sent (BUG-079). `manualDirty` suppresses that
    // re-seed while an edit is outstanding; it is cleared only when a command actually
    // completes (a pending true -> false transition).
    let manualDirty = false;
    let prevRangingPending = false;
    // A manual number field is being edited as long as it holds focus. The ~4 Hz
    // `ranging` echo must not overwrite a half-typed value: `manualDirty` alone did
    // NOT cover this because it is set only when an edit is COMMITTED (on `change`,
    // i.e. blur/Enter), so while the user is mid-typing "25" the echo re-seeded the
    // field to the applied value right after the "2" -- clearing the entry and
    // forcing a re-click (BUG-081). While a field has focus, suppress the re-seed.
    let manualEditing = false;

    function setSegActive(seg, attr, value) {
        if (!seg) return;
        for (const b of seg.querySelectorAll('button')) {
            b.classList.toggle('active', b.dataset[attr] === value);
        }
    }

    function sendManualParamsDebounced() {
        manualDirty = true;   // hold the local edit against the periodic re-seed (BUG-079)
        if (manualDebounce) clearTimeout(manualDebounce);
        manualDebounce = setTimeout(() => {
            manualDebounce = null;
            if (rangingPending) return;   // landed while a command was already in flight
            hub.send({
                type: 'set_manual_params',
                ranging_mode: manualRangingMode,
                fps: parseInt(numManualFps.value, 10),
                exposure_ms: parseInt(numManualExposure.value, 10),
                power_mode: manualPowerMode,
            });
        }, MANUAL_DEBOUNCE_MS);
    }

    segRangingProfile?.addEventListener('click', (e) => {
        const btn = e.target.closest('button[data-profile]');
        if (!btn || rangingPending) return;
        const profile = btn.dataset.profile;
        if (profile === 'manual') {
            // Progressive disclosure only -- opening the panel sends nothing.
            if (rangingManualPanel) rangingManualPanel.open = true;
            return;
        }
        hub.send({ type: 'set_profile', profile });
    });

    segManualRangingMode?.addEventListener('click', (e) => {
        const btn = e.target.closest('button[data-ranging-mode]');
        if (!btn || rangingPending) return;
        manualRangingMode = btn.dataset.rangingMode;
        setSegActive(segManualRangingMode, 'rangingMode', manualRangingMode);
        sendManualParamsDebounced();
    });

    segManualPower?.addEventListener('click', (e) => {
        const btn = e.target.closest('button[data-power-mode]');
        if (!btn || rangingPending) return;
        manualPowerMode = btn.dataset.powerMode;
        setSegActive(segManualPower, 'powerMode', manualPowerMode);
        sendManualParamsDebounced();
    });

    slManualFps?.addEventListener('input', () => {
        if (rangingPending) return;
        // The slider cannot enter the greyed range: clamp to the current exposure's
        // measured 1x ceiling (the number box below stays free for power users).
        const cap = currentFpsCeiling();
        if (Number(slManualFps.value) > cap) slManualFps.value = cap;
        if (numManualFps) numManualFps.value = slManualFps.value;
        if (manualFpsVal) manualFpsVal.textContent = slManualFps.value;
        sendManualParamsDebounced();
    });
    numManualFps?.addEventListener('change', () => {
        if (rangingPending) return;
        if (slManualFps) slManualFps.value = numManualFps.value;
        if (manualFpsVal) manualFpsVal.textContent = numManualFps.value;
        sendManualParamsDebounced();
    });
    slManualExposure?.addEventListener('input', () => {
        if (rangingPending) return;
        if (numManualExposure) numManualExposure.value = slManualExposure.value;
        if (manualExposureVal) manualExposureVal.textContent = slManualExposure.value + ' ms';
        applyFpsCeiling(ceilingForExposure(Number(slManualExposure.value)));
        sendManualParamsDebounced();
    });
    numManualExposure?.addEventListener('change', () => {
        if (rangingPending) return;
        if (slManualExposure) slManualExposure.value = numManualExposure.value;
        if (manualExposureVal) manualExposureVal.textContent = numManualExposure.value + ' ms';
        applyFpsCeiling(ceilingForExposure(Number(numManualExposure.value)));
        sendManualParamsDebounced();
    });

    // Focus/blur guard + live label tracking for the manual number fields. Focus
    // marks the field as being edited (suppresses the periodic re-seed, BUG-081);
    // `input` keeps the slider/label in step with each keystroke WITHOUT committing
    // (commit stays on `change` = blur/Enter, so no half-typed value is ever sent).
    numManualFps?.addEventListener('focus', () => { manualEditing = true; });
    numManualFps?.addEventListener('blur', () => { manualEditing = false; });
    numManualFps?.addEventListener('input', () => {
        if (numManualFps.value === '') return;   // mid-clear; don't disturb the slider
        if (slManualFps) slManualFps.value = numManualFps.value;
        if (manualFpsVal) manualFpsVal.textContent = numManualFps.value;
    });
    numManualExposure?.addEventListener('focus', () => { manualEditing = true; });
    numManualExposure?.addEventListener('blur', () => { manualEditing = false; });
    numManualExposure?.addEventListener('input', () => {
        if (numManualExposure.value === '') return;
        if (slManualExposure) slManualExposure.value = numManualExposure.value;
        if (manualExposureVal) manualExposureVal.textContent = numManualExposure.value + ' ms';
        applyFpsCeiling(ceilingForExposure(Number(numManualExposure.value)));
    });

    function sendImuEnvRateDebounced() {
        if (imuEnvDebounce) clearTimeout(imuEnvDebounce);
        imuEnvDebounce = setTimeout(() => {
            imuEnvDebounce = null;
            if (imuEnvPending) return;
            const rate = imuEnvMode === 'coupled' ? 0 : parseInt(numImuEnvRate.value, 10);
            hub.send({ type: 'set_imu_env_rate', rate_hz: rate });
        }, MANUAL_DEBOUNCE_MS);
    }

    segImuEnvMode?.addEventListener('click', (e) => {
        const btn = e.target.closest('button[data-imu-env-mode]');
        if (!btn || imuEnvPending) return;
        imuEnvMode = btn.dataset.imuEnvMode;
        setSegActive(segImuEnvMode, 'imuEnvMode', imuEnvMode);
        if (slImuEnvRate) slImuEnvRate.disabled = imuEnvMode === 'coupled';
        if (numImuEnvRate) numImuEnvRate.disabled = imuEnvMode === 'coupled';
        sendImuEnvRateDebounced();
    });
    slImuEnvRate?.addEventListener('input', () => {
        if (imuEnvPending) return;
        if (numImuEnvRate) numImuEnvRate.value = slImuEnvRate.value;
        sendImuEnvRateDebounced();
    });
    numImuEnvRate?.addEventListener('change', () => {
        if (imuEnvPending) return;
        if (slImuEnvRate) slImuEnvRate.value = numImuEnvRate.value;
        sendImuEnvRateDebounced();
    });

    hub.on('ranging', (msg) => {
        rangingPending = !!msg.pending;
        // Clear the local manual edit only when a command completes (pending true -> false);
        // until then, a periodic echo must not revert the in-progress edit (BUG-079).
        if (prevRangingPending && !rangingPending) manualDirty = false;
        prevRangingPending = rangingPending;
        setSegActive(segRangingProfile, 'profile', msg.applied ? msg.applied.profile : null);
        for (const b of (segRangingProfile ? segRangingProfile.querySelectorAll('button') : [])) {
            b.disabled = rangingPending;
        }
        for (const b of (segManualRangingMode ? segManualRangingMode.querySelectorAll('button') : [])) {
            b.disabled = rangingPending;
        }
        for (const b of (segManualPower ? segManualPower.querySelectorAll('button') : [])) {
            b.disabled = rangingPending;
        }
        if (slManualFps) slManualFps.disabled = rangingPending;
        if (numManualFps) numManualFps.disabled = rangingPending;
        if (slManualExposure) slManualExposure.disabled = rangingPending;
        if (numManualExposure) numManualExposure.disabled = rangingPending;

        if (rangingAppliedVal) {
            if (msg.applied) {
                const a = msg.applied;
                rangingAppliedVal.textContent =
                    `${PROFILE_LABELS[a.profile] || a.profile} (${a.ranging_mode}, ${a.fps} fps, ` +
                    `${a.exposure_ms} ms, ${a.power_mode})`;
                if (!rangingPending && !manualDirty && !manualEditing) {
                    manualRangingMode = a.ranging_mode;
                    manualPowerMode = a.power_mode;
                    setSegActive(segManualRangingMode, 'rangingMode', manualRangingMode);
                    setSegActive(segManualPower, 'powerMode', manualPowerMode);
                    if (slManualFps) slManualFps.value = a.fps;
                    if (numManualFps) numManualFps.value = a.fps;
                    if (manualFpsVal) manualFpsVal.textContent = a.fps;
                    if (slManualExposure) slManualExposure.value = a.exposure_ms;
                    if (numManualExposure) numManualExposure.value = a.exposure_ms;
                    if (manualExposureVal) manualExposureVal.textContent = a.exposure_ms + ' ms';
                }
            } else {
                rangingAppliedVal.textContent = msg.initialized ? '—' : 'not yet read back';
            }
        }
        // Benefit blurb for the applied preset (Manual has its own). Shown under the
        // selector so each preset explains itself once chosen (owner ask 2026-08-05).
        if (rangingBlurb) {
            const slug = msg.applied ? msg.applied.profile : null;
            const blurb = slug ? PROFILE_BLURBS[slug] : '';
            rangingBlurb.textContent = blurb || '';
            rangingBlurb.hidden = !blurb;
        }
        // Keep the fps cap bar in step with whatever exposure the manual box now holds
        // (prefer the server's measured ceiling for the applied config when idle).
        if (!manualEditing) {
            const ceiling = (msg.estimate && msg.estimate.ceiling_fps)
                ? msg.estimate.ceiling_fps : currentFpsCeiling();
            applyFpsCeiling(ceiling);
        }

        if (rangingRequestedStatus) {
            if (msg.pending && msg.requested) {
                const r = msg.requested;
                const desc = (r.kind === 'manual' && r.manual)
                    ? `manual ${r.manual.ranging_mode}, ${r.manual.fps} fps, ` +
                      `${r.manual.exposure_ms} ms, ${r.manual.power_mode}`
                    : (PROFILE_LABELS[r.profile] || r.profile || '');
                rangingRequestedStatus.textContent = `Requested: ${desc} …`;
                rangingRequestedStatus.hidden = false;
            } else {
                rangingRequestedStatus.hidden = true;
            }
        }
        if (rangingErrorEl) {
            rangingErrorEl.hidden = !msg.error;
            rangingErrorEl.textContent = msg.error ? `Error: ${msg.error}` : '';
        }

        const est = msg.estimate;
        if (est) {
            if (rangingRangeVal) rangingRangeVal.textContent = `${est.max_range_m.toFixed(1)} m`;
            if (rangingPowerVal) rangingPowerVal.textContent = `${Math.round(est.power_mw)} mW`;
            const frac = est.i3c_bus_utilization_pct / 100;
            if (rangingI3cFill) {
                rangingI3cFill.style.width = Math.min(100, est.i3c_bus_utilization_pct) + '%';
                rangingI3cFill.classList.toggle('is-warn', frac >= 0.70 && frac < 0.85);
                rangingI3cFill.classList.toggle('is-crit', frac >= 0.85);
            }
            if (rangingI3cCaption) {
                rangingI3cCaption.textContent =
                    `I3C Bus: ${est.i3c_bus_utilization_pct.toFixed(1)}% used ` +
                    `(${est.i3c_xfer_ms.toFixed(1)}ms ToF / ${(est.frame_period_us / 1000).toFixed(1)}ms frame) ` +
                    `• ${est.i3c_airtime_left_pct.toFixed(1)}% airtime left for IMU`;
            }
            if (rangingCdcWarning) {
                rangingCdcWarning.hidden = !est.transport_warning;
                rangingCdcWarning.textContent = est.transport_warning || '';
            }
            if (rangingExpectedFpsVal) {
                rangingExpectedFpsVal.textContent = `${est.expected_delivered_fps.toFixed(1)} fps`;
            }
            // Every OTHER estimate warning (e.g. the delivery-rate quantization
            // notice, or an IMU/env sub-sample notice folded into this same
            // profile's estimate) -- excludes transport_warning, which already
            // has its own dedicated row above.
            if (rangingEstimateWarnings) {
                const rest = (est.warnings || []).filter((w) => w !== est.transport_warning);
                rangingEstimateWarnings.hidden = rest.length === 0;
                rangingEstimateWarnings.textContent = rest.join(' ');
            }
        } else {
            if (rangingRangeVal) rangingRangeVal.textContent = '—';
            if (rangingPowerVal) rangingPowerVal.textContent = '—';
            if (rangingExpectedFpsVal) rangingExpectedFpsVal.textContent = '—';
            if (rangingI3cFill) rangingI3cFill.style.width = '0%';
            if (rangingI3cCaption) rangingI3cCaption.textContent = '';
            if (rangingCdcWarning) rangingCdcWarning.hidden = true;
            if (rangingEstimateWarnings) rangingEstimateWarnings.hidden = true;
        }

        if (rangingMeasuredVal) {
            rangingMeasuredVal.textContent = (msg.measured_fps !== null && msg.measured_fps !== undefined)
                ? `${msg.measured_fps.toFixed(1)} fps` : '—';
        }

        const ie = msg.imu_env || {};
        imuEnvPending = !!ie.pending;
        setSegActive(segImuEnvMode, 'imuEnvMode', ie.coupled ? 'coupled' : 'explicit');
        for (const b of (segImuEnvMode ? segImuEnvMode.querySelectorAll('button') : [])) {
            b.disabled = imuEnvPending;
        }
        if (slImuEnvRate) slImuEnvRate.disabled = imuEnvPending || !!ie.coupled;
        if (numImuEnvRate) numImuEnvRate.disabled = imuEnvPending || !!ie.coupled;
        if (imuEnvAppliedVal) {
            imuEnvAppliedVal.textContent = !ie.initialized
                ? 'not confirmed by firmware'
                : (ie.coupled ? 'coupled' : `${ie.applied_rate_hz} Hz`);
        }
        if (imuEnvSubsampleWarning) {
            imuEnvSubsampleWarning.hidden = !ie.warning;
            imuEnvSubsampleWarning.textContent = ie.warning || '';
        }
        if (imuEnvError) {
            imuEnvError.hidden = !ie.error;
            imuEnvError.textContent = ie.error ? `Error: ${ie.error}` : '';
        }
    });

    // --- View group: color mode segmented control (server-driven active) ---
    const segColor = $('seg-color');
    segColor?.addEventListener('click', (e) => {
        const btn = e.target.closest('button[data-mode]');
        if (btn) hub.send({ type: 'set_color', mode: btn.dataset.mode });
    });

    // Device: reflectance flat-field calibration toggle (server-driven active).
    segCalibrated?.addEventListener('click', (e) => {
        const btn = e.target.closest('button[data-calibrated]');
        if (btn) hub.send({ type: 'set_calibrated', enabled: btn.dataset.calibrated === 'on' });
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
        // Device: Calibrated/Uncalibrated. Disable + explain when no map is configured.
        setActive(segCalibrated, 'calibrated', msg.calibrated ? 'on' : 'off');
        const calAvail = msg.calibration_available !== false;
        if (segCalibrated) {
            for (const b of segCalibrated.querySelectorAll('button')) b.disabled = !calAvail;
        }
        if (calibratedNote) calibratedNote.hidden = calAvail;
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
        // The amplitude only has an effect once Oscillate is the selected
        // mode -- Orbit Speed also sets the oscillate SWING RATE (see
        // scene.js updateOscillate), so it stays gated on worldOnly alone,
        // but amplitude does nothing at all under Continuous. Issue #107:
        // it used to share the worldOnly-only gate above and stayed
        // interactive (and undimmed) with Continuous selected or Auto-orbit
        // off, which read as "live" when nothing it controls was running.
        const oscillateActive = worldOnly && msg.orbit_mode === 'oscillate';
        if (slOrbitAmplitude) slOrbitAmplitude.disabled = !oscillateActive;
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
