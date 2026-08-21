// comm-status.js — the per-chip Link/Device/Management modal for the top-bar
// comm-status cluster (issue #209).
//
// hud.js paints the chips themselves (dot colour, label, title) straight off
// every `comm` tick and owns none of this: a click there only `hub.emit`s
// 'comm_chip_click' with the target id, so hud.js's per-tick DOM churn never
// has to build a modal's worth of markup 3 seconds. This module instead caches
// the latest full `comm.targets` array and renders ONE shared modal (#comm-modal
// in index.html) from whichever target was clicked -- same schema for every
// chip (Bridge/Scanner/CrowPanel/ST-Link): `link` (reachability-layer facts),
// `device` (health, or nothing to show), `management` (control buttons, or
// none -- omitted rather than shown disabled, per the "only what is honestly
// controllable" rule).
//
// Today the only controllable device is the Pi bridge: Reboot and Fetch
// recent logs POST to /api/bridge/reboot and /api/bridge/logs, which call the
// exact same host/tools/pi_bridge.py functions the bridge_* MCP tools do --
// one implementation, two front ends, never a second copy of "how to reboot
// the bridge" living in this file.

export function createCommStatus(hub) {
    const $ = (id) => document.getElementById(id);
    const modal = $('comm-modal');
    if (!modal) return {};

    const titleEl = $('commx-title');
    const introEl = $('commx-intro');
    const linkEl = $('commx-link');
    const deviceEl = $('commx-device');
    const mgmtSection = $('commx-mgmt-section');
    const output = $('commx-output');
    const btnReboot = $('commx-reboot');
    const btnLogs = $('commx-logs');

    // Voxel view (issue #215): editable framing/tone for the server-rendered
    // /ws-thin point-cloud feed. CrowPanel-only, mirroring the Pi-only gating
    // of the Management section below. Edits go out as `set_thin_view`; values
    // are server-authoritative and come back on the `state` echo (never applied
    // optimistically), exactly like the browser View controls in controls.js.
    const voxelSection = $('commx-voxel-section');
    const segThinColormap = $('seg-thin-colormap');
    const chkThinPointAuto = $('chk-thin-point-auto');
    const slThinPointSize = $('sl-thin-point-size');
    const thinPointSizeVal = $('thin-point-size-val');
    const slThinDistance = $('sl-thin-distance');
    const slThinHeight = $('sl-thin-height');
    const slThinRotation = $('sl-thin-rotation');
    const chkThinOrbit = $('chk-thin-orbit');
    const slThinOrbitSpeed = $('sl-thin-orbit-speed');
    const slThinOrbitAmplitude = $('sl-thin-orbit-amplitude');
    const thinDistanceVal = $('thin-distance-val');
    const thinHeightVal = $('thin-height-val');
    const thinRotationVal = $('thin-rotation-val');
    const thinOrbitSpeedVal = $('thin-orbit-speed-val');
    const thinOrbitAmplitudeVal = $('thin-orbit-amplitude-val');

    let targets = [];    // latest comm.targets, cached for the click handler
    let current = null;  // the target object currently shown in the modal

    hub.on('comm', (msg) => {
        targets = Array.isArray(msg.targets) ? msg.targets : [];
        // Keep an open modal live: a reboot in progress should show the
        // bridge coming back without the owner having to reclick the chip.
        if (current && !modal.classList.contains('hidden')) {
            const fresh = targets.find((t) => t.id === current.id);
            if (fresh) render(fresh);
        }
    });

    // Internal bookkeeping keys from the raw bridge_status blob that read as
    // noise in a flat facts list (warnings/units get their own surfaces
    // elsewhere; `ok`/`target` are transport plumbing, not a device reading).
    const SKIP_KEYS = new Set(['warnings', 'units', 'unit_restarts', 'ok', 'target',
                               'raw', 'ssh_latency_ms']);

    function flatten(obj, prefix) {
        const out = [];
        if (!obj || typeof obj !== 'object') return out;
        for (const [k, v] of Object.entries(obj)) {
            if (SKIP_KEYS.has(k) || v === null || v === undefined) continue;
            const label = prefix ? prefix + '.' + k : k;
            if (Array.isArray(v)) {
                if (v.length) out.push([label, v.length + ' item(s)']);
            } else if (typeof v === 'object') {
                out.push(...flatten(v, label));
            } else {
                out.push([label, v]);
            }
        }
        return out;
    }

    function factsOrEmpty(container, obj, emptyText) {
        container.innerHTML = '';
        const entries = flatten(obj);
        if (entries.length === 0) {
            const p = document.createElement('p');
            p.className = 'commx-section__empty';
            p.textContent = emptyText;
            container.appendChild(p);
            return;
        }
        const dl = document.createElement('dl');
        dl.className = 'commx-facts';
        for (const [k, v] of entries) {
            const dt = document.createElement('dt');
            dt.textContent = k;
            const dd = document.createElement('dd');
            dd.textContent = String(v);
            dl.appendChild(dt);
            dl.appendChild(dd);
        }
        container.appendChild(dl);
    }

    function setThinView(fields) {
        hub.send({ type: 'set_thin_view', ...fields });
    }

    function applyThinView(tv) {
        if (!tv) return;
        if (segThinColormap) {
            for (const b of segThinColormap.querySelectorAll('button')) {
                b.classList.toggle('active', b.dataset.thincolormap === tv.colormap);
            }
        }
        if (slThinDistance !== null && tv.distance_m !== undefined) {
            slThinDistance.value = tv.distance_m;
            if (thinDistanceVal) thinDistanceVal.textContent = Number(tv.distance_m).toFixed(2) + ' m';
        }
        if (slThinHeight !== null && tv.height_m !== undefined) {
            slThinHeight.value = tv.height_m;
            if (thinHeightVal) thinHeightVal.textContent = Number(tv.height_m).toFixed(2) + ' m';
        }
        if (slThinRotation !== null && tv.rotation_deg !== undefined) {
            slThinRotation.value = tv.rotation_deg;
            if (thinRotationVal) thinRotationVal.textContent = Math.round(tv.rotation_deg) + '°';
        }
        if (chkThinPointAuto && tv.point_size_auto !== undefined) chkThinPointAuto.checked = !!tv.point_size_auto;
        if (slThinPointSize !== null && tv.point_size !== undefined) {
            slThinPointSize.value = tv.point_size;
            if (thinPointSizeVal) thinPointSizeVal.textContent = tv.point_size + ' px';
        }
        if (chkThinOrbit && tv.orbit_enabled !== undefined) chkThinOrbit.checked = !!tv.orbit_enabled;
        if (slThinOrbitSpeed !== null && tv.orbit_speed_deg_s !== undefined) {
            slThinOrbitSpeed.value = tv.orbit_speed_deg_s;
            if (thinOrbitSpeedVal) thinOrbitSpeedVal.textContent = tv.orbit_speed_deg_s + '°/s';
        }
        if (slThinOrbitAmplitude !== null && tv.orbit_amplitude_deg !== undefined) {
            slThinOrbitAmplitude.value = tv.orbit_amplitude_deg;
            if (thinOrbitAmplitudeVal) thinOrbitAmplitudeVal.textContent = Math.round(tv.orbit_amplitude_deg) + '°';
        }
    }

    let lastThinView = null;   // latest state.thin_view, for a modal opened later
    hub.on('state', (msg) => {
        if (msg && msg.thin_view) {
            lastThinView = msg.thin_view;
            applyThinView(lastThinView);
        }
    });

    if (segThinColormap) {
        segThinColormap.addEventListener('click', (e) => {
            const btn = e.target.closest('button[data-thincolormap]');
            if (btn) setThinView({ colormap: btn.dataset.thincolormap });
        });
    }
    chkThinPointAuto?.addEventListener('change', () => setThinView({ point_size_auto: chkThinPointAuto.checked }));
    slThinPointSize?.addEventListener('input', () => setThinView({ point_size: parseFloat(slThinPointSize.value) }));
    slThinDistance?.addEventListener('input', () => setThinView({ distance: parseFloat(slThinDistance.value) }));
    slThinHeight?.addEventListener('input', () => setThinView({ height: parseFloat(slThinHeight.value) }));
    slThinRotation?.addEventListener('input', () => setThinView({ rotation: parseFloat(slThinRotation.value) }));
    chkThinOrbit?.addEventListener('change', () => setThinView({ orbit: chkThinOrbit.checked }));
    slThinOrbitSpeed?.addEventListener('input', () => setThinView({ orbit_speed: parseFloat(slThinOrbitSpeed.value) }));
    slThinOrbitAmplitude?.addEventListener('input', () => setThinView({ orbit_amplitude: parseFloat(slThinOrbitAmplitude.value) }));

    function render(t) {
        current = t;
        titleEl.textContent = t.label || t.id;
        introEl.textContent = (t.addr ? t.addr + ' — ' : '') + (t.detail || '');

        // Voxel-view editor is CrowPanel-only (it renders that device's feed).
        if (voxelSection) {
            const isCrowPanel = t.id === 'crowpanel';
            voxelSection.classList.toggle('hidden', !isCrowPanel);
            if (isCrowPanel && lastThinView) applyThinView(lastThinView);
        }

        factsOrEmpty(linkEl, t.link, 'no link data');

        // Only show "not connected — informational" for genuinely absent targets
        // whose absence is expected/normal (state='absent' AND informational=true).
        // Down/up/unknown targets (or non-informational absent ones) show neutral text.
        const emptyDeviceText = (t.state === 'absent' && t.informational)
            ? 'not connected — informational, not a fault'
            : 'no device data yet';
        factsOrEmpty(deviceEl, t.device, emptyDeviceText);

        const hasMgmt = !!(t.management && (t.management.reboot || t.management.logs));
        mgmtSection.classList.toggle('hidden', !hasMgmt);
        if (btnReboot) btnReboot.classList.toggle('hidden', !(t.management && t.management.reboot));
        if (btnLogs) btnLogs.classList.toggle('hidden', !(t.management && t.management.logs));
        if (!hasMgmt) {
            output.classList.add('hidden');
            output.textContent = '';
        }
    }

    function open(id) {
        const t = targets.find((x) => x.id === id);
        if (!t) return;
        render(t);
        modal.classList.remove('hidden');
    }

    function close() {
        modal.classList.add('hidden');
        current = null;
    }

    hub.on('comm_chip_click', open);
    $('commx-close')?.addEventListener('click', close);
    modal.addEventListener('click', (e) => { if (e.target === modal) close(); });
    document.addEventListener('keydown', (e) => {
        if (e.key === 'Escape' && !modal.classList.contains('hidden')) close();
    });

    function showOutput(text, cls) {
        output.textContent = text;
        output.className = 'comm-output' + (cls ? ' ' + cls : '');
        output.classList.remove('hidden');
    }

    btnReboot?.addEventListener('click', async () => {
        if (!current) return;
        const label = current.addr || current.label || 'the bridge';
        if (!window.confirm('Reboot ' + label + '?\n\nIt takes roughly 30s to come back.')) return;
        btnReboot.disabled = true;
        showOutput('Rebooting…', null);
        try {
            const res = await fetch('/api/bridge/reboot', { method: 'POST' });
            const data = await res.json();
            const body = data.error ? data.error : (data.note || (data.ok ? 'reboot requested' : 'failed'));
            showOutput(body, data.ok ? 'ok' : 'err');
        } catch (err) {
            showOutput('request failed: ' + err, 'err');
        }
        btnReboot.disabled = false;
    });

    btnLogs?.addEventListener('click', async () => {
        btnLogs.disabled = true;
        showOutput('Fetching logs…', null);
        try {
            const res = await fetch('/api/bridge/logs', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ unit: 'roomscan-bridge', lines: 200 }),
            });
            const data = await res.json();
            showOutput(data.ok ? (data.text || '(empty)') : (data.error || 'failed'),
                       data.ok ? null : 'err');
        } catch (err) {
            showOutput('request failed: ' + err, 'err');
        }
        btnLogs.disabled = false;
    });

    return { open };
}
