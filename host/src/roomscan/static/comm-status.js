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

    function render(t) {
        current = t;
        titleEl.textContent = t.label || t.id;
        introEl.textContent = (t.addr ? t.addr + ' — ' : '') + (t.detail || '');

        factsOrEmpty(linkEl, t.link, 'no link data');
        factsOrEmpty(deviceEl, t.device,
            t.informational ? 'not connected — informational, not a fault'
                            : 'no device data yet');

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
