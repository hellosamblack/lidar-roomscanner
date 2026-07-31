// admin.js — the two top-bar maintenance actions (2026-07-31).
//
// Bridge Mode re-applies transparent-bridge mode on the RavPower FileHub;
// Restart Server relaunches the roomscan-web process.
//
// Unlike every other module here these do NOT go over the hub: they are plain
// POSTs to /api/bridge-mode and /api/restart, because both act on the server
// process/host rather than on the instrument's state. Restart in particular
// cannot use the socket -- the socket is what dies.
//
// Bridge Mode is gated behind a confirm modal on purpose. The script is step 3
// of a 4-step physical sequence (unplug -> power-cycle -> script -> replug) and
// running it out of order makes the FileHub treat its LAN port as WAN, so the
// modal states the ordering rather than letting one click imply "fix it".
//
// After a restart the UI needs no special handling: ws.js already reconnects
// with backoff, so the connection dot goes red and comes back on its own.

export function createAdmin(hub) {
    const $ = (id) => document.getElementById(id);

    const btnBridge = $('btn-bridge');
    const btnRestart = $('btn-restart');
    const modal = $('bridge-modal');
    const output = $('bridge-output');
    const btnRun = $('bridge-run');

    // Nothing to wire if the markup isn't present (e.g. a trimmed page).
    if (!btnBridge || !btnRestart || !modal) return;

    const log = (m) => { if (window.__diag) window.__diag(m); };

    function showOutput(text, cls) {
        output.textContent = text;
        output.className = 'bridge-output' + (cls ? ' ' + cls : '');
    }

    function openModal() {
        showOutput('', null);
        output.classList.add('hidden');
        btnRun.disabled = false;
        btnRun.textContent = 'Run step 3';
        modal.classList.remove('hidden');
    }

    function closeModal() { modal.classList.add('hidden'); }

    btnBridge.addEventListener('click', openModal);
    $('bridge-close')?.addEventListener('click', closeModal);
    $('bridge-cancel')?.addEventListener('click', closeModal);
    modal.addEventListener('click', (e) => { if (e.target === modal) closeModal(); });
    document.addEventListener('keydown', (e) => {
        if (e.key === 'Escape' && !modal.classList.contains('hidden')) closeModal();
    });

    btnRun.addEventListener('click', async () => {
        btnRun.disabled = true;
        btnRun.textContent = 'Running…';
        output.classList.remove('hidden');
        showOutput('Connecting to FileHub…', null);
        try {
            const res = await fetch('/api/bridge-mode', { method: 'POST' });
            const data = await res.json();
            // Report what happened, not what was asked for: on failure the
            // reason (missing expect, unreachable router, timeout) is the
            // whole value of the button.
            const body = data.error ? data.error : (data.output || '(no output)');
            showOutput(body, data.ok ? 'ok' : 'err');
            log('[bridge] ' + (data.ok ? 'ok' : 'failed: ' + (data.error || data.returncode)));
        } catch (err) {
            showOutput('request failed: ' + err, 'err');
            log('[bridge] request failed: ' + err);
        }
        btnRun.disabled = false;
        btnRun.textContent = 'Run again';
    });

    let restarting = false;

    // Registered ONCE, not per click: subscribing inside the handler would add
    // a duplicate listener on every restart.
    hub.on('conn', (st) => {
        if (restarting && st.state === 'open') {
            restarting = false;
            btnRestart.classList.remove('busy');
            btnRestart.textContent = 'Restart Server';
        }
    });

    btnRestart.addEventListener('click', async () => {
        if (restarting) return;
        // A restart drops the live stream and any in-progress recording, so it
        // takes a confirm -- but an inline one, since there is no ordering
        // hazard to explain the way Bridge Mode has.
        if (!window.confirm('Restart the roomscan-web server?\n\n'
                            + 'The live stream drops for a few seconds and the '
                            + 'page reconnects on its own.')) return;
        restarting = true;
        btnRestart.classList.add('busy');
        btnRestart.textContent = 'Restarting…';
        try {
            const res = await fetch('/api/restart', { method: 'POST' });
            const data = await res.json();
            log('[restart] ' + (data.ok ? 'server relaunching' : 'failed: ' + data.error));
        } catch (err) {
            // Expected: the process can exit before the response lands.
            log('[restart] connection closed (server exiting)');
        }
        // ws.js reconnects on its own, and the 'conn' handler above restores
        // the label — no page reload out from under the user.
    });
}
