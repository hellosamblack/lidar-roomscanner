// admin.js — the top-bar Restart Server maintenance action.
//
// Unlike every other module here this does NOT go over the hub: it is a
// plain POST to /api/restart, because it acts on the server process itself
// rather than on the instrument's state, and the socket is what dies.
//
// After a restart the UI needs no special handling: ws.js already reconnects
// with backoff, so the connection dot goes red and comes back on its own.
//
// The FileHub "Bridge Mode" button that used to live here was retired
// 2026-08-20 (#192): the Pi bridge (#191) replaced the FileHub as the
// scanner's wireless uplink, so there is no router left to re-bridge. The
// Pi bridge's own maintenance actions (Reboot, Fetch recent logs) live in
// the Bridge chip's modal instead -- see comm-status.js.

export function createAdmin(hub) {
    const $ = (id) => document.getElementById(id);

    const btnRestart = $('btn-restart');
    if (!btnRestart) return;   // nothing to wire if the markup isn't present

    const log = (m) => { if (window.__diag) window.__diag(m); };

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
