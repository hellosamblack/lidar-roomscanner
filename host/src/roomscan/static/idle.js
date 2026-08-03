// idle.js — parks the browser session when nobody's using it (owner ask,
// 2026-08-03).
//
// Two problems this fixes together: an idle tab left rendering forever burns
// client CPU for no reason (see the headless-Chrome-at-1100% pattern), and a
// `/ws` connection that never engages used to count as "a viewer" forever, so
// the server's sensor auto-idle never saw "nothing is using the live feed"
// even when nobody was actually looking (see web.py's `_active_viewer_count`
// comment). This module fixes the first directly (stop rendering) and reports
// the second over the wire (`idle_state`) so the server's viewer accounting
// reflects reality.
//
// Resume is either the button or the SAME activity listener firing again
// while parked -- moving the mouse un-idles immediately, no click required.
// The button exists for input that isn't a DOM event this module listens for
// (e.g. the OS refocusing the window without any mouse motion inside it).

const HEARTBEAT_MS = 30000;   // while active: how often to re-touch the
                               // server's per-connection activity clock so a
                               // merely-open (but unused) tab still ages out

export function createIdle(hub, sceneApi) {
    const $ = (id) => document.getElementById(id);
    const modal = $('idle-modal');
    const btnResume = $('idle-resume');
    if (!modal) return;   // trimmed page: nothing to wire

    let timeoutMs = 300000;        // overwritten once by the server's config
                                    // (see the 'state' handler below) -- NOT
                                    // re-applied on every later 'state' echo,
                                    // which fires on any unrelated UI change
                                    // and must not keep resetting this timer
    let configApplied = false;
    let parkTimer = null;
    let heartbeatTimer = null;
    let parked = false;

    function setRendering(on) {
        if (sceneApi && sceneApi.setRenderActive) sceneApi.setRenderActive(on);
    }

    function park() {
        if (parked) return;
        parked = true;
        clearTimeout(heartbeatTimer);
        setRendering(false);
        hub.send({ type: 'idle_state', active: false });
        modal.classList.remove('hidden');
    }

    function resume() {
        modal.classList.add('hidden');
        if (parked) {
            parked = false;
            setRendering(true);
        }
        hub.send({ type: 'idle_state', active: true });
        armParkTimer();
        armHeartbeat();
    }

    function armParkTimer() {
        clearTimeout(parkTimer);
        parkTimer = setTimeout(park, timeoutMs);
    }

    function armHeartbeat() {
        clearTimeout(heartbeatTimer);
        heartbeatTimer = setTimeout(() => {
            hub.send({ type: 'idle_state', active: true });
            armHeartbeat();
        }, HEARTBEAT_MS);
    }

    function onActivity() {
        if (parked) { resume(); return; }
        armParkTimer();
    }

    ['mousemove', 'mousedown', 'keydown', 'wheel', 'touchstart'].forEach((ev) =>
        window.addEventListener(ev, onActivity, { passive: true }));

    // A backgrounded tab is idle immediately, regardless of the timer -- no
    // point rendering (or counting as a viewer) something nobody can see.
    document.addEventListener('visibilitychange', () => {
        if (document.hidden) park();
        else onActivity();
    });

    btnResume?.addEventListener('click', resume);

    // Apply the server-configured timeout exactly once, from the first
    // 'state' message (sent immediately on connect) -- NOT on every later
    // broadcast, which fires on any client's unrelated setting change and
    // would otherwise keep extending this tab's own idle window forever.
    hub.on('state', (msg) => {
        if (configApplied) return;
        if (typeof msg.browser_idle_timeout_s === 'number' && msg.browser_idle_timeout_s > 0) {
            configApplied = true;
            timeoutMs = msg.browser_idle_timeout_s * 1000;
            if (!parked) armParkTimer();
        }
    });

    armParkTimer();
    armHeartbeat();

    // Diagnostics only (mirrors app.js's `window.__scene`) -- lets a driven
    // headless-Chrome check force park/resume instead of waiting out the real
    // multi-minute timeout.
    return { park, resume, isParked: () => parked };
}
