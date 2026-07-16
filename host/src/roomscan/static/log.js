// log.js — bottom event-log console (§7.1) + the transient toast layer.
//
// Console: scrolling, docked full-width at the bottom, collapsible to a thin bar
// so it never permanently eats canvas space. Capped at ~200 lines client-side
// (oldest dropped first, mirroring LogBus's bounded backlog). Subscribes to
// "log" and "event"; each line is timestamped with client-side Date.now() at
// receipt (the bus doesn't stamp wall-clock, consistent with logbus.py).
//
// Toast: on a "cmd" message show `label: detail` for ~2.5s, styled by status
// (ok=success, busy/timeout=warning, error=danger). The toast layer floats ABOVE
// the collapsible console so toasts stay visible even when the console is closed.
//
// Public surface:  createLog(hub) -> {}
// Hub events:  subscribes "log", "event", "cmd"

const MAX_LINES = 200;
const TOAST_MS = 2500;

function stamp() {
    const d = new Date();
    return d.toTimeString().slice(0, 8) + '.' + String(d.getMilliseconds()).padStart(3, '0');
}

export function createLog(hub) {
    const linesEl = document.getElementById('log-lines');
    const consoleEl = document.getElementById('log-console');
    const toggleEl = document.getElementById('log-toggle');
    const toastLayer = document.getElementById('toast-layer');

    function append(source, text) {
        if (!linesEl) return;
        const row = document.createElement('div');
        row.className = 'log-line log-line--' + source;
        row.innerHTML =
            `<span class="log-line__ts">${stamp()}</span>` +
            `<span class="log-line__src">${source}</span>` +
            `<span class="log-line__msg"></span>`;
        row.querySelector('.log-line__msg').textContent = text;   // textContent = safe
        linesEl.appendChild(row);
        while (linesEl.childElementCount > MAX_LINES) linesEl.removeChild(linesEl.firstChild);
        linesEl.scrollTop = linesEl.scrollHeight;   // autoscroll to newest
    }

    hub.on('log', (msg) => append('log', String(msg.line ?? '')));
    hub.on('event', (msg) => {
        const tail = msg.msg ? ' ' + msg.msg : '';
        append('event', `code=${msg.code} detail=${msg.detail}${tail}`);
    });

    // cmd -> event-log line AND a transient toast styled by status.
    hub.on('cmd', (msg) => {
        const status = msg.status || 'ok';
        append('cmd', `${msg.label}: ${msg.detail}`);
        showToast(`${msg.label}: ${msg.detail}`, status);
    });

    function showToast(text, status) {
        if (!toastLayer) return;
        const t = document.createElement('div');
        t.className = 'toast toast--' + status;
        t.textContent = text;
        toastLayer.appendChild(t);
        // Fade-in on next frame, remove after the dwell.
        requestAnimationFrame(() => t.classList.add('toast--show'));
        setTimeout(() => {
            t.classList.remove('toast--show');
            setTimeout(() => t.remove(), 250);
        }, TOAST_MS);
    }

    // Collapse toggle: flip a class on the console; header stays visible.
    if (toggleEl && consoleEl) {
        toggleEl.addEventListener('click', () => consoleEl.classList.toggle('collapsed'));
    }

    return {};
}
