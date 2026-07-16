// ws.js — the ONLY module that touches the raw WebSocket.
//
// Owns: connect + reconnect-with-backoff + the binary-tag/JSON demux (§6.3),
// and a minimal pub/sub hub (the frontend mirror of the backend LogBus /
// broadcast pattern). Every other module talks to the server *only* through
// this hub: they `on(type, handler)` to receive, and `send(obj)` to transmit.
//
// Demux rule (§6.3):
//   typeof event.data === "string"  -> JSON.parse -> emit(msg.type, msg)
//   ArrayBuffer                     -> LE uint32 tag in bytes[0..4)
//                                        tag 1 -> emit("point_cloud", buffer)
//                                        tag 2 -> emit("ir_image",    buffer)
//                                        else  -> console.warn + drop (never throw)
// The RAW buffer (header included) is handed to subscribers so each parses its
// own fixed layout; ws.js never needs to know point/pixel counts.
//
// Connection lifecycle is published as a local hub event: emit("conn", {state})
// with state ∈ {"connecting","open","closed","error"} so hud/topbar can render
// the connection dot without reaching into this module.

const D = (m, l) => { try { window.__diag && window.__diag('ws.js: ' + m, l); } catch (e) {} };

// Binary message type tags — mirror web.py TAG_POINT_CLOUD / TAG_IR_IMAGE.
const TAG_POINT_CLOUD = 1;
const TAG_IR_IMAGE = 2;

const RECONNECT_MS = 2000;

export function createHub() {
    const handlers = new Map();   // type -> Set<fn>
    let socket = null;

    function on(type, fn) {
        let set = handlers.get(type);
        if (!set) { set = new Set(); handlers.set(type, set); }
        set.add(fn);
        return () => set.delete(fn);   // unsubscribe
    }

    function emit(type, payload) {
        const set = handlers.get(type);
        if (!set) return;
        for (const fn of set) {
            try { fn(payload); }
            catch (e) { D('handler for "' + type + '" threw: ' + (e && e.message), 'error'); }
        }
    }

    // JSON-stringify + write if the socket is open; silently no-op otherwise
    // (a control fired while disconnected simply does nothing — the server is
    // the source of truth and the UI will re-sync on the next `state` echo).
    function send(obj) {
        if (socket && socket.readyState === WebSocket.OPEN) {
            try { socket.send(JSON.stringify(obj)); }
            catch (e) { D('send failed: ' + (e && e.message), 'error'); }
        }
    }

    function connect() {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const url = `${protocol}//${window.location.host}/ws`;
        D('connecting -> ' + url);
        emit('conn', { state: 'connecting' });
        try {
            socket = new WebSocket(url);
        } catch (e) {
            D('constructor threw: ' + e.message, 'error');
            emit('conn', { state: 'error' });
            setTimeout(connect, RECONNECT_MS);
            return;
        }
        socket.binaryType = 'arraybuffer';

        socket.onopen = () => { D('OPEN'); emit('conn', { state: 'open' }); };

        socket.onclose = (ev) => {
            // 1006 = abnormal (no close frame); 1000 = normal; 1015 = TLS.
            D('CLOSE code=' + ev.code + ' reason="' + (ev.reason || '') + '" wasClean=' + ev.wasClean, 'error');
            emit('conn', { state: 'closed' });
            setTimeout(connect, RECONNECT_MS);   // reconnect-with-backoff
        };

        socket.onerror = () => { D('ERROR (see close code next)', 'error'); emit('conn', { state: 'error' }); };

        socket.onmessage = (event) => {
            if (typeof event.data === 'string') {
                let msg;
                try { msg = JSON.parse(event.data); }
                catch (e) { D('non-JSON text frame dropped: ' + event.data.slice(0, 80), 'error'); return; }
                if (msg && typeof msg.type === 'string') emit(msg.type, msg);
                else D('JSON frame missing string `type`, dropped', 'error');
                return;
            }
            // Binary: first 4 bytes LE uint32 tag; hand the RAW buffer onward.
            const buffer = event.data;
            if (!(buffer instanceof ArrayBuffer) || buffer.byteLength < 4) {
                console.warn('[ws] binary frame too short, dropped');
                return;
            }
            const tag = new DataView(buffer).getUint32(0, true);
            if (tag === TAG_POINT_CLOUD) emit('point_cloud', buffer);
            else if (tag === TAG_IR_IMAGE) emit('ir_image', buffer);
            else console.warn('[ws] unrecognized binary tag ' + tag + ', dropped');
        };
    }

    return { on, emit, send, connect };
}
