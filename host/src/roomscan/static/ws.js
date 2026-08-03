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
//                                        tag 5 -> emit("magpose",     buffer)
//                                        else  -> console.warn + drop (never throw)
// The RAW buffer (header included) is handed to subscribers so each parses its
// own fixed layout; ws.js never needs to know point/pixel counts.
//
// Connection lifecycle is published as a local hub event: emit("conn", {state})
// with state ∈ {"connecting","open","closed","error"} so hud/topbar can render
// the connection dot without reaching into this module.
//
// Second socket, `/ws-mesh` (BUG-061, Part A): MESH (tag 3) moved off `/ws` so
// a whole-map re-send can never sit in front of the 30 Hz pose JSON. It is a
// SEPARATE WebSocket with its own connect/reconnect-with-backoff, mirroring
// the pattern above; its only inbound traffic is binary tag-3 frames (demuxed
// exactly like the main socket, -> emit('mesh', buffer)), and its only
// outbound traffic is `mesh_ack`, sent by `hub.ackMesh(seq)` after the client
// has actually consumed the mesh (uploaded it to the GPU) — see slam.js's
// `requestAnimationFrame(() => hub.ackMesh(seq))`. The main socket keeps its
// old tag-3 branch too: harmless (the server no longer sends tag 3 there),
// and it means a stale cached client that hasn't picked up ws-mesh yet still
// degrades to "no mesh" rather than throwing.

const D = (m, l) => { try { window.__diag && window.__diag('ws.js: ' + m, l); } catch (e) {} };

// Binary message type tags — mirror web.py TAG_POINT_CLOUD / TAG_IR_IMAGE /
// TAG_MESH / TAG_SURFACE / TAG_MAGPOSE.
const TAG_POINT_CLOUD = 1;
const TAG_IR_IMAGE = 2;
const TAG_MESH = 3;
const TAG_SURFACE = 4;
const TAG_MAGPOSE = 5;

const RECONNECT_MS = 2000;

// One id per page load, sent as a query param on BOTH sockets below, so the
// server can tell that a `/ws` connection and a `/ws-mesh` connection belong
// to the same tab and share one activity clock for idle-parking (Phase A.5,
// idle.js). Not a security token — just enough entropy to not collide across
// tabs in one browser session.
function makeClientId() {
    return 'c' + Math.random().toString(36).slice(2) + Date.now().toString(36);
}

export function createHub() {
    const handlers = new Map();   // type -> Set<fn>
    const clientId = makeClientId();
    let socket = null;
    let meshSocket = null;

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

    // Ack a consumed mesh on the DEDICATED mesh socket (never the main one —
    // the whole point of the split is that these two channels don't share a
    // queue). Silent no-op when the mesh socket isn't open: the server's
    // ack-timeout path (MESH_ACK_TIMEOUT_S) already covers "client vanished",
    // and a reconnect will get pushed the current latest mesh as a late
    // joiner, so there's nothing to retry here.
    function ackMesh(seq) {
        if (meshSocket && meshSocket.readyState === WebSocket.OPEN) {
            try { meshSocket.send(JSON.stringify({ type: 'mesh_ack', seq })); }
            catch (e) { D('mesh ack send failed: ' + (e && e.message), 'error'); }
        }
    }

    function connectMesh() {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const url = `${protocol}//${window.location.host}/ws-mesh?client_id=${clientId}`;
        D('mesh: connecting -> ' + url);
        try {
            meshSocket = new WebSocket(url);
        } catch (e) {
            D('mesh: constructor threw: ' + e.message, 'error');
            setTimeout(connectMesh, RECONNECT_MS);
            return;
        }
        meshSocket.binaryType = 'arraybuffer';

        meshSocket.onopen = () => { D('mesh: OPEN'); };

        meshSocket.onclose = (ev) => {
            D('mesh: CLOSE code=' + ev.code + ' reason="' + (ev.reason || '') + '" wasClean=' + ev.wasClean, 'error');
            setTimeout(connectMesh, RECONNECT_MS);   // reconnect-with-backoff
        };

        meshSocket.onerror = () => { D('mesh: ERROR (see close code next)', 'error'); };

        meshSocket.onmessage = (event) => {
            // The mesh socket only ever carries binary tag-3 frames, but demux
            // it the same defensive way as the main socket rather than assume.
            if (typeof event.data === 'string') {
                D('mesh: unexpected text frame dropped: ' + event.data.slice(0, 80), 'error');
                return;
            }
            const buffer = event.data;
            if (!(buffer instanceof ArrayBuffer) || buffer.byteLength < 4) {
                console.warn('[ws] mesh frame too short, dropped');
                return;
            }
            const tag = new DataView(buffer).getUint32(0, true);
            if (tag === TAG_MESH) emit('mesh', buffer);
            else console.warn('[ws] unexpected tag ' + tag + ' on /ws-mesh, dropped');
        };
    }

    function connect() {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const url = `${protocol}//${window.location.host}/ws?client_id=${clientId}`;
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
            else if (tag === TAG_MESH) emit('mesh', buffer);
            else if (tag === TAG_SURFACE) emit('surface_cloud', buffer);
            else if (tag === TAG_MAGPOSE) emit('magpose', buffer);
            else console.warn('[ws] unrecognized binary tag ' + tag + ', dropped');
        };
    }

    // One `connect()` call starts BOTH sockets — callers (app.js) don't need
    // to know the mesh socket exists.
    function connectAll() {
        connect();
        connectMesh();
    }

    return { on, emit, send, ackMesh, connect: connectAll };
}
