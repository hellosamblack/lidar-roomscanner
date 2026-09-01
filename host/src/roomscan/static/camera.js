// camera.js — the rig-camera monitor card (#226 Stage A, spec §5.5).
//
// PIXELS AND STATE TRAVEL SEPARATELY, and that separation is the whole design:
//
//   * pixels  -> one <img> pointed at the same-origin proxy
//                `/api/camera/stream.mjpg`, which the browser drives itself as
//                an ordinary HTTP multipart request. No camera byte ever
//                touches /ws, /ws-mesh or /ws-thin (§3 non-goals), so a webcam
//                cannot head-of-line block the 30 Hz pose JSON the way a mesh
//                once did (BUG-061).
//   * state   -> the low-rate `camera` hub message (<=4 Hz), which carries the
//                state word, the measured resolution/rates and the drop
//                counters, and nothing else.
//
// ONE STREAM PER TAB. The card's `<img>` is the only subscriber this page ever
// creates, and it is detached (`removeAttribute('src')`, which aborts the
// request) whenever the card is collapsed, hidden or frozen. That is the
// browser-side half of the §4.3 invariant "browser count must not multiply Pi
// camera opens" — the host already guarantees one upstream connection, and
// this guarantees the host is not paying for viewers that cannot see anything.
// A collapsed card whose <img> kept streaming would be invisible waste of
// exactly the kind BUG-060 measured on /ws.
//
// CACHE-BUSTING IS EDGE-TRIGGERED. The stream URL gains `?g=<generation>` and
// is reassigned ONLY when the server's camera generation actually changes (a
// reconnect, a camera reopen, a sequence reset) — never on a telemetry tick.
// Reassigning `src` tears down and reopens the multipart request, so doing it
// per message would give a permanently reconnecting picture that still reported
// a healthy state. Same trap as re-anchoring a periodic schedule to `now`
// (#216): the mechanism looks right and the far-side observable is wrong.
//
// FREEZE holds the last decoded bitmap by copying it into a <canvas>, then
// detaches the stream. It does NOT pretend the camera stopped: the state chip
// keeps rendering whatever the server reports, and a separate "Frozen" chip
// says the picture is held. Unfreezing re-attaches with the current generation.
//
// EXPAND floats the card large within the app (a class on the card, so the
// <img> is never re-parented and the request is never restarted). It is
// deliberately not native video Picture-in-Picture — that would need a <video>
// element and a product decision the spec explicitly defers.
//
// Public surface:  createCamera(hub) -> { setFrozen, isStreaming }
// Hub events:  subscribes "camera", "conn"

const D = (m, l) => { try { window.__diag && window.__diag('camera.js: ' + m, l); } catch (e) {} };

const STREAM_URL = '/api/camera/stream.mjpg';
const FRAME_URL = '/api/camera/frame.jpg';
// The four states Stage A can honestly report. Recording / Partial are named
// in spec §5.5 but belong to Stage B (#227): they describe a capture-bundle
// writer that does not exist yet, and a badge driven by a field no code
// populates would let "recording" read as healthy dual capture while only the
// scanner is being written. They arrive with the writer.
const STATES = ['offline', 'connecting', 'live', 'stale'];

export function createCamera(hub) {
    const card = document.getElementById('camera-card');
    const img = document.getElementById('camera-img');
    const hold = document.getElementById('camera-hold');
    const placeholder = document.getElementById('camera-placeholder');
    const chipState = document.getElementById('camera-state');
    const chipFrozen = document.getElementById('camera-frozen');
    const chkFreeze = document.getElementById('camera-freeze');
    const btnSnapshot = document.getElementById('camera-snapshot');
    const btnExpand = document.getElementById('camera-expand');
    const elRes = document.getElementById('camera-res');
    const elCaptureFps = document.getElementById('camera-capture-fps');
    const elHostFps = document.getElementById('camera-host-fps');
    const elAge = document.getElementById('camera-age');
    const elDrops = document.getElementById('camera-drops');
    const elError = document.getElementById('camera-error');
    if (!card || !img) { D('camera card DOM missing — skipping', 'error'); return {}; }

    let generation = null;        // server's camera generation, null until first message
    let attachedGeneration = null;  // the generation the current src was built for
    let frozen = false;

    // -- stream attach / detach ------------------------------------------

    function wanted() {
        // Every reason NOT to hold an open proxy request. Collapsed and hidden
        // are checked here rather than being trusted to CSS: `display:none`
        // does not abort an in-flight <img> request.
        return !frozen
            && generation !== null
            && !card.classList.contains('collapsed')
            && !card.classList.contains('hidden');
    }

    function attach() {
        if (attachedGeneration === generation) return;   // edge-triggered, see header
        img.src = STREAM_URL + '?g=' + generation;
        attachedGeneration = generation;
        img.classList.remove('hidden');
        D('stream attached at generation ' + generation);
    }

    function detach() {
        if (attachedGeneration === null) return;
        // removeAttribute, not `src = ''`: assigning the empty string re-resolves
        // against the document URL and re-requests the PAGE as an image in some
        // browsers. Removing the attribute aborts and leaves nothing pending.
        img.removeAttribute('src');
        attachedGeneration = null;
        D('stream detached');
    }

    function syncStream() {
        if (wanted()) attach(); else detach();
        const showHold = frozen && hold.width > 0;
        hold.classList.toggle('hidden', !showHold);
        img.classList.toggle('hidden', showHold || attachedGeneration === null);
        placeholder.classList.toggle('hidden', showHold || attachedGeneration !== null);
        chipFrozen.classList.toggle('hidden', !frozen);
    }

    // -- freeze ----------------------------------------------------------

    function captureHold() {
        // naturalWidth is 0 until at least one frame has decoded; freezing before
        // then must leave the placeholder up rather than paint a 0x0 canvas.
        const w = img.naturalWidth | 0, h = img.naturalHeight | 0;
        if (!w || !h) { hold.width = 0; return false; }
        hold.width = w; hold.height = h;
        try {
            hold.getContext('2d').drawImage(img, 0, 0, w, h);
        } catch (e) {
            D('could not hold the last frame: ' + e, 'warn');
            hold.width = 0;
            return false;
        }
        return true;
    }

    function setFrozen(on) {
        frozen = !!on;
        if (frozen) captureHold();
        if (chkFreeze) chkFreeze.checked = frozen;
        syncStream();
    }

    if (chkFreeze) chkFreeze.addEventListener('change', () => setFrozen(chkFreeze.checked));

    // -- snapshot --------------------------------------------------------

    if (btnSnapshot) btnSnapshot.addEventListener('click', async () => {
        // The host's newest ORIGINAL JPEG, not a canvas re-encode of what this
        // tab happens to have decoded: /api/camera/frame.jpg hands back the
        // camera's exact bytes, which is what Stage B will archive.
        try {
            const res = await fetch(FRAME_URL, { cache: 'no-store' });
            if (!res.ok) { D('snapshot unavailable: HTTP ' + res.status, 'warn'); return; }
            const gen = res.headers.get('X-Roomscan-Camera-Generation') || '0';
            const seq = res.headers.get('X-Roomscan-Camera-Seq') || '0';
            const blob = await res.blob();
            const url = URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = url;
            a.download = 'camera-g' + gen + '-s' + seq + '.jpg';
            document.body.appendChild(a);
            a.click();
            a.remove();
            setTimeout(() => URL.revokeObjectURL(url), 10000);
        } catch (e) {
            D('snapshot failed: ' + e, 'error');
        }
    });

    // -- expand ----------------------------------------------------------

    if (btnExpand) btnExpand.addEventListener('click', () => {
        const on = !card.classList.contains('is-expanded');
        card.classList.toggle('is-expanded', on);
        btnExpand.textContent = on ? 'Restore' : 'Expand';
        if (on) card.classList.remove('collapsed');
        syncStream();
        if (window.__relayout) window.__relayout();
    });

    document.addEventListener('keydown', (e) => {
        if (e.key === 'Escape' && card.classList.contains('is-expanded')) {
            card.classList.remove('is-expanded');
            if (btnExpand) btnExpand.textContent = 'Expand';
            syncStream();
        }
    });

    // The collapse toggle lives in layout.js (a classic script that owns every
    // card header), so the class is the only signal available here. Observing
    // it is what keeps a collapsed card from holding an open proxy request.
    new MutationObserver(syncStream).observe(card, {
        attributes: true, attributeFilter: ['class'],
    });

    // -- state message ---------------------------------------------------

    function fmt(value, digits, suffix) {
        if (value === null || value === undefined) return '--';
        return Number(value).toFixed(digits) + (suffix || '');
    }

    hub.on('camera', (msg) => {
        if (!msg) return;
        // Unknown state words fall back to Offline rather than being rendered
        // raw: an unstyled chip would read as "fine" (no colour = no alarm),
        // which is the wrong way to fail on a status readout.
        const state = STATES.indexOf(msg.state) >= 0 ? msg.state : 'offline';
        chipState.textContent = state.charAt(0).toUpperCase() + state.slice(1);
        chipState.className = 'camera-chip camera-chip--' + state;

        const nextGen = (typeof msg.generation === 'number') ? msg.generation : null;
        const genChanged = nextGen !== generation;
        generation = nextGen;
        // Only a real generation change reassigns src (see the header). A state
        // transition alone must not: `live -> stale -> live` on a jittery link
        // would otherwise restart the request every couple of seconds.
        if (genChanged) syncStream(); else if (wanted() && attachedGeneration === null) syncStream();

        elRes.textContent = (msg.width && msg.height) ? (msg.width + '×' + msg.height) : '--';
        elCaptureFps.textContent = fmt(msg.capture_fps, 1);
        elHostFps.textContent = fmt(msg.host_fps, 1);
        elAge.textContent = fmt(msg.age_ms, 0, ' ms');
        const src = (msg.source_drops === null || msg.source_drops === undefined) ? '--' : msg.source_drops;
        const ing = (msg.ingest_drops === null || msg.ingest_drops === undefined) ? '--' : msg.ingest_drops;
        elDrops.textContent = src + ' / ' + ing;
        elDrops.classList.toggle('warn', (msg.source_drops | 0) > 0 || (msg.ingest_drops | 0) > 0);
        elError.textContent = msg.last_error || '--';
        elError.classList.toggle('lost', !!msg.last_error);
    });

    // A dropped /ws socket says nothing about the camera proxy, which is a
    // separate HTTP request — but it does mean the state readouts have stopped
    // updating, and a stale "Live" chip beside a frozen picture is the exact
    // lie this card exists to avoid.
    hub.on('conn', (e) => {
        if (e && e.state !== 'open') {
            chipState.textContent = 'Offline';
            chipState.className = 'camera-chip camera-chip--offline';
        }
    });

    syncStream();
    return {
        setFrozen,
        isStreaming: () => attachedGeneration !== null,
    };
}
