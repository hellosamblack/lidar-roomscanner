// ir.js — the IR (reflectance) monitor <canvas> corner card (bottom-left).
//
// Subscribes to "ir_image", parses the tag+width+height+RGB layout (§6.1) and
// blits it with putImageData. The canvas backing store is sized to the incoming
// width/height (which can change with usecase/binning — read from the message,
// never assumed) and CSS `image-rendering: pixelated` upscales the low-res grid
// crisply, so no bytes are wasted upscaling on the wire.
//
// The card header carries a gray/turbo colormap toggle + a Freeze checkbox; both
// send `set_ir` via the hub and reflect the server `state` echo (never local
// click state) — one-way state flow (§8.3). The card is show/hide toggleable,
// driven by the right rail's IR group through the hub ("ir_show").
//
// GRAVITY ROLL. The pane is kept level with physical gravity so it agrees with the
// gravity-aligned point cloud. That happens in two parts: the server pre-rotates by
// whole quarter turns (`ir_gravity_rot`, pixel-exact and free, which is why the
// incoming width/height swap on a 90°/270° turn), and this module finishes the
// remaining ≤45° from the `sensor` message's `ir_roll_deg` using a CSS transform —
// so the 54x42 image is never resampled and stays pixel-crisp. `ir_roll_deg` is
// CCW-positive (np.rot90's sense); CSS rotates clockwise, hence the negation.
//
// The image spins inside a fixed SQUARE frame (#ir-frame), scaled so the rotated
// bounding box always fits: the card never changes shape as the board rolls and no
// field of view is cropped, at the cost of empty corners at intermediate angles.
//
// MIRROR. The Mirror view mode (`state.view_mode`) flips the pane left-right with
// a signed scale on the same transform, matching the server's X negation of the
// point cloud (the IR raster and the cloud grid share one index space). It is the
// only view mode this pane reacts to: World and FPV render identically.
//
// Public surface:  createIr(hub) -> {}
// Hub events:  subscribes "ir_image", "sensor", "state", "ir_show";  sends set_ir via hub.send

const D = (m, l) => { try { window.__diag && window.__diag('ir.js: ' + m, l); } catch (e) {} };

export function createIr(hub) {
    const card = document.getElementById('ir-card');
    const frame = document.getElementById('ir-frame');
    const canvas = document.getElementById('ir-canvas');
    const segColormap = document.getElementById('ir-card-colormap');
    const chkFreeze = document.getElementById('ir-card-freeze');
    const btnClose = document.getElementById('ir-card-close');
    if (!card || !canvas) { D('IR card DOM missing — skipping', 'error'); return {}; }

    const ctx = canvas.getContext('2d');
    let imageData = null;   // reused ImageData, reallocated only when size changes
    let imgW = 0, imgH = 0; // last image dims, for the fit math
    let rollDeg = 0;        // residual gravity roll, CCW-positive degrees
    let mirror = false;     // Mirror view mode: flip the pane left-right

    // Size + rotate the canvas so the rotated image is inscribed in the square
    // frame. Fit the unrotated image first (its long side spans the frame), then
    // shrink by however much the rotation grows the bounding box.
    function layout() {
        if (!frame || !imgW || !imgH) return;
        const side = frame.clientWidth;         // square, so width == height
        if (!side) return;                      // card hidden: nothing to lay out
        const base = side / Math.max(imgW, imgH);
        const cssW = imgW * base, cssH = imgH * base;
        const rad = Math.abs(rollDeg) * Math.PI / 180;
        const c = Math.abs(Math.cos(rad)), s = Math.abs(Math.sin(rad));
        const boxW = cssW * c + cssH * s;       // rotated bounding box
        const boxH = cssW * s + cssH * c;
        const k = side / Math.max(boxW, boxH, 1e-6);
        canvas.style.width = cssW + 'px';
        canvas.style.height = cssH + 'px';
        // translate(-50%,-50%) centres it; the frame's own CSS pins top/left to 50%.
        // ORDER MATTERS in Mirror: CSS applies the list right-to-left, so `scale`
        // listed BEFORE `rotate` flips the already-rotated pane — i.e. it mirrors
        // the finished, gravity-corrected image, which is what "mirror the IR
        // view" means. (A uniform scale commutes with rotate, which is why the
        // original order was harmless; a signed one does not.)
        canvas.style.transform =
            `translate(-50%, -50%) scale(${mirror ? -k : k}, ${k}) rotate(${-rollDeg}deg)`;
    }

    // The residual roll rides the `sensor` message (the orientation message), not
    // the IR binary, so no binary tag had to change. Silent on a ToF-only session.
    hub.on('sensor', (msg) => {
        const r = (msg && typeof msg.ir_roll_deg === 'number') ? msg.ir_roll_deg : 0;
        if (r === rollDeg) return;              // avoid needless style writes at 30 Hz
        rollDeg = r;
        layout();
    });

    // The frame is a % of the card, so a resize changes `side`.
    try { new ResizeObserver(layout).observe(frame); } catch (e) { /* older browser */ }

    hub.on('ir_image', (buffer) => {
        const view = new DataView(buffer);
        // u32 tag · u16 width · u16 height · u8[w*h*3] RGB, all little-endian.
        const width = view.getUint16(4, true);
        const height = view.getUint16(6, true);
        const rgb = new Uint8Array(buffer, 8, width * height * 3);
        if (width <= 0 || height <= 0 || rgb.length < width * height * 3) return;

        if (canvas.width !== width || canvas.height !== height) {
            canvas.width = width;
            canvas.height = height;
            imageData = ctx.createImageData(width, height);
            // Dims swap on a 90°/270° server-side snap (54x42 -> 42x54), so the
            // fit math has to re-run; never assume landscape.
            imgW = width; imgH = height;
            layout();
        }
        const out = imageData.data;   // RGBA
        for (let i = 0, j = 0; i < width * height; i++) {
            out[j++] = rgb[i * 3];
            out[j++] = rgb[i * 3 + 1];
            out[j++] = rgb[i * 3 + 2];
            out[j++] = 255;
        }
        ctx.putImageData(imageData, 0, 0);
        if (!window.__gotIr) { window.__gotIr = true; D('first IR frame: ' + width + 'x' + height); }
    });

    // Client reset barrier (issue #101 step 4): a generation/readiness change
    // means the pixels currently painted belong to a source the server has
    // already moved past. Blank the canvas and forget the last-known image
    // dims + gravity roll so the NEXT `ir_image` re-lays-out from scratch
    // rather than reusing stale orientation state.
    hub.on('stream_reset', () => {
        if (imgW && imgH) ctx.clearRect(0, 0, canvas.width, canvas.height);
        imgW = 0; imgH = 0;
        rollDeg = 0;
        window.__gotIr = false;
    });

    // Server state drives the toggles (not local clicks) so a second tab syncs.
    hub.on('state', (msg) => {
        if (segColormap) {
            for (const b of segColormap.querySelectorAll('button')) {
                b.classList.toggle('active', b.dataset.colormap === msg.ir_colormap);
            }
        }
        if (chkFreeze) chkFreeze.checked = !!msg.ir_freeze;
        // Mirror is the ONLY view mode that touches the IR pane (owner: "IR view
        // is unchanged for all but mirror") — World and FPV both leave the
        // gravity roll exactly as it is.
        const m = (msg.view_mode === 'mirror');
        if (m !== mirror) { mirror = m; layout(); }
    });

    // Read current desired settings from the DOM and emit one set_ir.
    function sendIr(colormap, freeze) {
        hub.send({ type: 'set_ir', colormap, freeze });
    }

    if (segColormap) {
        segColormap.addEventListener('click', (e) => {
            const btn = e.target.closest('button[data-colormap]');
            if (!btn) return;
            sendIr(btn.dataset.colormap, chkFreeze ? chkFreeze.checked : false);
        });
    }
    if (chkFreeze) {
        chkFreeze.addEventListener('change', () => {
            const active = segColormap && segColormap.querySelector('button.active');
            sendIr(active ? active.dataset.colormap : 'gray', chkFreeze.checked);
        });
    }

    // Close: COLLAPSE, don't hide. The right rail's "Show IR card" checkbox was
    // the only way back from a hidden card, and it went with the duplicated IR
    // control group (2026-07-31). Collapsing routes the card through layout.js's
    // squircle rail instead, which is both the way back and a persisted choice —
    // `.hidden` is neither. Click-bubbling to the header would also toggle, so
    // stop propagation rather than let the two fight.
    if (btnClose) {
        btnClose.addEventListener('click', (e) => {
            e.stopPropagation();
            card.classList.add('collapsed');
            try { localStorage.setItem('roomscan.card.ir-view.collapsed', '1'); } catch (err) {}
            window.dispatchEvent(new Event('resize'));   // let layout.js re-flow the dock
        });
    }

    return {};
}
