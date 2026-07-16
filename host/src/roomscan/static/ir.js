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
// Public surface:  createIr(hub) -> {}
// Hub events:  subscribes "ir_image", "state", "ir_show";  sends set_ir via hub.send

const D = (m, l) => { try { window.__diag && window.__diag('ir.js: ' + m, l); } catch (e) {} };

export function createIr(hub) {
    const card = document.getElementById('ir-card');
    const canvas = document.getElementById('ir-canvas');
    const segColormap = document.getElementById('ir-card-colormap');
    const chkFreeze = document.getElementById('ir-card-freeze');
    const btnClose = document.getElementById('ir-card-close');
    if (!card || !canvas) { D('IR card DOM missing — skipping', 'error'); return {}; }

    const ctx = canvas.getContext('2d');
    let imageData = null;   // reused ImageData, reallocated only when size changes

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

    // Server state drives the toggles (not local clicks) so a second tab syncs.
    hub.on('state', (msg) => {
        if (segColormap) {
            for (const b of segColormap.querySelectorAll('button')) {
                b.classList.toggle('active', b.dataset.colormap === msg.ir_colormap);
            }
        }
        if (chkFreeze) chkFreeze.checked = !!msg.ir_freeze;
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

    // Show/hide: local presentation only (not server state). Driven from the
    // right rail's IR group via the hub, plus the card's own close button.
    function setVisible(v) { card.classList.toggle('hidden', !v); }
    hub.on('ir_show', (v) => setVisible(!!v));
    if (btnClose) btnClose.addEventListener('click', () => { setVisible(false); hub.emit('ir_shown', false); });

    return {};
}
