// controls.js — the right-rail interactive control panel (§8.1 right rail).
//
// Turns DOM events into hub.send(...) messages and nothing else. Active state for
// anything the server tracks (color mode, IR colormap/freeze) is driven FROM the
// server's `state` echo, never from local click state — one-way state flow (§8.3),
// so a change in one tab reflects in every open tab for free. Usecase and the
// device buttons are one-shot actions (not persistent server state) so they just
// fire a `cmd`; their result surfaces as a toast/log line via log.js.
//
// Also owns generic control-group collapse (delegated header clicks) and the IR
// card show/hide toggle (a local presentation signal relayed over the hub).
//
// Public surface:  createControls(hub) -> {}
// Hub events:  subscribes "state", "ir_shown";  emits "reset_camera", "ir_show";
//              sends cmd / set_color / set_ir via hub.send

export function createControls(hub) {
    const $ = (id) => document.getElementById(id);

    // --- Device group: one-shot commands ---
    const cmd = (name, param = 0) => hub.send({ type: 'cmd', name, param });
    $('btn-ping')?.addEventListener('click', () => cmd('ping'));
    $('btn-calib')?.addEventListener('click', () => cmd('calib'));
    $('btn-reinit')?.addEventListener('click', () => cmd('reinit'));

    // --- Usecase segmented control (action, not persistent state) ---
    const segUsecase = $('seg-usecase');
    segUsecase?.addEventListener('click', (e) => {
        const btn = e.target.closest('button[data-uc]');
        if (btn) cmd('usecase', parseInt(btn.dataset.uc, 10));
    });

    // --- View group: color mode segmented control (server-driven active) ---
    const segColor = $('seg-color');
    segColor?.addEventListener('click', (e) => {
        const btn = e.target.closest('button[data-mode]');
        if (btn) hub.send({ type: 'set_color', mode: btn.dataset.mode });
    });
    $('btn-reset-cam')?.addEventListener('click', () => hub.emit('reset_camera'));

    // --- IR Monitor group: colormap + freeze (server-driven) + card show/hide ---
    const segIrColormap = $('seg-ir-colormap');
    const chkIrFreeze = $('chk-ir-freeze');
    const chkIrShow = $('chk-ir-show');

    function sendIr(colormap, freeze) { hub.send({ type: 'set_ir', colormap, freeze }); }
    segIrColormap?.addEventListener('click', (e) => {
        const btn = e.target.closest('button[data-colormap]');
        if (btn) sendIr(btn.dataset.colormap, chkIrFreeze ? chkIrFreeze.checked : false);
    });
    chkIrFreeze?.addEventListener('change', () => {
        const active = segIrColormap && segIrColormap.querySelector('button.active');
        sendIr(active ? active.dataset.colormap : 'gray', chkIrFreeze.checked);
    });
    // Card show/hide is local presentation; relay it to ir.js over the hub.
    chkIrShow?.addEventListener('change', () => hub.emit('ir_show', chkIrShow.checked));
    hub.on('ir_shown', (v) => { if (chkIrShow) chkIrShow.checked = !!v; });   // card's own close btn

    // --- server state echo drives active segments (§7.2) ---
    hub.on('state', (msg) => {
        setActive(segColor, 'mode', msg.color_mode);
        setActive(segIrColormap, 'colormap', msg.ir_colormap);
        if (chkIrFreeze) chkIrFreeze.checked = !!msg.ir_freeze;
    });

    function setActive(seg, attr, value) {
        if (!seg) return;
        for (const b of seg.querySelectorAll('button')) {
            b.classList.toggle('active', b.dataset[attr] === value);
        }
    }

    // --- generic control-group collapse (delegated on the right rail) ---
    const rail = $('right-rail');
    rail?.addEventListener('click', (e) => {
        const header = e.target.closest('.control-group__header');
        if (header && rail.contains(header)) {
            header.parentElement.classList.toggle('collapsed');
        }
    });

    return {};
}
