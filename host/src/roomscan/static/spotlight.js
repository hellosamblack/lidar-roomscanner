// spotlight.js — cursor-follow edge highlight on the chrome cards.
//
// Sets each card's --mx/--my to the pointer position (in card-local px); the CSS
// (.card[data-card-id]::after) paints a radial highlight masked to the 1px
// border. Pure DOM, no hub: it neither reads nor writes server state, so it
// takes no arguments and talks to nothing else (§8.3 — a module owns one
// concern). Cheap by construction: one delegated pointermove, coalesced to a
// frame, and the repaint area is the border only. Honours reduced-motion.

const D = (m) => { try { window.__diag && window.__diag('spotlight.js: ' + m); } catch (e) {} };

export function createSpotlight() {
    // Match the CSS scope exactly: cards that carry a data-card-id (modals and
    // the top bar deliberately do not).
    const SELECTOR = '.card[data-card-id]';

    if (window.matchMedia && window.matchMedia('(prefers-reduced-motion: reduce)').matches) {
        D('reduced-motion: spotlight disabled');
        return;
    }

    let lit = null;        // the card currently showing the glow
    let pending = null;    // {card, x, y} awaiting the next frame
    let raf = 0;

    const paint = () => {
        raf = 0;
        if (!pending) return;
        const { card, x, y } = pending;
        card.style.setProperty('--mx', x + 'px');
        card.style.setProperty('--my', y + 'px');
    };

    // One listener on document; find the card under the pointer per move. Passive
    // — we never preventDefault, so this can't interfere with scroll/drag.
    document.addEventListener('pointermove', (e) => {
        const card = e.target.closest ? e.target.closest(SELECTOR) : null;
        if (card !== lit) {
            if (lit) lit.classList.remove('is-spotlit');
            lit = card;
            if (lit) lit.classList.add('is-spotlit');
        }
        if (!card) return;
        const r = card.getBoundingClientRect();
        pending = { card, x: e.clientX - r.left, y: e.clientY - r.top };
        if (!raf) raf = requestAnimationFrame(paint);
    }, { passive: true });

    // Clear the glow when the pointer leaves the window entirely.
    document.addEventListener('pointerleave', () => {
        if (lit) { lit.classList.remove('is-spotlit'); lit = null; }
    });

    D('ready');
}
