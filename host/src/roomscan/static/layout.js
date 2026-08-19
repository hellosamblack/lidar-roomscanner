// layout.js — dock layout manager + card collapse toggles.
//
// A CLASSIC script (not an ES module) on purpose: the dock layout — and the
// event-log console that diagnostics fold into — must stay usable even when the
// module graph below fails to load, so neither may depend on it resolving.
//
// The anti-overlap contract (see the `.dock` block in index.html):
//
//   * every floating block lives in #left-dock or #right-rail, both of which
//     are height-bounded column-wrapping flex containers. CSS alone therefore
//     guarantees blocks never overlap each other, the top bar, or the console:
//     a stack that would run past the bottom of the band wraps to a NEW COLUMN.
//   * this file keeps that band honest (--dock-top / --dock-bottom track the
//     measured top-bar and event-log heights, so collapsing the console hands
//     its space back to the docks), and resolves the one collision CSS cannot
//     see: the left dock's columns marching into the right dock's.
//
// Collision resolution re-derives from a clean baseline every pass (so it never
// oscillates or leaves a stale degradation behind), degrading in cheapest-first
// order until both sides clear:
//
//   1. scroll the right dock            (one column; content still reachable)
//   2. collapse the IR monitor          (its own card, canvas hidden)
//   3. collapse the sensors card
//
// Public surface (for debugging / headless drivers):  window.__relayout()

(function () {
    'use strict';

    var GAP = 16;                               // min clear gap between the docks
    var EDGE = 16;                              // min clear gap to a viewport edge

    var root = document.documentElement;
    function $(id) { return document.getElementById(id); }

    var STORAGE_PREFIX = 'roomscan.card.';

    function getCardKey(cardId) {
        return STORAGE_PREFIX + cardId + '.collapsed';
    }

    var CARD_ICONS = {
        'telemetry': '<svg viewBox="0 0 24 24" width="20" height="20"><path fill="currentColor" d="M12,3C6.5,3 2,7.5 2,13C2,15.5 3,17.8 4.7,19.5L6.1,18.1C4.8,16.8 4,15 4,13C4,8.6 7.6,5 12,5C16.4,5 20,8.6 20,13C20,15 19.2,16.8 17.9,18.1L19.3,19.5C21,17.8 22,15.5 22,13C22,7.5 17.5,3 12,3M12,7C8.7,7 6,9.7 6,13C6,14.7 6.7,16.2 7.8,17.3L9.2,15.9C8.5,15.2 8,14.1 8,13C8,10.8 9.8,9 12,9C14.2,9 16,10.8 16,13C16,14.1 15.5,15.2 14.8,15.9L16.2,17.3C17.3,16.2 18,14.7 18,13C18,9.7 15.3,7 12,7M12,11A2,2 0 0,0 10,13A2,2 0 0,0 12,15A2,2 0 0,0 14,13A2,2 0 0,0 12,11Z"/></svg>',
        'sensors': '<svg viewBox="0 0 24 24" width="20" height="20"><path fill="currentColor" d="M12 2L4.5 20.29l.71.71L12 18l6.79 3 .71-.71L12 2zm0 4.18l4.41 10.7-3.91-1.73-.5-.22-.5.22-3.91 1.73L12 6.18z"/></svg>',
        'slam-hud': '<svg viewBox="0 0 24 24" width="20" height="20"><path fill="currentColor" d="M21,16.5C21,16.88 20.79,17.21 20.47,17.38L12.57,21.82C12.41,21.94 12.21,22 12,22C11.79,22 11.59,21.94 11.43,21.82L3.53,17.38C3.21,17.21 3,16.88 3,16.5V7.5C3,7.12 3.21,6.79 3.53,6.62L11.43,2.18C11.59,2.06 11.79,2 12,2C12.21,2 12.41,2.06 12.57,2.18L20.47,6.62C20.79,6.79 21,7.12 21,7.5V16.5M12,4.15L5.04,8.05L12,11.95L18.96,8.05L12,4.15Z"/></svg>',
        'ir-view': '<svg viewBox="0 0 24 24" width="20" height="20"><path fill="currentColor" d="M12,9A3,3 0 0,0 9,12A3,3 0 0,0 12,15A3,3 0 0,0 15,12A3,3 0 0,0 12,9M12,4.5C7,4.5 2.73,7.61 1,12C2.73,16.39 7,19.5 12,19.5C17,19.5 21.27,16.39 23,12C21.27,7.61 17,4.5 12,4.5M12,17.5A5.5,5.5 0 0,1 6.5,12A5.5,5.5 0 0,1 12,6.5A5.5,5.5 0 0,1 17.5,12A5.5,5.5 0 0,1 12,17.5Z"/></svg>',
        'device': '<svg viewBox="0 0 24 24" width="20" height="20"><path fill="currentColor" d="M6 2h12a2 2 0 0 1 2 2v16a2 2 0 0 1-2 2H6a2 2 0 0 1-2-2V4a2 2 0 0 1 2-2m3 2v2h2V4H9m4 0v2h2V4h-2m-4 14v2h2v-2H9m4 0v2h2v-2h-2M4 9h2v2H4V9m0 4h2v2H4v-2m14-4h2v2h-2V9m0 4h2v2h-2v-2z"/></svg>',
        'view': '<svg viewBox="0 0 24 24" width="20" height="20"><path fill="currentColor" d="M18 4l2 4h-3l-2-4h-2l2 4h-3l-2-4H8l2 4H7L5 4H4c-1.1 0-1.99.9-1.99 2L2 18c0 1.1.9 2 2 2h16c1.1 0 2-.9 2-2V4h-4zm-6 13l-5-3 5-3v6z"/></svg>',
        'browser': '<svg viewBox="0 0 24 24" width="20" height="20"><path fill="currentColor" d="M3,3H11V11H3V3M13,3H21V11H13V3M3,13H11V21H3V13M13,13H21V21H13V13Z"/></svg>',
        'preview': '<svg viewBox="0 0 24 24" width="20" height="20"><path fill="currentColor" d="M4,4H20A2,2 0 0,1 22,6V18A2,2 0 0,1 20,20H4A2,2 0 0,1 2,18V6A2,2 0 0,1 4,4M4,6V18H20V6H4M6,8H18V10H6V8M6,12H14V14H6V12Z"/></svg>',
        'slam-ctrl': '<svg viewBox="0 0 24 24" width="20" height="20"><path fill="currentColor" d="M12,2L4.5,20.29L5.21,21L12,18L18.79,21L19.5,20.29L12,2Z"/></svg>',
        'resources': '<svg viewBox="0 0 24 24" width="20" height="20"><path fill="currentColor" d="M4,2H20A2,2 0 0,1 22,4V20A2,2 0 0,1 20,22H4A2,2 0 0,1 2,20V4A2,2 0 0,1 4,2M6,6V18H8V6H6M10,10V18H12V10H10M14,8V18H16V8H14M18,13V18H20V13H18Z"/></svg>',
        'log': '<svg viewBox="0 0 24 24" width="20" height="20"><path fill="currentColor" d="M19,3H5C3.89,3 3,3.89 3,5V19C3,20.1 3.89,21 5,21H19C20.1,21 21,20.1 21,19V5C21,3.89 20.1,3 19,3M7,7H17V9H7V7M7,11H17V13H7V11M7,15H14V17H7V15Z"/></svg>',
        'splat': '<svg viewBox="0 0 24 24" width="20" height="20"><path fill="currentColor" d="M5 5a2 2 0 1 1 0 4 2 2 0 0 1 0-4m14 0a2 2 0 1 1 0 4 2 2 0 0 1 0-4M12 10a2 2 0 1 1 0 4 2 2 0 0 1 0-4M5 15a2 2 0 1 1 0 4 2 2 0 0 1 0-4m14 0a2 2 0 1 1 0 4 2 2 0 0 1 0-4Z"/></svg>'
    };

    var CARD_TITLES = {
        'telemetry': 'Stream',
        'sensors': 'Sensors',
        'slam-hud': 'SLAM HUD',
        'ir-view': 'IR Monitor',
        'device': 'Device',
        'view': 'View',
        'browser': 'Captures',
        'preview': 'Preview',
        'slam-ctrl': 'SLAM',
        'resources': 'Resources',
        'log': 'Event Log',
        'splat': 'Splat rooms'
    };

    // Primary (left) and Secondary (right) bar visibility toggling
    function togglePrimaryBar(show) {
        var bar = $('primary-bar') || document.querySelector('.sidebar--left');
        if (!bar) return;
        var isHidden = (show === undefined) ? !bar.classList.contains('is-hidden') : !show;
        bar.classList.toggle('is-hidden', isHidden);
        try { localStorage.setItem('roomscan.bar.primary.hidden', isHidden ? '1' : '0'); } catch (e) {}
        var btn = $('btn-toggle-primary-bar');
        if (btn) btn.classList.toggle('is-active', !isHidden);
    }

    function toggleSecondaryBar(show) {
        var bar = $('secondary-bar') || document.querySelector('.sidebar--right');
        if (!bar) return;
        var isHidden = (show === undefined) ? !bar.classList.contains('is-hidden') : !show;
        bar.classList.toggle('is-hidden', isHidden);
        try { localStorage.setItem('roomscan.bar.secondary.hidden', isHidden ? '1' : '0'); } catch (e) {}
        var btn = $('btn-toggle-secondary-bar');
        if (btn) btn.classList.toggle('is-active', !isHidden);
    }

    function setSidebarWidth(bar, width) {
        if (!bar) return;
        var minW = 220, maxW = Math.min(500, Math.floor(window.innerWidth * 0.45));
        var clampedW = Math.max(minW, Math.min(maxW, width));
        bar.style.width = clampedW + 'px';
        return clampedW;
    }

    function initSidebarStates() {
        try {
            var priHidden = localStorage.getItem('roomscan.bar.primary.hidden');
            if (priHidden === '1') togglePrimaryBar(false);
            var secHidden = localStorage.getItem('roomscan.bar.secondary.hidden');
            if (secHidden === '1') toggleSecondaryBar(false);

            var priW = parseInt(localStorage.getItem('roomscan.bar.primary.width'), 10);
            if (!isNaN(priW)) setSidebarWidth($('primary-bar'), priW);

            var secW = parseInt(localStorage.getItem('roomscan.bar.secondary.width'), 10);
            if (!isNaN(secW)) setSidebarWidth($('secondary-bar'), secW);
        } catch (e) {}
    }

    function setupSidebarResizing() {
        function attachHandle(handleId, barId, side) {
            var handle = $(handleId);
            var bar = $(barId);
            if (!handle || !bar) return;

            var startX = 0, startW = 0, dragged = false;

            function onMouseMove(e) {
                var dx = e.clientX - startX;
                if (Math.abs(dx) > 3) dragged = true;
                var newW = side === 'left' ? (startW + dx) : (startW - dx);
                setSidebarWidth(bar, newW);
                schedule();
            }

            function onMouseUp(e) {
                window.removeEventListener('mousemove', onMouseMove);
                window.removeEventListener('mouseup', onMouseUp);
                handle.classList.remove('is-dragging');
                document.body.style.cursor = '';
                document.body.style.userSelect = '';

                if (dragged) {
                    var finalW = bar.offsetWidth;
                    try {
                        localStorage.setItem('roomscan.bar.' + (side === 'left' ? 'primary' : 'secondary') + '.width', finalW);
                    } catch (err) {}
                }
            }

            handle.addEventListener('mousedown', function (e) {
                e.preventDefault();
                startX = e.clientX;
                startW = bar.offsetWidth;
                dragged = false;
                handle.classList.add('is-dragging');
                document.body.style.cursor = 'col-resize';
                document.body.style.userSelect = 'none';
                window.addEventListener('mousemove', onMouseMove);
                window.addEventListener('mouseup', onMouseUp);
            });

            handle.addEventListener('click', function (e) {
                if (!dragged) {
                    if (side === 'left') togglePrimaryBar();
                    else toggleSecondaryBar();
                    schedule();
                }
            });
        }

        attachHandle('primary-bar-handle', 'primary-bar', 'left');
        attachHandle('secondary-bar-handle', 'secondary-bar', 'right');
    }

    window.__togglePrimaryBar = togglePrimaryBar;
    window.__toggleSecondaryBar = toggleSecondaryBar;

    // The rail is a stable map of every panel that CAN exist, not a list of
    // what's missing: every registered card gets a permanent button, dimmed or
    // lit by its collapse state. `.hidden` is reserved for cards that are
    // genuinely absent from the DOM right now (e.g. slam-hud/slam-ctrl before
    // SLAM arms) -- a button for a card that cannot exist is a dead control.
    function updateSquircles() {
        // Disabled: sidebars use clean section accordions instead of floating squircle buttons
        return;
    }

    var HEADER_SELECTOR = '.control-group__header, .log-console__header, .ir-card__header';

    // Same SVG as the squircle rail (CARD_ICONS is the single source -- do not
    // duplicate the markup into index.html, the two copies would drift), sized
    // down via the `.card-icon` CSS class. Idempotent: safe to call repeatedly.
    function injectCardIcons() {
        var cards = document.querySelectorAll('[data-card-id]');
        for (var i = 0; i < cards.length; i++) {
            var card = cards[i];
            var id = card.getAttribute('data-card-id');
            if (!id || !CARD_ICONS[id]) continue;
            var header = card.querySelector(HEADER_SELECTOR);
            if (!header) continue;
            if (header.querySelector('.card-icon')) continue;
            header.insertAdjacentHTML('afterbegin', CARD_ICONS[id]);
            var icon = header.firstElementChild;
            if (icon) icon.classList.add('card-icon');
        }
    }

    /* ------------------------------------------------------------------ *
     * Card collapse persistence & delegation                            *
     * ------------------------------------------------------------------ */
    function initCardStates() {
        initSidebarStates();
        var cards = document.querySelectorAll('[data-card-id]');
        for (var i = 0; i < cards.length; i++) {
            var card = cards[i];
            var id = card.getAttribute('data-card-id');
            if (!id) continue;
            var key = getCardKey(id);
            var pref = null;
            try { pref = localStorage.getItem(key); } catch (e) {}
            if (pref !== null) {
                card.classList.toggle('collapsed', pref === '1');
            }
            // (The event-log console auto-opens on the first diagnostic error;
            // that is done by the head __diag script directly, before this runs.)
        }

        var subgroups = document.querySelectorAll('details[data-subgroup-id]');
        for (var j = 0; j < subgroups.length; j++) {
            var sg = subgroups[j];
            var sgId = sg.getAttribute('data-subgroup-id');
            if (!sgId) continue;
            var sgKey = 'roomscan.subgroup.' + sgId + '.open';
            var sgPref = null;
            try { sgPref = localStorage.getItem(sgKey); } catch (e) {}
            if (sgPref !== null) {
                sg.open = (sgPref === '1');
            }
        }
        injectCardIcons();
        updateSquircles();
    }

    function saveCardState(card) {
        var id = card.getAttribute('data-card-id');
        if (!id) return;
        var isCollapsed = card.classList.contains('collapsed');
        var key = getCardKey(id);
        try {
            localStorage.setItem(key, isCollapsed ? '1' : '0');
        } catch (e) {}
    }

    function setupCollapseDelegation() {
        var btnPri = $('btn-toggle-primary-bar');
        if (btnPri) {
            btnPri.addEventListener('click', function () { togglePrimaryBar(); schedule(); });
        }
        var btnSec = $('btn-toggle-secondary-bar');
        if (btnSec) {
            btnSec.addEventListener('click', function () { toggleSecondaryBar(); schedule(); });
        }

        document.addEventListener('keydown', function (e) {
            var tag = e.target && e.target.tagName ? e.target.tagName.toUpperCase() : '';
            if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT' || (e.target && e.target.isContentEditable)) return;
            if (e.ctrlKey || e.altKey || e.metaKey) return;

            if (e.key === '[') {
                togglePrimaryBar();
                schedule();
            } else if (e.key === ']') {
                toggleSecondaryBar();
                schedule();
            }
        });

        document.addEventListener('click', function (e) {
            var sqBtn = e.target.closest('.squircle-btn[data-for]');
            if (sqBtn) {
                var cardId = sqBtn.getAttribute('data-for');
                var card = document.querySelector('[data-card-id="' + cardId + '"]');
                if (card) {
                    // Toggle: a dim (collapsed) squircle expands its card, a
                    // bright (open) one collapses it -- the rail is a map AND
                    // a control, not just a "restore" button anymore.
                    card.classList.toggle('collapsed');
                    saveCardState(card);
                    updateSquircles();
                    schedule();
                }
                return;
            }

            var header = e.target.closest('.control-group__header, .log-console__header, .ir-card__header');
            if (!header) return;

            // Don't toggle collapse if clicking interactive controls inside header (e.g. colormap, freeze, close btn)
            if (e.target.closest('input, select, label, .segmented, .topbar-btn, .ir-card__close')) {
                return;
            }

            var card = header.closest('[data-card-id], .card, #log-console');
            if (!card) return;

            var collapsed = card.classList.toggle('collapsed');
            saveCardState(card);
            updateSquircles();
            schedule();
        });

        // Watch toggle events on collapsible sub-groups (<details data-subgroup-id="...">)
        document.addEventListener('toggle', function (e) {
            var sg = e.target;
            if (sg && sg.matches && sg.matches('details[data-subgroup-id]')) {
                var sgId = sg.getAttribute('data-subgroup-id');
                if (sgId) {
                    try {
                        localStorage.setItem('roomscan.subgroup.' + sgId + '.open', sg.open ? '1' : '0');
                    } catch (err) {}
                    schedule();
                }
            }
        }, true);
    }

    /* ------------------------------------------------------------------ *
     * View-focus policy (issue #121)                                     *
     *                                                                     *
     * Entering View makes the Captures browser the focal card: picking a *
     * capture is the first action there, so it should be the thing in    *
     * front of you, and every other sidebar card should get out of the   *
     * way. This is a pure card-COLLAPSE decision -- it never touches the *
     * hidden/visible class each card's own owning module already drives  *
     * off `state` (that stays exactly as it was).                        *
     *                                                                     *
     * The transition-EDGE part (only fire once, on Live->View, never on  *
     * every `state` re-broadcast, never fight a user who re-expands a    *
     * card afterward) lives in the caller (browser.js's `state` handler,  *
     * which already tracks the previous `source` for `collapseFor-        *
     * MapDisplay`). This function is just the mechanism: collapse every  *
     * OTHER card living in the two sidebars, expand the focal one, using *
     * the SAME localStorage-backed collapse persistence a manual header  *
     * click uses (`saveCardState`'s key format) so it behaves exactly    *
     * like the user had clicked each header themselves.                  *
     * ------------------------------------------------------------------ */
    function sidebarCardIds() {
        var out = [];
        var bars = [$('primary-bar'), $('secondary-bar')].filter(Boolean);
        for (var b = 0; b < bars.length; b++) {
            var cards = bars[b].querySelectorAll('[data-card-id]');
            for (var i = 0; i < cards.length; i++) {
                var id = cards[i].getAttribute('data-card-id');
                if (id) out.push(id);
            }
        }
        return out;
    }

    function setCardCollapsed(id, collapsed) {
        var card = document.querySelector('[data-card-id="' + id + '"]');
        if (!card) return;
        card.classList.toggle('collapsed', collapsed);
        try { localStorage.setItem(getCardKey(id), collapsed ? '1' : '0'); } catch (e) {}
    }

    // Collapses every other sidebar card and expands `focusId`. Calling this
    // IS the whole policy -- it does not itself gate on any transition, so a
    // caller that invoked it on every `state` echo would refight a user who
    // re-expanded a card a moment later. See browser.js's `state` handler.
    function focusSidebarCard(focusId) {
        var ids = sidebarCardIds();
        for (var i = 0; i < ids.length; i++) {
            if (ids[i] === focusId) continue;
            setCardCollapsed(ids[i], true);
        }
        setCardCollapsed(focusId, false);
        updateSquircles();
        schedule();
    }

    window.__focusSidebarCard = focusSidebarCard;

    /* ------------------------------------------------------------------ *
     * Measurement                                                        *
     * ------------------------------------------------------------------ */
    function shownChildren(dock) {
        var out = [];
        if (!dock) return out;
        for (var i = 0; i < dock.children.length; i++) {
            var rect = dock.children[i].getBoundingClientRect();
            if (rect.width > 0 && rect.height > 0) out.push(rect);
        }
        return out;
    }

    // Rightmost edge of anything in the left sidebar/dock, leftmost edge of anything in
    // the right sidebar/dock. Infinities mean "that side is empty".
    function edges() {
        var l = -Infinity, r = Infinity, i;
        var leftBar = $('primary-bar') || $('left-dock');
        var rightBar = $('secondary-bar') || $('right-rail');
        var lc = shownChildren(leftBar), rc = shownChildren(rightBar);
        for (i = 0; i < lc.length; i++) l = Math.max(l, lc[i].right);
        for (i = 0; i < rc.length; i++) r = Math.min(r, rc[i].left);
        return { left: l, right: r };
    }

    function fits() {
        var e = edges();
        var hasL = isFinite(e.left), hasR = isFinite(e.right);
        var w = window.innerWidth;
        if (hasL && e.left > w - EDGE) return false;          // left bar ran off the right edge
        if (hasR && e.right < EDGE) return false;             // right bar ran off the left edge
        if (hasL && hasR && e.left + GAP > e.right) return false;   // the two would overlap
        return true;
    }

    /* ------------------------------------------------------------------ *
     * Layout pass                                                        *
     * ------------------------------------------------------------------ */
    var AUTO = ['ir-card', 'sensors-card'];   // may be auto-collapsed

    function autoCollapse(id) {
        var el = $(id);
        if (!el) return false;
        el.classList.add('auto-collapsed');
        return true;
    }

    function relayout() {
        updateSquircles();
        // 1. Keep the dock band flush with the real top bar / event-log console.
        //    Collapsing the console therefore gives its height back to the docks.
        var topbar = $('topbar'), logConsole = $('log-console');
        root.style.setProperty('--dock-top', ((topbar ? topbar.offsetHeight : 48) + 12) + 'px');
        root.style.setProperty('--dock-bottom', ((logConsole ? logConsole.offsetHeight : 0) + 8) + 'px');

        // 2. Reset to the user's chosen state, then degrade only as far as needed.
        for (var i = 0; i < AUTO.length; i++) {
            var el = $(AUTO[i]);
            if (el) el.classList.remove('auto-collapsed');
        }
        var right = $('secondary-bar') || $('right-rail');
        if (right) right.classList.remove('dock--scroll');

        if (fits()) return;
        if (right) { right.classList.add('dock--scroll'); if (fits()) return; }
        if (autoCollapse('ir-card') && fits()) return;
        autoCollapse('sensors-card');
    }

    /* ------------------------------------------------------------------ *
     * Scheduling: coalesce to one pass per frame.                        *
     * ------------------------------------------------------------------ */
    var pending = false;
    function schedule() {
        if (pending) return;
        pending = true;
        requestAnimationFrame(function () {
            pending = false;
            try { relayout(); } catch (e) {
                if (window.__diag) window.__diag('layout.js: ' + (e && e.message || e), 'error');
            }
        });
    }
    window.__relayout = schedule;

    function observe() {
        window.addEventListener('resize', schedule);
        var docks = [$('primary-bar'), $('secondary-bar'), $('left-dock'), $('right-rail')].filter(Boolean);

        var ro = window.ResizeObserver ? new ResizeObserver(schedule) : null;
        function watch() {
            if (!ro) return;
            for (var d = 0; d < docks.length; d++) {
                for (var i = 0; i < docks[d].children.length; i++) ro.observe(docks[d].children[i]);
            }
        }
        watch();

        if (window.MutationObserver) {
            var mo = new MutationObserver(function () { watch(); schedule(); });
            for (var d = 0; d < docks.length; d++) {
                mo.observe(docks[d], { childList: true, subtree: true, attributes: true, attributeFilter: ['class'] });
            }
            var logConsole = $('log-console');
            if (logConsole) mo.observe(logConsole, { attributes: true, attributeFilter: ['class'] });
        }

        if (document.fonts && document.fonts.ready) document.fonts.ready.then(schedule).catch(function () {});
        window.addEventListener('load', schedule);
    }

    function init() {
        initCardStates();
        setupCollapseDelegation();
        setupSidebarResizing();
        observe();
        schedule();
        if (window.__diag) window.__diag('layout.js: sidebar layout manager ready');
    }

    if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init);
    else init();
})();
