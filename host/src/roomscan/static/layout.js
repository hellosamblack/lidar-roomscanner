// layout.js — dock layout manager + the diagnostics panel's collapse toggle.
//
// A CLASSIC script (not an ES module) on purpose: the diagnostics panel exists
// precisely for the case where the module graph below it fails to load, so its
// collapse toggle — and the layout that keeps it from covering anything else —
// must not depend on that graph resolving.
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
//   1. collapse the diagnostics panel   (debug-only, lowest value)
//   2. scroll the right dock            (one column; content still reachable)
//   3. collapse the IR monitor          (its own card, canvas hidden)
//   4. collapse the sensors card
//
// Public surface (for debugging / headless drivers):  window.__relayout()

(function () {
    'use strict';

    var GAP = 16;                               // min clear gap between the docks
    var EDGE = 16;                              // min clear gap to a viewport edge
    var DIAG_KEY = 'roomscan.diag.collapsed';   // '1' collapsed / '0' open / absent = default

    var root = document.documentElement;
    function $(id) { return document.getElementById(id); }

    var STORAGE_PREFIX = 'roomscan.card.';

    function getCardKey(cardId) {
        if (cardId === 'diag') return 'roomscan.diag.collapsed';
        return STORAGE_PREFIX + cardId + '.collapsed';
    }

    var CARD_ICONS = {
        'telemetry': '<svg viewBox="0 0 24 24" width="20" height="20"><path fill="currentColor" d="M12,2A10,10 0 0,0 2,12A10,10 0 0,0 12,22A10,10 0 0,0 22,12A10,10 0 0,0 12,2M12,4A8,8 0 0,1 20,12A8,8 0 0,1 12,20A8,8 0 0,1 4,12A8,8 0 0,1 12,4M12,6A6,6 0 0,0 6,12H8A4,4 0 0,1 12,8V6Z"/></svg>',
        'sensors': '<svg viewBox="0 0 24 24" width="20" height="20"><path fill="currentColor" d="M12,2A10,10 0 0,0 2,12A10,10 0 0,0 12,22A10,10 0 0,0 22,12A10,10 0 0,0 12,2M12,4A8,8 0 0,1 20,12A8,8 0 0,1 12,20A8,8 0 0,1 4,12A8,8 0 0,1 12,4M14.12,7.88L9.88,12.12L7.88,14.12L12.12,9.88L14.12,7.88Z"/></svg>',
        'slam-hud': '<svg viewBox="0 0 24 24" width="20" height="20"><path fill="currentColor" d="M21,16.5C21,16.88 20.79,17.21 20.47,17.38L12.57,21.82C12.41,21.94 12.21,22 12,22C11.79,22 11.59,21.94 11.43,21.82L3.53,17.38C3.21,17.21 3,16.88 3,16.5V7.5C3,7.12 3.21,6.79 3.53,6.62L11.43,2.18C11.59,2.06 11.79,2 12,2C12.21,2 12.41,2.06 12.57,2.18L20.47,6.62C20.79,6.79 21,7.12 21,7.5V16.5M12,4.15L5.04,8.05L12,11.95L18.96,8.05L12,4.15Z"/></svg>',
        'ir-view': '<svg viewBox="0 0 24 24" width="20" height="20"><path fill="currentColor" d="M12,9A3,3 0 0,0 9,12A3,3 0 0,0 12,15A3,3 0 0,0 15,12A3,3 0 0,0 12,9M12,4.5C7,4.5 2.73,7.61 1,12C2.73,16.39 7,19.5 12,19.5C17,19.5 21.27,16.39 23,12C21.27,7.61 17,4.5 12,4.5M12,17.5A5.5,5.5 0 0,1 6.5,12A5.5,5.5 0 0,1 12,6.5A5.5,5.5 0 0,1 17.5,12A5.5,5.5 0 0,1 12,17.5Z"/></svg>',
        'diag': '<svg viewBox="0 0 24 24" width="20" height="20"><path fill="currentColor" d="M20,4H4A2,2 0 0,0 2,6V18A2,2 0 0,0 4,20H20A2,2 0 0,0 22,18V6A2,2 0 0,0 20,4M20,18H4V6H20V18M6,8L10,12L6,16V14L8.5,12L6,10V8M11,15H17V17H11V15Z"/></svg>',
        'device': '<svg viewBox="0 0 24 24" width="20" height="20"><path fill="currentColor" d="M6,2H18A2,2 0 0,1 20,4V20A2,2 0 0,1 18,22H6A2,2 0 0,1 4,20V4A2,2 0 0,1 6,2M9,4V6H11V4H9M13,4V6H15V4H13M9,18V20H11V18H9M13,18V20H15V18H13M4,9H6V11H4V9M4,13H6V15H4V13M18,9H20V11H18V9M18,13H20V15H18V13Z"/></svg>',
        'view': '<svg viewBox="0 0 24 24" width="20" height="20"><path fill="currentColor" d="M12,2A10,10 0 0,0 2,12A10,10 0 0,0 12,22C13.1,22 14,21.1 14,20C14,19.5 13.8,19.05 13.47,18.7C13.12,18.33 12.92,17.84 12.92,17.3C12.92,16.2 13.82,15.3 14.92,15.3H16C19.31,15.3 22,12.61 22,9.3C22,5.27 17.52,2 12,2M6.5,11.5A1.5,1.5 0 0,1 5,10A1.5,1.5 0 0,1 6.5,8.5A1.5,1.5 0 0,1 8,10A1.5,1.5 0 0,1 6.5,11.5M9.5,7.5A1.5,1.5 0 0,1 8,6A1.5,1.5 0 0,1 9.5,4.5A1.5,1.5 0 0,1 11,6A1.5,1.5 0 0,1 9.5,7.5M14.5,7.5A1.5,1.5 0 0,1 13,6A1.5,1.5 0 0,1 14.5,4.5A1.5,1.5 0 0,1 16,6A1.5,1.5 0 0,1 14.5,7.5M17.5,11.5A1.5,1.5 0 0,1 16,10A1.5,1.5 0 0,1 17.5,8.5A1.5,1.5 0 0,1 19,10A1.5,1.5 0 0,1 17.5,11.5Z"/></svg>',
        'capture': '<svg viewBox="0 0 24 24" width="20" height="20"><path fill="currentColor" d="M18,4L20,8H17L15,4H13L15,8H12L10,4H8L10,8H7L5,4H4A2,2 0 0,0 2,6V18A2,2 0 0,0 4,20H20A2,2 0 0,0 22,18V4H18M10,10L16,13L10,16V10Z"/></svg>',
        'slam-ctrl': '<svg viewBox="0 0 24 24" width="20" height="20"><path fill="currentColor" d="M12,2L4.5,20.29L5.21,21L12,18L18.79,21L19.5,20.29L12,2Z"/></svg>',
        'resources': '<svg viewBox="0 0 24 24" width="20" height="20"><path fill="currentColor" d="M4,2H20A2,2 0 0,1 22,4V20A2,2 0 0,1 20,22H4A2,2 0 0,1 2,20V4A2,2 0 0,1 4,2M6,6V18H8V6H6M10,10V18H12V10H10M14,8V18H16V8H14M18,13V18H20V13H18Z"/></svg>',
        'log': '<svg viewBox="0 0 24 24" width="20" height="20"><path fill="currentColor" d="M19,3H5C3.89,3 3,3.89 3,5V19C3,20.1 3.89,21 5,21H19C20.1,21 21,20.1 21,19V5C21,3.89 20.1,3 19,3M7,7H17V9H7V7M7,11H17V13H7V11M7,15H14V17H7V15Z"/></svg>'
    };

    var CARD_TITLES = {
        'telemetry': 'Telemetry',
        'sensors': 'Sensors',
        'slam-hud': 'SLAM HUD',
        'ir-view': 'IR Monitor',
        'diag': 'Diagnostics',
        'device': 'Device',
        'view': 'View',
        'capture': 'Capture & Playback',
        'slam-ctrl': 'SLAM',
        'resources': 'Resources',
        'log': 'Event Log'
    };

    // The rail is a stable map of every panel that CAN exist, not a list of
    // what's missing: every registered card gets a permanent button, dimmed or
    // lit by its collapse state. `.hidden` is reserved for cards that are
    // genuinely absent from the DOM right now (e.g. slam-hud/slam-ctrl before
    // SLAM arms) -- a button for a card that cannot exist is a dead control.
    function updateSquircles() {
        var docks = document.querySelectorAll('.dock');
        for (var d = 0; d < docks.length; d++) {
            var dock = docks[d];
            var bar = dock.querySelector('.squircle-bar');
            if (!bar) {
                bar = document.createElement('div');
                bar.className = 'squircle-bar';
                dock.insertBefore(bar, dock.firstChild);
            }

            var cards = dock.querySelectorAll('[data-card-id]');
            for (var i = 0; i < cards.length; i++) {
                var card = cards[i];
                var cardId = card.getAttribute('data-card-id');
                if (!cardId || !CARD_ICONS[cardId]) continue;

                var isCollapsed = card.classList.contains('collapsed');
                var isHidden = card.classList.contains('hidden') || card.style.display === 'none';

                var btn = bar.querySelector('.squircle-btn[data-for="' + cardId + '"]');
                if (!btn) {
                    btn = document.createElement('button');
                    btn.type = 'button';
                    btn.className = 'squircle-btn';
                    btn.setAttribute('data-for', cardId);
                    btn.innerHTML = CARD_ICONS[cardId];
                    bar.appendChild(btn);
                }

                btn.classList.toggle('hidden', isHidden);
                var isOpen = !isCollapsed && !isHidden;
                btn.classList.toggle('is-open', isOpen);
                btn.classList.toggle('is-dim', !isOpen);
                var title = (CARD_TITLES[cardId] || cardId) + (isOpen ? ' (click to collapse)' : ' (click to expand)');
                btn.setAttribute('title', title);
            }
        }
    }

    var HEADER_SELECTOR = '.control-group__header, .diag__header, .log-console__header, .ir-card__header';

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
            } else if (id === 'diag' && window.__diagErrors > 0) {
                card.classList.remove('collapsed');   // an error already fired pre-DOM
            }
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

            var header = e.target.closest('.control-group__header, .diag__header, .log-console__header, .ir-card__header');
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

    // Rightmost edge of anything in the left dock, leftmost edge of anything in
    // the right dock. Infinities mean "that side is empty".
    function edges() {
        var l = -Infinity, r = Infinity, i;
        var lc = shownChildren($('left-dock')), rc = shownChildren($('right-rail'));
        for (i = 0; i < lc.length; i++) l = Math.max(l, lc[i].right);
        for (i = 0; i < rc.length; i++) r = Math.min(r, rc[i].left);
        return { left: l, right: r };
    }

    function fits() {
        var e = edges();
        var hasL = isFinite(e.left), hasR = isFinite(e.right);
        var w = window.innerWidth;
        if (hasL && e.left > w - EDGE) return false;          // left dock ran off the right edge
        if (hasR && e.right < EDGE) return false;             // right dock ran off the left edge
        if (hasL && hasR && e.left + GAP > e.right) return false;   // the two would overlap
        return true;
    }

    /* ------------------------------------------------------------------ *
     * Layout pass                                                        *
     * ------------------------------------------------------------------ */
    var AUTO = ['diag-card', 'ir-card', 'sensors-card'];   // may be auto-collapsed

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
        root.style.setProperty('--dock-top', ((topbar ? topbar.offsetHeight : 48) + 16) + 'px');
        root.style.setProperty('--dock-bottom', ((logConsole ? logConsole.offsetHeight : 0) + 8) + 'px');

        // 2. Reset to the user's chosen state, then degrade only as far as needed.
        for (var i = 0; i < AUTO.length; i++) {
            var el = $(AUTO[i]);
            if (el) el.classList.remove('auto-collapsed');
        }
        var right = $('right-rail');
        if (right) right.classList.remove('dock--scroll');

        if (fits()) return;
        if (autoCollapse('diag-card') && fits()) return;
        if (right) { right.classList.add('dock--scroll'); if (fits()) return; }
        if (autoCollapse('ir-card') && fits()) return;
        autoCollapse('sensors-card');
        // Nothing left to give: the viewport is narrower than one column a side.
        // The docks stay put (blocks may clip) rather than stacking on top of
        // each other — overlap is the one outcome this layout never allows.
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
        var docks = [$('left-dock'), $('right-rail')].filter(Boolean);

        // Size changes (a stream list growing, a group expanding, a capture list
        // filling) are what move the columns — watch the blocks themselves.
        // relayout() is idempotent from its clean baseline, so a pass triggered
        // by its own class changes settles on the same result and stops.
        var ro = window.ResizeObserver ? new ResizeObserver(schedule) : null;
        function watch() {
            if (!ro) return;
            for (var d = 0; d < docks.length; d++) {
                for (var i = 0; i < docks[d].children.length; i++) ro.observe(docks[d].children[i]);
            }
        }
        watch();

        // Class flips (hidden / collapsed) resize blocks without a size event on
        // the observed node itself; children added or removed need re-watching.
        if (window.MutationObserver) {
            var mo = new MutationObserver(function () { watch(); schedule(); });
            for (var d = 0; d < docks.length; d++) {
                mo.observe(docks[d], { childList: true, subtree: true, attributes: true, attributeFilter: ['class'] });
            }
            var logConsole = $('log-console');
            if (logConsole) mo.observe(logConsole, { attributes: true, attributeFilter: ['class'] });
        }

        // Web fonts land after first paint and change every block's height.
        if (document.fonts && document.fonts.ready) document.fonts.ready.then(schedule).catch(function () {});
        window.addEventListener('load', schedule);
    }

    function init() {
        initCardStates();
        setupCollapseDelegation();
        observe();
        schedule();
        if (window.__diag) window.__diag('layout.js: dock layout manager ready');
    }

    if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init);
    else init();
})();
