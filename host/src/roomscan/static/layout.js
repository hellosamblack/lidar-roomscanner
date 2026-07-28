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

    /* ------------------------------------------------------------------ *
     * Diagnostics panel: collapse toggle + persisted preference.          *
     * ------------------------------------------------------------------ */
    function initDiag() {
        var card = $('diag-card'), toggle = $('diag-toggle');
        if (!card || !toggle) return;
        var pref = null;
        try { pref = localStorage.getItem(DIAG_KEY); } catch (e) {}
        if (pref !== null) {
            card.classList.toggle('collapsed', pref === '1');
        } else if (window.__diagErrors > 0) {
            card.classList.remove('collapsed');   // an error already fired pre-DOM
        }
        toggle.addEventListener('click', function () {
            var collapsed = card.classList.toggle('collapsed');
            try { localStorage.setItem(DIAG_KEY, collapsed ? '1' : '0'); } catch (e) {}
            schedule();
        });
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
        initDiag();
        observe();
        schedule();
        if (window.__diag) window.__diag('layout.js: dock layout manager ready');
    }

    if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init);
    else init();
})();
