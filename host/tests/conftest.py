"""Auto-apply slow-family + xdist-safety markers to the suite.

Families are *derived* from each test file's own source at collection time, not a
hand-maintained list -- so they cannot drift out of sync with the tests (the repo's
"unenforced conventions drift" lesson). A file is tagged:

  * ``open3d``    -- gates itself on ``importorskip("open3d")``
  * ``subprocess``-- spawns a subprocess (C-shim / firmware / pi-bridge / MCP builds)
  * ``net``       -- binds a real socket / runs a ``uvicorn.Server``
  * ``slow``      -- union of the heavyweight families above

This lets ``-m 'not slow'`` carve a fast lane. The markers are registered in
``pyproject.toml``. Detection mirrors the patterns in ``roomscan.testtriage`` and
is intentionally cheap (one read per file).

``serial`` is different: it is an **xdist quarantine**, applied by exact node id to
the handful of tests that flake under ``pytest -n`` (CPU-contention-sensitive
threading/timing races), not derived from source. The vast majority of the suite --
including the Open3D and subprocess families, which use separate processes and
ephemeral ports -- is xdist-safe, so quarantining by ``time.sleep`` presence was
both wrong (those sleeps are thread-spin waits, not assertions) and slow (it forced
the heavy-but-safe tests serial). Keep this list minimal; add a node id only after
you have *seen* it flake under ``-n auto`` (the two-pass ``run_tests(parallel=True)``
runs the parallel lane, then this quarantine serially). The empirical procedure that
seeded it is in issue #208.
"""
from __future__ import annotations

import re
from functools import lru_cache
from pathlib import Path

import pytest

_PATTERNS = {
    "open3d": re.compile(r"""importorskip\(\s*["']open3d["']"""),
    "subprocess": re.compile(r"\bimport subprocess\b|\bsubprocess\."),
    "net": re.compile(r"""socket\.socket\(|\.bind\(\(|uvicorn\.Server\b"""),
}

# Exact node ids that flake under `pytest -n` and must run serially. Found by
# running `-n auto` repeatedly; each is a background-thread race a discontinuity
# callback can lose under load, not a genuine data dependency. Match is by suffix
# so the `tests/` prefix is optional.
SERIAL_NODEIDS = frozenset({
    "test_web.py::test_restart_resets_the_ephemeral_map",
})


@lru_cache(maxsize=512)
def _families(path: str) -> frozenset[str]:
    try:
        src = Path(path).read_text(encoding="utf-8")
    except OSError:
        return frozenset()
    fams = {name for name, pat in _PATTERNS.items() if pat.search(src)}
    if fams:
        fams.add("slow")
    return frozenset(fams)


def pytest_collection_modifyitems(config, items):
    for item in items:
        path = getattr(item, "path", None)
        if path is None:                      # pytest < 7 fallback
            path = Path(str(item.fspath))
        for fam in _families(str(path)):
            item.add_marker(getattr(pytest.mark, fam))
        if any(item.nodeid.endswith(n) for n in SERIAL_NODEIDS):
            item.add_marker(pytest.mark.serial)
