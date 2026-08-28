# E1 buggy-patch corpus (#223)

Each `bugNNN.py` reintroduces one historical defect, in a few lines, against the
real modules and the domain types from `roomscan.domain`. They are never imported
at runtime; `run_all.py` type-checks each one with Pyright and prints the tally.
A patch counts as REJECTED when Pyright reports at least one error in it.

Usage (from `host/`): `python tools/spikes/e1_bug_patches/run_all.py`
