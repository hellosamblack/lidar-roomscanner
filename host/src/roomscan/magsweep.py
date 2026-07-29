"""Magnetometer sweep coverage + calibration-quality diagnostics.

Backs the web app's "Calibrate Mag" modal (owner ask, 2026-07-29). The fit
itself is `magcal.fit_ellipsoid` reused verbatim; everything here is the
*diagnostic* layer around it -- the part that was missing and that let a bad
calibration ship silently.

Why this exists
---------------
A correctly calibrated magnetometer reports a CONSTANT field magnitude in every
orientation. On 2026-07-29 the shipped `mag_cal.json` was measured across a tilt
sweep and did not:

    tilt from vertical:  0.3   0.2   30.5  60.6  80.0  90.6  30.6  2.8   (deg)
    |B| after cal:       50.5  47.4  58.7  76.5  81.1  85.1  62.8  50.7  (uT)

Accurate ceiling-facing, degrading monotonically to ~1.7x toward horizontal --
the signature of an incomplete tumble: good coverage in ONE attitude family,
poor everywhere else. The ellipsoid fit is happy to interpolate a plausible
ellipsoid through a cap of samples; nothing downstream noticed. Hence: measure
the coverage during collection, and measure the |B| consistency of the result
before anyone accepts it.

Binning: a Fibonacci (golden-spiral) sphere lattice
---------------------------------------------------
Cells are the Voronoi cells of an N-point Fibonacci lattice, and a sample is
assigned to the cell whose lattice direction it is closest to (a single argmax
over dot products -- no wrap/branch logic, no boundary special cases).

Chosen over naive lat/lon bins because those are wildly UNequal in area (a
15-degree lat/lon bin at the pole is ~1/50 the area of one at the equator), so a
lat/lon coverage percentage would over-weight the poles and report "covered"
for a tumble that missed most of the actual sphere -- precisely the failure mode
this tool exists to catch. The Fibonacci lattice is the standard cheap
quasi-uniform sphere point set: every cell has near-identical area, so
"fraction of cells occupied" IS "fraction of the sphere covered", and a gap of
K cells means the same thing wherever it sits.

`SPHERE_CELLS = 92` gives ~0.137 sr per cell, i.e. a cell radius of ~12 deg
(diameter ~24 deg) -- coarse enough that a careful hand tumble can fill every
cell, fine enough that the measured defect above (a whole hemisphere-ish family
of attitudes missing) is many cells wide, not one.

What gets binned
----------------
The **calibrated, body-frame** field direction -- `unit(AXIS_CONVENTION @
cal.apply(raw))` -- NOT the raw direction. This matters and is not a detail:
the rig's hard-iron offset is ~65 uT against a ~50 uT field, so the raw vectors
all live in a cone around the offset and their directions cover barely half the
sphere no matter how thoroughly you tumble. Binned raw, coverage could never
reach 100% and the map would be meaningless. Binned calibrated, a cell IS "the
device was oriented such that the field entered it from this direction", which
is both what the user can act on and what the ellipsoid fit needs to span (full
body-frame direction coverage <=> the raw cloud wraps the whole ellipsoid).

With no calibration available at all (fresh install), `provisional_calibration`
derives a cheap hard-iron-only estimate from the sample bounding box so the map
still works on the very first tumble; it is used ONLY for binning/display and
is never saved.
"""
from __future__ import annotations

import math
import time

import numpy as np

from .magcal import MagCalibration, fit_ellipsoid
from .sensors import AXIS_CONVENTION

# Cell count of the Fibonacci sphere lattice -- see the module docstring for
# why 92 (cell radius ~12 deg, fillable by hand, gap-legible).
SPHERE_CELLS = 92

# Cap on retained raw samples (~11 min at the 30 Hz env-stream rate). Beyond
# this the oldest are dropped: an ellipsoid fit saturates long before, and the
# whole-array recompute in `build_report` stays a few ms.
MAX_SAMPLES = 20000

# --- quality thresholds (documented in docs/web-protocol.md) -----------------
#
# Field consistency is the HEADLINE metric: std(|B|)/mean(|B|) after applying
# the candidate calibration. The 2% "good" bar matches the existing convention
# in docs/yaw-fusion.md ("< 0.02 is a clean fit"); the defective calibration
# above scores ~20%.
FIELD_GOOD_PCT = 2.0
FIELD_MARGINAL_PCT = 5.0
# Fraction of sphere cells that must hold >=1 sample.
COVERAGE_GOOD = 0.85
COVERAGE_MARGINAL = 0.60
# Raw sample count. `fit_ellipsoid` needs 20 to be solvable at all; 300 is
# ~10 s of tumbling and the point where the 9-parameter fit stops being
# noise-dominated.
SAMPLES_GOOD = 300
SAMPLES_MARGINAL = 100
MIN_FIT_SAMPLES = 20

_VERDICT_RANK = {"good": 0, "marginal": 1, "bad": 2}

# Body-frame faces (SFLP body: X = Up, Y = Right, Z = Forward/boresight -- see
# docs/coordinate-frames.md). Used to name WHERE a coverage gap is, so the
# guidance is an instruction ("aim the Right face at the field") rather than a
# vector the user has to decode.
FACES = (
    ("Top", (1.0, 0.0, 0.0)),
    ("Bottom", (-1.0, 0.0, 0.0)),
    ("Right", (0.0, 1.0, 0.0)),
    ("Left", (0.0, -1.0, 0.0)),
    ("Front", (0.0, 0.0, 1.0)),
    ("Back", (0.0, 0.0, -1.0)),
)


# --- sphere lattice ---------------------------------------------------------

_LATTICE_CACHE: dict[int, np.ndarray] = {}
_NEIGHBOUR_CACHE: dict[tuple[int, int], list[tuple[int, ...]]] = {}


def sphere_lattice(n: int = SPHERE_CELLS) -> np.ndarray:
    """(n, 3) unit vectors on a Fibonacci (golden-spiral) sphere lattice --
    the cell centres. Deterministic and cached; callers must not mutate the
    returned array (it is returned read-only)."""
    cached = _LATTICE_CACHE.get(n)
    if cached is not None:
        return cached
    if n < 4:
        raise ValueError(f"need at least 4 cells, got {n}")
    i = np.arange(n, dtype=np.float64) + 0.5
    z = 1.0 - 2.0 * i / n
    r = np.sqrt(np.maximum(0.0, 1.0 - z * z))
    phi = math.pi * (1.0 + math.sqrt(5.0)) * i
    pts = np.column_stack([r * np.cos(phi), r * np.sin(phi), z])
    pts /= np.linalg.norm(pts, axis=1, keepdims=True)
    pts.setflags(write=False)
    _LATTICE_CACHE[n] = pts
    return pts


def cell_neighbours(n: int = SPHERE_CELLS, k: int = 6) -> list[tuple[int, ...]]:
    """Symmetrized k-nearest-neighbour adjacency of the lattice cells, used to
    grow connected empty regions. k=6 because a Fibonacci lattice is locally
    hexagonal; symmetrizing (union of both directions) keeps the graph
    undirected where the k-NN relation isn't mutual."""
    key = (n, k)
    cached = _NEIGHBOUR_CACHE.get(key)
    if cached is not None:
        return cached
    lat = sphere_lattice(n)
    dots = lat @ lat.T
    np.fill_diagonal(dots, -2.0)
    nearest = np.argsort(-dots, axis=1)[:, :k]
    sets: list[set[int]] = [set(row.tolist()) for row in nearest]
    for a, row in enumerate(nearest):
        for b in row.tolist():
            sets[b].add(a)
    adj = [tuple(sorted(s)) for s in sets]
    _NEIGHBOUR_CACHE[key] = adj
    return adj


def unit(vec) -> np.ndarray | None:
    """Normalize a 3-vector, or None if it is degenerately short."""
    v = np.asarray(vec, dtype=np.float64).reshape(3)
    n = float(np.linalg.norm(v))
    if not np.isfinite(n) or n < 1e-9:
        return None
    return v / n


def assign_cells(directions, n: int = SPHERE_CELLS) -> np.ndarray:
    """(M, 3) unit directions -> (M,) cell index (nearest lattice point).
    An empty input yields an empty int array."""
    d = np.asarray(directions, dtype=np.float64).reshape(-1, 3)
    if d.shape[0] == 0:
        return np.zeros(0, dtype=np.int64)
    return np.argmax(d @ sphere_lattice(n).T, axis=1)


# --- calibration helpers ----------------------------------------------------

def provisional_calibration(samples) -> MagCalibration | None:
    """Cheap hard-iron-only estimate: centre = per-axis midpoint of the sample
    bounding box, soft-iron = identity, field = mean radius. Used ONLY to bin
    directions for the coverage map when no real calibration exists yet (first
    ever tumble) -- never fitted, never saved, never fed to the yaw fusion. Its
    directions are good enough for 24-deg-wide cells long before the sample
    cloud is good enough for `fit_ellipsoid`.

    None when there is too little spread to centre anything meaningfully."""
    x = np.asarray(samples, dtype=np.float64).reshape(-1, 3)
    if x.shape[0] < 4:
        return None
    centre = (x.max(axis=0) + x.min(axis=0)) / 2.0
    radii = np.linalg.norm(x - centre, axis=1)
    field = float(np.mean(radii))
    if not np.isfinite(field) or field < 1e-6:
        return None
    return MagCalibration(
        offset=tuple(float(v) for v in centre),
        matrix=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
        field_ut=field,
    )


def calibrated_directions(samples, cal: MagCalibration | None) -> np.ndarray:
    """(N, 3) raw uT -> (N, 3) unit body-frame field directions under `cal`.

    `AXIS_CONVENTION` is applied so the directions are in the SFLP body frame
    (X=Up, Y=Right, Z=Forward), matching how the rest of the host consumes the
    mag (`web._mag_validity`, `build_sensor_message`, `YawFusion.update`) --
    the face names in `FACES` are only meaningful in that frame. With
    `cal=None` the raw vectors are used unchanged; see the module docstring for
    why that is a poor basis for binning and only a last resort."""
    x = np.asarray(samples, dtype=np.float64).reshape(-1, 3)
    if x.shape[0] == 0:
        return np.zeros((0, 3), dtype=np.float64)
    if cal is not None:
        m = np.asarray(cal.matrix, dtype=np.float64)
        b = np.asarray(cal.offset, dtype=np.float64)
        x = (x - b) @ m.T
    x = x @ np.asarray(AXIS_CONVENTION, dtype=np.float64).T
    norms = np.linalg.norm(x, axis=1, keepdims=True)
    norms[norms < 1e-9] = 1.0
    return x / norms


def calibrated_norms(samples, cal: MagCalibration) -> np.ndarray:
    """(N,) |B| in uT after applying `cal`. AXIS_CONVENTION is a signed
    permutation, so it cannot change a magnitude -- omitted deliberately."""
    x = np.asarray(samples, dtype=np.float64).reshape(-1, 3)
    if x.shape[0] == 0:
        return np.zeros(0, dtype=np.float64)
    m = np.asarray(cal.matrix, dtype=np.float64)
    b = np.asarray(cal.offset, dtype=np.float64)
    return np.linalg.norm((x - b) @ m.T, axis=1)


# --- metrics ----------------------------------------------------------------

def _verdict(value: float, good: float, marginal: float, higher_is_better: bool) -> str:
    if higher_is_better:
        if value >= good:
            return "good"
        return "marginal" if value >= marginal else "bad"
    if value < good:
        return "good"
    return "marginal" if value < marginal else "bad"


def field_consistency(samples, cal: MagCalibration | None) -> dict | None:
    """THE headline metric: |B| across every sample after applying `cal`.

    A correctly calibrated magnetometer reads the same field magnitude in every
    orientation, so `std_pct` (= std/mean, as a percent) is a direct, physical,
    assumption-free measure of calibration error -- it needs no ground truth,
    no reference heading, and no second instrument. `ratio` (max/min) is
    reported alongside because that is the number the 2026-07-29 tilt sweep
    exposed (~1.7x) and it is more legible than a std to a human eye.

    None when there are no samples or no calibration to apply."""
    x = np.asarray(samples, dtype=np.float64).reshape(-1, 3)
    if x.shape[0] == 0 or cal is None:
        return None
    norms = calibrated_norms(x, cal)
    mean = float(np.mean(norms))
    if not np.isfinite(mean) or mean < 1e-9:
        return None
    std = float(np.std(norms))
    lo, hi = float(np.min(norms)), float(np.max(norms))
    expected = float(cal.field_ut)
    std_pct = 100.0 * std / mean
    # Spread alone is not sufficient. A calibration can be perfectly SELF-
    # consistent (tiny std) while sitting at completely the wrong magnitude --
    # observed on-rig 2026-07-29: 255 stationary samples read |B| = 101.96 uT
    # against the saved calibration's own field_ut of 49.87 uT, a x2.04 bias,
    # yet scored std_pct = 0.22%. `bias_pct` is that second failure mode, and
    # the field verdict is the WORSE of the two so neither can hide the other.
    bias_pct = (100.0 * (mean - expected) / expected) if expected > 1e-9 else float("inf")
    spread_verdict = _verdict(std_pct, FIELD_GOOD_PCT, FIELD_MARGINAL_PCT, higher_is_better=False)
    bias_verdict = _verdict(abs(bias_pct), FIELD_GOOD_PCT, FIELD_MARGINAL_PCT, higher_is_better=False)
    return {
        "mean_ut": mean,
        "std_ut": std,
        "std_pct": std_pct,
        "bias_pct": bias_pct,
        "min_ut": lo,
        "max_ut": hi,
        "ratio": (hi / lo) if lo > 1e-9 else float("inf"),
        # RMS deviation of the calibrated samples from the sphere the fit
        # claims they lie on -- the fit residual, in physical uT. Combines both
        # failure modes above: sqrt(std^2 + bias^2).
        "residual_rms_ut": float(np.sqrt(np.mean((norms - expected) ** 2))),
        "expected_ut": expected,
        "spread_verdict": spread_verdict,
        "bias_verdict": bias_verdict,
        "verdict": max((spread_verdict, bias_verdict), key=lambda v: _VERDICT_RANK[v]),
    }


def coverage_stats(cell_idx, n: int = SPHERE_CELLS) -> dict:
    """Occupancy of the sphere cells: counts, occupied/empty, fraction, verdict."""
    idx = np.asarray(cell_idx, dtype=np.int64).reshape(-1)
    counts = np.bincount(idx, minlength=n)[:n] if idx.size else np.zeros(n, dtype=np.int64)
    occupied = int(np.count_nonzero(counts))
    fraction = occupied / float(n)
    return {
        "cells": n,
        "counts": [int(v) for v in counts],
        "occupied": occupied,
        "empty": n - occupied,
        "fraction": fraction,
        "verdict": _verdict(fraction, COVERAGE_GOOD, COVERAGE_MARGINAL, higher_is_better=True),
    }


def cell_deviation_pct(samples, cell_idx, cal: MagCalibration | None,
                       n: int = SPHERE_CELLS) -> list[float | None]:
    """Per-cell mean signed |B| error under `cal`, as a percent of `cal.field_ut`.
    None for cells with no samples.

    This is what turns the map from a wizard step into a diagnostic: colour the
    occupied cells by this and the 2026-07-29 defect (accurate ceiling-facing,
    +70% toward horizontal) is a picture, not a table."""
    x = np.asarray(samples, dtype=np.float64).reshape(-1, 3)
    idx = np.asarray(cell_idx, dtype=np.int64).reshape(-1)
    if x.shape[0] == 0 or cal is None or idx.size != x.shape[0]:
        return [None] * n
    expected = float(cal.field_ut)
    if expected < 1e-9:
        return [None] * n
    dev = 100.0 * (calibrated_norms(x, cal) - expected) / expected
    counts = np.bincount(idx, minlength=n)[:n]
    sums = np.bincount(idx, weights=dev, minlength=n)[:n]
    return [float(s / c) if c else None for s, c in zip(sums, counts)]


def empty_regions(counts, n: int = SPHERE_CELLS) -> list[dict]:
    """Connected components of the EMPTY cells, largest first.

    Each region: `{"cells": [...], "size": int, "fraction": float,
    "centroid": [x,y,z], "face": str}`. Connectivity uses `cell_neighbours`,
    so a region is a contiguous patch of missing sphere -- which is the thing
    worth pointing a user at, as opposed to N scattered singleton misses that
    happen to sum to the same count."""
    c = np.asarray(counts, dtype=np.int64).reshape(-1)
    if c.size != n:
        raise ValueError(f"counts must have {n} entries, got {c.size}")
    adj = cell_neighbours(n)
    lat = sphere_lattice(n)
    seen = set()
    regions: list[dict] = []
    for start in range(n):
        if c[start] or start in seen:
            continue
        stack = [start]
        seen.add(start)
        group: list[int] = []
        while stack:
            cur = stack.pop()
            group.append(cur)
            for nb in adj[cur]:
                if nb not in seen and not c[nb]:
                    seen.add(nb)
                    stack.append(nb)
        centroid = unit(lat[group].mean(axis=0))
        if centroid is None:      # antipodal-balanced group: fall back to a member
            centroid = lat[group[0]]
        regions.append({
            "cells": sorted(group),
            "size": len(group),
            "fraction": len(group) / float(n),
            "centroid": [float(v) for v in centroid],
            "face": nearest_face(centroid),
        })
    regions.sort(key=lambda r: -r["size"])
    return regions


def nearest_face(direction) -> str:
    """Name the body-frame face a direction points out of (see `FACES`)."""
    v = unit(direction)
    if v is None:
        return "?"
    return max(FACES, key=lambda f: float(np.dot(v, f[1])))[0]


def guidance_text(regions: list[dict], live_dir=None, n: int = SPHERE_CELLS) -> str:
    """One actionable sentence: where the biggest hole is and how to fill it.

    The instruction is in terms of the device's own faces because that is what
    the user can act on. "Aim <face> toward magnetic north and downward" is the
    concrete attitude that puts the field along that body axis in the northern
    hemisphere (the field dips downward there); the angle, when a live sample
    is available, says how far the current attitude is from that target so the
    user can steer with the live marker on the map."""
    if not regions:
        return "Full sphere covered — every orientation has samples."
    top = regions[0]
    if top["size"] >= n:
        # Nothing collected at all: the "largest gap" is the whole sphere and
        # its centroid is arbitrary, so naming a face would be noise.
        return ("Nothing collected yet — every one of the "
                f"{n} directions is missing. Press Start and rotate the device "
                "through every orientation you can reach.")
    pct = 100.0 * top["fraction"]
    parts = [
        f"Biggest gap: {top['size']} of {n} cells ({pct:.0f}% of the sphere) "
        f"around the device's {top['face']} face."
    ]
    live = unit(live_dir) if live_dir is not None else None
    if live is not None:
        cos = float(np.clip(np.dot(live, np.asarray(top["centroid"], dtype=np.float64)), -1.0, 1.0))
        parts.append(
            f"Turn the device so its {top['face']} face points toward magnetic north "
            f"and downward — about {math.degrees(math.acos(cos)):.0f}° of rotation from here."
        )
    else:
        parts.append(
            f"Turn the device so its {top['face']} face points toward magnetic north and downward."
        )
    if len(regions) > 1:
        parts.append(f"({len(regions) - 1} smaller gap{'s' if len(regions) > 2 else ''} remain.)")
    return " ".join(parts)


def quality_report(samples, cell_idx, cal: MagCalibration | None,
                   n: int = SPHERE_CELLS) -> dict:
    """The full quality block for ONE calibration: the three components plus a
    headline verdict that is explicitly the WORST of them, with `reason` naming
    which component drove it.

    Deliberately not a single opaque score -- the components are the point. A
    calibration can be perfectly self-consistent over a cap of the sphere
    (exactly the 2026-07-29 defect: it would score `field` good if you only
    re-measured the attitudes it was fitted from), so `coverage` has to be read
    next to `field`, not folded into it."""
    x = np.asarray(samples, dtype=np.float64).reshape(-1, 3)
    count = int(x.shape[0])
    field = field_consistency(x, cal)
    cov = coverage_stats(cell_idx, n)
    samples_verdict = _verdict(count, SAMPLES_GOOD, SAMPLES_MARGINAL, higher_is_better=True)
    # Reasons are per-component and phrased as what to DO about it, because the
    # three failures need opposite responses: a spread/bias failure means the
    # calibration is wrong, a coverage/sample failure means the MEASUREMENT is
    # not yet conclusive. "Limited by coverage" alone reads like an indictment
    # of the calibration when it is really an indictment of the sweep.
    components = [
        ("coverage",
         cov["verdict"],
         f"Only {100 * cov['fraction']:.0f}% of the sphere sampled ({cov['empty']} cells empty) — "
         "sweep more orientations before trusting this."),
        ("samples", samples_verdict,
         f"Only {count} samples — keep collecting."),
    ]
    if field is not None:
        worse_field = ("bias" if _VERDICT_RANK[field["bias_verdict"]]
                       > _VERDICT_RANK[field["spread_verdict"]] else "spread")
        components.insert(0, (
            "field consistency", field["verdict"],
            (f"|B| averages {field['bias_pct']:+.0f}% off this calibration's own expected "
             f"{field['expected_ut']:.1f} µT — the calibration is wrong.")
            if worse_field == "bias" else
            (f"|B| varies {field['std_pct']:.1f}% with orientation (×{field['ratio']:.2f} "
             "min-to-max) — the calibration is wrong.")))
    worst_name, worst, worst_why = max(components, key=lambda kv: _VERDICT_RANK[kv[1]])
    return {
        "samples": count,
        "samples_verdict": samples_verdict,
        "field": field,
        "coverage": {k: v for k, v in cov.items() if k != "counts"},
        "verdict": worst,
        "limited_by": worst_name,
        "reason": ("Field magnitude is consistent across a well-covered sphere."
                   if worst == "good" else worst_why),
    }


# --- collection session -----------------------------------------------------

class MagSweepSession:
    """Collects raw mag samples for one calibration attempt and reports on them.

    Lifecycle: `start()` -> `add()` per env sample -> `stop()` (fits a
    candidate) -> `accept()`/`discard()`. Samples survive `stop()` so the user
    can preview, reject, and resume tumbling into the SAME cloud rather than
    starting over.

    Threading: the web server drives this entirely from the asyncio event loop
    (broadcaster tick + inbound handler), so there is no lock. Anything calling
    it from a second thread must add one.
    """

    def __init__(self, n: int = SPHERE_CELLS, max_samples: int = MAX_SAMPLES):
        self.n = int(n)
        self.max_samples = int(max_samples)
        self._samples: list[tuple[float, float, float]] = []
        self.collecting = False
        self.started_at: float | None = None
        self.elapsed_s = 0.0
        self.candidate: MagCalibration | None = None
        self.fit_error: str | None = None
        self.last_t_us: int | None = None
        self.live_raw: tuple[float, float, float] | None = None

    # -- collection --
    def start(self) -> None:
        self.collecting = True
        self.started_at = time.monotonic()
        self.candidate = None
        self.fit_error = None

    def stop(self) -> None:
        """Stop collecting and fit a candidate from everything gathered so far.
        A degenerate/insufficient cloud leaves `candidate=None` and puts the
        reason in `fit_error` -- never raises, so a premature stop is a message
        in the UI, not a 500."""
        if self.collecting and self.started_at is not None:
            self.elapsed_s += time.monotonic() - self.started_at
        self.collecting = False
        self.started_at = None
        self.fit_candidate()

    def reset(self) -> None:
        self._samples.clear()
        self.collecting = False
        self.started_at = None
        self.elapsed_s = 0.0
        self.candidate = None
        self.fit_error = None
        self.last_t_us = None

    def add(self, mag_ut, t_us: int | None = None) -> bool:
        """Record one raw sample. `t_us` de-duplicates: the broadcaster polls
        `SensorState.latest_env()` rather than tapping the reader thread (which
        would mean touching the shared sensor path), so the same env sample can
        be seen on consecutive ticks. Returns True if it was stored."""
        v = np.asarray(mag_ut, dtype=np.float64).reshape(-1)
        if v.size != 3 or not np.all(np.isfinite(v)):
            return False
        self.live_raw = (float(v[0]), float(v[1]), float(v[2]))
        if not self.collecting:
            return False
        if t_us is not None and t_us == self.last_t_us:
            return False
        self.last_t_us = t_us
        self._samples.append(self.live_raw)
        if len(self._samples) > self.max_samples:
            del self._samples[0:len(self._samples) - self.max_samples]
        return True

    # -- derived --
    @property
    def samples(self) -> np.ndarray:
        return np.asarray(self._samples, dtype=np.float64).reshape(-1, 3)

    def elapsed(self) -> float:
        base = self.elapsed_s
        if self.collecting and self.started_at is not None:
            base += time.monotonic() - self.started_at
        return base

    def fit_candidate(self) -> MagCalibration | None:
        """Run `magcal.fit_ellipsoid` over the collected cloud. Sets
        `self.candidate` / `self.fit_error` and returns the candidate."""
        x = self.samples
        if x.shape[0] < MIN_FIT_SAMPLES:
            self.candidate = None
            self.fit_error = f"need at least {MIN_FIT_SAMPLES} samples, have {x.shape[0]}"
            return None
        try:
            self.candidate = fit_ellipsoid(x)
            self.fit_error = None
        except ValueError as exc:
            self.candidate = None
            self.fit_error = str(exc)
        return self.candidate

    def binning_calibration(self, current: MagCalibration | None) -> MagCalibration | None:
        """Which calibration to bin directions with (module docstring): the
        candidate if one has been fitted, else the saved one, else a
        provisional hard-iron estimate from the cloud itself."""
        if self.candidate is not None:
            return self.candidate
        if current is not None:
            return current
        return provisional_calibration(self.samples)


def build_report(session: MagSweepSession, current: MagCalibration | None,
                 view: str = "current", saved_path: str = "mag_cal.json") -> dict:
    """The whole `magcal` wire message: coverage map, both calibrations'
    quality blocks, gaps + guidance, and the live "you are here" cell.

    `view` picks which calibration colours the map ("current" | "candidate");
    both quality blocks are always sent so the saved calibration's defect stays
    on screen next to the candidate's, which is the comparison that makes a
    regression obvious before saving."""
    n = session.n
    x = session.samples
    bin_cal = session.binning_calibration(current)
    dirs = calibrated_directions(x, bin_cal)
    cell_idx = assign_cells(dirs, n)
    cov = coverage_stats(cell_idx, n)
    regions = empty_regions(cov["counts"], n)

    live_dir = None
    live_cell = None
    if session.live_raw is not None:
        ld = calibrated_directions([session.live_raw], bin_cal)
        if ld.shape[0]:
            live_dir = [round(float(v), 4) for v in ld[0]]
            live_cell = int(assign_cells(ld, n)[0])

    view = view if view in ("current", "candidate") else "current"
    view_cal = session.candidate if view == "candidate" else current
    if view_cal is None:      # asked for a calibration that doesn't exist yet
        view_cal = current if current is not None else session.candidate
        view = "candidate" if (current is None and session.candidate is not None) else "current"

    lat = sphere_lattice(n)
    return {
        "type": "magcal",
        "collecting": session.collecting,
        "sample_count": int(x.shape[0]),
        "elapsed_s": round(session.elapsed(), 1),
        "cells": n,
        "cell_dirs": [[round(float(c), 4) for c in row] for row in lat],
        "cell_counts": cov["counts"],
        "cell_dev_pct": [None if v is None else round(v, 2)
                         for v in cell_deviation_pct(x, cell_idx, view_cal, n)],
        "view": view,
        "live_cell": live_cell,
        "live_dir": live_dir,
        "gaps": [{k: (round(v, 4) if isinstance(v, float) else v)
                  for k, v in r.items() if k != "cells"} for r in regions[:5]],
        "guidance": guidance_text(regions, live_dir, n),
        "has_current": current is not None,
        "has_candidate": session.candidate is not None,
        "binning": ("candidate" if session.candidate is not None
                    else "current" if current is not None
                    else "provisional" if bin_cal is not None else "raw"),
        "fit_error": session.fit_error,
        # Both quality blocks are measured against THIS session's samples, so
        # with none collected there is nothing to say about either calibration
        # -- null rather than a "bad, 0% coverage" verdict that would read as a
        # claim about the saved calibration rather than about the empty cloud.
        "current": (quality_report(x, cell_idx, current, n)
                    if current is not None and x.shape[0] else None),
        "candidate": (quality_report(x, cell_idx, session.candidate, n)
                      if session.candidate is not None and x.shape[0] else None),
        "current_field_ut": round(float(current.field_ut), 3) if current is not None else None,
        "candidate_field_ut": (round(float(session.candidate.field_ut), 3)
                               if session.candidate is not None else None),
        "saved_path": str(saved_path),
    }
