"""Flat-field (per-zone fixed-pattern) correction for the ToF reflectance plane.

The VL53L9 is a multi-zone SPAD imager: each zone has a slightly different
response (per-zone gain from SPAD-count/optical-coupling/DSS differences), so a
uniform surface reads back with a stable, sensor-locked ripple -- fixed-pattern
noise (FPN). On a flat wall this measured at ~18% of signal, rock-stable across
frames, aligned to the sensor row/column axes (investigated 2026-07-16). It is
NOT display resampling and NOT scene texture.

This module models that as a multiplicative per-zone gain map and divides it out.
A correction is *built* from a flat-field capture -- the sensor slowly PANNED
across a uniform matte surface so real texture averages out while the FPN stays
locked to zones (`build_flatfield`) -- then saved, loaded, and applied to the
reflectance plane inside `pipeline.TransformStage` (so every consumer -- web,
panel, viewer, SLAM -- gets corrected reflectance uniformly). Disabled by
default: no map configured -> `load_configured()` returns None -> pipeline is a
no-op. See docs/flatfield-calibration.md and tools/build_flatfield.py.

Only reflectance is corrected here. Depth FPN (a per-zone *range* offset) and
confidence are different calibrations; the map format is per-plane-extensible
but v1 ships reflectance only.
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

# Sane bounds for a per-zone gain: reject a correction that would scale a zone by
# more than ~3x (a sign the build input wasn't a clean flat-field, e.g. a zone
# that was in shadow / no-return during the capture).
_GAIN_CLIP = (0.33, 3.0)


def _gaussian_kernel(sigma: float) -> np.ndarray:
    radius = max(1, int(round(3.0 * sigma)))
    x = np.arange(-radius, radius + 1, dtype=np.float64)
    k = np.exp(-(x * x) / (2.0 * sigma * sigma))
    return k / k.sum()


def _blur(img: np.ndarray, sigma: float) -> np.ndarray:
    """Separable Gaussian blur, reflect-padded, numpy-only (no scipy dependency)."""
    k = _gaussian_kernel(sigma)
    r = len(k) // 2
    out = img.astype(np.float64, copy=True)
    for axis in (0, 1):
        out = np.apply_along_axis(
            lambda m: np.convolve(np.pad(m, (r, r), mode="reflect"), k, mode="valid"),
            axis, out)
    return out


@dataclass(frozen=True)
class FlatField:
    """A per-zone multiplicative reflectance-gain correction, shape (H, W).

    `apply(plane)` returns `plane * gain` when shapes match, else the plane
    unchanged -- so a map built at one resolution/binning silently no-ops on a
    frame of a different shape rather than corrupting it. `gain` is unit-mean by
    construction, so correction redistributes per-zone response without shifting
    the overall reflectance level.
    """

    gain: np.ndarray                 # (H, W) float32, multiplicative, mean ~= 1.0
    meta: Optional[dict] = None      # provenance: n_frames, source, note

    @property
    def shape(self) -> tuple[int, int]:
        return self.gain.shape

    def apply(self, plane: np.ndarray) -> np.ndarray:
        p = np.asarray(plane)
        if p.shape != self.gain.shape:
            return plane                 # shape mismatch -> pass through untouched
        return (p * self.gain).astype(p.dtype, copy=False)

    def save(self, path) -> None:
        """Persist as .npz: the gain array + a JSON metadata blob."""
        np.savez(Path(path), gain=self.gain.astype(np.float32),
                 meta=json.dumps(self.meta or {}))

    @classmethod
    def load(cls, path) -> "FlatField | None":
        """Load a saved map. Any failure (missing/corrupt/wrong-shape) -> None,
        matching MagCalibration.load's tolerant contract -- a bad map disables
        correction rather than crashing the reader."""
        try:
            with np.load(Path(path), allow_pickle=False) as d:
                gain = np.asarray(d["gain"], dtype=np.float32)
                meta = json.loads(str(d["meta"])) if "meta" in d else {}
            if gain.ndim != 2 or not np.all(np.isfinite(gain)) or np.any(gain <= 0):
                return None
            return cls(gain=gain, meta=meta)
        except (OSError, ValueError, KeyError, json.JSONDecodeError, Exception):  # noqa: BLE001
            return None

    @classmethod
    def load_configured(cls, config=None) -> "FlatField | None":
        """Load the map named by ``[viewer] flatfield_path`` in roomscan.toml,
        or None if unset/missing. Import config lazily to keep the pipeline
        import-light. This is the one-liner the TransformStage construction
        sites call so a configured map goes live automatically."""
        try:
            from .config import ViewerConfig
            cfg = config if config is not None else ViewerConfig.load()
            path = getattr(cfg, "flatfield_path", None)
            if not path:
                return None
            return cls.load(path)
        except Exception:  # noqa: BLE001
            return None


# Ranging-mode keys for a mode-aware map set. These are the wire/`profiles`
# `RangingMode` values (VL53L9 context): 0 = Precision, 1 = Ambient. The
# 2026-08-04 cross-room study proved the FPN is mode-family-specific -- a
# Precision map leaves ~4% residual on Ambient and vice versa -- so the correct
# map is selected by the device's current ranging mode, not one global file.
RANGING_PRECISION = 0
RANGING_AMBIENT = 1


class FlatFieldSet:
    """A registry of per-ranging-mode `FlatField` maps + an optional default.

    `for_mode(ranging_mode)` returns the map to apply for the device's current
    ranging mode (`0` Precision / `1` Ambient), falling back to `default` when the
    mode is unknown (e.g. a replay with no device to read the mode from) or has no
    dedicated map. A single legacy `flatfield_path` becomes that default, so an old
    config keeps working; the mode-keyed paths take precedence when present.
    """

    def __init__(self, by_mode: dict[int, FlatField] | None = None,
                 default: Optional[FlatField] = None):
        self._by_mode = dict(by_mode or {})
        self._default = default

    @property
    def is_empty(self) -> bool:
        """True when no map is configured at all -- correction is then a no-op."""
        return not self._by_mode and self._default is None

    def for_mode(self, ranging_mode: Optional[int]) -> Optional[FlatField]:
        if ranging_mode is not None and ranging_mode in self._by_mode:
            return self._by_mode[ranging_mode]
        return self._default

    @classmethod
    def load_configured(cls, config=None) -> "FlatFieldSet":
        """Build the set from ``[viewer]`` config: ``flatfield_precision_path`` and
        ``flatfield_ambient_path`` key the two families, ``flatfield_path`` is the
        legacy single-map fallback/default. Any path that is unset or fails to load
        is simply absent (tolerant, matching `FlatField.load`), so a partial or
        missing configuration disables only the maps it lacks rather than raising.
        Always returns a set (possibly empty -- check `is_empty`)."""
        try:
            from .config import ViewerConfig
            cfg = config if config is not None else ViewerConfig.load()
        except Exception:  # noqa: BLE001
            return cls()
        by_mode: dict[int, FlatField] = {}
        for mode, attr in ((RANGING_PRECISION, "flatfield_precision_path"),
                           (RANGING_AMBIENT, "flatfield_ambient_path")):
            path = getattr(cfg, attr, None)
            if path:
                ff = FlatField.load(path)
                if ff is not None:
                    by_mode[mode] = ff
        default = None
        legacy = getattr(cfg, "flatfield_path", None)
        if legacy:
            default = FlatField.load(legacy)
        return cls(by_mode, default)


def build_flatfield(frames, *, smooth_sigma: float = 2.5,
                    note: str = "") -> FlatField:
    """Build a per-zone gain map from a flat-field capture.

    `frames`: an (N, H, W) stack (or iterable) of *reflectance* frames captured
    while slowly panning the sensor across a uniform matte surface. Panning is
    what makes this valid: real surface texture smears across zones and averages
    toward the smooth illumination, while the sensor's fixed per-zone response
    stays locked -- so the deviation of the per-zone average from its smoothed
    self IS the FPN.

    gain = smooth(avg) / avg, clipped and normalized to unit mean. Applying it
    (`raw * gain`) flattens the per-zone ripple while preserving the smooth
    illumination profile and the overall level.

    A single static frame (no panning) is NOT a valid input -- it bakes scene
    texture into the "correction". This is not enforceable from the data alone;
    it is the operator's responsibility (see docs/flatfield-calibration.md).
    """
    arr = np.asarray(frames, dtype=np.float64)
    if arr.ndim == 2:
        arr = arr[None]
    if arr.ndim != 3 or arr.shape[0] < 1:
        raise ValueError(f"need (N, H, W) reflectance frames, got shape {arr.shape}")
    avg = arr.mean(axis=0)
    eps = max(1e-6, 1e-3 * float(np.nanmedian(avg)))
    illum = _blur(avg, smooth_sigma)
    gain = illum / np.maximum(avg, eps)
    gain = np.clip(gain, *_GAIN_CLIP)
    gain /= float(gain.mean())               # unit-mean: correction preserves level
    meta = {"n_frames": int(arr.shape[0]), "smooth_sigma": float(smooth_sigma),
            "residual_pct": float(100.0 * (avg / illum - 1.0).std()), "note": note}
    return FlatField(gain=gain.astype(np.float32), meta=meta)
