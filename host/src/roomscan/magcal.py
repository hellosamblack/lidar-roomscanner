"""Hard/soft-iron magnetometer calibration for the LIS2MDL (stream 10 mag).

calibrated = matrix @ (raw - offset), where `offset` removes hard-iron bias and
`matrix` removes soft-iron scale/skew so calibrated samples lie on a sphere of
radius `field_ut`. Fit from a cloud of raw samples collected while rotating the
rig through all orientations (see `fit_ellipsoid`)."""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class MagCalibration:
    offset: tuple[float, float, float]
    matrix: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]
    field_ut: float

    def apply(self, raw_ut) -> np.ndarray:
        raw = np.asarray(raw_ut, dtype=np.float64)
        m = np.asarray(self.matrix, dtype=np.float64)
        b = np.asarray(self.offset, dtype=np.float64)
        return m @ (raw - b)

    def save(self, path) -> None:
        Path(path).write_text(json.dumps({
            "offset": list(self.offset),
            "matrix": [list(row) for row in self.matrix],
            "field_ut": self.field_ut,
        }), encoding="utf-8")

    @classmethod
    def load(cls, path) -> "MagCalibration | None":
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
            offset = tuple(float(v) for v in data["offset"])
            matrix = tuple(tuple(float(v) for v in row) for row in data["matrix"])
            field_ut = float(data["field_ut"])
            if len(offset) != 3 or len(matrix) != 3 or any(len(r) != 3 for r in matrix):
                return None
            return cls(offset=offset, matrix=matrix, field_ut=field_ut)
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError):
            return None
