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


def fit_ellipsoid(samples: np.ndarray) -> MagCalibration:
    """Least-squares fit of a 3-D point cloud to an ellipsoid, returned as a
    MagCalibration that maps the cloud onto a sphere.

    Fits a*x^2 + b*y^2 + c*z^2 + 2f*yz + 2g*xz + 2h*xy + 2p*x + 2q*y + 2r*z = 1,
    recovers the center (hard-iron) and a symmetric shape matrix, then forms the
    soft-iron correction S = field * sqrtm(Q_n) so that S @ (raw - center) lies on
    a sphere of radius `field` = geometric mean of the ellipsoid semi-axes."""
    X = np.asarray(samples, dtype=np.float64)
    if X.ndim != 2 or X.shape[1] != 3 or X.shape[0] < 20:
        raise ValueError(f"need an (N>=20, 3) sample array, got {X.shape}")
    x, y, z = X[:, 0], X[:, 1], X[:, 2]
    D = np.column_stack([x * x, y * y, z * z, 2 * y * z, 2 * x * z, 2 * x * y,
                         2 * x, 2 * y, 2 * z])
    v, _res, rank, _sv = np.linalg.lstsq(D, np.ones(X.shape[0]), rcond=None)
    if rank < 9:
        # Rank-deficient design matrix: the samples don't span 3-D (e.g. confined
        # to a plane or line), so the ellipsoid is underdetermined.
        raise ValueError("degenerate ellipsoid fit (rank-deficient sample cloud)")
    a, b, c, f, g, h, p, q, r = v
    Q = np.array([[a, h, g], [h, b, f], [g, f, c]])
    u = np.array([p, q, r])
    try:
        center = -np.linalg.solve(Q, u)
    except np.linalg.LinAlgError as exc:
        raise ValueError("degenerate ellipsoid fit (singular shape matrix)") from exc
    d = 1.0 + center @ Q @ center
    if d <= 0:
        raise ValueError("degenerate ellipsoid fit (non-positive scale)")
    Q_n = Q / d
    evals, evecs = np.linalg.eigh(Q_n)
    if np.any(evals <= 0):
        raise ValueError("degenerate ellipsoid fit (non-positive-definite)")
    semi_axes = 1.0 / np.sqrt(evals)
    field = float(np.prod(semi_axes) ** (1.0 / 3.0))
    sqrt_Qn = evecs @ np.diag(np.sqrt(evals)) @ evecs.T
    S = field * sqrt_Qn
    return MagCalibration(
        offset=tuple(float(v) for v in center),
        matrix=tuple(tuple(float(v) for v in row) for row in S),
        field_ut=field,
    )
