"""BUG-026: a body/CV-frame point cloud was drawn as if it were already in the
render world (the display rotation was computed and then not applied).

E1 (2026-08-27) could not encode this -- frame-tagged arrays needed a generic
wrapper. #225 added `domain.Cloud[F]` / `Rotation[To, From]`; with the CV cloud
tagged and the world cloud declared, Pyright REJECTS the defect. The typed
sensors API is `display_frame_rotation` (the bare-matrix `display_rotation`
still exists for numpy consumers and is an accepted escape hatch, like
`decode_env(payload)[0]` for BUG-083).
"""
import numpy as np

from roomscan.domain import CV, Cloud, CvWorld
from roomscan.sensors import display_frame_rotation


def world_cloud(cv_pts: np.ndarray, quat) -> Cloud[CvWorld]:
    cloud = Cloud(cv_pts, CV)
    _R = display_frame_rotation(quat)   # computed and ignored -- the actual defect
    return cloud                        # CV-frame points returned as the world cloud
