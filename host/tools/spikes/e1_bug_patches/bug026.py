"""BUG-026: a body-frame point cloud was drawn as if it were world-frame (the
display rotation was not applied). NOT ENCODED in E1: frame-tagged arrays need a
generic wrapper around numpy arrays (spec item #4), which is out of E1's scope.
Pyright accepts this file."""
import numpy as np

from roomscan.sensors import display_rotation


def world_cloud(body_pts: np.ndarray, quat) -> np.ndarray:
    _R = display_rotation(quat)      # computed and ignored -- the actual defect
    return body_pts                  # body-frame points returned as world-frame
