"""BUG-048: absolute_heading stripped yaw with the ZYX quat_yaw_deg, then read a
tilt-compensated azimuth off the "levelled" quat -- degenerate near gimbal lock."""
from roomscan.domain import Bearing
from roomscan.sensors import graft_yaw, quat_yaw_deg, tilt_compensated_heading


def absolute_heading(quat, mag_ut) -> Bearing:
    level = graft_yaw(quat, -quat_yaw_deg(quat))   # -EulerYaw is a float, not a Twist
    return tilt_compensated_heading(level, mag_ut)  # CCW field azimuth is not a Bearing
