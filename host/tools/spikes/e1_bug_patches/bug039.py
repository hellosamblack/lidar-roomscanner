"""BUG-039: imufusion's yaw loop nulled a difference of ZYX Euler yaws (heading of
body Z on a body frame whose X is Up) and grafted that onto the attitude."""
from roomscan.sensors import graft_yaw, quat_yaw_deg


def correct_yaw(q, yaw_ref, gain: float):
    err = quat_yaw_deg(yaw_ref) - quat_yaw_deg(q)   # EulerYaw - EulerYaw -> float
    return graft_yaw(q, gain * err)                  # graft_yaw wants a Twist
