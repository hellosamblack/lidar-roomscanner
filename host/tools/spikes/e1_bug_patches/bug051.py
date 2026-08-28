"""BUG-051: the shipped live path fed quat_yaw_deg (ZYX yaw) to the heading
consumer -- 18.4 deg of systematic error at a true bearing of 45 deg."""
from roomscan.sensors import YawFusion, quat_yaw_deg


def feed(fusion: YawFusion, quat, dt: float) -> None:
    fusion._absorb_north_bearing(quat_yaw_deg(quat), dt)   # EulerYaw where a Bearing is required
