"""BUG-058: BUG-051's fix used yaw_twist_deg (swing-twist about world Z) as the
heading; with a horizontal boresight it absorbs roll one-for-one. YawFusion
differenced a CCW field azimuth against that twist ("heading - yaw")."""
from roomscan.sensors import YawFusion, graft_yaw, tilt_compensated_heading, yaw_twist_deg


def feed(fusion: YawFusion, quat, mag_ut, dt: float) -> None:
    heading = tilt_compensated_heading(quat, mag_ut)
    yaw = yaw_twist_deg(quat)
    fusion._absorb_north_bearing(heading - yaw, dt)      # float where a Bearing is required


def level(quat):
    return graft_yaw(quat, -yaw_twist_deg(quat))          # -Twist is a float, not a Twist
