from roomscan.config import ViewerConfig


def test_sensor_config_defaults():
    c = ViewerConfig()
    assert c.imu_gizmo is True
    assert c.sensors_panel is True
    assert c.gizmo_scale == 0.15
