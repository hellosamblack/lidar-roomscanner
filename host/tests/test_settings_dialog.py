"""The GUI dialog build is supervised-run verified; here we only assert the
pure section ordering contract that the menubar relies on."""
from roomscan import settings_dialog


def test_sections_order():
    assert settings_dialog.SECTIONS[0] == "View"
    assert "Device" in settings_dialog.SECTIONS
    assert "IR Monitor" in settings_dialog.SECTIONS
