"""One menu-opened settings dialog carrying the former sidebar's grouped
controls (spec 5.4). The panel holds no always-visible side panel anymore.

The grouped controls are built ONCE by ``ControlPanel._build_settings_widgets``
into a detached root (``panel._settings_root``) with each section recorded in
``panel._settings_groups``. ``build_settings_dialog`` wraps that single root in a
fresh ``gui.Dialog`` and expands the requested section, so the live control
widgets keep their single, canonical instance (the tick and the ``_on_*``
handlers reference them by attribute on the panel). This avoids rebuilding the
widget tree -- and re-wiring every handler -- on each open.
"""
from __future__ import annotations

# Menu-openable section ordering contract the menubar relies on. Extra sections
# present in the dialog (SLAM, Showcase, Sensors, Events) are not menu targets.
SECTIONS = ["View", "Surface", "IR Monitor", "Device", "Capture"]


def build_settings_dialog(panel, *, section: str | None = None):
    """Wrap the panel's persistent settings root in a fresh ``gui.Dialog``.

    When ``section`` is given, that CollapsableVert is expanded and every other
    tracked group is collapsed so the dialog opens focused on the menu the user
    picked (View / Device). ``section=None`` leaves each group at its
    construction-time open state.
    """
    gui = panel._gui
    dlg = gui.Dialog("Settings")
    groups = getattr(panel, "_settings_groups", {}) or {}
    if section is not None:
        for title, group in groups.items():
            group.set_is_open(title == section)
    dlg.add_child(panel._settings_root)
    return dlg
