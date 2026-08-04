"""ST ProfileTuning vs the rig's own model.

The tool's whole job is to REPORT disagreement rather than resolve it toward the
more optimistic number, so these pin the disagreements it must name -- and the
shape it reports them in, which is shared with `profile_estimate()`/`rig_profile()`.
"""
from tools.profile_tuning import analyze_profile


def test_dss_off_i3c_report_names_the_unverified_layout_conflict():
    report = analyze_profile(dss=False, output_interface="I3C", fps=66, exposure_ms=10)

    assert report["st_profiletuning"]["frame_bytes"] == 106.0
    assert report["st_profiletuning"]["readout_ms"] == 0.0848
    assert not report["agree"]
    assert any("DSS-off I3C conflict" in message for message in report["disagreements"])


def test_dss_on_i3c_report_includes_hardware_delivery_warning():
    report = analyze_profile(dss=True, output_interface="I3C", fps=90, exposure_ms=4)

    assert report["roomscanner_full_map"]["expected_delivered_fps"] == 45.0
    assert any("Rate conflict" in message for message in report["disagreements"])


def test_the_rig_side_speaks_the_shared_estimate_vocabulary():
    """`roomscanner_full_map` is the same shape `profile_estimate()` returns.

    It used to be `dataclasses.asdict`, which emitted raw enum ints for
    ranging/power mode and silently dropped `ok` (a property, not a field) --
    a second spelling of one estimate, which is how two readers of the same
    quantity drift apart.
    """
    from roomscan import profiles

    report = analyze_profile(ranging_mode="Precision", power_config="Regular",
                             fps=30, exposure_ms=10)
    rig = report["roomscanner_full_map"]

    assert rig == profiles.estimate_to_json(profiles.estimate_manual(
        profiles.ManualParams(profiles.RangingMode.PRECISION, 30, 10,
                              profiles.PowerMode.REGULAR)))
    assert rig["ranging_mode"] == "precision" and rig["power_mode"] == "regular"
    assert rig["ok"] is True


def test_ambient_lux_reaches_the_rig_power_model():
    """It was accepted by the CLI and then `del`'d, so the flag moved nothing.
    Only the rig side may move: ST's timing model has no ambient term."""
    dark = analyze_profile(ambient_lux=0.0)
    bright = analyze_profile(ambient_lux=100000.0)

    assert bright["roomscanner_full_map"]["power_mw"] > dark["roomscanner_full_map"]["power_mw"]
    assert bright["st_profiletuning"] == dark["st_profiletuning"]
    assert bright["request"]["ambient_lux"] == 100000.0


def test_a_non_i3c_interface_is_flagged_as_out_of_the_rigs_scope():
    """CSI-2 is not wired on this board (zero occurrences in the schematic), so a
    CSI-2 row is an ST planning estimate and must not read as a rig prediction."""
    report = analyze_profile(output_interface="CSI2", fps=30, exposure_ms=10)

    assert any("Interface scope conflict" in m for m in report["disagreements"])


def test_a_dss_request_that_contradicts_the_rate_policy_is_named():
    """DSS is derived from fps (on at <=60, off above), never user-set, so asking
    for DSS on at 90 fps is a request the rig cannot honour."""
    report = analyze_profile(dss=True, fps=90, exposure_ms=2)

    assert report["roomscanner_full_map"]["dss_enabled"] is False
    assert any("DSS policy conflict" in m for m in report["disagreements"])
