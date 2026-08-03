"""Table-driven tests for roomscan.profiles — the truthful ranging-profile contract.

Covers all four profile definitions, enum wire values, FPS<->period conversion,
DSS/context rules, the transport warning threshold, and invalid manual
combinations, with explicit boundary cases at 1/60/61/90/100 fps and 1/16 ms
exposure (Task 1 of docs/superpowers/plans/2026-07-31-high-framerate-and-manual-
ranging-modes.md).
"""
from __future__ import annotations

import pytest

from roomscan.profiles import (BLANKING_MARGIN_US_PENDING_HW, DSS_FPS_CEILING,
                               EXPOSURE_MS_MAX, EXPOSURE_MS_MIN, FPS_MAX, FPS_MIN,
                               IMU_ENV_HUB_CYCLE_HZ, IMU_ENV_RATE_MAX_HZ,
                               MAX_RANGE_M, MIN_DISTANCE_MM, PRESETS,
                               TRANSPORT_CDC_FPS_CEILING, ManualParams,
                               PowerMode, ProfileConfig, ProfileId, RangingMode,
                               dss_enabled_for_fps, duty_cycle, estimate_manual,
                               estimate_power_mw, estimate_preset,
                               estimate_profile, fps_to_period_us,
                               manual_profile_config, period_us_to_fps,
                               transport_warning_message, validate_imu_env_rate,
                               validate_manual_params)

# ---------------------------------------------------------------------------
# Enum wire values — these are the SET_RANGING_PROFILE / SET_MANUAL_PARAMS /
# vl53l9_context_t / vl53l9_power_mode_t encodings other tasks' codecs depend on.
# ---------------------------------------------------------------------------


def test_profile_id_wire_values():
    assert ProfileId.ROOM_MAPPING == 0
    assert ProfileId.PRECISION == 1
    assert ProfileId.HIGH_FRAMERATE == 2
    assert ProfileId.MANUAL == 3


def test_ranging_mode_matches_vl53l9_context_t():
    # VL53L9_CONTEXT_SHORT = 0 (Precision), VL53L9_CONTEXT_LONG = 1 (Ambient)
    # firmware/vendor/53L9A1/Drivers/BSP/Components/vl53l9/vl53l9.h:103-106
    assert RangingMode.PRECISION == 0
    assert RangingMode.AMBIENT == 1


def test_power_mode_matches_vl53l9_power_mode_t():
    # vl53l9.h:85-89
    assert PowerMode.REGULAR == 0
    assert PowerMode.LOW == 1
    assert PowerMode.ULTRA_LOW == 2


# ---------------------------------------------------------------------------
# The four profile definitions
# ---------------------------------------------------------------------------

# (profile_id, ranging_mode, fps, exposure_ms, power_mode) exactly as amended
# in docs/superpowers/specs/2026-07-31-high-framerate-and-manual-ranging-modes.md
PRESET_TABLE = [
    (ProfileId.ROOM_MAPPING, RangingMode.AMBIENT, 30, 6, PowerMode.ULTRA_LOW),
    (ProfileId.PRECISION, RangingMode.PRECISION, 30, 10, PowerMode.ULTRA_LOW),
    (ProfileId.HIGH_FRAMERATE, RangingMode.PRECISION, 90, 4, PowerMode.REGULAR),
]


@pytest.mark.parametrize("profile_id,ranging_mode,fps,exposure_ms,power_mode", PRESET_TABLE)
def test_preset_fields(profile_id, ranging_mode, fps, exposure_ms, power_mode):
    cfg = PRESETS[profile_id]
    assert cfg.ranging_mode is ranging_mode
    assert cfg.fps == fps
    assert cfg.exposure_ms == exposure_ms
    assert cfg.power_mode is power_mode


def test_presets_cover_exactly_the_three_non_manual_ids():
    assert set(PRESETS) == {ProfileId.ROOM_MAPPING, ProfileId.PRECISION,
                            ProfileId.HIGH_FRAMERATE}


def test_room_mapping_dss_enabled():
    assert PRESETS[ProfileId.ROOM_MAPPING].dss_enabled is True


def test_precision_preset_dss_enabled():
    assert PRESETS[ProfileId.PRECISION].dss_enabled is True


def test_high_framerate_preset_dss_disabled():
    # 90 fps > DSS_FPS_CEILING (60) -> DSS forced off, per the global constraint.
    assert PRESETS[ProfileId.HIGH_FRAMERATE].dss_enabled is False


@pytest.mark.parametrize("profile_id", list(PRESETS))
def test_every_preset_validates_clean(profile_id):
    """A preset must never trip its own manual-mode validation — same formula,
    no special-casing (plan Task 1 step 3: "Do NOT special-case preset labels
    over a different manual formula")."""
    cfg = PRESETS[profile_id]
    params = ManualParams(cfg.ranging_mode, cfg.fps, cfg.exposure_ms, cfg.power_mode)
    result = validate_manual_params(params)
    assert result.ok, result.errors


# --- reconciled anchors: presets must reproduce DS14879 Table 9's own figures ------


def test_room_mapping_power_matches_table9_anchor_exactly():
    # Table 9 "Profile examples": Room Mapping, Ambient, 30 fps, 6 ms, ULP -> 200 mW.
    # This anchor was used to DERIVE the ULP/Ambient intercept, so it must reproduce
    # exactly (within float rounding), not approximately.
    est = estimate_preset(ProfileId.ROOM_MAPPING)
    assert est.power_mw == pytest.approx(200.0, abs=0.1)


def test_room_mapping_range_matches_table9_anchor():
    est = estimate_preset(ProfileId.ROOM_MAPPING)
    assert est.max_range_m == pytest.approx(8.0)


def test_precision_preset_range_matches_datasheet_ceiling():
    # DS14879's own stated ranging ceiling ("up to 8.8 m") and Table 23 gray 62%.
    est = estimate_preset(ProfileId.PRECISION)
    assert est.max_range_m == pytest.approx(8.8)


def test_high_framerate_range_matches_table9_gaming_anchor_exactly():
    # Table 9 Gaming: Precision (no DSS), 100 fps, 4 ms, Regular -> 5 m max range.
    # Range does not depend on fps in this model, so our 90 fps preset still hits
    # the exact Table 9 figure.
    est = estimate_preset(ProfileId.HIGH_FRAMERATE)
    assert est.max_range_m == pytest.approx(5.0)


def test_high_framerate_power_close_to_table9_gaming_anchor():
    # Table 9 Gaming anchor is 420 mW @ 100 fps; our preset deliberately runs at
    # 90 fps (see module docstring), so this is a proximity check, not an exact
    # reproduction: predicted 415.0 mW, 1.2% off the nearby real anchor.
    est = estimate_preset(ProfileId.HIGH_FRAMERATE)
    assert est.power_mw == pytest.approx(415.0, abs=0.5)
    assert abs(est.power_mw - 420.0) / 420.0 < 0.02


def test_min_distance_ambient_vs_precision():
    # DS14879 Table 22, exact.
    assert MIN_DISTANCE_MM[RangingMode.AMBIENT] == 450.0
    assert MIN_DISTANCE_MM[RangingMode.PRECISION] == 50.0


# ---------------------------------------------------------------------------
# FPS <-> frame period conversion
# ---------------------------------------------------------------------------

FPS_PERIOD_TABLE = [
    (1, 1_000_000),
    (30, 33333),
    (60, 16667),
    (61, 16393),
    (90, 11111),
    (100, 10000),
]


@pytest.mark.parametrize("fps,period_us", FPS_PERIOD_TABLE)
def test_fps_to_period_us(fps, period_us):
    assert fps_to_period_us(fps) == period_us


@pytest.mark.parametrize("fps", [1, 30, 60, 61, 90, 100])
def test_period_round_trips_to_fps_within_rounding(fps):
    period_us = fps_to_period_us(fps)
    assert period_us_to_fps(period_us) == pytest.approx(fps, abs=0.01)


# ---------------------------------------------------------------------------
# DSS / context rules
# ---------------------------------------------------------------------------

DSS_BOUNDARY_TABLE = [
    (1, True),
    (60, True),
    (61, False),
    (90, False),
    (100, False),
]


@pytest.mark.parametrize("fps,expected_dss", DSS_BOUNDARY_TABLE)
def test_dss_enabled_for_fps_boundary(fps, expected_dss):
    assert dss_enabled_for_fps(fps) is expected_dss


@pytest.mark.parametrize("fps", [61, 90, 100])
def test_ambient_above_dss_ceiling_is_rejected(fps):
    params = ManualParams(RangingMode.AMBIENT, fps, 4, PowerMode.REGULAR)
    result = validate_manual_params(params)
    assert not result.ok
    assert any("Precision" in e for e in result.errors)


@pytest.mark.parametrize("fps", [61, 90, 100])
def test_precision_above_dss_ceiling_is_accepted(fps):
    params = ManualParams(RangingMode.PRECISION, fps, 4, PowerMode.REGULAR)
    result = validate_manual_params(params)
    assert result.ok, result.errors


@pytest.mark.parametrize("fps", [1, 30, 60])
def test_ambient_at_or_below_dss_ceiling_is_accepted(fps):
    params = ManualParams(RangingMode.AMBIENT, fps, 4, PowerMode.REGULAR)
    result = validate_manual_params(params)
    assert result.ok, result.errors


def test_above_60_hz_only_valid_combination_is_precision():
    # (AMBIENT, dss_off) is deliberately absent from real use (kept only so the
    # MAX_RANGE_M lookup never KeyErrors on an already-invalid config).
    assert (RangingMode.PRECISION, False) in MAX_RANGE_M
    assert (RangingMode.AMBIENT, False) in MAX_RANGE_M  # present, but unreachable
                                                         # through valid manual params


# ---------------------------------------------------------------------------
# Transport warning threshold
# ---------------------------------------------------------------------------

TRANSPORT_TABLE = [
    ("cdc", 1, False),
    ("cdc", 60, False),
    ("cdc", 61, True),
    ("cdc", 90, True),
    ("cdc", 100, True),
    ("ethernet", 61, False),
    ("ethernet", 90, False),
    ("ethernet", 100, False),
    ("replay", 90, False),
]


@pytest.mark.parametrize("transport,fps,expect_warning", TRANSPORT_TABLE)
def test_transport_warning_threshold(transport, fps, expect_warning):
    msg = transport_warning_message(transport, fps)
    assert (msg is not None) is expect_warning


def test_transport_warning_is_non_blocking_on_estimate():
    # The spec asks for a warning, not a hard ban: a >60 fps CDC estimate must
    # still be `ok` (assuming the rest of the config is valid).
    cfg = ProfileConfig(ProfileId.MANUAL, RangingMode.PRECISION, 90, 4, PowerMode.REGULAR)
    est = estimate_profile(cfg, transport="cdc")
    assert est.ok
    assert est.transport_warning is not None
    assert est.transport_warning in est.warnings


def test_transport_case_insensitive():
    assert transport_warning_message("CDC", 90) is not None


def test_transport_ceiling_matches_dss_ceiling():
    # Both the DSS/context rule and the CDC transport warning use 60 Hz as their
    # boundary; pinning the relationship catches an accidental drift if either
    # constant is edited independently.
    assert TRANSPORT_CDC_FPS_CEILING == DSS_FPS_CEILING == 60


# ---------------------------------------------------------------------------
# Invalid manual combinations
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("fps", [FPS_MIN - 1, 0, FPS_MAX + 1, 101, -5])
def test_manual_fps_out_of_range_rejected(fps):
    params = ManualParams(RangingMode.PRECISION, fps, 4, PowerMode.REGULAR)
    result = validate_manual_params(params)
    assert not result.ok


@pytest.mark.parametrize("fps", [FPS_MIN, FPS_MAX])
def test_manual_fps_boundary_accepted(fps):
    # Boundary values themselves (1 and 100 fps) with compatible exposure/mode.
    ranging = RangingMode.PRECISION if fps > DSS_FPS_CEILING else RangingMode.AMBIENT
    params = ManualParams(ranging, fps, 1, PowerMode.REGULAR)
    result = validate_manual_params(params)
    assert result.ok, result.errors


@pytest.mark.parametrize("exposure_ms", [EXPOSURE_MS_MIN - 1, 0, EXPOSURE_MS_MAX + 1, 17])
def test_manual_exposure_out_of_range_rejected(exposure_ms):
    params = ManualParams(RangingMode.AMBIENT, 30, exposure_ms, PowerMode.REGULAR)
    result = validate_manual_params(params)
    assert not result.ok


@pytest.mark.parametrize("exposure_ms", [EXPOSURE_MS_MIN, EXPOSURE_MS_MAX])
def test_manual_exposure_boundary_accepted_at_low_fps(exposure_ms):
    # 1 ms and 16 ms both fit comfortably inside a 30 fps (33333 us) period.
    params = ManualParams(RangingMode.AMBIENT, 30, exposure_ms, PowerMode.REGULAR)
    result = validate_manual_params(params)
    assert result.ok, result.errors


def test_manual_16ms_exposure_rejected_at_90fps_exceeds_period():
    # period = 11111 us; 16 ms = 16000 us alone exceeds the period.
    params = ManualParams(RangingMode.PRECISION, 90, 16, PowerMode.REGULAR)
    result = validate_manual_params(params)
    assert not result.ok
    assert any("does not fit" in e for e in result.errors)


def test_manual_16ms_exposure_rejected_at_100fps():
    params = ManualParams(RangingMode.PRECISION, 100, 16, PowerMode.REGULAR)
    result = validate_manual_params(params)
    assert not result.ok


def test_manual_blanking_margin_boundary():
    # At 60 fps (period 16667 us), 16 ms exposure (16000 us) + the 500 us margin
    # placeholder = 16500 us, which fits; one tick more would not.
    ok_params = ManualParams(RangingMode.AMBIENT, 60, 16, PowerMode.REGULAR)
    assert validate_manual_params(ok_params).ok

    period_us = fps_to_period_us(61)
    exposure_us_that_just_fails = period_us - BLANKING_MARGIN_US_PENDING_HW + 1
    exposure_ms_that_just_fails = -(-exposure_us_that_just_fails // 1000)  # ceil
    if exposure_ms_that_just_fails <= EXPOSURE_MS_MAX:
        bad = ManualParams(RangingMode.PRECISION, 61, exposure_ms_that_just_fails,
                           PowerMode.REGULAR)
        assert not validate_manual_params(bad).ok


def test_manual_valid_combination_accepted():
    params = ManualParams(RangingMode.PRECISION, 61, 4, PowerMode.REGULAR)
    result = validate_manual_params(params)
    assert result.ok
    assert result.errors == ()


def test_manual_multiple_errors_all_reported():
    # fps out of range AND ambient above the DSS ceiling AND exposure out of range:
    # every independent problem should surface, not just the first one found.
    params = ManualParams(RangingMode.AMBIENT, 200, 99, PowerMode.REGULAR)
    result = validate_manual_params(params)
    assert not result.ok
    assert len(result.errors) >= 2


# ---------------------------------------------------------------------------
# IMU/env rate validation (Task 7 cross-reference: profiles.py owns this too)
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("rate", [None, 0])
def test_imu_env_rate_coupled_always_valid(rate):
    result = validate_imu_env_rate(rate)
    assert result.ok
    assert result.warnings == ()


@pytest.mark.parametrize("rate", [1, 30, 60, IMU_ENV_HUB_CYCLE_HZ, IMU_ENV_RATE_MAX_HZ, 480])
def test_imu_env_rate_in_range_valid(rate):
    assert validate_imu_env_rate(rate).ok


@pytest.mark.parametrize("rate", [-1, 481, 1000])
def test_imu_env_rate_out_of_range_rejected(rate):
    result = validate_imu_env_rate(rate)
    assert not result.ok


def test_imu_env_rate_above_hub_cycle_warns_not_rejects():
    # 61 Hz exceeds the 60 Hz sensor-hub cycle -- reported, not silently dropped
    # nor rejected (global constraint).
    result = validate_imu_env_rate(61)
    assert result.ok
    assert result.warnings
    assert "sub-sample" in result.warnings[0]


def test_imu_env_rate_at_hub_cycle_boundary_no_warning():
    result = validate_imu_env_rate(60)
    assert result.ok
    assert result.warnings == ()


def test_manual_params_imu_env_rate_defaults_to_coupled():
    params = ManualParams(RangingMode.AMBIENT, 30, 6, PowerMode.ULTRA_LOW)
    assert params.imu_env_rate_hz is None


def test_manual_params_with_decoupled_imu_env_rate_flows_into_estimate():
    params = ManualParams(RangingMode.AMBIENT, 30, 6, PowerMode.ULTRA_LOW, imu_env_rate_hz=90)
    est = estimate_manual(params)
    assert est.ok
    assert est.imu_env_rate_hz == 90
    assert est.imu_env_coupled is False
    assert any("sub-sample" in w for w in est.warnings)


def test_estimate_defaults_to_coupled_imu_env():
    est = estimate_preset(ProfileId.ROOM_MAPPING)
    assert est.imu_env_coupled is True
    assert est.imu_env_rate_hz is None


# ---------------------------------------------------------------------------
# duty_cycle / power model sanity
# ---------------------------------------------------------------------------


def test_duty_cycle_examples():
    assert duty_cycle(6, 30) == pytest.approx(0.18)
    assert duty_cycle(4, 100) == pytest.approx(0.4)
    assert duty_cycle(16, 100) == 1.0  # clamped -- 1.6 raw would be nonsensical


def test_duty_cycle_clamped_to_unit_interval():
    assert duty_cycle(1000, 100) == 1.0
    assert duty_cycle(0, 0) == 0.0


def test_power_increases_monotonically_with_duty():
    low = estimate_power_mw(RangingMode.PRECISION, PowerMode.REGULAR, 4, 30)
    high = estimate_power_mw(RangingMode.PRECISION, PowerMode.REGULAR, 10, 30)
    assert high > low


def test_power_regular_exceeds_ultra_low_at_same_duty():
    reg = estimate_power_mw(RangingMode.AMBIENT, PowerMode.REGULAR, 6, 30)
    ulp = estimate_power_mw(RangingMode.AMBIENT, PowerMode.ULTRA_LOW, 6, 30)
    assert reg > ulp


# ---------------------------------------------------------------------------
# I3C bus airtime
# ---------------------------------------------------------------------------

I3C_UTIL_TABLE = [
    (30, 28.5),
    (60, 57.0),
    (90, 85.5),
    (100, 95.0),
]


@pytest.mark.parametrize("fps,expected_pct", I3C_UTIL_TABLE)
def test_i3c_bus_utilization_matches_spec_percentages(fps, expected_pct):
    from roomscan.profiles import i3c_bus_utilization_pct
    assert i3c_bus_utilization_pct(fps) == pytest.approx(expected_pct, abs=0.1)


def test_i3c_airtime_left_complements_utilization():
    est = estimate_preset(ProfileId.HIGH_FRAMERATE)
    assert est.i3c_bus_utilization_pct + est.i3c_airtime_left_pct == pytest.approx(100.0)


# ---------------------------------------------------------------------------
# ProfileConfig / manual_profile_config plumbing
# ---------------------------------------------------------------------------


def test_manual_profile_config_round_trip():
    params = ManualParams(RangingMode.PRECISION, 75, 3, PowerMode.LOW, imu_env_rate_hz=30)
    cfg = manual_profile_config(params)
    assert cfg.profile_id is ProfileId.MANUAL
    assert cfg.ranging_mode is params.ranging_mode
    assert cfg.fps == params.fps
    assert cfg.exposure_ms == params.exposure_ms
    assert cfg.power_mode is params.power_mode
    assert cfg.imu_env_rate_hz == 30


def test_estimate_preset_rejects_manual_id():
    with pytest.raises(ValueError):
        estimate_preset(ProfileId.MANUAL)


def test_estimate_manual_invalid_params_reports_errors_not_raises():
    params = ManualParams(RangingMode.AMBIENT, 90, 16, PowerMode.REGULAR)
    est = estimate_manual(params)
    assert not est.ok
    assert est.errors
