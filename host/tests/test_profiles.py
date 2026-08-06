"""Table-driven tests for roomscan.profiles — the truthful ranging-profile contract.

Covers all four profile definitions, enum wire values, FPS<->period conversion,
DSS/context rules, the transport warning threshold, and invalid manual
combinations, with explicit boundary cases at 1/60/61/90/100 fps and 1/16 ms
exposure (Task 1 of docs/superpowers/plans/2026-07-31-high-framerate-and-manual-
ranging-modes.md).
"""
from __future__ import annotations

import pytest

from roomscan.profiles import (AMBIENT_LUX_DEFAULT, BLANKING_MARGIN_US_PENDING_HW,
                               DSS_FPS_CEILING,
                               EXPOSURE_MS_MAX, EXPOSURE_MS_MIN, FPS_MAX, FPS_MIN,
                               IMU_ENV_HUB_CYCLE_HZ, IMU_ENV_RATE_MAX_HZ,
                               MAX_RANGE_M, MIN_DISTANCE_MM, PRESETS,
                               TRANSPORT_CDC_FPS_CEILING, ManualParams,
                               PowerMode, ProfileConfig, ProfileId, RangingMode,
                               ceiling_fps_for_exposure, dss_enabled_for_fps,
                               duty_cycle, estimate_manual, estimate_power_mw,
                               estimate_preset, estimate_profile,
                               expected_delivered_fps, fps_to_period_us,
                               manual_profile_config, measured_floor_ms,
                               period_us_to_fps, transport_warning_message,
                               validate_imu_env_rate, validate_manual_params)

# ---------------------------------------------------------------------------
# Enum wire values — these are the SET_RANGING_PROFILE / SET_MANUAL_PARAMS /
# vl53l9_context_t / vl53l9_power_mode_t encodings other tasks' codecs depend on.
# ---------------------------------------------------------------------------


def test_profile_id_wire_values():
    assert ProfileId.STABILITY == 0
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

# (profile_id, ranging_mode, fps, exposure_ms, power_mode, imu_env_rate_hz) --
# USE-CASE presets (owner-directed 2026-08-05, grounded in an on-rig exposure/fps
# sweep): all Precision/Regular/60 Hz IMU, differing only in the exposure-vs-fps
# tradeoff. STABILITY (12 ms) is the default; Precision (15 ms) is the longest
# exposure that holds 30 fps; High Frame-Rate is 4 ms/46 fps.
PRESET_TABLE = [
    (ProfileId.STABILITY, RangingMode.PRECISION, 30, 12, PowerMode.REGULAR, 60),
    (ProfileId.PRECISION, RangingMode.PRECISION, 30, 15, PowerMode.REGULAR, 60),
    (ProfileId.HIGH_FRAMERATE, RangingMode.PRECISION, 46, 4, PowerMode.REGULAR, 60),
]


@pytest.mark.parametrize("profile_id,ranging_mode,fps,exposure_ms,power_mode,imu_hz", PRESET_TABLE)
def test_preset_fields(profile_id, ranging_mode, fps, exposure_ms, power_mode, imu_hz):
    cfg = PRESETS[profile_id]
    assert cfg.ranging_mode is ranging_mode
    assert cfg.fps == fps
    assert cfg.exposure_ms == exposure_ms
    assert cfg.power_mode is power_mode
    assert cfg.imu_env_rate_hz == imu_hz


def test_presets_cover_exactly_the_three_non_manual_ids():
    assert set(PRESETS) == {ProfileId.STABILITY, ProfileId.PRECISION,
                            ProfileId.HIGH_FRAMERATE}


def test_default_profile_is_stability():
    from roomscan.profiles import DEFAULT_PROFILE_ID
    assert DEFAULT_PROFILE_ID is ProfileId.STABILITY
    assert PRESETS[DEFAULT_PROFILE_ID].exposure_ms == 12


def test_stability_dss_enabled():
    assert PRESETS[ProfileId.STABILITY].dss_enabled is True


def test_precision_preset_dss_enabled():
    assert PRESETS[ProfileId.PRECISION].dss_enabled is True


def test_high_framerate_preset_dss_enabled():
    # Amended 2026-08-03 (measured hardware ceiling): 46 fps <= DSS_FPS_CEILING
    # (60) -> DSS on, per the global constraint. Was DSS-off at the old 90 fps
    # design point.
    assert PRESETS[ProfileId.HIGH_FRAMERATE].dss_enabled is True


@pytest.mark.parametrize("profile_id", list(PRESETS))
def test_every_preset_validates_clean(profile_id):
    """A preset must never trip its own manual-mode validation — same formula,
    no special-casing (plan Task 1 step 3: "Do NOT special-case preset labels
    over a different manual formula")."""
    cfg = PRESETS[profile_id]
    params = ManualParams(cfg.ranging_mode, cfg.fps, cfg.exposure_ms, cfg.power_mode)
    result = validate_manual_params(params)
    assert result.ok, result.errors


# --- reconciled anchors: presets must reproduce DS14879 Table 9's own range figures ---
# (power anchors moved to "Power model (decompiled ProfileTuning.exe)" below, 2026-08-03
# -- the power model itself was REPLACED, not just re-tuned, so it no longer targets
# DS14879's Table 9/36 rows the way the retired fitted model did by construction.)


def test_ambient_dss_range_matches_table9_anchor():
    # The 8.0 m Ambient+DSS anchor is a MAX_RANGE_M model check -- no longer tied to a
    # preset (all presets are Precision since 2026-08-05), so exercise it directly.
    est = estimate_manual(ManualParams(RangingMode.AMBIENT, 30, 6, PowerMode.ULTRA_LOW))
    assert est.max_range_m == pytest.approx(8.0)


def test_stability_and_precision_presets_reach_datasheet_precision_ceiling():
    # Both default-family presets are Precision + DSS on -> 8.8 m (past the ~6 m/20 ft
    # this rig scans), so Ambient's longer range is never needed.
    assert estimate_preset(ProfileId.STABILITY).max_range_m == pytest.approx(8.8)
    assert estimate_preset(ProfileId.PRECISION).max_range_m == pytest.approx(8.8)


def test_precision_preset_range_matches_datasheet_ceiling():
    # DS14879's own stated ranging ceiling ("up to 8.8 m") and Table 23 gray 62%.
    est = estimate_preset(ProfileId.PRECISION)
    assert est.max_range_m == pytest.approx(8.8)


def test_precision_dss_off_range_matches_table9_gaming_anchor_exactly():
    # Table 9 Gaming: Precision (no DSS), 100 fps, 4 ms, Regular -> 5 m max range.
    # This is a MAX_RANGE_M lookup check, not the High Frame-Rate preset (amended
    # 2026-08-03 to 46 fps/DSS-on, which no longer hits this lookup cell).
    assert MAX_RANGE_M[(RangingMode.PRECISION, False)] == pytest.approx(5.0)


# --- Power model (decompiled ProfileTuning.exe, 2026-08-03) -----------------------
# REPLACES the earlier DS14879-two-point-fit model (see profiles.py module docstring
# "Power — SUPERSEDED 2026-08-03" / "Power model, decompiled ProfileTuning.exe
# (2026-08-03)"). All five owner-run ProfileTuning.exe tool readings this model was
# gated on: 54x42, I3C, DSS Enable, ambient = "Home, Theatres - 100 Lux"
# (AMBIENT_LUX_DEFAULT) unless noted. Model reproduces every one to within 0.01%,
# an order of magnitude inside the ~2% bar the replacement was gated on.

POWER_ANCHOR_TABLE = [
    # (ranging_mode, power_mode, exposure_ms, fps, tool_reading_mw)
    (RangingMode.AMBIENT, PowerMode.ULTRA_LOW, 6, 30, 208.4),
    (RangingMode.PRECISION, PowerMode.ULTRA_LOW, 10, 30, 255.7),
    (RangingMode.PRECISION, PowerMode.REGULAR, 4, 37, 243.7),
    (RangingMode.AMBIENT, PowerMode.REGULAR, 2, 37, 210.5),
    (RangingMode.AMBIENT, PowerMode.REGULAR, 4, 37, 260.0),
]


@pytest.mark.parametrize("ranging_mode,power_mode,exposure_ms,fps,tool_reading_mw",
                         POWER_ANCHOR_TABLE)
def test_power_matches_decompiled_profiletuning_anchor(
        ranging_mode, power_mode, exposure_ms, fps, tool_reading_mw):
    power_mw = estimate_power_mw(ranging_mode, power_mode, exposure_ms, fps)
    assert power_mw == pytest.approx(tool_reading_mw, abs=tool_reading_mw * 0.02)


def test_ambient_6ms_power_matches_profiletuning_not_old_ds14879_anchor():
    # The Ambient/ULP/6ms/30fps operating point (the retired Room Mapping preset)
    # remains a valid POWER MODEL anchor even though no preset uses it now: DS14879
    # Table 9's footnoted 200 mW "(5 klx)" is not ProfileTuning's 100-lux default, and
    # the model gives ~208.4 mW, matching POWER_ANCHOR_TABLE, not the retired 200.0.
    est = estimate_manual(ManualParams(RangingMode.AMBIENT, 30, 6, PowerMode.ULTRA_LOW))
    assert est.power_mw == pytest.approx(208.4, abs=0.1)
    assert est.power_mw != pytest.approx(200.0, abs=0.1)


def test_ambient_lux_is_a_genuine_power_input():
    # The old fitted model had no ambient-light parameter at all; the decompiled
    # model's VBAT_Rx term scales with it directly, so darker/brighter readings must
    # move the estimate apart at fixed ranging/power/exposure/fps.
    dark = estimate_power_mw(RangingMode.AMBIENT, PowerMode.REGULAR, 6, 30, ambient_lux=0.0)
    bright = estimate_power_mw(RangingMode.AMBIENT, PowerMode.REGULAR, 6, 30,
                               ambient_lux=100000.0)
    assert bright > dark


def test_ambient_lux_reaches_the_estimate_and_not_just_the_power_function():
    """The knob existed on `estimate_power_mw` but no estimate could pass it, so
    a caller offering an ambient-lux option (the ProfileTuning comparison CLI did)
    silently dropped it and reported the default. Assert the value MOVES the
    estimate, not merely that the parameter is accepted."""
    params = ManualParams(RangingMode.AMBIENT, 30, 6, PowerMode.REGULAR)
    dark = estimate_manual(params, ambient_lux=0.0)
    bright = estimate_manual(params, ambient_lux=100000.0)
    default = estimate_manual(params)

    assert bright.power_mw > dark.power_mw
    assert default.power_mw == pytest.approx(
        estimate_manual(params, ambient_lux=AMBIENT_LUX_DEFAULT).power_mw)
    ambient_params = ManualParams(RangingMode.AMBIENT, 30, 6, PowerMode.ULTRA_LOW)
    assert (estimate_manual(ambient_params, ambient_lux=100000.0).power_mw
            > estimate_manual(ambient_params).power_mw)


# --- High Frame-Rate preset, amended 2026-08-03: measured hardware ceiling ---------
# (Task 5's on-target sweep found 90 fps requests deliver only ~44.85 fps -- a clean
# 2x period multiple, not 1:1 -- so the preset moved to 46 fps, the measured 1x
# ceiling at 4 ms exposure. See profiles.py module docstring "Measured hardware
# ceiling" and docs/superpowers/specs/2026-07-31-high-framerate-and-manual-ranging-
# modes.md Sec 2.1/8.)


def test_high_framerate_preset_is_46fps_not_90fps():
    cfg = PRESETS[ProfileId.HIGH_FRAMERATE]
    assert cfg.fps == 46
    assert cfg.exposure_ms == 4
    assert cfg.ranging_mode is RangingMode.PRECISION
    assert cfg.power_mode is PowerMode.REGULAR


def test_high_framerate_preset_range_is_precision_dss_on_now():
    # DSS is now ON at 46 fps, so the preset gets the Precision/DSS-on 8.8 m
    # figure (same lookup cell as the Precision preset), not the old 5.0 m
    # DSS-off figure.
    est = estimate_preset(ProfileId.HIGH_FRAMERATE)
    assert est.max_range_m == pytest.approx(8.8)


def test_high_framerate_preset_power_recomputes_at_46fps():
    # Decompiled ProfileTuning.exe model (2026-08-03, replaces the retired
    # DS14879-fit model): Precision/Regular/4 ms/46 fps/DSS-on, ambient =
    # AMBIENT_LUX_DEFAULT -> 273.6 mW. Distinct from the same config's own
    # 90 fps/DSS-off figure under this model (369.8 mW, computed directly below)
    # -- the power estimate must actually move with the preset's own fps/DSS
    # state, not stay pinned to the retired 90 fps design point.
    est = estimate_preset(ProfileId.HIGH_FRAMERATE)
    assert est.power_mw == pytest.approx(273.6, abs=0.1)
    ninety_fps_dss_off = estimate_power_mw(RangingMode.PRECISION, PowerMode.REGULAR, 4, 90)
    assert est.power_mw != pytest.approx(ninety_fps_dss_off, abs=0.1)


def test_high_framerate_preset_delivers_1x_no_quantization():
    # The whole point of the amendment: the preset itself must sit AT or below
    # its own exposure's measured 1x ceiling, so it is delivered 1:1 with no
    # warning and no quantization.
    est = estimate_preset(ProfileId.HIGH_FRAMERATE)
    assert est.expected_delivered_fps == pytest.approx(46.0, abs=0.05)
    assert est.warnings == ()
    assert est.ok


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


def test_estimate_preset_uses_the_presets_own_imu_rate():
    # Presets now carry an IMU/env rate (60 Hz), applied alongside the ranging config;
    # the estimate must reflect it. An explicit override (0 = coupled) still wins.
    est = estimate_preset(ProfileId.STABILITY)
    assert est.imu_env_rate_hz == 60
    assert est.imu_env_coupled is False
    coupled = estimate_preset(ProfileId.STABILITY, imu_env_rate_hz=0)
    assert coupled.imu_env_coupled is True


def test_estimate_manual_defaults_to_coupled_imu_env():
    est = estimate_manual(ManualParams(RangingMode.PRECISION, 30, 12, PowerMode.REGULAR))
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
# REFINEMENT (2026-08-03): I3C_XFER_MS moved from the raw 12.5 MHz SDR clock
# (9.49888 ms) to the documented EFFECTIVE 10 Mbps throughput (11.8736 ms) --
# see profiles.py module docstring "I3C ToF bus airtime", corroborated by AN6522
# Table 5 ("54x42 -> I3C (10 Mbps): 11.8 ms") and the decompiled ProfileTuning.exe
# planning tool's own readout formula. Expected percentages below are DERIVED from
# I3C_XFER_MS rather than hand-computed literals, so this table tracks the
# constant rather than needing hand-updating if it is ever refined again.

I3C_UTIL_FPS_TABLE = [30, 46, 60, 90, 100]


@pytest.mark.parametrize("fps", I3C_UTIL_FPS_TABLE)
def test_i3c_bus_utilization_matches_derived_percentage(fps):
    from roomscan.profiles import I3C_XFER_MS, i3c_bus_utilization_pct
    expected_pct = min(100.0, I3C_XFER_MS * fps / 10.0)
    assert i3c_bus_utilization_pct(fps) == pytest.approx(expected_pct, abs=0.05)


def test_i3c_xfer_ms_is_the_effective_10mbps_figure_not_the_raw_clock():
    # AN6522 Table 5: "54x42 -> I3C (10 Mbps): 11.8 ms" -- pins the corrected
    # coefficient so a future edit can't silently drift back to the raw-clock
    # figure (9.49888 ms) this replaced.
    from roomscan.profiles import I3C_XFER_MS
    assert I3C_XFER_MS == pytest.approx(11.8736, abs=1e-4)
    assert I3C_XFER_MS == pytest.approx(11.8, abs=0.1)  # AN6522's own rounded figure


@pytest.mark.parametrize("fps,expect_saturated", [(60, False), (90, True), (100, True)])
def test_i3c_bus_saturates_above_60fps_at_the_effective_rate(fps, expect_saturated):
    # At the effective 10 Mbps rate, one raw transfer alone (11.8736 ms) no longer
    # fits inside a 90/100 fps period (11.111/10.0 ms) at all -- the utilization
    # clamps to 100%, not merely "near" it as the old raw-clock coefficient implied.
    from roomscan.profiles import i3c_bus_utilization_pct
    assert (i3c_bus_utilization_pct(fps) == pytest.approx(100.0)) is expect_saturated


def test_i3c_airtime_left_complements_utilization():
    est = estimate_preset(ProfileId.HIGH_FRAMERATE)
    assert est.i3c_bus_utilization_pct + est.i3c_airtime_left_pct == pytest.approx(100.0)


# ---------------------------------------------------------------------------
# Measured hardware ceiling (2026-08-03): integer-multiple quantization below
# the sensor's per-frame floor, and the manual-request warning it produces.
# See profiles.py module docstring "Measured hardware ceiling" -- floor
# brackets are (exposure<=2ms -> 20.0ms, <=4ms -> 21.739ms, <=8ms -> 23.529ms),
# each the bracket's own conservative (under-promising) upper bound.
# ---------------------------------------------------------------------------

# FULL-CURVE remeasurement (2026-08-05, on-rig sweep): floor = 1000/ceiling at each
# MEASURED exposure, floor linearly interpolated between them. Supersedes the old
# 3-bracket table (<=8 ms) AND its derived extrapolation line (>8 ms).
FLOOR_MEASURED_TABLE = [
    (2, 1000.0 / 48),   # 20.833
    (4, 1000.0 / 46),   # 21.739
    (6, 1000.0 / 42),   # 23.810
    (8, 1000.0 / 40),   # 25.000
    (10, 1000.0 / 36),  # 27.778
    (12, 1000.0 / 34),  # 29.412
    (14, 1000.0 / 31),  # 32.258
    (15, 1000.0 / 30),  # 33.333
    (16, 1000.0 / 29),  # 34.483
]


@pytest.mark.parametrize("exposure_ms,expected_floor_ms", FLOOR_MEASURED_TABLE)
def test_measured_floor_ms_at_measured_points(exposure_ms, expected_floor_ms):
    assert measured_floor_ms(exposure_ms) == pytest.approx(expected_floor_ms, abs=1e-3)


def test_measured_floor_ms_interpolates_and_holds_below_first_point():
    # <=2 ms holds the fastest measured ceiling's floor.
    assert measured_floor_ms(1) == pytest.approx(1000.0 / 48, abs=1e-3)
    # 3 ms is the linear midpoint of the 2/4 ms floors (no direct 3 ms measurement).
    assert measured_floor_ms(3) == pytest.approx((1000.0 / 48 + 1000.0 / 46) / 2, abs=1e-3)
    # Monotonic and clearly above the retired derived-line values it replaced.
    assert measured_floor_ms(16) > measured_floor_ms(9) > measured_floor_ms(4)
    assert measured_floor_ms(16) != pytest.approx(32.4736)  # the retired derived 16 ms value


def test_measured_floor_ms_16ms_measured_value():
    # 16 ms tops out at 29 fps 1x -> 1000/29 = 34.483 ms (measured 2026-08-05); this
    # is why a 30 fps request at 16 ms cannot be delivered 1:1.
    assert measured_floor_ms(16) == pytest.approx(1000.0 / 29, abs=1e-3)


# (requested_fps, exposure_ms, expected_delivered_fps) -- pins the measured
# hardware points (90/100 fps @ 4 ms) and the boundary cases the amended
# preset and its neighbors sit on.
QUANTIZATION_TABLE = [
    (90, 4, 45.0),          # measured 44.85 fps on hardware, clean 2x
    (100, 4, 100.0 / 3.0),  # measured 33.2 fps on hardware, clean 3x
    (46, 4, 46.0),          # High Frame-Rate preset: exactly 1x, no quantization
    (48, 4, 24.0),          # just above the 4ms ceiling (46) -> 2x
    (48, 2, 48.0),          # exactly the 2ms ceiling (2026-08-05) -> 1x
    (60, 2, 30.0),          # above the 2ms ceiling (48) -> 2x
    (30, 15, 30.0),         # 15 ms holds 30 fps exactly (1x) -- the Precision preset
    (30, 16, 15.0),         # 16 ms cannot hold 30 fps -> quantizes down (see BUG-notes)
]


@pytest.mark.parametrize("requested_fps,exposure_ms,expected_fps", QUANTIZATION_TABLE)
def test_expected_delivered_fps_quantization(requested_fps, exposure_ms, expected_fps):
    assert expected_delivered_fps(requested_fps, exposure_ms) == pytest.approx(
        expected_fps, abs=0.05)


def test_expected_delivered_fps_never_exceeds_requested():
    # A period-multiple quantized rate is always requested_fps / N for integer
    # N >= 1 -- it can equal the request but never exceed it.
    for fps, exposure_ms in [(90, 4), (100, 4), (100, 16), (46, 4)]:
        assert expected_delivered_fps(fps, exposure_ms) <= fps + 1e-9


# (exposure_ms, ceiling_fps) -- the fps just at/below the ceiling must not warn;
# just above it must.
WARNING_BOUNDARY_TABLE = [
    (2, 48),   # measured 2026-08-05: ceiling 48 fps -- 48 fits, 49 doesn't
    (4, 46),   # 46 fits, 47 doesn't
    (8, 40),   # 40 fits, 41 doesn't
    (15, 30),  # 30 fits (Precision preset), 31 doesn't
]


@pytest.mark.parametrize("exposure_ms,at_ceiling_fps", WARNING_BOUNDARY_TABLE)
def test_manual_warning_boundary_per_exposure(exposure_ms, at_ceiling_fps):
    ranging = (RangingMode.PRECISION if at_ceiling_fps > DSS_FPS_CEILING
              else RangingMode.AMBIENT)
    at_params = ManualParams(ranging, at_ceiling_fps, exposure_ms, PowerMode.REGULAR)
    at_result = validate_manual_params(at_params)
    assert at_result.ok
    assert at_result.warnings == ()

    over_params = ManualParams(ranging, at_ceiling_fps + 1, exposure_ms, PowerMode.REGULAR)
    over_result = validate_manual_params(over_params)
    assert over_result.ok  # warning, not rejection
    assert over_result.warnings
    assert "1x delivery ceiling" in over_result.warnings[0]
    assert "ACCEPTS" in over_result.warnings[0]


def test_manual_warning_reports_expected_delivered_not_requested():
    params = ManualParams(RangingMode.PRECISION, 90, 4, PowerMode.REGULAR)
    result = validate_manual_params(params)
    assert result.ok
    assert any("~45.0" in w or "~45" in w for w in result.warnings)


def test_ceiling_fps_for_exposure_matches_measured_floor():
    assert ceiling_fps_for_exposure(4) == pytest.approx(46.0, abs=0.05)
    assert ceiling_fps_for_exposure(2) == pytest.approx(48.0, abs=0.05)


def test_ceiling_fps_int_uses_the_measured_table_verbatim():
    from roomscan.profiles import ceiling_fps_int
    # A float 1000/33.3333 = 29.9997 must NOT shave 15 ms to 29 -- the table says 30.
    assert ceiling_fps_int(15) == 30
    assert ceiling_fps_int(16) == 29
    assert ceiling_fps_int(12) == 34
    assert ceiling_fps_int(4) == 46


def test_estimate_manual_reports_expected_delivered_fps_field():
    # ProfileEstimate.expected_delivered_fps must report what the sensor will
    # actually deliver, not echo the request, once past the 1x ceiling.
    params = ManualParams(RangingMode.PRECISION, 90, 4, PowerMode.REGULAR)
    est = estimate_manual(params)
    assert est.ok
    assert est.fps == 90  # the request is preserved...
    assert est.expected_delivered_fps == pytest.approx(45.0, abs=0.05)  # ...but not echoed here
    assert est.expected_delivered_fps != est.fps


def test_estimate_preset_expected_delivered_fps_matches_request_when_under_ceiling():
    for profile_id in PRESETS:
        est = estimate_preset(profile_id)
        assert est.expected_delivered_fps == pytest.approx(est.fps, abs=0.05)


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
