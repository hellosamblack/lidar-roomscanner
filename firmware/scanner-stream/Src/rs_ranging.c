#include "rs_ranging.h"

#include <string.h>

#include "rs_protocol.h"
#include "vl53l9.h"
#include "vl53l9_platform.h"
#include "vl53l9_reg.h"

vl53l9_context_t rs_ranging_context_for_wire_mode(uint8_t wire_ranging_mode) {
    return (wire_ranging_mode == RS_RANGING_MODE_PRECISION) ? VL53L9_CONTEXT_SHORT : VL53L9_CONTEXT_LONG;
}

uint8_t rs_ranging_wire_mode_for_context(vl53l9_context_t context) {
    return (context == VL53L9_CONTEXT_SHORT) ? RS_RANGING_MODE_PRECISION : RS_RANGING_MODE_AMBIENT;
}

vl53l9_power_mode_t rs_ranging_vendor_power(uint8_t wire_power_mode) {
    switch (wire_power_mode) {
    case RS_POWER_MODE_ULP:
        return VL53L9_POWER_ULTRA_LOW;
    case RS_POWER_MODE_LP:
        return VL53L9_POWER_LOW;
    case RS_POWER_MODE_REGULAR:
    default:
        return VL53L9_POWER_REGULAR;
    }
}

uint8_t rs_ranging_wire_power(vl53l9_power_mode_t power) {
    switch (power) {
    case VL53L9_POWER_ULTRA_LOW:
        return RS_POWER_MODE_ULP;
    case VL53L9_POWER_LOW:
        return RS_POWER_MODE_LP;
    case VL53L9_POWER_REGULAR:
    default:
        return RS_POWER_MODE_REGULAR;
    }
}

uint32_t rs_ranging_fps_from_period(uint32_t frame_period_us) {
    if (frame_period_us == 0u) {
        return 0u;
    }
    /* round(1e6 / period_us), the inverse of the host's round(1e6/fps) --
     * integer round-to-nearest via a half-period bias. */
    return (1000000u + (frame_period_us / 2u)) / frame_period_us;
}

uint8_t rs_ranging_dss_enabled_for_period(uint32_t frame_period_us) {
    return (rs_ranging_fps_from_period(frame_period_us) <= RS_RANGING_DSS_FPS_CEILING) ? 1u : 0u;
}

uint32_t rs_ranging_frame_timeout_ms(uint32_t frame_period_us) {
    uint32_t period_ms = frame_period_us / 1000u;
    uint32_t margin_ms = period_ms * RS_RANGING_TIMEOUT_MARGIN_MULT;
    return (margin_ms > RS_RANGING_TIMEOUT_FLOOR_MS) ? margin_ms : RS_RANGING_TIMEOUT_FLOOR_MS;
}

/* Preset table -- exactly host/src/roomscan/profiles.py's PRESETS dict (Table 2.1
 * of the amended spec, reconciled Task 1), in vendor units instead of wire units
 * since this is what actually gets written to hardware. Binning fixed at 2
 * (54x42) for every preset -- plan's global constraint. `.sync` is
 * VL53L9_SYNC_AUTONOMOUS (Task 5): the production/raw-only acquisition loop no
 * longer calls vl53l9_trigger_frame() at all -- the sensor free-runs FRAME_READY at
 * its own configured frame_period_us once started, which is the only way to
 * actually realize a target FPS (non-negotiable finding #2: frame_period_us is
 * inert under VL53L9_SYNC_MANUAL). This matches the vendor's own g_ranging_profiles[]
 * default (vl53l9_utils.c) that this scanner-owned table used to diverge from.
 * `.id` is the vendor vl53l9_profile_t's own (unused anywhere in the vendor tree)
 * usecase-id field -- left 0, since our host-facing identity is `profile_id`
 * below, not this vendor field.
 *
 * RS_PROFILE_HIGH_FRAMERATE amended 2026-08-03 (measured hardware ceiling, Task 5):
 * an on-target sweep found the sensor has an intrinsic per-frame floor DS14879 does
 * not document -- a requested period shorter than it is accepted, not rejected, and
 * silently delivered as an integer MULTIPLE of the request (90 Hz measured 44.85 fps,
 * a clean 2x; 100 Hz measured 33.2 fps, a clean 3x). The preset moves 90 -> 46 fps,
 * the measured 1x delivery ceiling at this preset's own 4 ms exposure, with DSS now
 * ON (46 <= the 60 fps DSS ceiling -- see rs_ranging_dss_enabled_for_period()).
 * Mirrors host/src/roomscan/profiles.py's PRESETS[ProfileId.HIGH_FRAMERATE] exactly;
 * see that module's docstring "Measured hardware ceiling" and
 * docs/superpowers/specs/2026-07-31-high-framerate-and-manual-ranging-modes.md
 * Sec 2.1/3.2.1/8.1 for the full investigation. */
static const rs_ranging_profile_t rs_ranging_presets[3] = {
    { /* RS_PROFILE_ROOM_MAPPING: Ambient, DSS on, 30 fps, 6 ms, ULP */
        .vendor = { .id = 0u, .sync = VL53L9_SYNC_AUTONOMOUS, .power = VL53L9_POWER_ULTRA_LOW,
                    .context = VL53L9_CONTEXT_LONG, .frame_period_us = 33333u, .binning = 2u,
                    .exposure_ms = 6u },
        .profile_id = RS_PROFILE_ROOM_MAPPING,
        .dss_enabled = 1u,
    },
    { /* RS_PROFILE_PRECISION: Precision, DSS on, 30 fps, 10 ms, ULP */
        .vendor = { .id = 0u, .sync = VL53L9_SYNC_AUTONOMOUS, .power = VL53L9_POWER_ULTRA_LOW,
                    .context = VL53L9_CONTEXT_SHORT, .frame_period_us = 33333u, .binning = 2u,
                    .exposure_ms = 10u },
        .profile_id = RS_PROFILE_PRECISION,
        .dss_enabled = 1u,
    },
    { /* RS_PROFILE_HIGH_FRAMERATE: Precision, DSS on (46fps <= 60fps ceiling), 46 fps,
       * 4 ms, Regular -- amended 2026-08-03, measured hardware ceiling (was 90 fps /
       * DSS off / 11111 us, which the sensor never actually delivered 1:1). */
        .vendor = { .id = 0u, .sync = VL53L9_SYNC_AUTONOMOUS, .power = VL53L9_POWER_REGULAR,
                    .context = VL53L9_CONTEXT_SHORT, .frame_period_us = 21739u, .binning = 2u,
                    .exposure_ms = 4u },
        .profile_id = RS_PROFILE_HIGH_FRAMERATE,
        .dss_enabled = 1u,
    },
};

int rs_ranging_preset(uint32_t profile_id, rs_ranging_profile_t *out) {
    if (profile_id > RS_PROFILE_HIGH_FRAMERATE) {
        return -1;
    }
    *out = rs_ranging_presets[profile_id];
    return 0;
}

void rs_ranging_boot_default(rs_ranging_profile_t *out) {
    *out = rs_ranging_presets[RS_PROFILE_ROOM_MAPPING];
}

uint32_t rs_ranging_validate_manual(const rs_ranging_manual_params_t *params) {
    if (params->ranging_mode != RS_RANGING_MODE_AMBIENT && params->ranging_mode != RS_RANGING_MODE_PRECISION) {
        return RS_RESULT_BAD_PARAM;
    }
    if (params->power_mode > RS_POWER_MODE_REGULAR) {
        return RS_RESULT_BAD_PARAM;
    }
    if (params->frame_period_us < RS_RANGING_FRAME_PERIOD_MIN_US ||
        params->frame_period_us > RS_RANGING_FRAME_PERIOD_MAX_US) {
        return RS_RESULT_BAD_PARAM;
    }
    if (params->exposure_ms < RS_RANGING_EXPOSURE_MS_MIN || params->exposure_ms > RS_RANGING_EXPOSURE_MS_MAX) {
        return RS_RESULT_BAD_PARAM;
    }

    /* Global constraint: >60fps forces DSS off, which is valid only with
     * Precision ranging mode. */
    uint32_t fps = rs_ranging_fps_from_period(params->frame_period_us);
    if (fps > RS_RANGING_DSS_FPS_CEILING && params->ranging_mode != RS_RANGING_MODE_PRECISION) {
        return RS_RESULT_BAD_PARAM;
    }

    /* PENDING hardware measurement (Task 5) -- see RS_RANGING_BLANKING_MARGIN_US.
     * Rejects only exposure that plainly cannot fit inside the requested period. */
    uint32_t exposure_us = (uint32_t)params->exposure_ms * 1000u;
    if (exposure_us + RS_RANGING_BLANKING_MARGIN_US > params->frame_period_us) {
        return RS_RESULT_BAD_PARAM;
    }

    return RS_RESULT_OK;
}

void rs_ranging_manual_candidate(const rs_ranging_manual_params_t *params, rs_ranging_profile_t *out) {
    out->vendor.id = 0u;
    out->vendor.sync = VL53L9_SYNC_AUTONOMOUS; /* Task 5: see the preset table's comment above */
    out->vendor.power = rs_ranging_vendor_power(params->power_mode);
    out->vendor.context = rs_ranging_context_for_wire_mode(params->ranging_mode);
    out->vendor.frame_period_us = params->frame_period_us;
    out->vendor.binning = 2u;
    out->vendor.exposure_ms = params->exposure_ms;
    out->profile_id = RS_PROFILE_MANUAL;
    out->dss_enabled = rs_ranging_dss_enabled_for_period(params->frame_period_us);
}

int rs_ranging_apply_dss(vl53l9_device_t *p_dev, vl53l9_context_t context, uint8_t enable) {
    /* The vendor's own DSS-mode enum (_dss_mode_t: DSS_DISABLE=0, DSS_LONG=1,
     * DSS_SHORT=2) is PRIVATE to vl53l9.c (`static`, declared inline, no header)
     * -- vl53l9_set_binning() (vl53l9.c:455-506) is the only place those values
     * are visible, and it writes them precisely like this. Hardcoded here rather
     * than re-declared, so there is exactly one place (read-only vendor code we
     * never edit) that could ever disagree with these numbers -- a local
     * re-declaration would not have protected against that anyway. */
    uint8_t val;
    if (!enable) {
        val = 0u; /* DSS_DISABLE */
    } else {
        val = (context == VL53L9_CONTEXT_SHORT) ? 2u /* DSS_SHORT */ : 1u /* DSS_LONG */;
    }
    return vl53l9_write8(p_dev, VL53L9_REGADDR_STANDBY_DSS_MODE((uint16_t)context), val);
}

int rs_ranging_write_profile(vl53l9_device_t *p_dev, const rs_ranging_profile_t *profile) {
    vl53l9_profile_t vendor = profile->vendor;
    int ret = vl53l9_utils_set_profile(p_dev, &vendor);
    if (ret) {
        return ret;
    }
    /* MUST run after vl53l9_utils_set_profile(): it calls vl53l9_set_binning(),
     * which unconditionally rewrites this same register to DSS-enabled
     * (vl53l9.c:499) -- plan step 2. */
    return rs_ranging_apply_dss(p_dev, vendor.context, profile->dss_enabled);
}

int rs_ranging_read_config(vl53l9_device_t *p_dev, rs_ranging_readback_t *out) {
    int ret;
    memset(out, 0, sizeof(*out));

    ret = vl53l9_get_sync_mode(p_dev, &out->sync_mode);
    if (ret) {
        return ret;
    }

    vl53l9_context_t context;
    ret = vl53l9_get_context(p_dev, &context);
    if (ret) {
        return ret;
    }
    out->ranging_mode = rs_ranging_wire_mode_for_context(context);

    vl53l9_power_mode_t power;
    ret = vl53l9_get_power_mode(p_dev, &power);
    if (ret) {
        return ret;
    }
    out->power_mode = rs_ranging_wire_power(power);

    ret = vl53l9_get_frame_period(p_dev, &out->frame_period_us);
    if (ret) {
        return ret;
    }

    ret = vl53l9_get_binning(p_dev, context, &out->binning);
    if (ret) {
        return ret;
    }

    ret = vl53l9_get_exposure(p_dev, context, &out->exposure_ms);
    if (ret) {
        return ret;
    }

    uint8_t dss_raw = 0u;
    ret = vl53l9_read8(p_dev, VL53L9_REGADDR_STANDBY_DSS_MODE((uint16_t)context), &dss_raw);
    if (ret) {
        return ret;
    }
    out->dss_enabled = (dss_raw != 0u) ? 1u : 0u;

    return 0;
}
