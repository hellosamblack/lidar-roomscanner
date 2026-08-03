/* Scanner-owned ranging-profile layer: presets, an app-local DSS setter, and
 * live readback. Task 4 of
 * docs/superpowers/plans/2026-07-31-high-framerate-and-manual-ranging-modes.md.
 * See docs/protocol.md (commands 8-12, the ranging-mode/power-mode/profile
 * registries) and
 * docs/superpowers/specs/2026-07-31-high-framerate-and-manual-ranging-modes.md
 * (Table 2.1 -- the preset numbers below are exactly
 * host/src/roomscan/profiles.py's PRESETS dict, the canonical model this
 * mirrors, in vendor units instead of wire units).
 *
 * Deliberately separate from firmware/vendor/53L9A1/ (READ-ONLY reference, per
 * CLAUDE.md): this adds a local DSS setter and a profile layer on top of the
 * vendor's own vl53l9_utils_set_profile()/g_ranging_profiles[] without editing
 * either. HAL-free -- only vl53l9's own public register-I/O API
 * (vl53l9_read8/write8, vl53l9_utils_set_profile(), the vl53l9_get_* readback
 * functions) and rs_protocol.h's RS_* wire constants.
 */
#ifndef RS_RANGING_H
#define RS_RANGING_H

#include <stdint.h>

#include "vl53l9_interface.h"
#include "vl53l9_utils.h"

/* Validation constants. Mirror host/src/roomscan/profiles.py where a host-side
 * anchor exists; RS_RANGING_EXPOSURE_MS_MAX deliberately does NOT (see its own
 * comment). */
#define RS_RANGING_DSS_FPS_CEILING     (60u)   /* DSS on <=60fps, forced off above */
#define RS_RANGING_BLANKING_MARGIN_US  (500u)  /* PENDING hardware measurement (Task 5);
                                                 * matches host's
                                                 * BLANKING_MARGIN_US_PENDING_HW */
#define RS_RANGING_EXPOSURE_MS_MIN     (1u)
/* The driver's real ceiling (vl53l9_set_exposure(), vl53l9.c:550) -- matches this
 * file's existing (pre-Task-4) RS_CMD_SET_EXPOSURE_MS check, one line down in
 * vl53l9_app.c. The host UI/profiles.py caps Manual requests lower (16 ms) as a
 * product choice; firmware validates what the sensor can actually schedule, not
 * the UI's narrower contract -- a compliant host never sends above 16 ms anyway,
 * so this is strictly less restrictive, never a hardware-vs-host disagreement in
 * the direction that would reject a valid host request. */
#define RS_RANGING_EXPOSURE_MS_MAX     (30u)
#define RS_RANGING_FRAME_PERIOD_MIN_US (10000u)   /* vl53l9_set_frame_period()'s own bound */
#define RS_RANGING_FRAME_PERIOD_MAX_US (1000000u)

/* Per-frame wait-timeout sizing (Task 5, plan step 5): a bounded margin over the
 * applied frame period, not the old fixed 1000 ms wait. A fixed 1000 ms window at
 * 90/100 fps would silently absorb ~90-100 missed frames before a genuine fault (a
 * wedged sensor, a dropped I3C transaction) is ever detected; at 1 fps (manual's
 * floor) it would fire spuriously on perfectly healthy hardware. MARGIN_MULT/FLOOR_MS
 * are empirical (Task 5 on-target tuning against real FRAME_READY/DMA jitter, not a
 * datasheet number) -- see rs_ranging_frame_timeout_ms()'s call sites in
 * vl53l9_app.c for where GPIO_IT/DMA_RX event waits use this. */
#define RS_RANGING_TIMEOUT_MARGIN_MULT (4u)
#define RS_RANGING_TIMEOUT_FLOOR_MS    (60u)

/* Decoded SET_MANUAL_PARAMS (cmd 9) fields, wire units throughout (ranging_mode:
 * RS_RANGING_MODE_*, power_mode: RS_POWER_MODE_*, exposure_ms: integer ms,
 * frame_period_us: round(1e6/fps)) -- mirrors rs_parsed_command_t's `u.manual`
 * member (rs_protocol.h) so callers can build one straight from it. */
typedef struct {
    uint8_t  ranging_mode;
    uint32_t frame_period_us;
    uint16_t exposure_ms;
    uint8_t  power_mode;
} rs_ranging_manual_params_t;

/* One fully-resolved ranging configuration: the vendor fields
 * vl53l9_utils_set_profile() consumes, plus the two scanner-owned fields the
 * vendor type has no room for -- profile_id (the host-facing SET_RANGING_PROFILE
 * enum / ACK applied value, RS_PROFILE_*) and dss_enabled (vl53l9_set_binning(),
 * called from vl53l9_utils_set_profile(), ALWAYS turns DSS on; this scanner-owned
 * flag is what rs_ranging_apply_dss() applies immediately afterward to actually
 * turn it off above 60 fps -- non-negotiable finding #3 of the plan). */
typedef struct {
    vl53l9_profile_t vendor;
    uint32_t profile_id;
    uint8_t  dss_enabled;
} rs_ranging_profile_t;

/* Live register readback of the config currently in force -- sync mode, ranging
 * mode (derived from context), power mode, frame period, exposure, binning, and
 * DSS (plan step 3). None of vl53l9's own getters gate on FSM state (only the
 * setters do -- see vl53l9.c: vl53l9_get_context/_power_mode/_sync_mode/
 * _frame_period/_binning/_exposure never check `_get_fsm_state`), so this is
 * safe to call whether the sensor is standing by or streaming, as long as the
 * I3C bus is otherwise idle (the command-poll safe point every caller uses
 * already guarantees that). Wire-unit fields (ranging_mode/power_mode) are what
 * docs/protocol.md's cmd 9/10 ACK carries; sync_mode/binning/dss_enabled have no
 * ACK field (the wire's 16-byte ranging-config shape is cmd+result+ranging_mode+
 * frame_period_us+exposure_ms+power_mode only) -- kept here for firmware-internal
 * validation/diagnostics, per the plan's own read-back list. */
typedef struct {
    vl53l9_sync_mode_t sync_mode;
    uint8_t  ranging_mode;   /* RS_RANGING_MODE_* */
    uint32_t frame_period_us;
    uint16_t exposure_ms;
    uint8_t  power_mode;     /* RS_POWER_MODE_* */
    uint8_t  binning;
    uint8_t  dss_enabled;
} rs_ranging_readback_t;

/* --- enum conversions: wire <-> vendor -------------------------------------
 *
 * Neither vl53l9_context_t nor vl53l9_power_mode_t is numbered the same as the
 * wire's RS_RANGING_MODE_* / RS_POWER_MODE_* registries (docs/protocol.md):
 *   - vl53l9_context_t is INVERTED: wire AMBIENT=0 -> VL53L9_CONTEXT_LONG=1,
 *     wire PRECISION=1 -> VL53L9_CONTEXT_SHORT=0 (vl53l9.h:103).
 *   - vl53l9_power_mode_t is REVERSED end-to-end: wire ULP=0/LP=1/REGULAR=2 vs
 *     vendor VL53L9_POWER_REGULAR=0/LOW=1/ULTRA_LOW=2 (vl53l9.h:85).
 * Get either wrong and a profile silently applies the OPPOSITE context or power
 * mode from the one requested -- never compare/assign across the two without
 * going through these. */
vl53l9_context_t rs_ranging_context_for_wire_mode(uint8_t wire_ranging_mode);
uint8_t rs_ranging_wire_mode_for_context(vl53l9_context_t context);
vl53l9_power_mode_t rs_ranging_vendor_power(uint8_t wire_power_mode);
uint8_t rs_ranging_wire_power(vl53l9_power_mode_t power);

/* fps derived from frame_period_us by rounding (round(1e6/period_us), the
 * inverse of the host's fps_to_period_us()) -- used only to decide the
 * DSS/>60Hz rule below; never sent on the wire itself. */
uint32_t rs_ranging_fps_from_period(uint32_t frame_period_us);

/* Bounded per-frame event-wait timeout (Task 5, plan step 5): max(FLOOR_MS,
 * MARGIN_MULT * period_ms). Used for both the FRAME_READY (PLATFORM_GPIO_IT_EVT) and
 * DMA-complete (PLATFORM_I3C_DMA_RX_EVT) waits in the autonomous acquisition loop --
 * see vl53l9_app.c. A frame_period_us of 0 (should never occur; g_active_profile is
 * always seeded before use) resolves to the floor, not a divide-by-zero. */
uint32_t rs_ranging_frame_timeout_ms(uint32_t frame_period_us);

/* Global constraint: DSS enabled at <=60 fps, forced off above it. */
uint8_t rs_ranging_dss_enabled_for_period(uint32_t frame_period_us);

/* Preset table (RS_PROFILE_ROOM_MAPPING/PRECISION/HIGH_FRAMERATE). Returns 0 and
 * fills *out for a valid preset id, -1 (out untouched) for RS_PROFILE_MANUAL or
 * an out-of-range id -- MANUAL has no fixed preset; callers reapply the last
 * accepted SET_MANUAL_PARAMS candidate instead. */
int rs_ranging_preset(uint32_t profile_id, rs_ranging_profile_t *out);

/* Room Mapping -- the boot default (plan step 6). */
void rs_ranging_boot_default(rs_ranging_profile_t *out);

/* Full validation of a decoded SET_MANUAL_PARAMS candidate: field ranges,
 * frame-period bounds, the >60fps-precision-only DSS rule, and the blanking-
 * margin placeholder -- the same checks host/src/roomscan/profiles.py's
 * validate_manual_params() runs, reproduced host-independently so a malformed
 * or adversarial COMMAND is rejected here even if no compliant host would ever
 * send it. Never touches hardware. Returns RS_RESULT_OK or RS_RESULT_BAD_PARAM
 * (rs_protocol.h). */
uint32_t rs_ranging_validate_manual(const rs_ranging_manual_params_t *params);

/* Resolves a validated manual candidate into a full rs_ranging_profile_t
 * (profile_id = RS_PROFILE_MANUAL, dss_enabled derived per the rule above).
 * Does not validate -- call rs_ranging_validate_manual() first. */
void rs_ranging_manual_candidate(const rs_ranging_manual_params_t *params, rs_ranging_profile_t *out);

/* App-local DSS setter (plan step 2): vl53l9_write8() through the vendor's own
 * public register-I/O, at the exact register vl53l9_set_binning() (called from
 * vl53l9_utils_set_profile()) always rewrites -- so this MUST run AFTER that
 * call to have any effect (rs_ranging_write_profile() below does so). Device
 * must be in standby, the same requirement vl53l9_set_binning() itself has --
 * this function does not check it; callers apply it from the same
 * already-stopped context vl53l9_utils_set_profile() needs. */
int rs_ranging_apply_dss(vl53l9_device_t *p_dev, vl53l9_context_t context, uint8_t enable);

/* Writes profile->vendor via vl53l9_utils_set_profile() then applies
 * profile->dss_enabled via rs_ranging_apply_dss() -- the whole scanner-owned
 * apply sequence for one profile, in the one order that works (plan step 2).
 * Device must already be in standby (caller has already called vl53l9_stop());
 * this function does not stop/start/trigger, and does not touch g_active_profile
 * or attempt any restore on failure -- that orchestration (validate-before-stop,
 * restore-the-whole-previous-profile-on-failure, re-trigger) stays in
 * vl53l9_app.c alongside the existing REINIT/recovery machinery it must compose
 * with (rs_sensor_reinit/rs_recover/handle_error). This file must NEVER call
 * handle_error() (recursion-guard discipline -- see vl53l9_app.c's block comment
 * above rs_recover()). Returns 0 on success, the first nonzero vl53l9 error
 * code otherwise. */
int rs_ranging_write_profile(vl53l9_device_t *p_dev, const rs_ranging_profile_t *profile);

/* Live readback (plan step 3): sync mode, ranging mode/context, power, frame
 * period, exposure, binning, DSS. Safe any time the I3C bus is idle (see the
 * struct's own comment -- no FSM-state gate on any getter this uses). Returns 0
 * and a fully-populated *out on success, or the first nonzero vl53l9 error with
 * *out zeroed (memset at entry, so a failure never leaks a stale/partial value). */
int rs_ranging_read_config(vl53l9_device_t *p_dev, rs_ranging_readback_t *out);

#endif /* RS_RANGING_H */
