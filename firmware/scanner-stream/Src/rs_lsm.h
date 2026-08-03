/* LSM6DSV16X (IKS4A1 HUB1) driver for the scanner-stream fork.
 *
 * Reads the LSM's SFLP game-rotation-vector (orientation quaternion) and, once
 * enabled, its I2C sensor-hub environmental slaves (baro/mag/temp) over the shared
 * native-I3C bus at dynamic address 0x50. See
 * docs/superpowers/specs/completed/2026-07-09-lsm6dsv16x-orientation-env-panel-design.md. */
#ifndef RS_LSM_H
#define RS_LSM_H

#include <stdint.h>

typedef struct {
    float quat[4];       /* [w, x, y, z] unit quaternion, LSM body frame */
    float pressure_pa;   /* LPS22DF, Pa */
    float mag_ut[3];     /* LIS2MDL, [x, y, z] µT */
    float temp_c;        /* STTS22H, °C */
    uint8_t have_quat;   /* 1 if quat was updated this call */
    uint8_t have_env;    /* 1 if env fields were updated this call */
    /* WHEN `quat` is, on the LSM's own clock. `quat` is the MEAN of `quat_n` SFLP samples
     * spread across this drain (RS_LSM_SFLP_AVERAGE, shipped for BUG-027's 2.8x noise cut),
     * so the orientation it carries is the batch's MIDPOINT -- not the ToF frame's t_us, and
     * not the drain. Measured on this rig 2026-07-30: the midpoint leads the frame-ready edge
     * by +7.8 ms (the drain is +24.3 ms past it), i.e. ~0.3 deg at 38.5 deg/s -- an order of
     * magnitude above BUG-031's residual, and in the opposite direction to "stale". Shipped on
     * stream 13 so a host can propagate the average to the frame instant instead of guessing
     * (and so nobody has to re-derive the sign from a capture). */
    uint32_t quat_mid_ticks;  /* midpoint of this drain's TIMESTAMP span; 0 if none seen */
    uint16_t quat_n;          /* SFLP samples averaged into `quat` */
} rs_lsm_sample_t;

/* One verbatim LSM6DSV16X FIFO word, laid out exactly as stream 11 (RS_STREAM_IMU_RAW)
 * puts it on the wire — see docs/protocol.md. `tag` is the FIFO_DATA_OUT_TAG *register*
 * byte, reconstructed as (tag_sensor << 3) | (tag_cnt << 1): the ST driver's
 * lsm6dsv16x_fifo_out_raw_get() decodes the register into a bitfield and drops bit0
 * (not_used0), so bit0 is always 0 here. `data` is the 6 payload bytes untouched
 * (little-endian, sensor encoding). sizeof == RS_IMU_RAW_REC_SIZE (8), no padding, so an
 * array of these is memcpy-free wire payload. */
typedef struct {
    uint8_t tag;
    uint8_t data[6];
    uint8_t reserved;   /* always 0 */
} rs_lsm_raw_word_t;

/* FIFO depth in words (LSM6DSV16X): the largest batch one drain can ever produce. */
#define RS_LSM_RAW_FIFO_MAX (256u)

/* Configure the LSM (SFLP + sensor-hub). Returns 0 on success, <0 on failure.
 * Must run after the ToF bring-up has assigned the LSM its dynamic address (0x50). */
int rs_lsm_init(void);

/* Drain the LSM FIFO, demux by tag, return the newest quaternion + env sample.
 * Never blocks. Returns 0 if any data was obtained, <0 if the FIFO yielded nothing
 * usable. Fields are only meaningful when the matching have_* flag is set. */
int rs_lsm_read_latest(rs_lsm_sample_t *out);

/* As rs_lsm_read_latest, but additionally copies out the raw FIFO words this drain saw
 * for the stream-11 tags (GY_NC 0x01, XL_NC 0x02, TIMESTAMP 0x04, SFLP gbias 0x16, SFLP
 * gravity 0x17) — the game-rotation word (0x13) stays on stream 9 and the sensor-hub
 * words stay on stream 10. `raw` may be NULL (then raw_max/raw_count are ignored and the
 * behaviour is byte-for-byte rs_lsm_read_latest). Words beyond raw_max are dropped, the
 * FIFO is still fully drained. Returns 0 if any quat/env/raw data was obtained. */
int rs_lsm_read_latest_raw(rs_lsm_sample_t *out, rs_lsm_raw_word_t *raw, uint16_t raw_max,
                           uint16_t *raw_count);

/* Read the LSM timestamp counter (TIMESTAMP0..3) into *ticks. Returns 0 on success, <0 on a
 * bus/read failure. One 4-byte register read — cheap enough to issue at the ToF's FRAME_READY
 * edge, which is the point: it puts that edge on the LSM's clock directly instead of inferring
 * it from whichever FIFO words a later drain happens to hold (BUG-031). Ticks are in the same
 * units as stream 11's TIMESTAMP words, i.e. scaled by stream 12's INTERNAL_FREQ_FINE. */
int rs_lsm_read_timestamp(uint32_t *ticks);

/* INTERNAL_FREQ_FINE (register 0x4F), latched once by rs_lsm_init(). Factory trim of the
 * internal oscillator that clocks the ODRs and the FIFO timestamp counter; the true tick
 * period is 1 / (46080 * (1 + 0.0013 * freq_fine)) seconds (AN5763 6.4), NOT the nominal
 * 21.7 us. `valid` is 0 until a successful read — the wire (stream 12) carries both. */
extern int8_t  g_lsm_freq_fine;
extern uint8_t g_lsm_freq_fine_valid;

/* Poll the LSM's embedded Wake-Up function (auto-idle motion wake, 2026-08-03 -- see the
 * RS_LSM_WAKE_* tuning block in rs_lsm.c). One register read (WAKE_UP_SRC, 0x45), safe to call
 * on any cadence including from the ToF idle loop while ranging is stopped: this hardware block
 * is independent of SFLP and of the ToF's own I3C frame cadence. Returns 1 if WU_IA (bit3) is
 * set (motion detected), 0 if not, <0 on a bus read failure. `*wake_up_src_out` (may be NULL)
 * gets the raw register byte, exactly what ships as RS_EVT_AUTO_WAKE_MOTION's detail. */
int rs_lsm_check_wake_up(uint8_t *wake_up_src_out);

#endif /* RS_LSM_H */
