/* LSM6DSV16X (IKS4A1 HUB1) driver for the scanner-stream fork.
 *
 * Reads the LSM's SFLP game-rotation-vector (orientation quaternion) and, once
 * enabled, its I2C sensor-hub environmental slaves (baro/mag/temp) over the shared
 * native-I3C bus at dynamic address 0x50. See
 * docs/superpowers/specs/2026-07-09-lsm6dsv16x-orientation-env-panel-design.md. */
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
} rs_lsm_sample_t;

/* Configure the LSM (SFLP + sensor-hub). Returns 0 on success, <0 on failure.
 * Must run after the ToF bring-up has assigned the LSM its dynamic address (0x50). */
int rs_lsm_init(void);

/* Drain the LSM FIFO, demux by tag, return the newest quaternion + env sample.
 * Never blocks. Returns 0 if any data was obtained, <0 if the FIFO yielded nothing
 * usable. Fields are only meaningful when the matching have_* flag is set. */
int rs_lsm_read_latest(rs_lsm_sample_t *out);

#endif /* RS_LSM_H */
