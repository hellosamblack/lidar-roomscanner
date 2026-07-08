/**
 * rs_transform_shim.h
 *
 * PC-side shim exposing the vl53l9-transform-c pipeline as a small, stable
 * C ABI so it can be loaded from Python via ctypes. Mirrors the transform
 * setup sequence in firmware/scanner-stream/Src/vl53l9_app.c (capabilities,
 * calib-buffer control, prepare) but replaces sensor-driven double-buffered
 * DMA with a single caller-supplied raw buffer per call.
 *
 * Not part of the read-only 53L9A1 reference package: this file (and its
 * .c counterpart) live in host/transform/ and only #include vendor headers
 * from ../../../53L9A1 (PKG_ROOT) -- the vendor tree itself is never
 * copied or edited.
 */
#ifndef RS_TRANSFORM_SHIM_H_
#define RS_TRANSFORM_SHIM_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32)
#define RST_API __declspec(dllexport)
#else
#define RST_API
#endif

/**
 * @brief Create and prepare a transform pipeline instance.
 *
 * Replicates vl53l9_app.c's setup: transform_initialize -> set "raw"/3DMD
 * input capabilities (width x height, e.g. 14842 x 1 at binning 2) -> set
 * "depth"/ZF32 output capabilities (54 x 42) -> set the mandatory
 * "calib-buffer" control -> transform_prepare.
 *
 * @param[in] calib      Pointer to the VL53L9_CALIB_DATA_SIZE (2332-byte)
 *                        calibration blob. Copied internally; the caller's
 *                        buffer need not outlive this call.
 * @param[in] calib_len  Length of @p calib in bytes. Must equal 2332.
 * @param[in] in_width   Raw input stream width in pixels (binning 2 -> 14842).
 * @param[in] in_height  Raw input stream height in pixels (binning 2 -> 1).
 *
 * @return Opaque handle on success, or NULL on failure (bad calib_len,
 *         allocation failure, or any transform setup step returning non-zero).
 */
RST_API void *rst_create(const uint8_t *calib, uint32_t calib_len, uint32_t in_width, uint32_t in_height);

/**
 * @brief Process one raw frame into a depth (ZF32) frame.
 *
 * @param[in]  h          Handle from rst_create().
 * @param[in]  raw        Raw 3DMD input buffer (raw_len bytes).
 * @param[in]  raw_len    Length of @p raw in bytes; must match the input
 *                         capabilities set at rst_create() time.
 * @param[out] depth_out  Caller-allocated output buffer of 54*42 floats
 *                         (output resolution is fixed by the firmware's
 *                         binning-2 profile: width=54, height=42).
 *
 * @return 0 (MEDIA_ERROR_NONE) on success, non-zero transform error code
 *         otherwise.
 */
RST_API int rst_process(void *h, const uint8_t *raw, uint32_t raw_len, float *depth_out);

/**
 * @brief Tear down a transform instance created by rst_create().
 *
 * Attempts the full teardown the firmware itself never exercises
 * (transform_finalize -> transform_release -> vl53l9_transform_destroy).
 * Safe to call with h == NULL (no-op).
 */
RST_API void rst_destroy(void *h);

#ifdef __cplusplus
}
#endif

#endif /* RS_TRANSFORM_SHIM_H_ */
