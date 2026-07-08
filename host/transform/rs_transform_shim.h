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
 *
 * --- v2: multi-output ---------------------------------------------------
 *
 * The transform library (vl53l9_transform.c, docs/transform-streams.md)
 * exposes four *independent* named output streams that can all be
 * negotiated and produced by ONE prepared instance in a single
 * transform_process_stream() call, because
 * _check_streams_consistency()/_do_process_stream() gate each output on
 * "was a stream_buffer supplied for this name", not on some mutual
 * exclusivity: "depth" (ZF32), "reflectance" (RF32), "confidence" (CF32),
 * "ambient" (IF32). All four are float32 planes at the same resolution as
 * depth (54x42 for the binning-2 profile this shim targets), so they share
 * one calib-buffer control and one raw input.
 *
 * ZAPC is different: it is a *format* of the "depth" stream itself, not a
 * separate stream ("Android depth point cloud": 4x float32 per zone,
 * [x, y, z, confidence]). _do_set_stream_capabilities() only accepts one
 * capability per stream name, and _do_process_stream() branches on a
 * single is_pointcloud_requested flag when filling the "depth" output
 * buffer -- so one instance's "depth" stream is ZF32 XOR ZAPC, never both,
 * for the same transform_process_stream() call. Getting ZF32 *and* ZAPC
 * out of the same raw frame therefore requires a SECOND transform
 * instance, prepared with "depth" capability = ZAPC and no other outputs.
 * This shim creates that second instance lazily, only when the caller's
 * out_mask includes RST_OUT_ZAPC.
 *
 * Cost of the second instance: it re-runs the full per-frame pipeline
 * (extract -> distance_calibration -> TNR -> confidence -> ratenorm ->
 * reflectance -> radial_to_perp -> dmax -> sharpener -> flying_pixel ->
 * distance_check -> pointcloud) on the same raw bytes, plus its own set of
 * calibration maps (5 x resolution floats) and, if TNR is not bypassed, its
 * own TNR history buffer -- roughly 2x the compute and a second (small,
 * 54x42-scale) working set. Each instance's TNR state is fed the same raw
 * stream independently, so each instance's own output stays internally
 * self-consistent (its own ZF32-vs-ZF32 or ZAPC-vs-ZAPC frames are
 * temporally coherent) but the two instances' frame-N outputs are NOT
 * derived from shared TNR state -- they're two independent stateful
 * filters converging from the same raw inputs. That only matters if a
 * caller cross-compares DEPTH and ZAPC on the same frame (Task 3's
 * validation use case); the live viewer only ever asks for one of the two
 * per instance, so this is a non-issue there.
 *
 * Output plane formats/dtypes (all at RST_OUT_WIDTH x RST_OUT_HEIGHT
 * unless noted):
 *   - depth        ZF32   float32, 1 value/zone   (RST_OUT_COUNT floats)
 *   - reflectance  RF32   float32, 1 value/zone   (RST_OUT_COUNT floats)
 *   - confidence   CF32   float32, 1 value/zone   (RST_OUT_COUNT floats)
 *   - ambient      IF32   float32, 1 value/zone   (RST_OUT_COUNT floats)
 *   - zapc         ZAPC   float32, 4 values/zone: [x, y, z, confidence]
 *                         (RST_OUT_COUNT * 4 floats) -- second instance
 *
 * status (CU32) and amplitude (AF32) are exposed by the library but not
 * wired into this shim's mask -- not part of this task's --color deliverable
 * (reflectance/confidence); add analogously if a future task needs them.
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

/** @name rst_create2() out_mask bits (bitwise-OR any combination). */
/**@{*/
#define RST_OUT_DEPTH       (1u << 0) /**< "depth" stream, format ZF32 (primary instance). */
#define RST_OUT_REFLECTANCE (1u << 1) /**< "reflectance" stream, format RF32 (primary instance). */
#define RST_OUT_CONFIDENCE  (1u << 2) /**< "confidence" stream, format CF32 (primary instance). */
#define RST_OUT_AMBIENT     (1u << 3) /**< "ambient" stream, format IF32 (primary instance). */
#define RST_OUT_ZAPC        (1u << 4) /**< "depth" stream, format ZAPC (separate second instance -- see header comment above). */
#define RST_OUT_MASK_ALL    (RST_OUT_DEPTH | RST_OUT_REFLECTANCE | RST_OUT_CONFIDENCE | RST_OUT_AMBIENT | RST_OUT_ZAPC)
/**@}*/

/**
 * @brief Create and prepare a transform pipeline instance.
 *
 * Replicates vl53l9_app.c's setup: transform_initialize -> set "raw"/3DMD
 * input capabilities (width x height, e.g. 14842 x 1 at binning 2) -> set
 * "depth"/ZF32 output capabilities (54 x 42) -> set the mandatory
 * "calib-buffer" control -> transform_prepare.
 *
 * Depth-only convenience wrapper over rst_create2() with
 * out_mask = RST_OUT_DEPTH; kept so the byte-identity equivalence test
 * (host/tests/test_equivalence.py) exercises the same code path unmodified.
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
 * Depth-only convenience wrapper over rst_process2() (all other outputs NULL).
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
 * @brief Tear down a transform instance created by rst_create() or rst_create2().
 *
 * Attempts the full teardown the firmware itself never exercises
 * (transform_finalize -> transform_release -> vl53l9_transform_destroy),
 * for every underlying transform instance (primary and, if present, the
 * ZAPC secondary instance -- see the v2 header comment above).
 * Safe to call with h == NULL (no-op).
 */
RST_API void rst_destroy(void *h);

/**
 * @brief Create and prepare a transform pipeline instance emitting any
 *        combination of depth/reflectance/confidence/ambient/ZAPC.
 *
 * @param[in] calib      Same as rst_create().
 * @param[in] calib_len  Same as rst_create().
 * @param[in] in_width   Same as rst_create().
 * @param[in] in_height  Same as rst_create().
 * @param[in] out_mask   Bitwise-OR of RST_OUT_* bits selecting which output
 *                        planes rst_process2() will fill. Must be nonzero.
 *                        RST_OUT_DEPTH/REFLECTANCE/CONFIDENCE/AMBIENT are
 *                        negotiated on one shared "primary" instance (any
 *                        subset, all in a single transform_process_stream()
 *                        call per frame); RST_OUT_ZAPC is negotiated on a
 *                        second, independently-stateful instance (see the
 *                        ZAPC cost note at the top of this file).
 *
 * @return Opaque handle on success, or NULL on failure (bad calib_len, zero
 *         out_mask, allocation failure, or any transform setup step
 *         returning non-zero).
 */
RST_API void *rst_create2(const uint8_t *calib, uint32_t calib_len, uint32_t in_width, uint32_t in_height,
                          uint32_t out_mask);

/**
 * @brief Process one raw frame into the output planes selected at
 *        rst_create2() time.
 *
 * Each output pointer must be non-NULL if and only if the corresponding
 * RST_OUT_* bit was set in @p out_mask at create time; passing NULL for a
 * selected output (or non-NULL for an unselected one) is a usage error.
 *
 * @param[in]  h               Handle from rst_create2().
 * @param[in]  raw             Raw 3DMD input buffer (raw_len bytes); fed to
 *                               every underlying instance (primary and, if
 *                               present, ZAPC).
 * @param[in]  raw_len         Length of @p raw in bytes.
 * @param[out] depth_out       RST_OUT_COUNT floats (ZF32), or NULL.
 * @param[out] reflectance_out RST_OUT_COUNT floats (RF32), or NULL.
 * @param[out] confidence_out  RST_OUT_COUNT floats (CF32), or NULL.
 * @param[out] ambient_out     RST_OUT_COUNT floats (IF32), or NULL.
 * @param[out] zapc_out        RST_OUT_COUNT * 4 floats ([x,y,z,confidence]
 *                               per zone, ZAPC), or NULL.
 *
 * @return 0 (MEDIA_ERROR_NONE) on success, non-zero transform error code
 *         (or MEDIA_ERROR_INVALID_PARAMETER for a mask/pointer mismatch)
 *         otherwise.
 */
RST_API int rst_process2(void *h, const uint8_t *raw, uint32_t raw_len, float *depth_out, void *reflectance_out,
                         void *confidence_out, void *ambient_out, float *zapc_out);

#ifdef __cplusplus
}
#endif

#endif /* RS_TRANSFORM_SHIM_H_ */
