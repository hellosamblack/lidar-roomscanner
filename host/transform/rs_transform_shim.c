/**
 * rs_transform_shim.c
 *
 * See rs_transform_shim.h. Replicates the transform setup/process sequence
 * from firmware/scanner-stream/Src/vl53l9_app.c (capabilities section
 * through transform_prepare, then per-frame transform_process_stream),
 * with the sensor/DMA/double-buffering code removed: the caller supplies
 * one raw buffer and one output buffer per rst_process() call.
 */
#include "rs_transform_shim.h"

#include <stdlib.h>
#include <string.h>

#include "media.h"
#include "vl53l9_transform.h"

#define RST_CALIB_SIZE (2332u)  /* VL53L9_CALIB_DATA_SIZE, see Drivers/BSP/Components/vl53l9/vl53l9.h */
#define RST_OUT_WIDTH (54u)
#define RST_OUT_HEIGHT (42u)
#define RST_OUT_COUNT (RST_OUT_WIDTH * RST_OUT_HEIGHT)
#define RST_OUT_BYTES (RST_OUT_COUNT * (uint32_t)sizeof(float))

typedef struct {
    transform_t *p_transform;
    memory_t in_mem, out_mem;
    memories_t in_mems, out_mems;
    stream_buffers_t stream_buffers;
    stream_buffer_t bufs[2];
    uint8_t calib[RST_CALIB_SIZE];
    uint32_t expected_raw_len; /* in_width * in_height, validated on each rst_process() call */
} rst_ctx_t;

static void rst_free_ctx(rst_ctx_t *ctx) {
    if (ctx != NULL) {
        if (ctx->p_transform != NULL) {
            vl53l9_transform_destroy(ctx->p_transform);
        }
        free(ctx);
    }
}

void *rst_create(const uint8_t *calib, uint32_t calib_len, uint32_t in_width, uint32_t in_height) {
    if ((calib == NULL) || (calib_len != RST_CALIB_SIZE) || (in_width == 0u) || (in_height == 0u)) {
        return NULL;
    }

    rst_ctx_t *ctx = (rst_ctx_t *)calloc(1, sizeof(rst_ctx_t));
    if (ctx == NULL) {
        return NULL;
    }
    memcpy(ctx->calib, calib, RST_CALIB_SIZE);
    ctx->expected_raw_len = in_width * in_height;

    ctx->p_transform = vl53l9_transform_create();
    if (ctx->p_transform == NULL) {
        free(ctx);
        return NULL;
    }

    int ret = transform_initialize(ctx->p_transform);
    if (ret != 0) {
        rst_free_ctx(ctx);
        return NULL;
    }

    /* set capabilities -- mirrors vl53l9_app.c: input stream before output,
     * no defaults, both mandatory. */

    /* build raw stream capabilities */
    property_t raw_format = { "format", { .val.v_string = "3DMD", .tid = VTID_STRING } };
    property_t raw_width = { "width", { .val.v_uint32 = in_width, .tid = VTID_UINT32 } };
    property_t raw_height = { "height", { .val.v_uint32 = in_height, .tid = VTID_UINT32 } };

    properties_t *raw_props = properties_new(3);
    properties_add(raw_props, &raw_format);
    properties_add(raw_props, &raw_width);
    properties_add(raw_props, &raw_height);
    capabilities_t *raw_caps = capabilities_new_simple(&raw_props);

    /* build depth stream capabilities */
    property_t depth_format = { "format", { .val.v_string = "ZF32", .tid = VTID_STRING } };
    property_t depth_width = { "width", { .val.v_uint32 = RST_OUT_WIDTH, .tid = VTID_UINT32 } };
    property_t depth_height = { "height", { .val.v_uint32 = RST_OUT_HEIGHT, .tid = VTID_UINT32 } };

    properties_t *depth_props = properties_new(3);
    properties_add(depth_props, &depth_format);
    properties_add(depth_props, &depth_width);
    properties_add(depth_props, &depth_height);
    capabilities_t *depth_caps = capabilities_new_simple(&depth_props);

    ret = transform_set_stream_capabilities(ctx->p_transform, "raw", raw_caps);
    int ret2 = (ret == 0) ? transform_set_stream_capabilities(ctx->p_transform, "depth", depth_caps) : ret;

    properties_free(raw_props, NULL);
    properties_free(depth_props, NULL);
    capabilities_free(raw_caps, NULL);
    capabilities_free(depth_caps, NULL);

    if (ret2 != 0) {
        rst_free_ctx(ctx);
        return NULL;
    }

    /* mandatory static control, must be set before transform_prepare() */
    ret = transform_set_control(ctx->p_transform, "calib-buffer",
                                 (value_t){ .val.v_ptr = ctx->calib, .tid = VTID_POINTER });
    if (ret != 0) {
        rst_free_ctx(ctx);
        return NULL;
    }

    ret = transform_prepare(ctx->p_transform);
    if (ret != 0) {
        rst_free_ctx(ctx);
        return NULL;
    }

    /* wire up the (single-buffer, no DMA) stream_buffers container reused by every rst_process() call */
    ctx->in_mems = (memories_t){ .items = &ctx->in_mem, .size = 1, .capacity = 1, .item_size = sizeof(memory_t) };
    ctx->out_mems = (memories_t){ .items = &ctx->out_mem, .size = 1, .capacity = 1, .item_size = sizeof(memory_t) };

    ctx->bufs[0] = (stream_buffer_t){ .name = "raw", .buffer = { .memories = &ctx->in_mems, .nb = 0, .timestamp = 0, .metadata = NULL } };
    ctx->bufs[1] = (stream_buffer_t){ .name = "depth", .buffer = { .memories = &ctx->out_mems, .nb = 0, .timestamp = 0, .metadata = NULL } };

    ctx->stream_buffers = (stream_buffers_t){ .items = ctx->bufs, .size = 2, .capacity = 2, .item_size = sizeof(stream_buffer_t) };

    return ctx;
}

int rst_process(void *h, const uint8_t *raw, uint32_t raw_len, float *depth_out) {
    if ((h == NULL) || (raw == NULL) || (depth_out == NULL)) {
        return MEDIA_ERROR_INVALID_PARAMETER;
    }
    rst_ctx_t *ctx = (rst_ctx_t *)h;
    if (raw_len != ctx->expected_raw_len) {
        return MEDIA_ERROR_INVALID_PARAMETER;
    }

    /* transform_process_stream takes non-const memory_t.data; the transform
     * only reads the "raw" stream, it does not write back into it. */
    ctx->in_mem.data = (uint8_t *)raw; /* transform only reads "raw"; explicit cast drops const */
    ctx->in_mem.offset = 0;
    ctx->in_mem.size = raw_len;
    ctx->in_mem.maxsize = raw_len;
    ctx->in_mem.flags = MEM_FLAG_NONE;

    ctx->out_mem.data = (uint8_t *)depth_out;
    ctx->out_mem.offset = 0;
    ctx->out_mem.size = RST_OUT_BYTES;
    ctx->out_mem.maxsize = RST_OUT_BYTES;
    ctx->out_mem.flags = MEM_FLAG_NONE;

    return transform_process_stream(ctx->p_transform, &ctx->stream_buffers);
}

void rst_destroy(void *h) {
    if (h == NULL) {
        return;
    }
    rst_ctx_t *ctx = (rst_ctx_t *)h;

    /* Full teardown path: the firmware never exercises this (it spins
     * forever in the acquisition loop -- see reference bug #6), so this is
     * unvalidated by the vendor. transform_finalize/-release delegate to
     * media_finalize/media_release which are no-ops beyond state-machine
     * bookkeeping and _process_free_buffers() (frees only pipeline-internal
     * scratch buffers, never caller memory), so this is expected to be
     * safe; verified by the create/process/destroy smoke test. */
    if (ctx->p_transform != NULL) {
        (void)transform_finalize(ctx->p_transform);
        (void)transform_release(ctx->p_transform);
        vl53l9_transform_destroy(ctx->p_transform);
        ctx->p_transform = NULL;
    }
    free(ctx);
}
