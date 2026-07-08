/**
 * rs_transform_shim.c
 *
 * See rs_transform_shim.h. Replicates the transform setup/process sequence
 * from firmware/scanner-stream/Src/vl53l9_app.c (capabilities section
 * through transform_prepare, then per-frame transform_process_stream),
 * with the sensor/DMA/double-buffering code removed: the caller supplies
 * one raw buffer and one output buffer per stream, per rst_process()/
 * rst_process2() call.
 *
 * v2 adds a multi-output mask (rst_create2/rst_process2): depth,
 * reflectance, confidence and ambient are independent named output streams
 * that one prepared transform instance can all emit in a single
 * transform_process_stream() call, so they share one "primary" instance.
 * ZAPC is a second *format* of the "depth" stream, mutually exclusive with
 * ZF32 on the same instance, so it lives on its own lazily-created "zapc"
 * instance -- see the cost/consistency discussion in rs_transform_shim.h.
 * rst_create()/rst_process() are now thin depth-only wrappers over
 * rst_create2()/rst_process2() so the byte-identity equivalence test keeps
 * exercising this same code path unmodified.
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
#define RST_OUT_BYTES (RST_OUT_COUNT * (uint32_t)sizeof(float))          /* depth/reflectance/confidence/ambient plane */
#define RST_OUT_ZAPC_BYTES (RST_OUT_COUNT * 4u * (uint32_t)sizeof(float)) /* ZAPC: [x,y,z,confidence] per zone */

#define RST_PRIMARY_MASK (RST_OUT_DEPTH | RST_OUT_REFLECTANCE | RST_OUT_CONFIDENCE | RST_OUT_AMBIENT)

typedef struct {
    transform_t *p_transform;      /* primary instance: depth/reflectance/confidence/ambient (whichever requested) */
    transform_t *p_transform_zapc; /* secondary instance: depth/ZAPC only; NULL unless out_mask & RST_OUT_ZAPC */

    uint32_t out_mask;
    uint32_t expected_raw_len; /* in_width * in_height, validated on each rst_process2() call */
    uint8_t calib[RST_CALIB_SIZE];

    /* primary instance I/O plumbing -- wired once in rst_create2(), .data updated every rst_process2() call */
    memory_t in_mem, depth_mem, refl_mem, conf_mem, amb_mem;
    memories_t in_mems, depth_mems, refl_mems, conf_mems, amb_mems;
    stream_buffer_t bufs[5]; /* "raw" + up to 4 active outputs */
    stream_buffers_t stream_buffers;

    /* secondary (ZAPC) instance I/O plumbing */
    memory_t in_mem_zapc, zapc_mem;
    memories_t in_mems_zapc, zapc_mems;
    stream_buffer_t bufs_zapc[2]; /* "raw" + "depth" (ZAPC format) */
    stream_buffers_t stream_buffers_zapc;
} rst_ctx_t;

static void rst_free_ctx(rst_ctx_t *ctx) {
    if (ctx != NULL) {
        if (ctx->p_transform != NULL) {
            vl53l9_transform_destroy(ctx->p_transform);
        }
        if (ctx->p_transform_zapc != NULL) {
            vl53l9_transform_destroy(ctx->p_transform_zapc);
        }
        free(ctx);
    }
}

/* Builds a 1-capability {format, width, height} property set and applies it to one named stream. */
static int _set_stream_caps(transform_t *t, const char *stream_name, const char *format, uint32_t width,
                            uint32_t height) {
    property_t p_format = { "format", { .val.v_string = format, .tid = VTID_STRING } };
    property_t p_width = { "width", { .val.v_uint32 = width, .tid = VTID_UINT32 } };
    property_t p_height = { "height", { .val.v_uint32 = height, .tid = VTID_UINT32 } };

    properties_t *props = properties_new(3);
    properties_add(props, &p_format);
    properties_add(props, &p_width);
    properties_add(props, &p_height);
    capabilities_t *caps = capabilities_new_simple(&props);

    int ret = transform_set_stream_capabilities(t, stream_name, caps);

    properties_free(props, NULL);
    capabilities_free(caps, NULL);
    return ret;
}

/* Creates, initializes and prepares one transform instance: raw/3DMD input capabilities, then
 * n_outs named output stream capabilities (out_names[i]/out_formats[i], all at RST_OUT_WIDTH x
 * RST_OUT_HEIGHT), then the mandatory calib-buffer control, then transform_prepare. Mirrors
 * vl53l9_app.c's setup ordering (input before output, no defaults, both mandatory). Returns NULL
 * and destroys any partially-built instance on the first failing step. */
static transform_t *_create_prepared_instance(uint32_t in_width, uint32_t in_height, const char *const *out_names,
                                              const char *const *out_formats, uint32_t n_outs, uint8_t *calib_buf) {
    transform_t *t = vl53l9_transform_create();
    if (t == NULL) {
        return NULL;
    }

    if (transform_initialize(t) != 0) {
        vl53l9_transform_destroy(t);
        return NULL;
    }

    if (_set_stream_caps(t, "raw", "3DMD", in_width, in_height) != 0) {
        vl53l9_transform_destroy(t);
        return NULL;
    }

    for (uint32_t i = 0; i < n_outs; i++) {
        if (_set_stream_caps(t, out_names[i], out_formats[i], RST_OUT_WIDTH, RST_OUT_HEIGHT) != 0) {
            vl53l9_transform_destroy(t);
            return NULL;
        }
    }

    /* mandatory static control, must be set before transform_prepare() */
    if (transform_set_control(t, "calib-buffer", (value_t){ .val.v_ptr = calib_buf, .tid = VTID_POINTER }) != 0) {
        vl53l9_transform_destroy(t);
        return NULL;
    }

    if (transform_prepare(t) != 0) {
        vl53l9_transform_destroy(t);
        return NULL;
    }

    return t;
}

void *rst_create2(const uint8_t *calib, uint32_t calib_len, uint32_t in_width, uint32_t in_height,
                  uint32_t out_mask) {
    if ((calib == NULL) || (calib_len != RST_CALIB_SIZE) || (in_width == 0u) || (in_height == 0u) ||
        (out_mask == 0u) || ((out_mask & ~RST_OUT_MASK_ALL) != 0u)) {
        return NULL;
    }

    rst_ctx_t *ctx = (rst_ctx_t *)calloc(1, sizeof(rst_ctx_t));
    if (ctx == NULL) {
        return NULL;
    }
    memcpy(ctx->calib, calib, RST_CALIB_SIZE);
    ctx->expected_raw_len = in_width * in_height;
    ctx->out_mask = out_mask;

    uint32_t primary_mask = out_mask & RST_PRIMARY_MASK;

    if (primary_mask != 0u) {
        const char *names[4];
        const char *formats[4];
        uint32_t n = 0;
        if ((primary_mask & RST_OUT_DEPTH) != 0u) {
            names[n] = "depth";
            formats[n] = "ZF32";
            n++;
        }
        if ((primary_mask & RST_OUT_REFLECTANCE) != 0u) {
            names[n] = "reflectance";
            formats[n] = "RF32";
            n++;
        }
        if ((primary_mask & RST_OUT_CONFIDENCE) != 0u) {
            names[n] = "confidence";
            formats[n] = "CF32";
            n++;
        }
        if ((primary_mask & RST_OUT_AMBIENT) != 0u) {
            names[n] = "ambient";
            formats[n] = "IF32";
            n++;
        }

        ctx->p_transform = _create_prepared_instance(in_width, in_height, names, formats, n, ctx->calib);
        if (ctx->p_transform == NULL) {
            rst_free_ctx(ctx);
            return NULL;
        }

        /* wire up the (single-buffer, no DMA) stream_buffers container reused by every rst_process2() call */
        ctx->in_mems = (memories_t){ .items = &ctx->in_mem, .size = 1, .capacity = 1, .item_size = sizeof(memory_t) };
        uint32_t nb = 0;
        ctx->bufs[nb++] = (stream_buffer_t){
            .name = "raw", .buffer = { .memories = &ctx->in_mems, .nb = 0, .timestamp = 0, .metadata = NULL }
        };

        if ((primary_mask & RST_OUT_DEPTH) != 0u) {
            ctx->depth_mems =
                (memories_t){ .items = &ctx->depth_mem, .size = 1, .capacity = 1, .item_size = sizeof(memory_t) };
            ctx->bufs[nb++] = (stream_buffer_t){
                .name = "depth", .buffer = { .memories = &ctx->depth_mems, .nb = 0, .timestamp = 0, .metadata = NULL }
            };
        }
        if ((primary_mask & RST_OUT_REFLECTANCE) != 0u) {
            ctx->refl_mems =
                (memories_t){ .items = &ctx->refl_mem, .size = 1, .capacity = 1, .item_size = sizeof(memory_t) };
            ctx->bufs[nb++] = (stream_buffer_t){
                .name = "reflectance",
                .buffer = { .memories = &ctx->refl_mems, .nb = 0, .timestamp = 0, .metadata = NULL }
            };
        }
        if ((primary_mask & RST_OUT_CONFIDENCE) != 0u) {
            ctx->conf_mems =
                (memories_t){ .items = &ctx->conf_mem, .size = 1, .capacity = 1, .item_size = sizeof(memory_t) };
            ctx->bufs[nb++] = (stream_buffer_t){
                .name = "confidence",
                .buffer = { .memories = &ctx->conf_mems, .nb = 0, .timestamp = 0, .metadata = NULL }
            };
        }
        if ((primary_mask & RST_OUT_AMBIENT) != 0u) {
            ctx->amb_mems =
                (memories_t){ .items = &ctx->amb_mem, .size = 1, .capacity = 1, .item_size = sizeof(memory_t) };
            ctx->bufs[nb++] = (stream_buffer_t){
                .name = "ambient", .buffer = { .memories = &ctx->amb_mems, .nb = 0, .timestamp = 0, .metadata = NULL }
            };
        }

        ctx->stream_buffers =
            (stream_buffers_t){ .items = ctx->bufs, .size = nb, .capacity = nb, .item_size = sizeof(stream_buffer_t) };
    }

    if ((out_mask & RST_OUT_ZAPC) != 0u) {
        const char *names[1] = { "depth" };
        const char *formats[1] = { "ZAPC" };

        ctx->p_transform_zapc = _create_prepared_instance(in_width, in_height, names, formats, 1u, ctx->calib);
        if (ctx->p_transform_zapc == NULL) {
            rst_free_ctx(ctx);
            return NULL;
        }

        ctx->in_mems_zapc =
            (memories_t){ .items = &ctx->in_mem_zapc, .size = 1, .capacity = 1, .item_size = sizeof(memory_t) };
        ctx->zapc_mems =
            (memories_t){ .items = &ctx->zapc_mem, .size = 1, .capacity = 1, .item_size = sizeof(memory_t) };

        ctx->bufs_zapc[0] = (stream_buffer_t){
            .name = "raw", .buffer = { .memories = &ctx->in_mems_zapc, .nb = 0, .timestamp = 0, .metadata = NULL }
        };
        ctx->bufs_zapc[1] = (stream_buffer_t){
            .name = "depth", .buffer = { .memories = &ctx->zapc_mems, .nb = 0, .timestamp = 0, .metadata = NULL }
        };

        ctx->stream_buffers_zapc =
            (stream_buffers_t){ .items = ctx->bufs_zapc, .size = 2, .capacity = 2, .item_size = sizeof(stream_buffer_t) };
    }

    return ctx;
}

int rst_process2(void *h, const uint8_t *raw, uint32_t raw_len, float *depth_out, void *reflectance_out,
                 void *confidence_out, void *ambient_out, float *zapc_out) {
    if ((h == NULL) || (raw == NULL)) {
        return MEDIA_ERROR_INVALID_PARAMETER;
    }
    rst_ctx_t *ctx = (rst_ctx_t *)h;
    if (raw_len != ctx->expected_raw_len) {
        return MEDIA_ERROR_INVALID_PARAMETER;
    }

    /* every selected output must have a buffer, every unselected output must not: catches
     * mask/call mismatches instead of silently ignoring extra pointers or under-filling data. */
    if (((ctx->out_mask & RST_OUT_DEPTH) != 0u) != (depth_out != NULL)) {
        return MEDIA_ERROR_INVALID_PARAMETER;
    }
    if (((ctx->out_mask & RST_OUT_REFLECTANCE) != 0u) != (reflectance_out != NULL)) {
        return MEDIA_ERROR_INVALID_PARAMETER;
    }
    if (((ctx->out_mask & RST_OUT_CONFIDENCE) != 0u) != (confidence_out != NULL)) {
        return MEDIA_ERROR_INVALID_PARAMETER;
    }
    if (((ctx->out_mask & RST_OUT_AMBIENT) != 0u) != (ambient_out != NULL)) {
        return MEDIA_ERROR_INVALID_PARAMETER;
    }
    if (((ctx->out_mask & RST_OUT_ZAPC) != 0u) != (zapc_out != NULL)) {
        return MEDIA_ERROR_INVALID_PARAMETER;
    }

    int ret;

    if (ctx->p_transform != NULL) {
        /* transform_process_stream takes non-const memory_t.data; the transform only reads the
         * "raw" stream, it does not write back into it. */
        ctx->in_mem = (memory_t){
            .data = (uint8_t *)raw, .offset = 0, .size = raw_len, .maxsize = raw_len, .flags = MEM_FLAG_NONE
        };

        if (depth_out != NULL) {
            ctx->depth_mem = (memory_t){ .data = (uint8_t *)depth_out,
                                         .offset = 0,
                                         .size = RST_OUT_BYTES,
                                         .maxsize = RST_OUT_BYTES,
                                         .flags = MEM_FLAG_NONE };
        }
        if (reflectance_out != NULL) {
            ctx->refl_mem = (memory_t){ .data = (uint8_t *)reflectance_out,
                                        .offset = 0,
                                        .size = RST_OUT_BYTES,
                                        .maxsize = RST_OUT_BYTES,
                                        .flags = MEM_FLAG_NONE };
        }
        if (confidence_out != NULL) {
            ctx->conf_mem = (memory_t){ .data = (uint8_t *)confidence_out,
                                        .offset = 0,
                                        .size = RST_OUT_BYTES,
                                        .maxsize = RST_OUT_BYTES,
                                        .flags = MEM_FLAG_NONE };
        }
        if (ambient_out != NULL) {
            ctx->amb_mem = (memory_t){ .data = (uint8_t *)ambient_out,
                                       .offset = 0,
                                       .size = RST_OUT_BYTES,
                                       .maxsize = RST_OUT_BYTES,
                                       .flags = MEM_FLAG_NONE };
        }

        ret = transform_process_stream(ctx->p_transform, &ctx->stream_buffers);
        if (ret != 0) {
            return ret;
        }
    }

    if (ctx->p_transform_zapc != NULL) {
        ctx->in_mem_zapc = (memory_t){
            .data = (uint8_t *)raw, .offset = 0, .size = raw_len, .maxsize = raw_len, .flags = MEM_FLAG_NONE
        };
        ctx->zapc_mem = (memory_t){ .data = (uint8_t *)zapc_out,
                                    .offset = 0,
                                    .size = RST_OUT_ZAPC_BYTES,
                                    .maxsize = RST_OUT_ZAPC_BYTES,
                                    .flags = MEM_FLAG_NONE };

        ret = transform_process_stream(ctx->p_transform_zapc, &ctx->stream_buffers_zapc);
        if (ret != 0) {
            return ret;
        }
    }

    return 0;
}

void *rst_create(const uint8_t *calib, uint32_t calib_len, uint32_t in_width, uint32_t in_height) {
    return rst_create2(calib, calib_len, in_width, in_height, RST_OUT_DEPTH);
}

int rst_process(void *h, const uint8_t *raw, uint32_t raw_len, float *depth_out) {
    if (depth_out == NULL) {
        return MEDIA_ERROR_INVALID_PARAMETER;
    }
    return rst_process2(h, raw, raw_len, depth_out, NULL, NULL, NULL, NULL);
}

void rst_destroy(void *h) {
    if (h == NULL) {
        return;
    }
    rst_ctx_t *ctx = (rst_ctx_t *)h;

    /* Full teardown path: the firmware never exercises this (it spins forever in the acquisition
     * loop -- see reference bug #6), so this is unvalidated by the vendor. transform_finalize/
     * -release delegate to media_finalize/media_release which are no-ops beyond state-machine
     * bookkeeping and _process_free_buffers() (frees only pipeline-internal scratch buffers,
     * never caller memory), so this is expected to be safe; verified by the create/process/
     * destroy smoke test. Applied to both the primary and (if present) ZAPC instance. */
    if (ctx->p_transform != NULL) {
        (void)transform_finalize(ctx->p_transform);
        (void)transform_release(ctx->p_transform);
        vl53l9_transform_destroy(ctx->p_transform);
        ctx->p_transform = NULL;
    }
    if (ctx->p_transform_zapc != NULL) {
        (void)transform_finalize(ctx->p_transform_zapc);
        (void)transform_release(ctx->p_transform_zapc);
        vl53l9_transform_destroy(ctx->p_transform_zapc);
        ctx->p_transform_zapc = NULL;
    }
    free(ctx);
}
