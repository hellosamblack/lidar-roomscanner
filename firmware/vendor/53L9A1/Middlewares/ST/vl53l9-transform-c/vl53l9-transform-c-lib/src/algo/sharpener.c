/**
 ******************************************************************************
 * @file    sharpener.c
 * @author  IMD Software Team
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "algo/sharpener.h"

#include <errno.h>
#include <float.h>
#include <math.h>

#ifndef EINVAL
#define EINVAL 22
#endif

#define CHECK_ERRNO(expression) \
    errno = 0;                  \
    expression;                 \
    if ((bool)errno)            \
    return

typedef int32_t i32;
typedef uint32_t u32;

typedef float_t f32;

static inline f32 minf(f32 a, f32 b) {
    return (a < b) ? a : b;
}

static inline f32 maxf(f32 a, f32 b) {
    return (a > b) ? a : b;
}

static inline f32 clampf(f32 val, f32 min, f32 max) {
    return maxf(min, minf(max, val));
}

#define MAX_IMAGE_SIZE   (2268)
#define MAX_GROUP_AMOUNT (42)

typedef enum info_id {
    MAX_SIGNAL_TOP,
    SUM_SIGNAL_TOP,
    X_BAR_TOP,
    Y_BAR_TOP,
    SIG_SCORE_TOP,
    MAX_SIGNAL_BOT,
    SUM_SIGNAL_BOT,
    X_BAR_BOT,
    Y_BAR_BOT,
    SIG_SCORE_BOT,
    INFO_ID_MAX
} info_id;

void vl53l9_algo_sharpener_init_default_params(sharpener_params_t *params) {
    params->mode = SHARPENER_MODE_SORT;
    params->invalid_distance = 12000.0f;
    params->min_range_threshold_mm = 300.0f;
    params->scale_range_threshd_by_range = true;
    params->range_threshold_factor = 0.3f;
    params->enable_max_range_threshold = true;
    params->max_range_threshold_mm_6_step = 1200.0f;
    params->max_range_threshold_mm_7_step = 600.0f;
    params->enable_distance = true;
    params->enable_gaussian = true;
    params->channel_ratio = 19.23f;
    params->sigma_factor = 0.8f;
    params->distance_power = 0.1f;
    params->signal_threshold_factor = 0.05f;
    params->threshold_includes_glare = false;
    params->glare_ratio = 0.0000138871530f;
    params->leak_shift_range_grouping = 3;
    params->nb_lines_overlap = 1;
    params->max_distance = 15.0f;
    params->min_distance_grouping = 50.0f;
    params->th_score_double_sharp = 4.0f;
}

static void sharpener_optim(const f32 *depth, const f32 *signal,

                            bool *sharp_valid, f32 *sharp_score,

                            const sharpener_params_t *params, u32 width, u32 height, u32 step_number,
                            f32 min_distance_grouping);

i32 vl53l9_algo_sharpener(const f32 *depth, const f32 *signal,

                          bool *sharp_valid, f32 *sharp_score,

                          const sharpener_params_t *params, u32 width, u32 height, u32 step_number) {
    // if one of the input is missing, return
    // if all outputs are missing, return
    if ((depth == NULL) || (signal == NULL) || !((sharp_valid != NULL) || (sharp_score != NULL))) {
        errno = EINVAL;
        return EXIT_FAILURE;
    }

    i32 exit_val = EXIT_SUCCESS;
    switch (params->mode) {
    default:
    case SHARPENER_MODE_SORT:
        errno = EINVAL;
        exit_val = EXIT_FAILURE;
        break;

    case SHARPENER_MODE_OPTIM:
        sharpener_optim(depth, signal, sharp_valid, sharp_score, params, width, height, step_number, 0.0f);
        break;

    case SHARPENER_MODE_DOUBLE_SHARP: {
            bool sharp_valids[2][MAX_IMAGE_SIZE] = { 0 };
            f32 sharp_scores[2][MAX_IMAGE_SIZE] = { 0 };

            sharpener_optim(depth, signal, sharp_valids[0], sharp_scores[0], params, width, height, step_number, 0.0f);
            sharpener_optim(depth, signal, sharp_valids[1], sharp_scores[1], params, width, height, step_number,
                            params->min_distance_grouping);

            for (u32 i = 0; i < (width * height); ++i) {
                const f32 final_score = sharp_scores[0][i] * sharp_scores[1][i];

                if (sharp_score) {
                    sharp_score[i] = final_score;
                }
                if (sharp_valid) {
                    sharp_valid[i] = final_score > params->th_score_double_sharp;
                    if (sharp_valids[0][i] && sharp_valids[1][i]) {
                        sharp_valid[i] = true;
                    }
                }
            }
        }
        break;
    }

    return exit_val;
}

void sharpener_optim(const f32 *depth, const f32 *signal,

                     bool *sharp_valid, f32 *sharp_score,

                     const sharpener_params_t *params, u32 width, u32 height, u32 step_number,
                     f32 min_distance_grouping) {
    u32 group_id[MAX_IMAGE_SIZE] = { 0u };

    f32 group_infos[MAX_GROUP_AMOUNT][INFO_ID_MAX] = { 0 };

    // grouping pre-processing
    f32 max_range_threshold_mm =
        (step_number == 7u) ? params->max_range_threshold_mm_7_step : params->max_range_threshold_mm_6_step;
    const f32 max_nb_group_m1 = params->invalid_distance / max_range_threshold_mm;
    const u32 max_nb_group = (u32)max_nb_group_m1 + 1u;

    // grouping
    for (u32 y = 0; (y < height); ++y) {
        for (u32 x = 0; (x < width); ++x) {
            const u32 i = (y * width) + x;

            u32 gid = 0;
            if (depth[i] >= min_distance_grouping) {
                const f32 fgid =
                    minf(depth[i] - min_distance_grouping, params->invalid_distance) / max_range_threshold_mm;
                gid = 1u + (u32)fgid;
            }

            group_id[i] = gid;

            if (y < ((height / 2u) + (u32)params->nb_lines_overlap)) {
                group_infos[gid][MAX_SIGNAL_TOP] = maxf(group_infos[gid][MAX_SIGNAL_TOP], signal[i]);
                group_infos[gid][SUM_SIGNAL_TOP] += signal[i];
                group_infos[gid][X_BAR_TOP] += signal[i] * (f32)x;
                group_infos[gid][Y_BAR_TOP] += signal[i] * (f32)y;
            } else {
                group_infos[gid][MAX_SIGNAL_TOP] =
                    maxf(group_infos[gid][MAX_SIGNAL_TOP], signal[i] / params->channel_ratio);
                group_infos[gid][SUM_SIGNAL_TOP] += signal[i] / params->channel_ratio;
                group_infos[gid][X_BAR_TOP] += signal[i] * (f32)x / params->channel_ratio;
                group_infos[gid][Y_BAR_TOP] += signal[i] * (f32)y / params->channel_ratio;
            }

            if (y >= ((height / 2u) - (u32)params->nb_lines_overlap)) {
                group_infos[gid][MAX_SIGNAL_BOT] = maxf(group_infos[gid][MAX_SIGNAL_BOT], signal[i]);
                group_infos[gid][SUM_SIGNAL_BOT] += signal[i];
                group_infos[gid][X_BAR_BOT] += signal[i] * (f32)x;
                group_infos[gid][Y_BAR_BOT] += signal[i] * (f32)y;
            } else {
                group_infos[gid][MAX_SIGNAL_BOT] =
                    maxf(group_infos[gid][MAX_SIGNAL_BOT], signal[i] / params->channel_ratio);
                group_infos[gid][SUM_SIGNAL_BOT] += signal[i] / params->channel_ratio;
                group_infos[gid][X_BAR_BOT] += signal[i] * (f32)x / params->channel_ratio;
                group_infos[gid][Y_BAR_BOT] += signal[i] * (f32)y / params->channel_ratio;
            }
        }
    }

    for (u32 i = 0; i < max_nb_group; ++i) {
        group_infos[i][X_BAR_TOP] =
            (bool)group_infos[i][SUM_SIGNAL_TOP] ? (group_infos[i][X_BAR_TOP] / group_infos[i][SUM_SIGNAL_TOP]) : 0.0f;
        group_infos[i][Y_BAR_TOP] =
            (bool)group_infos[i][SUM_SIGNAL_TOP] ? (group_infos[i][Y_BAR_TOP] / group_infos[i][SUM_SIGNAL_TOP]) : 0.0f;
        group_infos[i][SIG_SCORE_TOP] = (bool)group_infos[i][MAX_SIGNAL_TOP]
                                            ? (group_infos[i][SUM_SIGNAL_TOP] / group_infos[i][MAX_SIGNAL_TOP])
                                            : 1.0f;
        group_infos[i][X_BAR_BOT] =
            (bool)group_infos[i][SUM_SIGNAL_BOT] ? (group_infos[i][X_BAR_BOT] / group_infos[i][SUM_SIGNAL_BOT]) : 0.0f;
        group_infos[i][Y_BAR_BOT] =
            (bool)group_infos[i][SUM_SIGNAL_BOT] ? (group_infos[i][Y_BAR_BOT] / group_infos[i][SUM_SIGNAL_BOT]) : 0.0f;
        group_infos[i][SIG_SCORE_BOT] = (bool)group_infos[i][MAX_SIGNAL_BOT]
                                            ? (group_infos[i][SUM_SIGNAL_BOT] / group_infos[i][MAX_SIGNAL_BOT])
                                            : 1.0f;
    }

    // set status pre-processing
    const f32 distance_power = params->distance_power / 2.0f;
    const f32 glare_ratio = params->glare_ratio * (f32)params->threshold_includes_glare;

    // set status
    for (u32 y = 0; y < height; ++y) {
        for (u32 x = 0; x < width; ++x) {
            const u32 i = (y * width) + x;

            if (depth[i] >= params->invalid_distance) {
                sharp_score[i] = 1.0f;
            }

            const u32 gid = group_id[i];
            info_id max_signal_id, sum_signal_id, x_bar_id, y_bar_id, sig_score_id;
            if (y < (height / 2u)) {
                max_signal_id = MAX_SIGNAL_TOP;
                sum_signal_id = SUM_SIGNAL_TOP;
                x_bar_id = X_BAR_TOP;
                y_bar_id = Y_BAR_TOP;
                sig_score_id = SIG_SCORE_TOP;
            } else {
                max_signal_id = MAX_SIGNAL_BOT;
                sum_signal_id = SUM_SIGNAL_BOT;
                x_bar_id = X_BAR_BOT;
                y_bar_id = Y_BAR_BOT;
                sig_score_id = SIG_SCORE_BOT;
            }

            const f32 sig_score_sq = group_infos[gid][sig_score_id] * group_infos[gid][sig_score_id] *
                                     params->sigma_factor * params->sigma_factor;

            f32 distance = 1.0f;
            if (params->enable_distance) {
                if (params->enable_gaussian) {
                    CHECK_ERRNO(const f32 distance_before_expf = ((powf((f32)x - group_infos[gid][x_bar_id], 2.0f) +
                                                                   powf((f32)y - group_infos[gid][y_bar_id], 2.0f)) /
                                                                  (2.0f * sig_score_sq)));
                    CHECK_ERRNO(distance = (distance_before_expf < 88.7228f) ? expf(distance_before_expf) : FLT_MAX);
                } else {
                    CHECK_ERRNO(distance = powf(powf((f32)x - group_infos[gid][x_bar_id], 2.0f) +
                                                    powf((f32)y - group_infos[gid][y_bar_id], 2.0f),
                                                distance_power));
                }

                distance = clampf(distance, 1.0f, params->max_distance);
            }

            const f32 signal_threshold = (group_infos[gid][max_signal_id] * params->signal_threshold_factor) +
                                         (glare_ratio * group_infos[gid][sum_signal_id]);
            const f32 score = (bool)signal_threshold ? (signal[i] * (distance / signal_threshold)) : 0.0f;
            if (sharp_score) {
                sharp_score[i] = score;
            }
            if (sharp_valid) {
                sharp_valid[i] = (bool)((i32)score);
            }
        }
    }
}
