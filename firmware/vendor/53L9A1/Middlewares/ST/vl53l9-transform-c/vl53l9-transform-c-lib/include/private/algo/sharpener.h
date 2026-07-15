/**
 ******************************************************************************
 * @file    sharpener.h
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

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/**
 * @brief sharpener mode enum
 *
 * @note Based on Python algo R_1.4.0
 *
 * @invariant SHARPENER_MODE_SORT sharpener sorting mode (not supported)
 * @invariant SHARPENER_MODE_OPTIM sharpener optimized mode (not supported)
 * @invariant SHARPENER_MODE_DOUBLE_SHARP sharpener optimized mode run twice for quality purposes (not supported)
 */
typedef enum sharpener_mode_e {
    SHARPENER_MODE_SORT,
    SHARPENER_MODE_OPTIM,
    SHARPENER_MODE_DOUBLE_SHARP
} sharpener_mode_e;

/**
 * @brief sharpener module constants and constants computed and/or extracted from OTP
 *
 * @note Based on Python algo R_1.4.0
 *
 * @param mode sharpener mode
 * @param invalid_distance distance where a pixel is considered invalid
 * @param min_range_threshold_mm the minimum range threshold, for grouping, to be used in the event of very close
 *  targets
 * @param scale_range_threshd_by_range when set, scale the range threshold according to the target range, so that
 *  targets with the same degree of tilt will be grouped together
 * @param range_threshold_factor proportion of the current range to use as a threshold for grouping
 * @param enable_max_range_threshold when set, the maximum range threshold, for grouping. Distance above this threshold
 *  regarding the first element of group is considering to be a new group
 * @param max_range_threshold_mm_6_step the maximum range threshold between first pixel and the current one, for
 *  grouping on 6 step capture
 * @param max_range_threshold_mm_7_step the maximum range threshold between first pixel and the current one, for
 *  grouping on 7 step capture
 * @param enable_distance distance between current pixel and barycenter of the group is taking into account
 * @param enable_gaussian gaussian
 * @param sigma_factor gaussian sigma factor
 * @param distance_power power
 * @param signal_threshold_factor the threshold relative to the maximum signal for  the group, below which signals are
 *  blurred
 * @param threshold_includes_glare when set, the signal threshold is modified by the predicted glare on the zone
 * @param glare_ratio lens glare ratio input based on the edge-spread-function with 50% of the field of
 *  view (FoV) covered
 * @param leak_shift_range_grouping leaky integrator control for the grouping by range
 * @param max_distance max distance
 * @param min_distance_grouping distance of the first group
 * @param th_score_double_sharp threshold used when 2 sharpeners in a row
 */
typedef struct sharpener_params_t {
    sharpener_mode_e mode;
    float_t invalid_distance;
    float_t min_range_threshold_mm;
    bool scale_range_threshd_by_range;
    float_t range_threshold_factor;
    bool enable_max_range_threshold;
    float_t max_range_threshold_mm_6_step;
    float_t max_range_threshold_mm_7_step;
    bool enable_distance;
    bool enable_gaussian;
    float_t channel_ratio;
    float_t sigma_factor;
    float_t distance_power;
    float_t signal_threshold_factor;
    bool threshold_includes_glare;
    float_t glare_ratio;
    int32_t leak_shift_range_grouping;
    int32_t nb_lines_overlap;
    float_t max_distance;
    float_t min_distance_grouping;
    float_t th_score_double_sharp;
} sharpener_params_t;

/**
 * @brief init params default values
 *
 * @note Based on Python algo R_1.4.0
 */
void vl53l9_algo_sharpener_init_default_params(sharpener_params_t *params);

/**
 * @brief compute sharpener filter map
 *
 * @note Based on Python algo R_1.4.0
 *
 * @retval EXIT_SUCCESS algo ran successfuly
 * @retval EXIT_FAILURE algo ran into an issue, check errno
 *
 * @param depth input depth
 * @param signal input signal
 *
 * @param sharp_valid output sharpener valid flag buffer
 * @param sharp_score output sharpener score buffer
 *
 * @param params sharpener constant parameters
 * @param width image width
 * @param height image height
 * @param step_number number of dToF capture steps
 */
int32_t vl53l9_algo_sharpener(const float_t *depth, const float_t *signal,

                              bool *sharp_valid, float_t *sharp_score,

                              const sharpener_params_t *params, uint32_t width, uint32_t height, uint32_t step_number);

#ifdef __cplusplus
}
#endif
