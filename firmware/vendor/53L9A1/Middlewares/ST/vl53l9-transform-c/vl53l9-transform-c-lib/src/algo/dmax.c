/**
 ******************************************************************************
 * @file    dmax.c
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

#include "algo/dmax.h"

#include <errno.h>

#ifndef EINVAL
#define EINVAL 22
#endif

#define CHECK_SQRTF_DOM(val) \
    if ((val) < 0.0f) {      \
        errno = EDOM;        \
        return EXIT_FAILURE; \
    }

#define CHECK_ERRNO_FAIL(expression) \
    errno = 0;                       \
    expression;                      \
    if ((bool)errno)                 \
    return EXIT_FAILURE

static inline float_t minf(float_t a, float_t b) {
    return (a < b) ? a : b;
}

void vl53l9_algo_dmax_init_default_params(dmax_params_t *params) {
    params->cover_glass = false;
    params->ambient_scaling = 0u;
    params->ambient_clip = 2.0f;
    params->max_spads = 10.427f;
    params->signal_factor = 1.4f;
    params->six_step_scaler = 2.9f;
    params->max_distance = 9601.0f;
    params->dmax_reflectance = 5.0f;
    params->dmax_correction = 0.8f;
}

int32_t vl53l9_algo_dmax(const float_t *ambient, const float_t *amp_norm, const float_t *r2p_coefs,
                         const float_t *effective_spads, const float_t *conf_xtalk_est, const float_t *reflectance,
                         const bool *conf_valid, const float_t *sig_correction_factor,

                         float_t *dmax,

                         const dmax_params_t *params, uint32_t size, uint32_t step_number, float_t conf_threshold_main,
                         bool auto_expo) {
    // if cover_glass is enabled and conf_xtalk_est is missing, return
    if (params->cover_glass && (conf_xtalk_est == NULL)) {
        errno = EINVAL;
        return EXIT_FAILURE;
    }
    // if auto_expo is enabled and reflectance or conf_valid is missing, return
    if (auto_expo && ((reflectance == NULL) || (conf_valid == NULL))) {
        errno = EINVAL;
        return EXIT_FAILURE;
    }
    // if one of the other input is missing, return
    // if all outputs are missing, return
    if ((ambient == NULL) || (amp_norm == NULL) || (r2p_coefs == NULL) || (effective_spads == NULL) ||
        (sig_correction_factor == NULL) || (dmax == NULL)) {
        errno = EINVAL;
        return EXIT_FAILURE;
    }

    const float_t sig_factor =
        params->dmax_reflectance * params->signal_factor * ((step_number == 6u) ? params->six_step_scaler : 1.0f);
    const float_t cth_sq = conf_threshold_main * conf_threshold_main;
    for (uint32_t i = 0u; i < size; ++i) {
        float_t ambient_tmp = ((params->ambient_scaling != 0u) && (ambient[i] == 0.0f))
                                  ? params->ambient_clip
                                  : (ambient[i] * params->max_spads / effective_spads[i]);
        if (auto_expo) {
            if (conf_valid[i]) {
                ambient_tmp *= params->dmax_reflectance / reflectance[i];
            }
        }
        const float_t signal_1000mm =
            sig_correction_factor[i] * sig_factor * amp_norm[i] / (r2p_coefs[i] * r2p_coefs[i]);

        float_t dmax_den_sq = 0.0f;
        if (params->cover_glass) {
            dmax_den_sq = (cth_sq * cth_sq) +
                          (4.0f * (ambient_tmp + conf_xtalk_est[i]) * cth_sq * conf_xtalk_est[i] * conf_xtalk_est[i]);
        } else {
            dmax_den_sq = (cth_sq * cth_sq) + (4.0f * ambient_tmp * cth_sq);
        }
        CHECK_SQRTF_DOM(dmax_den_sq)
        CHECK_ERRNO_FAIL(const float_t dmax_den = cth_sq + sqrtf(dmax_den_sq));

        const float_t dmax_factor_sq = 2.0f * signal_1000mm / dmax_den;
        CHECK_SQRTF_DOM(dmax_factor_sq)
        CHECK_ERRNO_FAIL(dmax[i] =
                             minf(params->dmax_correction * 1000.0f * sqrtf(dmax_factor_sq), params->max_distance));
    }

    return EXIT_SUCCESS;
}
