/**
 * @file hr_algo_api.h
 * @brief Public API for heart rate algorithm.
 *
 * [Strong Freeze] All function signatures in this file are frozen.
 * External callers should only include this file.
 */

#ifndef HR_ALGO_API_H
#define HR_ALGO_API_H

#include "hr_algo_types.h"
#include "hr_algo_params.h"
#include "hr_algo_debug.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/*  hr_algo_ctx_t  [Medium Freeze]                                    */
/*  Opaque context. Full definition lives in src/ (not public).       */
/* ------------------------------------------------------------------ */
typedef struct hr_algo_ctx hr_algo_ctx_t;

/* ------------------------------------------------------------------ */
/*  Public API  [Strong Freeze]                                       */
/* ------------------------------------------------------------------ */

/** Return a hr_params_t filled with compile-time default values. */
hr_params_t hr_algo_default_params(void);

/** Initialize algorithm context with given parameters. */
void hr_algo_init(hr_algo_ctx_t *ctx, const hr_params_t *params);

/** Feed one ACC sample (call at 25 Hz). */
void hr_algo_feed_acc(hr_algo_ctx_t *ctx,
                      int16_t ax, int16_t ay, int16_t az);

/** Feed one PPG sample, all 4 channels (call at 25 Hz). */
void hr_algo_feed_ppg(hr_algo_ctx_t *ctx,
                      int32_t ch0, int32_t ch1,
                      int32_t ch2, int32_t ch3);

/** Run the full heart-rate estimation pipeline (call once per second). */
void hr_algo_process_1s(hr_algo_ctx_t *ctx);

/** Get latest output (valid after hr_algo_process_1s). */
const hr_output_t *hr_algo_get_output(const hr_algo_ctx_t *ctx);

/** Get latest debug frame (valid after hr_algo_process_1s). */
const hr_debug_frame_t *hr_algo_get_debug(const hr_algo_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* HR_ALGO_API_H */
