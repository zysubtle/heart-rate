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
#include <stddef.h>        /* size_t for hr_algo_ctx_sizeof */

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

/**
 * Return sizeof(hr_algo_ctx_t) for external allocation.
 *
 * hr_algo_ctx_t is an opaque type (incomplete in this header).
 * External callers need this to allocate ctx storage without
 * including internal headers:
 *
 *   #include <stdint.h>
 *   static uint8_t ctx_buf[HR_ALGO_CTX_MAX_SIZE];
 *   hr_algo_ctx_t *ctx = (hr_algo_ctx_t *)ctx_buf;
 *
 * Or at runtime:
 *   size_t sz = hr_algo_ctx_sizeof();
 *   hr_algo_ctx_t *ctx = (hr_algo_ctx_t *)my_alloc(sz);
 *
 * Alignment: the returned size accounts for the struct's natural
 * alignment.  Callers using static buffers should ensure the buffer
 * is aligned to at least _Alignof(max_align_t) or 8 bytes.
 */
size_t hr_algo_ctx_sizeof(void);

#ifdef __cplusplus
}
#endif

#endif /* HR_ALGO_API_H */
