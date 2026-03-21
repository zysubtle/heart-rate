/**
 * @file hr_preproc.h
 * @brief M2 Preprocessing — internal API.
 *
 * This header is internal to src/ and MUST NOT be exposed as public API.
 *
 * M2 receives the most recent 200-point ACC/PPG raw window from M1 and
 * produces filtered float outputs for downstream M3 (SQI), M4 (Motion),
 * and M5 (MAC).
 *
 * Responsibilities:
 *   - PPG: DC removal / baseline drift rejection (high-pass) +
 *          high-frequency noise rejection (low-pass)
 *   - ACC: smoothing (low-pass), preserving DC (gravity component)
 *
 * NOT responsible for: SQI, channel selection, motion classification,
 * MAC, candidate estimation, fusion, state machine, or final HR output.
 *
 * Filter design: 2nd-order IIR Butterworth (architecture doc O8).
 * Implementation: Direct Form II Transposed (DF2T) biquad.
 * Processing mode: block-stateless with DC warm-up per call.
 */

#ifndef HR_PREPROC_H
#define HR_PREPROC_H

#include "hr_sampling.h"       /* acc_sample_t, ppg_sample_t, HR_WINDOW_SIZE */
#include "hr_algo_params.h"    /* hr_params_t */

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/*  System constant — sampling rate shared across DSP modules          */
/* ------------------------------------------------------------------ */

#define HR_PREPROC_SAMPLE_RATE_HZ  25.0f

/* ------------------------------------------------------------------ */
/*  Biquad filter primitives (internal to M2, not public)              */
/* ------------------------------------------------------------------ */

/** 2nd-order IIR biquad coefficients (a0 normalised to 1). */
typedef struct {
    float b0, b1, b2;    /* numerator   */
    float a1, a2;         /* denominator */
} biquad_coeff_t;

/** DF2T biquad delay-element state. */
typedef struct {
    float w1, w2;
} biquad_state_t;

/* ------------------------------------------------------------------ */
/*  M2 output structures                                               */
/*                                                                     */
/*  200 points, ordered oldest-to-newest, channel/axis order preserved */
/*  from M1.  float type for downstream SQI / MAC / FFT / ACF.        */
/* ------------------------------------------------------------------ */

/** Preprocessed ACC output: 200 samples x 3 axes (x, y, z). */
typedef struct {
    float data[HR_WINDOW_SIZE][3];
} acc_preproc_out_t;

/** Preprocessed PPG output: 200 samples x 4 channels (ch0..ch3). */
typedef struct {
    float data[HR_WINDOW_SIZE][4];
} ppg_preproc_out_t;

/* ------------------------------------------------------------------ */
/*  M2 sub-context                                                     */
/*                                                                     */
/*  Intended to be embedded inside hr_algo_ctx_t.                      */
/*  All large buffers live here — never on the stack.                   */
/*  No dynamic memory.                                                 */
/* ------------------------------------------------------------------ */

typedef struct {
    /* Cached filter coefficients (computed once at init from params) */
    biquad_coeff_t ppg_hp_coeff;    /* PPG high-pass  */
    biquad_coeff_t ppg_lp_coeff;    /* PPG low-pass   */
    biquad_coeff_t acc_lp_coeff;    /* ACC low-pass   */

    /* Per-channel / per-axis filter states (used within each run) */
    biquad_state_t ppg_hp_state[4]; /* 4 PPG channels, independent */
    biquad_state_t ppg_lp_state[4];
    biquad_state_t acc_lp_state[3]; /* 3 ACC axes, independent     */

    /* Output buffers (kept off the stack) */
    acc_preproc_out_t acc_out;
    ppg_preproc_out_t ppg_out;

    bool initialized;
} hr_preproc_ctx_t;

/* ================================================================== */
/*  Internal API                                                       */
/* ================================================================== */

/**
 * Initialize M2 sub-context.
 *
 * - Clears all filter states and output buffers to zero.
 * - Computes Butterworth biquad coefficients from params->preproc.*
 *   using bilinear transform.  Coefficients are cached for subsequent
 *   hr_preproc_run() calls.
 * - If a cutoff frequency is out of valid range (<=0 or >=Nyquist),
 *   the corresponding filter degrades to all-pass.
 */
void hr_preproc_init(hr_preproc_ctx_t *ctx, const hr_params_t *params);

/**
 * Run M2 preprocessing on one window.
 *
 * @param ctx      M2 sub-context (must have been initialised)
 * @param acc_in   Linear buffer of n ACC samples (oldest first)
 * @param ppg_in   Linear buffer of n PPG samples (oldest first)
 * @param n        Number of samples (typically HR_WINDOW_SIZE = 200)
 *
 * On return, ctx->acc_out and ctx->ppg_out contain the preprocessed
 * results, 1:1 aligned with the input, same sample count and order.
 *
 * Input data is not modified.
 *
 * Processing is block-stateless: filter states are re-initialised
 * (with DC warm-up) at the start of each call.  No cross-call state
 * is carried over, avoiding issues with overlapping windows.
 */
void hr_preproc_run(hr_preproc_ctx_t *ctx,
                    const acc_sample_t *acc_in,
                    const ppg_sample_t *ppg_in,
                    uint16_t n);

/**
 * Reset M2 sub-context.
 * Semantically equivalent to hr_preproc_init; re-computes coefficients
 * from the given params.
 */
void hr_preproc_reset(hr_preproc_ctx_t *ctx, const hr_params_t *params);

#ifdef __cplusplus
}
#endif

#endif /* HR_PREPROC_H */
