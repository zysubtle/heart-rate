/**
 * @file hr_mac.h
 * @brief M5 Motion Artifact Cancellation (MAC) — internal API.
 *
 * This header is internal to src/ and MUST NOT be exposed as public API.
 *
 * M5 receives:
 *   - Preprocessed PPG window from M2 (200 points x 4 channels, float)
 *   - Preprocessed ACC window from M2 (200 points x 3 axes, float)
 *   - main_ch from M3 (SQI / channel selection)
 *   - motion_state from M4 (motion detection)
 *
 * M5 produces:
 *   - ppg_mac[200]: artifact-suppressed waveform for the main PPG channel
 *   - mac_result_t: validity, bypass/divergence flags, energy statistics
 *
 * Algorithm (O6 first version): Multi-input NLMS adaptive filter.
 *   Target:    d[n] = ppg_preproc[n][main_ch]
 *   Reference: 3-axis dynamic ACC (per-axis de-meaned within window)
 *   Output:    e[n] = d[n] - w^T u[n]  (error = cleaned signal)
 *
 * raw/mac dual-branch contract:
 *   - raw branch = ppg_in->data[:, main_ch]  (M5 does NOT modify it)
 *   - mac branch = ctx->mac.out.data[:]
 *   - M6 should check ctx->mac.result.valid before using mac branch
 *   - When valid==false, out.data may contain a copy of raw for
 *     downstream convenience, but must NOT be treated as MAC output
 *
 * Responsibilities:
 *   - Adaptive artifact cancellation on main_ch PPG using ACC reference
 *   - Produce valid/bypassed/diverged result for downstream decision
 *   - Bypass MAC in REST state to protect clean signals
 *   - Detect and handle filter divergence safely
 *
 * NOT responsible for: SQI, channel selection, motion classification,
 * candidate estimation, fusion, state machine, confidence, or final
 * HR output.  M5 does NOT write to hr_output_t or hr_debug_frame_t.
 *
 * Input contract:
 *   - n samples (typically HR_WINDOW_SIZE = 200), ordered oldest to newest
 *   - M5 does NOT read from ring buffers directly
 *   - M5 does NOT perform preprocessing (M2 owns that)
 *   - M5 only reads main_ch from sqi_result (M3 is the sole writer)
 *   - M5 only reads motion_state from motion_result (M4 is the sole writer)
 */

#ifndef HR_MAC_H
#define HR_MAC_H

#include "hr_preproc.h"        /* ppg_preproc_out_t, acc_preproc_out_t,
                                  HR_WINDOW_SIZE */
#include "hr_sqi.h"            /* sqi_result_t */
#include "hr_motion.h"         /* motion_result_t */
#include "hr_algo_params.h"    /* hr_params_t */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/*  Compile-time upper bound for NLMS filter order                     */
/*                                                                     */
/*  Dynamic memory is prohibited; weight and delay-line arrays must be */
/*  sized at compile time.  Runtime effective_order is clamped to this  */
/*  limit: effective_order = min(params->mac.filter_order, MAX_ORDER). */
/*                                                                     */
/*  Total NLMS taps = 3 axes * effective_order.                        */
/*  With MAX_ORDER=16: max 48 taps, 384 bytes for weights + delay.     */
/* ------------------------------------------------------------------ */

#define HR_MAC_MAX_ORDER   16
#define HR_MAC_MAX_TAPS    (3 * HR_MAC_MAX_ORDER)   /* 48 */

/* ------------------------------------------------------------------ */
/*  MAC output waveform                                                */
/* ------------------------------------------------------------------ */

typedef struct {
    float data[HR_WINDOW_SIZE];
} ppg_mac_out_t;

/* ------------------------------------------------------------------ */
/*  MAC per-cycle result                                               */
/*                                                                     */
/*  valid:              true  → M6 may use mac branch for candidates   */
/*                      false → M6 must rely on raw branch only        */
/*  bypassed:           true  → MAC intentionally skipped (e.g. REST)  */
/*  diverged:           true  → MAC ran but was numerically unstable;  */
/*                              weights have been reset for recovery    */
/*  main_ch_used:       PPG channel used as NLMS target this cycle     */
/*  input_energy:       mean(d[n]^2)  — target signal power            */
/*  output_energy:      mean(e[n]^2)  — MAC output signal power        */
/*  artifact_est_energy:mean(y[n]^2)  — estimated artifact power       */
/* ------------------------------------------------------------------ */

typedef struct {
    bool    valid;
    bool    bypassed;
    bool    diverged;
    uint8_t main_ch_used;
    float   input_energy;
    float   output_energy;
    float   artifact_est_energy;
} mac_result_t;

/* ------------------------------------------------------------------ */
/*  M5 sub-context                                                     */
/*                                                                     */
/*  Embedded inside hr_algo_ctx_t.  No dynamic memory.                 */
/*  All large buffers (output waveform, weight/delay arrays) live      */
/*  here — never on the stack.                                         */
/*                                                                     */
/*  NLMS weights persist across calls (essential for convergence).     */
/*  Delay lines are reset at the start of each hr_mac_run() because   */
/*  M2 is block-stateless (DC warm-up per call) — cross-call delay    */
/*  continuity is not guaranteed.                                      */
/* ------------------------------------------------------------------ */

typedef struct {
    ppg_mac_out_t out;
    mac_result_t  result;

    /* NLMS adaptive filter weights.
     * Layout: [axis0_tap0 .. axis0_tapL-1, axis1_tap0 .. axis2_tapL-1]
     * where L = effective_order.  Only [0..total_taps-1] are active. */
    float         weights[HR_MAC_MAX_TAPS];

    /* NLMS tap delay lines (same layout as weights).
     * tap0 = newest sample, tapL-1 = oldest. */
    float         delay_line[HR_MAC_MAX_TAPS];

    uint16_t      effective_order;   /* min(param, MAX_ORDER) */
    uint16_t      total_taps;        /* 3 * effective_order   */

    bool          initialized;
} hr_mac_ctx_t;

/* ================================================================== */
/*  Internal API                                                       */
/* ================================================================== */

/**
 * Initialize M5 sub-context.
 *
 * - Clears all output, result, weights, and delay lines to zero.
 * - Computes effective_order = min(params->mac.filter_order, MAX_ORDER).
 * - Sets initialized = true.
 *
 * Must be called before the first hr_mac_run().
 */
void hr_mac_init(hr_mac_ctx_t *ctx, const hr_params_t *params);

/**
 * Run M5: motion artifact cancellation on one window.
 *
 * @param ctx        M5 sub-context (must have been initialized)
 * @param ppg_in     Preprocessed PPG window from M2 (n x 4 channels)
 * @param acc_in     Preprocessed ACC window from M2 (n x 3 axes)
 * @param sqi_in     SQI result from M3 (reads main_ch only)
 * @param motion_in  Motion result from M4 (reads state only)
 * @param n          Number of samples (typically HR_WINDOW_SIZE = 200)
 * @param params     Algorithm parameters (reads params->mac.*)
 *
 * On return:
 *   ctx->out.data[]          MAC output waveform (or raw copy if bypassed)
 *   ctx->result.valid        whether mac branch is usable
 *   ctx->result.bypassed     whether MAC was intentionally skipped
 *   ctx->result.diverged     whether filter was numerically unstable
 *   ctx->result.main_ch_used which PPG channel was processed
 *   ctx->result.*_energy     energy statistics for diagnostics
 *
 * Behavior by motion_state:
 *   REST:       bypass (valid=false, bypassed=true)
 *   WALK/RUN/IRREGULAR: attempt MAC; on divergence → valid=false, diverged=true
 */
void hr_mac_run(hr_mac_ctx_t *ctx,
                const ppg_preproc_out_t *ppg_in,
                const acc_preproc_out_t *acc_in,
                const sqi_result_t *sqi_in,
                const motion_result_t *motion_in,
                uint16_t n,
                const hr_params_t *params);

/**
 * Reset M5 sub-context to cold-start state.
 * Clears weights (forces re-convergence) and all outputs.
 * Semantically equivalent to hr_mac_init() with the same params.
 */
void hr_mac_reset(hr_mac_ctx_t *ctx, const hr_params_t *params);

#ifdef __cplusplus
}
#endif

#endif /* HR_MAC_H */
