/**
 * @file hr_motion.h
 * @brief M4 Motion State Detection -- internal API.
 *
 * This header is internal to src/ and MUST NOT be exposed as public API.
 *
 * M4 receives the preprocessed ACC window from M2 (200 points x 3 axes,
 * float, oldest-to-newest) and produces:
 *   - motion_state_t:  REST / WALK / RUN / IRREGULAR
 *   - Feature summary: dynamic_energy, dominant_freq_hz, periodicity
 *
 * Feature extraction (O9 first version -- energy + periodicity + frequency):
 *   1. Dynamic energy: mean power of de-meaned 3-axis ACC (gravity removed)
 *   2. Periodicity: max normalized ACF peak in activity frequency range
 *   3. Dominant frequency: derived from ACF peak lag
 *
 * Classification: rule-based.  Energy thresholds from hr_params_t.motion
 * combined with periodicity and dominant frequency design constants.
 *
 * Responsibilities:
 *   - Extract motion features from M2 ACC output
 *   - Classify motion state per processing cycle
 *   - Store results in M4 sub-context for downstream M5/M6/M7/M8/M9
 *
 * NOT responsible for: SQI, channel selection, MAC, candidate estimation,
 * fusion, state machine, confidence, or final HR output.
 *
 * Input contract:
 *   - n samples (typically 200), 3 axes, float, oldest-to-newest
 *   - M4 does NOT read from the ring buffer directly
 *   - M4 does NOT perform preprocessing (M2 owns that)
 *   - M4 only processes the linear window that M2 has already output
 *
 * Output contract:
 *   - One motion_state per cycle, always a valid enum value
 *   - Results stored in ctx->result only; M4 does NOT write to
 *     hr_output_t or hr_debug_frame_t (that is M9's job)
 */

#ifndef HR_MOTION_H
#define HR_MOTION_H

#include "hr_preproc.h"        /* acc_preproc_out_t, HR_WINDOW_SIZE,
                                  HR_PREPROC_SAMPLE_RATE_HZ */
#include "hr_algo_params.h"    /* hr_params_t */
#include "hr_algo_types.h"     /* motion_state_t */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/*  M4 output: per-cycle motion analysis result                        */
/*                                                                     */
/*  state:            final classification for this cycle              */
/*  dynamic_energy:   mean power of de-meaned 3-axis ACC               */
/*                    unit: (sensor-unit)^2, always >= 0               */
/*  dominant_freq_hz: estimated dominant activity frequency [Hz]       */
/*                    0.0 when unreliable (low energy or aperiodic)    */
/*  periodicity:      normalized ACF peak in activity band [0, 1]      */
/*  periodic_motion:  true if periodicity >= internal threshold        */
/* ------------------------------------------------------------------ */

typedef struct {
    motion_state_t state;
    float          dynamic_energy;
    float          dominant_freq_hz;
    float          periodicity;
    bool           periodic_motion;
} motion_result_t;

/* ------------------------------------------------------------------ */
/*  M4 sub-context                                                     */
/*                                                                     */
/*  Embedded inside hr_algo_ctx_t.  No dynamic memory.                 */
/*  mag_dyn_buf keeps the 200-sample dynamic magnitude sequence off    */
/*  the stack (800 bytes).  It is a scratch buffer: only meaningful    */
/*  during hr_motion_run().                                            */
/* ------------------------------------------------------------------ */

typedef struct {
    motion_result_t result;

    /*
     * Staging buffer: dynamic acceleration magnitude (n samples).
     * Populated and consumed within a single hr_motion_run() call.
     * Content is undefined between calls.
     */
    float           mag_dyn_buf[HR_WINDOW_SIZE];

    /*
     * Cross-cycle minimal state.
     * V1 records prev_state for downstream reference only -- the V1
     * classification logic itself is purely per-window and does NOT
     * use prev_state for hysteresis.  M8 (state machine) is the
     * proper owner of cross-cycle decision logic.
     */
    motion_state_t  prev_state;
    bool            initialized;
} hr_motion_ctx_t;

/* ================================================================== */
/*  Internal API                                                       */
/* ================================================================== */

/**
 * Initialize M4 sub-context.
 *
 * Clears all results, staging buffer, and cross-cycle state.
 * Sets initialized = false; first hr_motion_run() will set it.
 */
void hr_motion_init(hr_motion_ctx_t *ctx);

/**
 * Run M4: extract motion features and classify motion state.
 *
 * @param ctx      M4 sub-context (must have been initialized)
 * @param acc_in   Preprocessed ACC window from M2 (n x 3 axes)
 * @param n        Number of samples (typically HR_WINDOW_SIZE = 200)
 * @param params   Algorithm parameters (reads params->motion.*)
 *
 * On return:
 *   ctx->result.state            motion classification
 *   ctx->result.dynamic_energy   window-level dynamic energy
 *   ctx->result.dominant_freq_hz dominant activity frequency (or 0)
 *   ctx->result.periodicity      normalized periodicity score [0,1]
 *   ctx->result.periodic_motion  periodicity >= internal threshold
 *   ctx->prev_state              updated to current state
 *
 * Results stored in ctx->result only.  M4 does NOT write to
 * hr_output_t or hr_debug_frame_t; that is M9's responsibility.
 */
void hr_motion_run(hr_motion_ctx_t *ctx,
                   const acc_preproc_out_t *acc_in,
                   uint16_t n,
                   const hr_params_t *params);

/**
 * Reset M4 sub-context to cold-start state.
 * Semantically equivalent to hr_motion_init().
 */
void hr_motion_reset(hr_motion_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* HR_MOTION_H */
