/**
 * @file hr_candidate.h
 * @brief M6 Candidate Estimation — internal API.
 *
 * This header is internal to src/ and MUST NOT be exposed as public API.
 *
 * M6 receives:
 *   - Preprocessed PPG window from M2 (200 points x 4 channels, float)
 *   - MAC output from M5 (200 points, validity flag)
 *   - SQI result from M3 (main_ch, sqi_main)
 *   - Motion result from M4 (motion_state)
 *
 * M6 produces:
 *   - 7 candidates: peak_raw, peak_mac, fft_raw, fft_mac,
 *                    acf_raw, acf_mac, pred
 *   - Each candidate: valid, bpm, score [0,1], source
 *
 * raw/mac dual-branch contract:
 *   - raw branch: ppg_in->data[:, main_ch]  (always available)
 *   - mac branch: mac_out->data[:]           (only when mac_result->valid)
 *   - mac_result->valid == false → all 3 mac candidates are invalid
 *
 * Candidate validity contract (frozen):
 *   - valid == true:  bpm in [hr_min_bpm, hr_max_bpm], score in [0,1]
 *   - valid == false: bpm = 0.0f, score = 0.0f, source still identifies
 *                     the originating method (for debug traceability)
 *
 * Methods:
 *   - Peak: local-maxima detection with prominence, IBI → BPM
 *   - FFT:  selective DFT via Goertzel on 256-point grid (O1: 256 zero-pad)
 *   - ACF:  normalized autocorrelation in valid HR lag range
 *   - Pred: historical stable-output seed (O5: no hardcoded BPM)
 *
 * Score normalisation (O4): each method normalises internally to [0,1].
 * Pred strategy (O5): pred_seed must come from M7/M8 historical output;
 *                      first version defaults to invalid (no seed source).
 *
 * NOT responsible for: SQI computation, channel selection, motion detection,
 * MAC filtering, fusion, state machine, confidence, or final HR output.
 * M6 does NOT write to hr_output_t or hr_debug_frame_t.
 */

#ifndef HR_CANDIDATE_H
#define HR_CANDIDATE_H

#include "hr_preproc.h"        /* ppg_preproc_out_t, HR_WINDOW_SIZE,
                                  HR_PREPROC_SAMPLE_RATE_HZ */
#include "hr_mac.h"            /* ppg_mac_out_t, mac_result_t */
#include "hr_sqi.h"            /* sqi_result_t */
#include "hr_motion.h"         /* motion_result_t */
#include "hr_algo_types.h"     /* hr_candidate_t, hr_source_t */
#include "hr_algo_params.h"    /* hr_params_t */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/*  Compile-time constants                                             */
/* ------------------------------------------------------------------ */

/**
 * DFT length for Goertzel selective-DFT.
 * 200 data points zero-padded to 256 gives frequency resolution
 * of 25/256 ≈ 0.098 Hz (~5.9 BPM), sufficient for V1.
 */
#define HR_CAND_FFT_LEN   256

/* ------------------------------------------------------------------ */
/*  M6 output: per-cycle candidate results                             */
/*                                                                     */
/*  Field names align with hr_debug_frame_t for downstream copying.    */
/* ------------------------------------------------------------------ */

typedef struct {
    hr_candidate_t cand_peak_raw;
    hr_candidate_t cand_peak_mac;
    hr_candidate_t cand_fft_raw;
    hr_candidate_t cand_fft_mac;
    hr_candidate_t cand_acf_raw;
    hr_candidate_t cand_acf_mac;
    hr_candidate_t cand_pred;
    bool           any_valid;       /* true if >= 1 candidate is valid */
} hr_candidate_result_t;

/* ------------------------------------------------------------------ */
/*  M6 sub-context                                                     */
/*                                                                     */
/*  Embedded inside hr_algo_ctx_t.  No dynamic memory.                 */
/*  work_buf keeps the extracted 1D signal off the stack (800 bytes).  */
/* ------------------------------------------------------------------ */

typedef struct {
    hr_candidate_result_t result;

    /**
     * Scratch buffer for 1D signal extraction + mean subtraction.
     * Shared across peak / FFT / ACF methods.  Overwritten each branch.
     * Content is undefined between hr_candidate_run() calls.
     */
    float work_buf[HR_WINDOW_SIZE];

    /**
     * Pred candidate seed state.
     *
     * Populated by M7/M8 via hr_candidate_set_pred_seed() once stable
     * tracking is achieved.  M6's hr_candidate_run() only reads these.
     *
     * First version: no upstream writer exists yet, so cand_pred
     * remains invalid until M7/M8 are implemented.
     */
    bool  pred_seed_valid;
    float pred_seed_bpm;

    bool  initialized;
} hr_candidate_ctx_t;

/* ================================================================== */
/*  Internal API                                                       */
/* ================================================================== */

/**
 * Initialize M6 sub-context.
 *
 * Clears all results, work buffer, and pred seed state.
 * Sets initialized = true.  Must be called before hr_candidate_run().
 */
void hr_candidate_init(hr_candidate_ctx_t *ctx);

/**
 * Run M6: generate 7 heart-rate candidates for one processing cycle.
 *
 * @param ctx        M6 sub-context (must have been initialised)
 * @param ppg_in     Preprocessed PPG window from M2 (n x 4 channels)
 * @param mac_result MAC validity/stats from M5
 * @param mac_out    MAC output waveform from M5
 * @param sqi_in     SQI result from M3 (reads main_ch, sqi_main)
 * @param motion_in  Motion result from M4 (reads state)
 * @param n          Number of samples (typically HR_WINDOW_SIZE = 200)
 * @param params     Algorithm parameters (reads params->candidate.*)
 *
 * On return:
 *   ctx->result.cand_*     7 candidates with valid/bpm/score/source
 *   ctx->result.any_valid  true if at least one candidate is valid
 *
 * Results stored in ctx->result only.  M6 does NOT write to
 * hr_output_t or hr_debug_frame_t; that is M9's responsibility.
 */
void hr_candidate_run(hr_candidate_ctx_t *ctx,
                      const ppg_preproc_out_t *ppg_in,
                      const mac_result_t *mac_result,
                      const ppg_mac_out_t *mac_out,
                      const sqi_result_t *sqi_in,
                      const motion_result_t *motion_in,
                      uint16_t n,
                      const hr_params_t *params);

/**
 * Reset M6 sub-context to cold-start state.
 * Clears pred seed and all results.
 * Semantically equivalent to hr_candidate_init().
 */
void hr_candidate_reset(hr_candidate_ctx_t *ctx);

/**
 * Set pred candidate seed from external source (M7/M8).
 *
 * This is the only legitimate path for injecting a historical stable
 * BPM into the pred candidate.  M6 itself never generates this value.
 *
 * @param ctx  M6 sub-context
 * @param bpm  Stable BPM from M7/M8 tracking (must be in valid range)
 */
void hr_candidate_set_pred_seed(hr_candidate_ctx_t *ctx, float bpm);

/**
 * Invalidate pred seed (e.g., on tracking loss).
 *
 * After this call, cand_pred will be invalid until a new seed is set.
 */
void hr_candidate_clear_pred_seed(hr_candidate_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* HR_CANDIDATE_H */
