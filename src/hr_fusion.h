/**
 * @file hr_fusion.h
 * @brief M7 Fusion & Smoothing — internal API.
 *
 * This header is internal to src/ and MUST NOT be exposed as public API.
 *
 * M7 receives:
 *   - 7 candidates from M6 (hr_candidate_result_t)
 *   - hr_state from M8 (read-only)
 *   - motion_state from M4 (read-only)
 *   - Parameters from hr_params_t.fusion
 *
 * M7 produces:
 *   - Fused & smoothed hr_bpm (the sole legitimate writer of final hr_bpm)
 *   - Pred seed update for M6 (via hr_candidate_set/clear_pred_seed)
 *
 * Fusion strategy (O12):
 *   rank = W_SCORE * candidate.score + W_HIST * history_consistency
 *   Select candidate with highest rank among valid ones.
 *   history_consistency = max(0, 1 - |bpm - prev_hr| / DELTA_MAX)
 *
 * Smoothing:
 *   1. Clamp jump: clamp(raw_bpm, prev ± bpm_jump_threshold)
 *   2. EMA: hr_bpm = alpha * clamped + (1 - alpha) * prev
 *
 * Pred seed (O5 downstream):
 *   Updated only when TRACK + valid + stable for >= N cycles.
 *   Source is the smoothed final hr_bpm (never a single raw candidate).
 *
 * Responsibilities:
 *   - Compare and fuse 7 candidates
 *   - Smooth output with history
 *   - Maintain minimal cross-cycle state (prev_hr, stable_count)
 *   - Provide pred seed to M6
 *
 * NOT responsible for:
 *   - SQI, channel selection, motion detection, MAC, candidate generation
 *   - hr_state transitions (M8 owns this)
 *   - confidence computation (M9 owns this)
 *   - Writing to hr_output_t or hr_debug_frame_t (M9 owns this)
 */

#ifndef HR_FUSION_H
#define HR_FUSION_H

#include "hr_candidate.h"      /* hr_candidate_result_t, hr_candidate_ctx_t */
#include "hr_algo_types.h"     /* hr_state_t, motion_state_t, hr_source_t,
                                  hr_candidate_t */
#include "hr_algo_params.h"    /* hr_params_t */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/*  Compile-time constants                                             */
/* ------------------------------------------------------------------ */

/** Number of candidates produced by M6 per cycle. */
#define HR_FUSION_NUM_CANDIDATES  7

/* ------------------------------------------------------------------ */
/*  M7 output: per-cycle fusion result                                 */
/*                                                                     */
/*  valid:            true when M7 produced a fused BPM this cycle     */
/*  hr_bpm:           final smoothed BPM; 0.0f when invalid            */
/*  raw_selected_bpm: pre-smoothing BPM of the selected candidate      */
/*  selected_source:  source method of selected candidate              */
/*  selected_score:   score of selected candidate [0,1]                */
/*  selected_index:   index in [0..6] corresponding to fixed order:    */
/*                    0=peak_raw 1=peak_mac 2=fft_raw 3=fft_mac        */
/*                    4=acf_raw  5=acf_mac  6=pred                     */
/*  smoothed:         true if EMA smoothing was applied                */
/* ------------------------------------------------------------------ */

typedef struct {
    bool         valid;
    float        hr_bpm;
    float        raw_selected_bpm;
    hr_source_t  selected_source;
    float        selected_score;
    uint8_t      selected_index;
    bool         smoothed;
} hr_fusion_result_t;

/* ------------------------------------------------------------------ */
/*  M7 sub-context                                                     */
/*                                                                     */
/*  Embedded inside hr_algo_ctx_t.  No dynamic memory.                 */
/*  Cross-cycle state is minimal: previous BPM, source, and a          */
/*  stable-output counter for pred seed eligibility.                   */
/* ------------------------------------------------------------------ */

typedef struct {
    hr_fusion_result_t result;

    /* --- Historical state ------------------------------------------ */
    bool         has_prev_hr;       /* true after first valid output   */
    float        prev_hr_bpm;       /* previous cycle's final hr_bpm   */
    hr_source_t  prev_source;       /* previous cycle's source method  */
    uint8_t      stable_count;      /* consecutive valid-output cycles */

    bool         initialized;
} hr_fusion_ctx_t;

/* ================================================================== */
/*  Internal API                                                       */
/* ================================================================== */

/**
 * Initialize M7 sub-context.
 *
 * Clears all results and historical state.
 * Sets initialized = true.  Must be called before hr_fusion_run().
 */
void hr_fusion_init(hr_fusion_ctx_t *ctx);

/**
 * Run M7: fuse candidates and produce smoothed hr_bpm.
 *
 * @param ctx            M7 sub-context (must have been initialised)
 * @param cand_in        M6 candidate results (7 candidates + any_valid)
 * @param hr_state       Current hr_state from M8 (read-only by M7)
 * @param motion_state   Current motion_state from M4 (read-only by M7)
 * @param params         Algorithm parameters (reads params->fusion.*)
 * @param candidate_ctx  M6 sub-context for pred seed update; may be NULL
 *                       when pred seed bridging is not yet wired
 *
 * On return:
 *   ctx->result.*          fusion output for this cycle
 *   ctx->prev_hr_bpm       updated if result is valid
 *   ctx->has_prev_hr       set true on first valid output
 *   ctx->stable_count      incremented on valid, reset on invalid
 *   candidate_ctx->pred_*  may be updated (set or cleared)
 *
 * M7 does NOT write to hr_output_t or hr_debug_frame_t.
 * M7 does NOT compute confidence.
 * M7 does NOT transition hr_state.
 */
void hr_fusion_run(hr_fusion_ctx_t *ctx,
                   const hr_candidate_result_t *cand_in,
                   hr_state_t hr_state,
                   motion_state_t motion_state,
                   const hr_params_t *params,
                   hr_candidate_ctx_t *candidate_ctx);

/**
 * Reset M7 sub-context to cold-start state.
 * Clears historical state and result.
 * Semantically equivalent to hr_fusion_init().
 */
void hr_fusion_reset(hr_fusion_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* HR_FUSION_H */
