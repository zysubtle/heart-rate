/**
 * @file hr_statemachine.h
 * @brief M8 State Machine — internal API.
 *
 * This header is internal to src/ and MUST NOT be exposed as public API.
 *
 * M8 receives (per cycle):
 *   - data_ready: whether ring buffer contains a full 200-pt window
 *   - Candidate results from M6 (hr_candidate_result_t)
 *   - SQI result from M3 (sqi_result_t)
 *   - Motion result from M4 (motion_result_t)
 *   - Fusion historical state from M7 (hr_fusion_ctx_t, read-only,
 *     previous cycle only)
 *   - Parameters from hr_params_t.statemachine
 *
 * M8 produces:
 *   - Current-cycle hr_state (the sole legitimate writer of hr_state)
 *   - Auxiliary counts/flags for downstream M7 and future M9
 *
 * State graph (architecture doc §7.1):
 *
 *            ┌────────────────────────┐
 *            ▼                        │
 *   INIT ──→ ACQUIRE ──→ TRACK ──→ HOLDOVER
 *                          ▲          │
 *                          │          ▼
 *                          └── REACQUIRE
 *
 * Responsibilities:
 *   - Manage hr_state transitions exclusively
 *   - Record minimal cross-cycle counts for state transitions
 *   - Provide holdover_count so M9 can implement O7 confidence decay
 *
 * NOT responsible for:
 *   - SQI, channel selection, motion detection, MAC, candidates
 *   - Fusion / smoothing / pred seed update
 *   - Confidence computation (forbidden: §5.1 causal constraint)
 *   - Writing to hr_output_t or hr_debug_frame_t
 */

#ifndef HR_STATEMACHINE_H
#define HR_STATEMACHINE_H

#include "hr_candidate.h"      /* hr_candidate_result_t */
#include "hr_sqi.h"            /* sqi_result_t */
#include "hr_motion.h"         /* motion_result_t */
#include "hr_fusion.h"         /* hr_fusion_ctx_t */
#include "hr_algo_types.h"     /* hr_state_t */
#include "hr_algo_params.h"    /* hr_params_t */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/*  M8 output: per-cycle state machine result                          */
/*                                                                     */
/*  state:                primary output — current hr_state            */
/*  data_ready:           echo of input flag                           */
/*  candidate_usable:     true when candidate quality meets "good"     */
/*                        threshold (score + SQI + motion modulation)  */
/*  best_candidate_score: highest score among valid candidates [0,1]   */
/*                        0.0f when no valid candidate exists          */
/*  acquire_good_count:   consecutive "good" cycles accumulated in     */
/*                        ACQUIRE state; reset on non-good cycle       */
/*  holdover_count:       cycles spent in HOLDOVER (incremented per    */
/*                        cycle while in HOLDOVER); preserved after    */
/*                        leaving HOLDOVER so M9 can reference it      */
/*  reacquire_good_count: consecutive "good" cycles in REACQUIRE       */
/*  signal_recovered:     true only on HOLDOVER→REACQUIRE transition   */
/*                        triggered by signal recovery (not timeout)   */
/* ------------------------------------------------------------------ */

typedef struct {
    hr_state_t  state;
    bool        data_ready;
    bool        candidate_usable;
    float       best_candidate_score;
    uint8_t     acquire_good_count;
    uint8_t     holdover_count;
    uint8_t     reacquire_good_count;
    bool        signal_recovered;
} hr_sm_result_t;

/* ------------------------------------------------------------------ */
/*  M8 sub-context                                                     */
/*                                                                     */
/*  Embedded inside hr_algo_ctx_t.  No dynamic memory.                 */
/*  prev_state:   state at the end of the previous cycle (before this  */
/*                update).  Enables transition detection.              */
/*  ever_ready:   latched true once data_ready has been true at least  */
/*                once.  Prevents spurious INIT re-entry.              */
/* ------------------------------------------------------------------ */

typedef struct {
    hr_sm_result_t  result;
    hr_state_t      prev_state;
    bool            ever_ready;
    bool            initialized;
} hr_sm_ctx_t;

/* ================================================================== */
/*  Internal API                                                       */
/* ================================================================== */

/**
 * Initialize M8 sub-context.
 *
 * Sets initial state to HR_STATE_INIT, clears all counters and flags.
 * Must be called before hr_sm_update().
 */
void hr_sm_init(hr_sm_ctx_t *ctx);

/**
 * Run M8: compute current-cycle hr_state.
 *
 * @param ctx          M8 sub-context (must have been initialised)
 * @param data_ready   true when ring buffer has >= 200 samples
 * @param cand_in      M6 candidate results; may be NULL when !data_ready
 * @param sqi_in       M3 SQI result; may be NULL when !data_ready
 * @param motion_in    M4 motion result; may be NULL when !data_ready
 * @param fusion_hist  M7 sub-context (read-only, previous-cycle state);
 *                     may be NULL during early cycles
 * @param params       Algorithm parameters (reads statemachine + sqi)
 *
 * On return:
 *   ctx->result.*   fully populated for this cycle
 *   ctx->prev_state updated to the state that was current BEFORE
 *                   this update (useful for transition detection)
 *
 * M8 does NOT write to hr_output_t or hr_debug_frame_t.
 * M8 does NOT read or compute confidence.
 */
void hr_sm_update(hr_sm_ctx_t *ctx,
                  bool data_ready,
                  const hr_candidate_result_t *cand_in,
                  const sqi_result_t *sqi_in,
                  const motion_result_t *motion_in,
                  const hr_fusion_ctx_t *fusion_hist,
                  const hr_params_t *params);

/**
 * Reset M8 sub-context to cold-start state.
 * Semantically equivalent to hr_sm_init().
 */
void hr_sm_reset(hr_sm_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* HR_STATEMACHINE_H */
