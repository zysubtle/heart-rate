/**
 * @file hr_statemachine.c
 * @brief M8 State Machine — implementation.
 *
 * Manages hr_state transitions per processing cycle.
 * M8 is the sole writer of hr_state; all other modules read only.
 *
 * State graph:
 *   INIT → ACQUIRE → TRACK → HOLDOVER → REACQUIRE → TRACK (cycle)
 *
 * Candidate quality is evaluated on a three-tier model:
 *   - "good":  any_valid + best score ≥ good threshold + SQI acceptable
 *              → used for ACQUIRE/REACQUIRE stable-count accumulation
 *   - "poor":  no valid candidates, OR (score AND SQI both very low)
 *              → triggers TRACK → HOLDOVER
 *   - neutral: between good and poor
 *              → no state change; counters reset in ACQUIRE/REACQUIRE
 *
 * This asymmetry provides natural hysteresis: it is harder to enter
 * TRACK than to remain in it.
 *
 * No dynamic memory.  No large arrays on the stack.
 * All cross-cycle state lives in hr_sm_ctx_t.
 */

#include "hr_statemachine.h"

#include <string.h>

/* ================================================================== */
/*  V1 internal design constants                                       */
/*                                                                     */
/*  Score thresholds for candidate quality tiers.                      */
/*  If future tuning requires per-device adjustment, promote to        */
/*  hr_params_t.statemachine.                                          */
/*                                                                     */
/*  SM_SCORE_GOOD:                                                     */
/*    Minimum best_candidate_score for a cycle to count as "good".     */
/*    Typical M6 scores for clean signals: peak 0.4–0.8, FFT 0.3–0.7, */
/*    ACF 0.3–0.9.  0.3 admits moderate-quality candidates while       */
/*    rejecting clearly noisy ones.                                    */
/*                                                                     */
/*  SM_SCORE_POOR:                                                     */
/*    Below this AND SQI below poor threshold → signal quality is so   */
/*    degraded that TRACK should yield to HOLDOVER.  Set lower than    */
/*    SM_SCORE_GOOD to avoid oscillation between TRACK and HOLDOVER.   */
/*                                                                     */
/*  SM_MOTION_SCORE_BOOST:                                             */
/*    In RUN / IRREGULAR, the "good" score threshold is raised by this */
/*    amount.  More conservative acceptance prevents false lock-on     */
/*    during high-motion artifacts.                                    */
/* ================================================================== */

#define SM_SCORE_GOOD           0.3f
#define SM_SCORE_POOR           0.15f
#define SM_MOTION_SCORE_BOOST   0.1f

/* ================================================================== */
/*  Helper: best valid candidate score from M6 results                 */
/*                                                                     */
/*  Scans all 7 candidates; returns max score among valid ones.        */
/*  Returns 0.0f when no valid candidate exists.                       */
/* ================================================================== */

static float compute_best_score(const hr_candidate_result_t *cand)
{
    const hr_candidate_t *arr[7] = {
        &cand->cand_peak_raw, &cand->cand_peak_mac,
        &cand->cand_fft_raw,  &cand->cand_fft_mac,
        &cand->cand_acf_raw,  &cand->cand_acf_mac,
        &cand->cand_pred
    };

    float best = 0.0f;
    for (int i = 0; i < 7; i++) {
        if (arr[i]->valid && arr[i]->score > best) {
            best = arr[i]->score;
        }
    }
    return best;
}

/* ================================================================== */
/*  Public (internal) API                                              */
/* ================================================================== */

void hr_sm_init(hr_sm_ctx_t *ctx)
{
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
    ctx->result.state = HR_STATE_INIT;
    ctx->prev_state   = HR_STATE_INIT;
    ctx->initialized  = true;
}

void hr_sm_update(hr_sm_ctx_t *ctx,
                  bool data_ready,
                  const hr_candidate_result_t *cand_in,
                  const sqi_result_t *sqi_in,
                  const motion_result_t *motion_in,
                  const hr_fusion_ctx_t *fusion_hist,
                  const hr_params_t *params)
{
    if (!ctx || !ctx->initialized || !params) return;

    /*
     * fusion_hist is accepted for interface completeness and future use.
     * V1 state transitions are driven solely by candidate quality, SQI,
     * and motion_state.  fusion_hist->has_prev_hr / prev_hr_bpm may be
     * consulted in future iterations for richer HOLDOVER/REACQUIRE logic.
     */
    (void)fusion_hist;

    /* ------------------------------------------------------------ */
    /*  Snapshot previous state for transition detection              */
    /* ------------------------------------------------------------ */
    ctx->prev_state = ctx->result.state;
    hr_state_t state = ctx->prev_state;

    if (data_ready) ctx->ever_ready = true;

    /* ------------------------------------------------------------ */
    /*  Candidate quality summary                                    */
    /*                                                               */
    /*  Three tiers: good / neutral / poor.                          */
    /*  Evaluated only when data_ready and all upstream pointers     */
    /*  are valid; otherwise defaults to poor.                       */
    /* ------------------------------------------------------------ */
    float best_score   = 0.0f;
    bool  cand_usable  = false;   /* "good" tier  */
    bool  cand_poor    = true;    /* "poor" tier   */
    bool  sig_recovered = false;

    const bool inputs_ok = data_ready && cand_in && sqi_in && motion_in;

    if (inputs_ok) {
        best_score = compute_best_score(cand_in);

        /* --- "good" tier: strict enough to gate ACQUIRE/REACQUIRE --- */
        float good_thr = SM_SCORE_GOOD;
        if (motion_in->state == MOTION_STATE_RUN ||
            motion_in->state == MOTION_STATE_IRREGULAR) {
            good_thr += SM_MOTION_SCORE_BOOST;
        }

        cand_usable = cand_in->any_valid &&
                      best_score >= good_thr &&
                      sqi_in->sqi_main >= params->sqi.sqi_poor_threshold;

        /* --- "poor" tier: only when BOTH score AND SQI are bad ----- */
        if (cand_in->any_valid) {
            cand_poor = (best_score < SM_SCORE_POOR &&
                         sqi_in->sqi_main < params->sqi.sqi_poor_threshold);
        }
        /* !any_valid → cand_poor remains true */

        /* --- signal recovery: valid + decent quality (no motion     */
        /*     penalty — we want to detect recovery regardless of     */
        /*     current activity level)                                */
        sig_recovered = cand_in->any_valid &&
                        best_score >= SM_SCORE_GOOD &&
                        sqi_in->sqi_main >= params->sqi.sqi_poor_threshold;
    }

    /* ------------------------------------------------------------ */
    /*  State transition logic                                       */
    /* ------------------------------------------------------------ */
    switch (state) {

    /* -------------------------------------------------------------- */
    /*  INIT: cold-start state set by hr_sm_init().                   */
    /*  Unconditionally advance to ACQUIRE on first update.           */
    /* -------------------------------------------------------------- */
    case HR_STATE_INIT:
        state = HR_STATE_ACQUIRE;
        ctx->result.acquire_good_count = 0;
        break;

    /* -------------------------------------------------------------- */
    /*  ACQUIRE: accumulating data / waiting for stable candidates.    */
    /*                                                                */
    /*  While !data_ready: ring buffer not yet full — stay, reset     */
    /*  good count (candidates are not meaningful yet).               */
    /*                                                                */
    /*  While data_ready:                                             */
    /*    good quality → increment counter; if reaches threshold →    */
    /*                   transition to TRACK.                         */
    /*    not good     → reset counter, stay in ACQUIRE.              */
    /* -------------------------------------------------------------- */
    case HR_STATE_ACQUIRE:
        if (!data_ready) {
            ctx->result.acquire_good_count = 0;
        } else if (cand_usable) {
            if (ctx->result.acquire_good_count < 255)
                ctx->result.acquire_good_count++;
            if (ctx->result.acquire_good_count >=
                params->statemachine.acquire_stable_count) {
                state = HR_STATE_TRACK;
            }
        } else {
            ctx->result.acquire_good_count = 0;
        }
        break;

    /* -------------------------------------------------------------- */
    /*  TRACK: normal tracking — output is trustworthy.               */
    /*                                                                */
    /*  If data becomes unavailable or candidate quality drops to     */
    /*  "poor" tier → HOLDOVER.                                      */
    /*  "Neutral" quality (not good, not poor) keeps TRACK alive;     */
    /*  this prevents oscillation on borderline cycles.               */
    /* -------------------------------------------------------------- */
    case HR_STATE_TRACK:
        if (!data_ready || cand_poor) {
            state = HR_STATE_HOLDOVER;
            ctx->result.holdover_count = 0;
        }
        break;

    /* -------------------------------------------------------------- */
    /*  HOLDOVER: signal lost, using historical output.               */
    /*                                                                */
    /*  holdover_count increments every cycle (supports O7 in M9).    */
    /*                                                                */
    /*  Exit to REACQUIRE on:                                         */
    /*    a) Signal recovery detected (valid + decent quality)        */
    /*    b) Timeout: holdover_count >= holdover_max_count            */
    /*                                                                */
    /*  On signal-recovered exit, reacquire_good_count starts at 1    */
    /*  (the recovery cycle counts as the first good cycle).          */
    /*  On timeout exit, reacquire_good_count starts at 0.            */
    /* -------------------------------------------------------------- */
    case HR_STATE_HOLDOVER:
        if (ctx->result.holdover_count < 255)
            ctx->result.holdover_count++;

        if (sig_recovered) {
            state = HR_STATE_REACQUIRE;
            ctx->result.reacquire_good_count = 1;
        } else if (ctx->result.holdover_count >=
                   params->statemachine.holdover_max_count) {
            state = HR_STATE_REACQUIRE;
            ctx->result.reacquire_good_count = 0;
        }
        break;

    /* -------------------------------------------------------------- */
    /*  REACQUIRE: attempting to re-lock after holdover.              */
    /*                                                                */
    /*  Consecutive good cycles → TRACK.                              */
    /*  Non-good cycle → reset counter, stay in REACQUIRE.            */
    /*  Separate counter from ACQUIRE (no mixing).                    */
    /* -------------------------------------------------------------- */
    case HR_STATE_REACQUIRE:
        if (cand_usable) {
            if (ctx->result.reacquire_good_count < 255)
                ctx->result.reacquire_good_count++;
            if (ctx->result.reacquire_good_count >=
                params->statemachine.reacquire_stable_count) {
                state = HR_STATE_TRACK;
            }
        } else {
            ctx->result.reacquire_good_count = 0;
        }
        break;

    /* -------------------------------------------------------------- */
    /*  Unknown: defensive fallback to INIT.                          */
    /* -------------------------------------------------------------- */
    default:
        state = HR_STATE_INIT;
        break;
    }

    /* ------------------------------------------------------------ */
    /*  Populate result                                               */
    /* ------------------------------------------------------------ */
    ctx->result.state                = state;
    ctx->result.data_ready           = data_ready;
    ctx->result.candidate_usable     = cand_usable;
    ctx->result.best_candidate_score = best_score;

    /*
     * signal_recovered is meaningful only on the exact cycle where
     * HOLDOVER transitions to REACQUIRE via recovery detection.
     */
    ctx->result.signal_recovered =
        (ctx->prev_state == HR_STATE_HOLDOVER &&
         state == HR_STATE_REACQUIRE &&
         sig_recovered);
}

void hr_sm_reset(hr_sm_ctx_t *ctx)
{
    hr_sm_init(ctx);
}
