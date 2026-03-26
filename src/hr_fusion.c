/**
 * @file hr_fusion.c
 * @brief M7 Fusion & Smoothing — implementation.
 *
 * Fuses 7 candidates from M6 into a single smoothed hr_bpm per cycle.
 *
 * Fusion pipeline:
 *   1. Collect valid candidates
 *   2. Rank each: rank = W_SCORE * score + W_HIST * consistency
 *   3. Select highest-rank candidate
 *   4. Apply jump-limit clamping + EMA smoothing
 *   5. Update pred seed for M6 (when eligible)
 *
 * State-aware behaviour (lightweight, not a state machine):
 *   INIT:       no output (too early for meaningful fusion)
 *   ACQUIRE:    normal fusion, no pred seed update
 *   TRACK:      normal fusion, pred seed eligible
 *   HOLDOVER:   history-biased fusion weights, no pred seed update
 *   REACQUIRE:  normal fusion, no pred seed update
 *
 * No dynamic memory.  No large arrays on the stack.
 * All cross-cycle state lives in hr_fusion_ctx_t.
 */

#include "hr_fusion.h"

#include <string.h>
#include <math.h>

/* ================================================================== */
/*  V1 internal design constants                                       */
/*                                                                     */
/*  Semantic meaning documented per constant.  If future tuning        */
/*  requires per-device adjustment, promote to hr_params_t.fusion.     */
/* ================================================================== */

/*
 * Default ranking weights for candidate selection (O12).
 *
 * W_SCORE: importance of M6's intrinsic candidate score.
 * W_HIST:  importance of BPM proximity to previous output.
 *
 * Both terms are in [0,1] so rank is in [0, W_SCORE+W_HIST].
 * Relative ratio matters, not absolute magnitude.
 */
#define FUSION_W_SCORE_DEFAULT   0.6f
#define FUSION_W_HIST_DEFAULT    0.4f

/*
 * HOLDOVER ranking weights: prefer history-consistent candidates.
 * In HOLDOVER the signal is degraded; trusting history more avoids
 * jumping to a noisy new estimate.
 */
#define FUSION_W_SCORE_HOLDOVER  0.4f
#define FUSION_W_HIST_HOLDOVER   0.6f

/*
 * History consistency mapping: linear ramp from 1.0 (delta=0)
 * to 0.0 (delta >= DELTA_MAX).
 *
 * 30 BPM means candidates within 30 BPM of prev_hr get partial
 * consistency credit; beyond 30 BPM the consistency term is zero.
 *
 * Physiological basis: heart rate rarely changes > 30 BPM in one
 * second under normal conditions.  Larger jumps likely indicate
 * method error rather than true physiological change.
 */
#define FUSION_HIST_DELTA_MAX_BPM  30.0f

/*
 * Minimum consecutive valid-output cycles before pred seed is
 * eligible for update.  Prevents transient spikes from poisoning
 * the pred candidate in subsequent cycles.
 */
#define FUSION_PRED_SEED_STABLE_MIN  3

/* ================================================================== */
/*  Helper: access candidate by fixed index                            */
/*                                                                     */
/*  Index order mirrors hr_candidate_result_t field order and aligns   */
/*  with hr_debug_frame_t candidate order:                             */
/*    0=peak_raw  1=peak_mac  2=fft_raw  3=fft_mac                     */
/*    4=acf_raw   5=acf_mac   6=pred                                   */
/* ================================================================== */

static const hr_candidate_t *get_candidate(
    const hr_candidate_result_t *r, uint8_t idx)
{
    switch (idx) {
    case 0: return &r->cand_peak_raw;
    case 1: return &r->cand_peak_mac;
    case 2: return &r->cand_fft_raw;
    case 3: return &r->cand_fft_mac;
    case 4: return &r->cand_acf_raw;
    case 5: return &r->cand_acf_mac;
    case 6: return &r->cand_pred;
    default: return NULL;
    }
}

/* ================================================================== */
/*  Helper: historical consistency score [0, 1]                        */
/*                                                                     */
/*  Linear ramp: delta=0 → 1.0,  delta>=DELTA_MAX → 0.0               */
/* ================================================================== */

static float compute_consistency(float cand_bpm, float prev_bpm)
{
    float delta = fabsf(cand_bpm - prev_bpm);
    if (delta >= FUSION_HIST_DELTA_MAX_BPM) return 0.0f;
    return 1.0f - delta / FUSION_HIST_DELTA_MAX_BPM;
}

/* ================================================================== */
/*  Helper: select best candidate by composite rank                    */
/*                                                                     */
/*  Linear scan over 7 candidates; only valid ones participate.        */
/*  Ties are broken by first-encountered (lower index wins).           */
/*  Returns true if at least one valid candidate was found.            */
/* ================================================================== */

static bool select_best_candidate(
    const hr_candidate_result_t *cand_in,
    bool has_prev, float prev_bpm,
    float w_score, float w_hist,
    uint8_t *out_index)
{
    bool  found     = false;
    float best_rank = -1.0f;
    uint8_t best_idx = 0;

    for (uint8_t i = 0; i < HR_FUSION_NUM_CANDIDATES; i++) {
        const hr_candidate_t *c = get_candidate(cand_in, i);
        if (!c || !c->valid) continue;

        float hist = has_prev ? compute_consistency(c->bpm, prev_bpm)
                              : 0.0f;
        float rank = w_score * c->score + w_hist * hist;

        if (!found || rank > best_rank) {
            best_rank = rank;
            best_idx  = i;
            found     = true;
        }
    }

    if (found) {
        *out_index = best_idx;
    }
    return found;
}

/* ================================================================== */
/*  Helper: jump-limit clamping + EMA smoothing                        */
/*                                                                     */
/*  Order: clamp first, then smooth.  This ensures:                    */
/*   - Large outliers are capped before they affect the EMA            */
/*   - The EMA then gradually incorporates the clamped value           */
/*                                                                     */
/*  smooth_alpha interpretation:                                       */
/*   α = 1.0 → output follows clamped input immediately               */
/*   α = 0.0 → output locked to previous (pure holdover)              */
/*   typical α = 0.3 → moderate tracking with smoothing                */
/* ================================================================== */

static float apply_smoothing(float raw_bpm, float prev_bpm,
                             float alpha, float jump_threshold)
{
    /* Clamp alpha to [0, 1] for safety */
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;

    /* Step 1: jump-limit clamping */
    float clamped = raw_bpm;
    if (jump_threshold > 0.0f) {
        float delta = raw_bpm - prev_bpm;
        if (delta > jump_threshold) {
            clamped = prev_bpm + jump_threshold;
        } else if (delta < -jump_threshold) {
            clamped = prev_bpm - jump_threshold;
        }
    }

    /* Step 2: first-order EMA */
    return alpha * clamped + (1.0f - alpha) * prev_bpm;
}

/* ================================================================== */
/*  Helper: pred seed update                                           */
/*                                                                     */
/*  Pred seed source is the smoothed final hr_bpm (never a raw         */
/*  single-candidate BPM).                                             */
/*                                                                     */
/*  Eligibility:                                                       */
/*   - fusion valid + TRACK state + stable for >= N cycles → set seed  */
/*   - INIT state → clear seed (cold start / reset)                    */
/*   - all other cases → preserve existing seed (don't touch)          */
/* ================================================================== */

static void update_pred_seed(const hr_fusion_ctx_t *ctx,
                             hr_state_t hr_state,
                             hr_candidate_ctx_t *candidate_ctx)
{
    if (!candidate_ctx) return;

    if (ctx->result.valid &&
        hr_state == HR_STATE_TRACK &&
        ctx->stable_count >= FUSION_PRED_SEED_STABLE_MIN) {
        hr_candidate_set_pred_seed(candidate_ctx, ctx->result.hr_bpm);
    } else if (hr_state == HR_STATE_INIT) {
        hr_candidate_clear_pred_seed(candidate_ctx);
    }
    /* Other states: don't touch — preserve last good seed. */
}

/* ================================================================== */
/*  Helper: clear result to invalid defaults                           */
/* ================================================================== */

static void clear_result(hr_fusion_result_t *r)
{
    memset(r, 0, sizeof(*r));
    r->selected_source = HR_SOURCE_NONE;
}

/* ================================================================== */
/*  Public (internal) API                                              */
/* ================================================================== */

void hr_fusion_init(hr_fusion_ctx_t *ctx)
{
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
    ctx->result.selected_source = HR_SOURCE_NONE;
    ctx->initialized = true;
}

void hr_fusion_run(hr_fusion_ctx_t *ctx,
                   const hr_candidate_result_t *cand_in,
                   hr_state_t hr_state,
                   motion_state_t motion_state,
                   const hr_params_t *params,
                   hr_candidate_ctx_t *candidate_ctx)
{
    if (!ctx || !ctx->initialized || !cand_in || !params) return;

    /*
     * V1: motion_state is accepted for interface completeness but not
     * used directly in fusion logic.  M6 already applies motion-based
     * score modulation; M7 trusts the scores as received.
     * Future: motion_state may modulate smoothing aggressiveness.
     */
    (void)motion_state;

    clear_result(&ctx->result);

    /* ------------------------------------------------------------ */
    /*  INIT: algorithm has not started processing yet.              */
    /*  No meaningful candidates expected; output invalid and clear  */
    /*  pred seed to ensure clean slate.                             */
    /* ------------------------------------------------------------ */
    if (hr_state == HR_STATE_INIT) {
        ctx->stable_count = 0;
        update_pred_seed(ctx, hr_state, candidate_ctx);
        return;
    }

    /* ------------------------------------------------------------ */
    /*  Determine fusion ranking weights (state-aware, lightweight)  */
    /*                                                               */
    /*  HOLDOVER: bias toward history to avoid jumping to a noisy    */
    /*  estimate when signal quality is poor.                        */
    /*  All other states: default score-dominant weighting.          */
    /* ------------------------------------------------------------ */
    float w_score = FUSION_W_SCORE_DEFAULT;
    float w_hist  = FUSION_W_HIST_DEFAULT;

    if (hr_state == HR_STATE_HOLDOVER) {
        w_score = FUSION_W_SCORE_HOLDOVER;
        w_hist  = FUSION_W_HIST_HOLDOVER;
    }

    /* ------------------------------------------------------------ */
    /*  Select best candidate                                        */
    /* ------------------------------------------------------------ */
    uint8_t best_idx = 0;
    bool found = select_best_candidate(cand_in,
                                       ctx->has_prev_hr,
                                       ctx->prev_hr_bpm,
                                       w_score, w_hist,
                                       &best_idx);

    if (!found) {
        /* All 7 candidates invalid — no fusion output this cycle.
         * prev_hr_bpm is preserved for potential HOLDOVER use by M9.
         * stable_count resets; pred seed is not corrupted. */
        ctx->stable_count = 0;
        update_pred_seed(ctx, hr_state, candidate_ctx);
        return;
    }

    /* ------------------------------------------------------------ */
    /*  Build result from selected candidate                         */
    /* ------------------------------------------------------------ */
    const hr_candidate_t *selected = get_candidate(cand_in, best_idx);

    ctx->result.valid            = true;
    ctx->result.raw_selected_bpm = selected->bpm;
    ctx->result.selected_source  = selected->source;
    ctx->result.selected_score   = selected->score;
    ctx->result.selected_index   = best_idx;

    /* ------------------------------------------------------------ */
    /*  Smoothing: only when historical reference exists              */
    /* ------------------------------------------------------------ */
    if (ctx->has_prev_hr) {
        ctx->result.hr_bpm = apply_smoothing(
            selected->bpm,
            ctx->prev_hr_bpm,
            params->fusion.smooth_alpha,
            params->fusion.bpm_jump_threshold);
        ctx->result.smoothed = true;
    } else {
        ctx->result.hr_bpm   = selected->bpm;
        ctx->result.smoothed = false;
    }

    /* ------------------------------------------------------------ */
    /*  Safety: reject non-finite results                            */
    /* ------------------------------------------------------------ */
    if (!isfinite(ctx->result.hr_bpm) || ctx->result.hr_bpm <= 0.0f) {
        clear_result(&ctx->result);
        ctx->stable_count = 0;
        update_pred_seed(ctx, hr_state, candidate_ctx);
        return;
    }

    /* ------------------------------------------------------------ */
    /*  Update cross-cycle historical state                          */
    /* ------------------------------------------------------------ */
    ctx->prev_hr_bpm = ctx->result.hr_bpm;
    ctx->prev_source = ctx->result.selected_source;
    ctx->has_prev_hr = true;

    if (ctx->stable_count < 255) {
        ctx->stable_count++;
    }

    /* ------------------------------------------------------------ */
    /*  Pred seed update (must come after stable_count increment)    */
    /* ------------------------------------------------------------ */
    update_pred_seed(ctx, hr_state, candidate_ctx);
}

void hr_fusion_reset(hr_fusion_ctx_t *ctx)
{
    hr_fusion_init(ctx);
}
