/**
 * @file test_hr_fusion.c
 * @brief M7 Fusion & Smoothing — test suite.
 *
 * Tests the real implementation in src/hr_fusion.c against:
 *   [A] Contract tests:        init / reset / basic run / invalid args
 *   [B] INIT & all-invalid:    INIT blocks output, all-invalid behaviour
 *   [C] Candidate ranking:     score priority, history consistency, tie-break
 *   [D] Jump limit + EMA:      smoothing formula, alpha edges, threshold edges
 *   [E] State-aware behaviour:  HOLDOVER bias, motion_state unused
 *   [F] Pred seed semantics:   set / clear / preserve / NULL safety
 *   [G] Multi-cycle:           stable_count tracking, repeated run stability
 *
 * All inputs are directly constructed — no upstream M1–M5 pipeline needed.
 *
 * Test framework: tests/test_framework.h (project-local, zero deps).
 *
 * ---------------------------------------------------------------
 * REAL implementation constants used for expected-value calculation
 * (read from src/hr_fusion.c, not from any public API):
 *
 *   FUSION_W_SCORE_DEFAULT   = 0.6
 *   FUSION_W_HIST_DEFAULT    = 0.4
 *   FUSION_W_SCORE_HOLDOVER  = 0.4
 *   FUSION_W_HIST_HOLDOVER   = 0.6
 *   FUSION_HIST_DELTA_MAX_BPM = 30.0
 *   FUSION_PRED_SEED_STABLE_MIN = 3
 * ---------------------------------------------------------------
 */

#include "test_framework.h"
#include "hr_fusion.h"
#include "hr_candidate.h"   /* hr_candidate_ctx_t, set/clear_pred_seed */

#include <string.h>
#include <math.h>

/* ================================================================== */
/*  Mirror of real design constants for expected-value computation.    */
/*  If M7 changes these, update here and re-verify.                   */
/* ================================================================== */

#define T_W_SCORE_DEF    0.6f
#define T_W_HIST_DEF     0.4f
#define T_W_SCORE_HO     0.4f
#define T_W_HIST_HO      0.6f
#define T_DELTA_MAX      30.0f
#define T_SEED_STABLE    3

/* ================================================================== */
/*  Test helpers                                                       */
/* ================================================================== */

/** Build hr_params_t with explicit fusion parameters. */
static hr_params_t make_fusion_params(float alpha, float jump_thresh)
{
    hr_params_t p;
    memset(&p, 0, sizeof(p));
    p.fusion.smooth_alpha       = alpha;
    p.fusion.bpm_jump_threshold = jump_thresh;
    return p;
}

/** Build an invalid candidate with the given source. */
static hr_candidate_t make_invalid_cand(hr_source_t src)
{
    hr_candidate_t c;
    c.valid  = false;
    c.bpm    = 0.0f;
    c.score  = 0.0f;
    c.source = src;
    return c;
}

/** Build a valid candidate. */
static hr_candidate_t make_valid_cand(float bpm, float score, hr_source_t src)
{
    hr_candidate_t c;
    c.valid  = true;
    c.bpm    = bpm;
    c.score  = score;
    c.source = src;
    return c;
}

/** Build a candidate result where all 7 are invalid. */
static hr_candidate_result_t make_all_invalid(void)
{
    hr_candidate_result_t r;
    r.cand_peak_raw = make_invalid_cand(HR_SOURCE_PEAK_RAW);
    r.cand_peak_mac = make_invalid_cand(HR_SOURCE_PEAK_MAC);
    r.cand_fft_raw  = make_invalid_cand(HR_SOURCE_FFT_RAW);
    r.cand_fft_mac  = make_invalid_cand(HR_SOURCE_FFT_MAC);
    r.cand_acf_raw  = make_invalid_cand(HR_SOURCE_ACF_RAW);
    r.cand_acf_mac  = make_invalid_cand(HR_SOURCE_ACF_MAC);
    r.cand_pred     = make_invalid_cand(HR_SOURCE_PRED);
    r.any_valid     = false;
    return r;
}

/**
 * Set a specific candidate slot [0..6] to a valid candidate.
 * Index order: 0=peak_raw 1=peak_mac 2=fft_raw 3=fft_mac
 *              4=acf_raw  5=acf_mac  6=pred
 */
static void set_slot_valid(hr_candidate_result_t *r,
                           uint8_t slot, float bpm, float score)
{
    hr_source_t src_map[] = {
        HR_SOURCE_PEAK_RAW, HR_SOURCE_PEAK_MAC,
        HR_SOURCE_FFT_RAW,  HR_SOURCE_FFT_MAC,
        HR_SOURCE_ACF_RAW,  HR_SOURCE_ACF_MAC,
        HR_SOURCE_PRED
    };
    hr_candidate_t vc = make_valid_cand(bpm, score, src_map[slot]);

    switch (slot) {
    case 0: r->cand_peak_raw = vc; break;
    case 1: r->cand_peak_mac = vc; break;
    case 2: r->cand_fft_raw  = vc; break;
    case 3: r->cand_fft_mac  = vc; break;
    case 4: r->cand_acf_raw  = vc; break;
    case 5: r->cand_acf_mac  = vc; break;
    case 6: r->cand_pred     = vc; break;
    default: break;
    }
    r->any_valid = true;
}

/** Inject historical state into a fusion context (post-init). */
static void inject_history(hr_fusion_ctx_t *ctx,
                           float prev_bpm, hr_source_t prev_src,
                           uint8_t stable)
{
    ctx->has_prev_hr = true;
    ctx->prev_hr_bpm = prev_bpm;
    ctx->prev_source = prev_src;
    ctx->stable_count = stable;
}

/* ================================================================== */
/*  [A] Contract tests                                                 */
/* ================================================================== */

static int test_fusion_init_state(void)
{
    hr_fusion_ctx_t ctx;
    hr_fusion_init(&ctx);

    ASSERT_TRUE(ctx.initialized);
    ASSERT_FALSE(ctx.has_prev_hr);
    ASSERT_FLOAT_NEAR(ctx.prev_hr_bpm, 0.0f, 1e-6f);
    ASSERT_INT_EQ(ctx.stable_count, 0);
    ASSERT_FALSE(ctx.result.valid);
    ASSERT_FLOAT_NEAR(ctx.result.hr_bpm, 0.0f, 1e-6f);
    ASSERT_INT_EQ(ctx.result.selected_source, HR_SOURCE_NONE);
    ASSERT_FALSE(ctx.result.smoothed);
    return 0;
}

static int test_fusion_reset_state(void)
{
    hr_fusion_ctx_t ctx;
    hr_fusion_init(&ctx);

    /* Pollute state */
    ctx.has_prev_hr  = true;
    ctx.prev_hr_bpm  = 99.0f;
    ctx.stable_count = 42;
    ctx.result.valid = true;
    ctx.result.hr_bpm = 75.0f;

    hr_fusion_reset(&ctx);

    ASSERT_TRUE(ctx.initialized);
    ASSERT_FALSE(ctx.has_prev_hr);
    ASSERT_FLOAT_NEAR(ctx.prev_hr_bpm, 0.0f, 1e-6f);
    ASSERT_INT_EQ(ctx.stable_count, 0);
    ASSERT_FALSE(ctx.result.valid);
    ASSERT_FLOAT_NEAR(ctx.result.hr_bpm, 0.0f, 1e-6f);
    ASSERT_INT_EQ(ctx.result.selected_source, HR_SOURCE_NONE);
    return 0;
}

static int test_fusion_basic_run_contract(void)
{
    hr_fusion_ctx_t ctx;
    hr_fusion_init(&ctx);

    hr_candidate_result_t cands = make_all_invalid();
    set_slot_valid(&cands, 2, 75.0f, 0.8f); /* fft_raw */

    hr_params_t p = make_fusion_params(0.3f, 20.0f);

    hr_fusion_run(&ctx, &cands, HR_STATE_ACQUIRE,
                  MOTION_STATE_REST, &p, NULL);

    ASSERT_TRUE(ctx.result.valid);
    ASSERT_TRUE(isfinite(ctx.result.hr_bpm));
    ASSERT_TRUE(ctx.result.hr_bpm > 0.0f);
    ASSERT_FLOAT_NEAR(ctx.result.raw_selected_bpm, 75.0f, 1e-6f);
    ASSERT_INT_EQ(ctx.result.selected_source, HR_SOURCE_FFT_RAW);
    ASSERT_FLOAT_NEAR(ctx.result.selected_score, 0.8f, 1e-6f);
    ASSERT_INT_EQ(ctx.result.selected_index, 2);
    ASSERT_FALSE(ctx.result.smoothed); /* first output, no history */
    return 0;
}

static int test_fusion_invalid_args(void)
{
    hr_fusion_ctx_t ctx;
    hr_fusion_init(&ctx);
    hr_candidate_result_t cands = make_all_invalid();
    set_slot_valid(&cands, 0, 75.0f, 0.8f);
    hr_params_t p = make_fusion_params(0.3f, 20.0f);

    /* NULL ctx */
    hr_fusion_run(NULL, &cands, HR_STATE_TRACK, MOTION_STATE_REST, &p, NULL);

    /* NULL cand_in */
    hr_fusion_run(&ctx, NULL, HR_STATE_TRACK, MOTION_STATE_REST, &p, NULL);
    ASSERT_FALSE(ctx.result.valid);

    /* NULL params */
    hr_fusion_init(&ctx);
    hr_fusion_run(&ctx, &cands, HR_STATE_TRACK, MOTION_STATE_REST, NULL, NULL);
    ASSERT_FALSE(ctx.result.valid);

    /* Uninitialized ctx */
    memset(&ctx, 0, sizeof(ctx));
    ctx.initialized = false;
    hr_fusion_run(&ctx, &cands, HR_STATE_TRACK, MOTION_STATE_REST, &p, NULL);
    ASSERT_FALSE(ctx.result.valid);

    return 0;
}

/* ================================================================== */
/*  [B] INIT & all-invalid candidates                                  */
/* ================================================================== */

static int test_fusion_init_blocks_output_and_clears_seed(void)
{
    hr_fusion_ctx_t ctx;
    hr_fusion_init(&ctx);

    hr_candidate_result_t cands = make_all_invalid();
    set_slot_valid(&cands, 0, 80.0f, 0.9f); /* high-quality candidate */

    hr_params_t p = make_fusion_params(0.3f, 20.0f);

    /* Prepare a candidate_ctx with an existing seed */
    hr_candidate_ctx_t cand_ctx;
    hr_candidate_init(&cand_ctx);
    cand_ctx.pred_seed_valid = true;
    cand_ctx.pred_seed_bpm   = 99.0f;

    hr_fusion_run(&ctx, &cands, HR_STATE_INIT,
                  MOTION_STATE_REST, &p, &cand_ctx);

    /* INIT must block output despite valid candidates */
    ASSERT_FALSE(ctx.result.valid);
    ASSERT_FLOAT_NEAR(ctx.result.hr_bpm, 0.0f, 1e-6f);
    ASSERT_INT_EQ(ctx.stable_count, 0);

    /* INIT must clear pred seed */
    ASSERT_FALSE(cand_ctx.pred_seed_valid);

    return 0;
}

static int test_fusion_all_invalid_candidates(void)
{
    hr_fusion_ctx_t ctx;
    hr_fusion_init(&ctx);

    hr_candidate_result_t cands = make_all_invalid();
    hr_params_t p = make_fusion_params(0.3f, 20.0f);

    hr_fusion_run(&ctx, &cands, HR_STATE_TRACK,
                  MOTION_STATE_REST, &p, NULL);

    ASSERT_FALSE(ctx.result.valid);
    ASSERT_FLOAT_NEAR(ctx.result.hr_bpm, 0.0f, 1e-6f);
    ASSERT_INT_EQ(ctx.stable_count, 0);
    ASSERT_FALSE(ctx.has_prev_hr); /* was never set */
    return 0;
}

static int test_fusion_all_invalid_preserves_prev_hr(void)
{
    hr_fusion_ctx_t ctx;
    hr_fusion_init(&ctx);
    inject_history(&ctx, 88.0f, HR_SOURCE_FFT_RAW, 5);

    hr_candidate_result_t cands = make_all_invalid();
    hr_params_t p = make_fusion_params(0.3f, 20.0f);

    hr_fusion_run(&ctx, &cands, HR_STATE_TRACK,
                  MOTION_STATE_REST, &p, NULL);

    ASSERT_FALSE(ctx.result.valid);
    ASSERT_INT_EQ(ctx.stable_count, 0);
    /* prev_hr must be preserved for potential HOLDOVER use by M9 */
    ASSERT_TRUE(ctx.has_prev_hr);
    ASSERT_FLOAT_NEAR(ctx.prev_hr_bpm, 88.0f, 1e-6f);
    return 0;
}

/* ================================================================== */
/*  [C] Candidate ranking & history consistency                        */
/*                                                                     */
/*  Expected values calculated from real design constants:             */
/*    rank = W_SCORE * score + W_HIST * consistency                    */
/*    consistency = max(0, 1 - |bpm-prev| / 30)                       */
/* ================================================================== */

static int test_fusion_selects_highest_score_without_history(void)
{
    hr_fusion_ctx_t ctx;
    hr_fusion_init(&ctx);
    /* No prev_hr → history consistency = 0 for all candidates.
     * Ranking reduces to 0.6 * score.  Highest score wins. */

    hr_candidate_result_t cands = make_all_invalid();
    set_slot_valid(&cands, 0, 70.0f, 0.5f); /* peak_raw: rank=0.30 */
    set_slot_valid(&cands, 2, 80.0f, 0.9f); /* fft_raw:  rank=0.54 */
    set_slot_valid(&cands, 4, 90.0f, 0.6f); /* acf_raw:  rank=0.36 */

    hr_params_t p = make_fusion_params(0.3f, 20.0f);

    hr_fusion_run(&ctx, &cands, HR_STATE_ACQUIRE,
                  MOTION_STATE_REST, &p, NULL);

    ASSERT_TRUE(ctx.result.valid);
    ASSERT_INT_EQ(ctx.result.selected_index, 2);
    ASSERT_INT_EQ(ctx.result.selected_source, HR_SOURCE_FFT_RAW);
    ASSERT_FLOAT_NEAR(ctx.result.raw_selected_bpm, 80.0f, 1e-6f);
    ASSERT_FLOAT_NEAR(ctx.result.selected_score, 0.9f, 1e-6f);
    return 0;
}

static int test_fusion_history_consistency_affects_ranking(void)
{
    hr_fusion_ctx_t ctx;
    hr_fusion_init(&ctx);
    inject_history(&ctx, 75.0f, HR_SOURCE_PEAK_RAW, 2);

    /*
     * prev_hr_bpm = 75.  TRACK mode (w_score=0.6, w_hist=0.4).
     *
     * Slot 0 (peak_raw): bpm=100, score=0.8
     *   delta=25, consistency = 1-25/30 = 0.1667
     *   rank = 0.6*0.8 + 0.4*0.1667 = 0.5467
     *
     * Slot 2 (fft_raw): bpm=77, score=0.6
     *   delta=2, consistency = 1-2/30 = 0.9333
     *   rank = 0.6*0.6 + 0.4*0.9333 = 0.7333
     *
     * Without history, slot 0 (score 0.8) would win.
     * With history, slot 2 (close to prev) wins.
     */

    hr_candidate_result_t cands = make_all_invalid();
    set_slot_valid(&cands, 0, 100.0f, 0.8f);
    set_slot_valid(&cands, 2,  77.0f, 0.6f);

    hr_params_t p = make_fusion_params(1.0f, 100.0f);

    hr_fusion_run(&ctx, &cands, HR_STATE_TRACK,
                  MOTION_STATE_REST, &p, NULL);

    ASSERT_TRUE(ctx.result.valid);
    ASSERT_INT_EQ(ctx.result.selected_index, 2);
    ASSERT_INT_EQ(ctx.result.selected_source, HR_SOURCE_FFT_RAW);
    ASSERT_FLOAT_NEAR(ctx.result.raw_selected_bpm, 77.0f, 1e-6f);
    return 0;
}

static int test_fusion_tie_breaks_by_lower_index(void)
{
    hr_fusion_ctx_t ctx;
    hr_fusion_init(&ctx);
    /* No prev_hr → rank = 0.6 * score.
     * Two candidates with identical score → identical rank.
     * Real code: `rank > best_rank` (strictly greater) → lower index wins. */

    hr_candidate_result_t cands = make_all_invalid();
    set_slot_valid(&cands, 2, 80.0f, 0.7f); /* fft_raw */
    set_slot_valid(&cands, 4, 85.0f, 0.7f); /* acf_raw */

    hr_params_t p = make_fusion_params(0.3f, 20.0f);

    hr_fusion_run(&ctx, &cands, HR_STATE_ACQUIRE,
                  MOTION_STATE_REST, &p, NULL);

    ASSERT_TRUE(ctx.result.valid);
    ASSERT_INT_EQ(ctx.result.selected_index, 2);
    return 0;
}

/* ================================================================== */
/*  [D] Jump limit + EMA                                               */
/*                                                                     */
/*  Smoothing formula (real code):                                     */
/*    1. clamped = clamp(raw, prev ± threshold)   [if threshold > 0]   */
/*    2. hr_bpm  = alpha * clamped + (1-alpha) * prev                  */
/* ================================================================== */

static int test_fusion_first_valid_not_smoothed(void)
{
    hr_fusion_ctx_t ctx;
    hr_fusion_init(&ctx);

    hr_candidate_result_t cands = make_all_invalid();
    set_slot_valid(&cands, 0, 85.0f, 0.7f);

    hr_params_t p = make_fusion_params(0.3f, 10.0f);

    hr_fusion_run(&ctx, &cands, HR_STATE_ACQUIRE,
                  MOTION_STATE_REST, &p, NULL);

    ASSERT_TRUE(ctx.result.valid);
    ASSERT_FALSE(ctx.result.smoothed);
    /* First output: hr_bpm == raw_selected_bpm (no smoothing) */
    ASSERT_FLOAT_NEAR(ctx.result.hr_bpm, 85.0f, 1e-6f);
    ASSERT_FLOAT_NEAR(ctx.result.raw_selected_bpm, 85.0f, 1e-6f);
    return 0;
}

static int test_fusion_clamp_then_ema(void)
{
    hr_fusion_ctx_t ctx;
    hr_fusion_init(&ctx);
    inject_history(&ctx, 80.0f, HR_SOURCE_PEAK_RAW, 1);

    /*
     * prev = 80, candidate bpm = 120, threshold = 10, alpha = 0.5
     *
     * Step 1 clamp: delta = 40 > 10  →  clamped = 80 + 10 = 90
     * Step 2 EMA:   0.5 * 90 + 0.5 * 80 = 85.0
     */

    hr_candidate_result_t cands = make_all_invalid();
    set_slot_valid(&cands, 0, 120.0f, 0.8f);

    hr_params_t p = make_fusion_params(0.5f, 10.0f);

    hr_fusion_run(&ctx, &cands, HR_STATE_TRACK,
                  MOTION_STATE_REST, &p, NULL);

    ASSERT_TRUE(ctx.result.valid);
    ASSERT_TRUE(ctx.result.smoothed);
    ASSERT_FLOAT_NEAR(ctx.result.raw_selected_bpm, 120.0f, 1e-6f);
    ASSERT_FLOAT_NEAR(ctx.result.hr_bpm, 85.0f, 0.01f);
    return 0;
}

static int test_fusion_alpha_edge_cases(void)
{
    hr_candidate_result_t cands = make_all_invalid();
    set_slot_valid(&cands, 0, 120.0f, 0.8f);

    /* --- alpha = 1.0: output follows clamped input immediately --- */
    {
        hr_fusion_ctx_t ctx;
        hr_fusion_init(&ctx);
        inject_history(&ctx, 80.0f, HR_SOURCE_PEAK_RAW, 1);
        hr_params_t p = make_fusion_params(1.0f, 10.0f);

        hr_fusion_run(&ctx, &cands, HR_STATE_TRACK,
                      MOTION_STATE_REST, &p, NULL);

        /* clamp: 120→90,  EMA: 1.0*90 + 0.0*80 = 90 */
        ASSERT_FLOAT_NEAR(ctx.result.hr_bpm, 90.0f, 0.01f);
    }

    /* --- alpha = 0.0: output locked to previous --- */
    {
        hr_fusion_ctx_t ctx;
        hr_fusion_init(&ctx);
        inject_history(&ctx, 80.0f, HR_SOURCE_PEAK_RAW, 1);
        hr_params_t p = make_fusion_params(0.0f, 10.0f);

        hr_fusion_run(&ctx, &cands, HR_STATE_TRACK,
                      MOTION_STATE_REST, &p, NULL);

        /* clamp: 120→90,  EMA: 0.0*90 + 1.0*80 = 80 */
        ASSERT_FLOAT_NEAR(ctx.result.hr_bpm, 80.0f, 0.01f);
    }

    /* --- alpha = 2.0 (out of range): clamped to 1.0 --- */
    {
        hr_fusion_ctx_t ctx;
        hr_fusion_init(&ctx);
        inject_history(&ctx, 80.0f, HR_SOURCE_PEAK_RAW, 1);
        hr_params_t p = make_fusion_params(2.0f, 10.0f);

        hr_fusion_run(&ctx, &cands, HR_STATE_TRACK,
                      MOTION_STATE_REST, &p, NULL);

        /* alpha clamped to 1.0 → same as alpha=1.0 case: 90 */
        ASSERT_FLOAT_NEAR(ctx.result.hr_bpm, 90.0f, 0.01f);
    }

    /* --- alpha = -0.5 (negative): clamped to 0.0 --- */
    {
        hr_fusion_ctx_t ctx;
        hr_fusion_init(&ctx);
        inject_history(&ctx, 80.0f, HR_SOURCE_PEAK_RAW, 1);
        hr_params_t p = make_fusion_params(-0.5f, 10.0f);

        hr_fusion_run(&ctx, &cands, HR_STATE_TRACK,
                      MOTION_STATE_REST, &p, NULL);

        /* alpha clamped to 0.0 → same as alpha=0.0 case: 80 */
        ASSERT_FLOAT_NEAR(ctx.result.hr_bpm, 80.0f, 0.01f);
    }

    return 0;
}

static int test_fusion_jump_threshold_zero_skips_clamp(void)
{
    hr_candidate_result_t cands = make_all_invalid();
    set_slot_valid(&cands, 0, 120.0f, 0.8f);

    /* --- threshold = 0: code skips clamp (jump_threshold > 0.0f) --- */
    {
        hr_fusion_ctx_t ctx;
        hr_fusion_init(&ctx);
        inject_history(&ctx, 80.0f, HR_SOURCE_PEAK_RAW, 1);
        hr_params_t p = make_fusion_params(0.5f, 0.0f);

        hr_fusion_run(&ctx, &cands, HR_STATE_TRACK,
                      MOTION_STATE_REST, &p, NULL);

        /* No clamp: clamped=120,  EMA: 0.5*120+0.5*80 = 100 */
        ASSERT_FLOAT_NEAR(ctx.result.hr_bpm, 100.0f, 0.01f);
    }

    /* --- threshold = -5: also skips clamp --- */
    {
        hr_fusion_ctx_t ctx;
        hr_fusion_init(&ctx);
        inject_history(&ctx, 80.0f, HR_SOURCE_PEAK_RAW, 1);
        hr_params_t p = make_fusion_params(0.5f, -5.0f);

        hr_fusion_run(&ctx, &cands, HR_STATE_TRACK,
                      MOTION_STATE_REST, &p, NULL);

        /* Same as threshold=0: EMA 0.5*120+0.5*80 = 100 */
        ASSERT_FLOAT_NEAR(ctx.result.hr_bpm, 100.0f, 0.01f);
    }

    return 0;
}

/* ================================================================== */
/*  [E] State-aware behaviour                                          */
/* ================================================================== */

static int test_fusion_holdover_biases_toward_history(void)
{
    /*
     * Same candidates, same prev_hr.  Only hr_state differs.
     *
     * prev_hr_bpm = 80
     * Slot 0 (peak_raw): bpm=95, score=0.9
     *   delta=15, consistency = 1 - 15/30 = 0.5
     *   TRACK:    rank = 0.6*0.9 + 0.4*0.5  = 0.74
     *   HOLDOVER: rank = 0.4*0.9 + 0.6*0.5  = 0.66
     *
     * Slot 2 (fft_raw):  bpm=82, score=0.55
     *   delta=2, consistency = 1 - 2/30 = 0.9333
     *   TRACK:    rank = 0.6*0.55 + 0.4*0.9333 = 0.7033
     *   HOLDOVER: rank = 0.4*0.55 + 0.6*0.9333 = 0.78
     *
     * TRACK → slot 0 wins (0.74 > 0.7033)
     * HOLDOVER → slot 2 wins (0.78 > 0.66)
     */

    hr_candidate_result_t cands = make_all_invalid();
    set_slot_valid(&cands, 0, 95.0f, 0.9f);
    set_slot_valid(&cands, 2, 82.0f, 0.55f);

    /* alpha=1.0, large threshold → smoothing effectively passes through */
    hr_params_t p = make_fusion_params(1.0f, 100.0f);

    /* --- TRACK --- */
    hr_fusion_ctx_t ctx_track;
    hr_fusion_init(&ctx_track);
    inject_history(&ctx_track, 80.0f, HR_SOURCE_PEAK_RAW, 2);

    hr_fusion_run(&ctx_track, &cands, HR_STATE_TRACK,
                  MOTION_STATE_REST, &p, NULL);

    ASSERT_TRUE(ctx_track.result.valid);
    ASSERT_INT_EQ(ctx_track.result.selected_index, 0);
    ASSERT_INT_EQ(ctx_track.result.selected_source, HR_SOURCE_PEAK_RAW);

    /* --- HOLDOVER --- */
    hr_fusion_ctx_t ctx_ho;
    hr_fusion_init(&ctx_ho);
    inject_history(&ctx_ho, 80.0f, HR_SOURCE_PEAK_RAW, 2);

    hr_fusion_run(&ctx_ho, &cands, HR_STATE_HOLDOVER,
                  MOTION_STATE_REST, &p, NULL);

    ASSERT_TRUE(ctx_ho.result.valid);
    ASSERT_INT_EQ(ctx_ho.result.selected_index, 2);
    ASSERT_INT_EQ(ctx_ho.result.selected_source, HR_SOURCE_FFT_RAW);

    return 0;
}

static int test_fusion_motion_state_currently_unused(void)
{
    /*
     * Run fusion twice with identical inputs except motion_state.
     * Real code: `(void)motion_state;`  →  outputs must be identical.
     */
    hr_candidate_result_t cands = make_all_invalid();
    set_slot_valid(&cands, 0, 75.0f, 0.8f);
    hr_params_t p = make_fusion_params(0.5f, 20.0f);

    hr_fusion_ctx_t ctx_rest;
    hr_fusion_init(&ctx_rest);
    hr_fusion_run(&ctx_rest, &cands, HR_STATE_ACQUIRE,
                  MOTION_STATE_REST, &p, NULL);

    hr_fusion_ctx_t ctx_run;
    hr_fusion_init(&ctx_run);
    hr_fusion_run(&ctx_run, &cands, HR_STATE_ACQUIRE,
                  MOTION_STATE_RUN, &p, NULL);

    ASSERT_TRUE(ctx_rest.result.valid);
    ASSERT_TRUE(ctx_run.result.valid);
    ASSERT_FLOAT_NEAR(ctx_rest.result.hr_bpm, ctx_run.result.hr_bpm, 1e-9f);
    ASSERT_INT_EQ(ctx_rest.result.selected_index,
                  ctx_run.result.selected_index);
    ASSERT_INT_EQ(ctx_rest.result.selected_source,
                  ctx_run.result.selected_source);
    ASSERT_FLOAT_NEAR(ctx_rest.result.selected_score,
                      ctx_run.result.selected_score, 1e-9f);

    return 0;
}

/* ================================================================== */
/*  [F] Pred seed semantics                                            */
/* ================================================================== */

static int test_fusion_track_updates_pred_seed_after_stable(void)
{
    hr_fusion_ctx_t ctx;
    hr_fusion_init(&ctx);

    hr_candidate_ctx_t cand_ctx;
    hr_candidate_init(&cand_ctx);

    hr_candidate_result_t cands = make_all_invalid();
    set_slot_valid(&cands, 0, 75.0f, 0.8f);

    hr_params_t p = make_fusion_params(0.3f, 20.0f);

    /* Run 3 cycles under TRACK with valid output each time.
     * stable_count: 0→1 (run1), 1→2 (run2), 2→3 (run3).
     * Pred seed threshold = 3, so seed should be set after run 3. */

    hr_fusion_run(&ctx, &cands, HR_STATE_TRACK,
                  MOTION_STATE_REST, &p, &cand_ctx);
    ASSERT_INT_EQ(ctx.stable_count, 1);
    ASSERT_FALSE(cand_ctx.pred_seed_valid); /* 1 < 3 */

    hr_fusion_run(&ctx, &cands, HR_STATE_TRACK,
                  MOTION_STATE_REST, &p, &cand_ctx);
    ASSERT_INT_EQ(ctx.stable_count, 2);
    ASSERT_FALSE(cand_ctx.pred_seed_valid); /* 2 < 3 */

    hr_fusion_run(&ctx, &cands, HR_STATE_TRACK,
                  MOTION_STATE_REST, &p, &cand_ctx);
    ASSERT_INT_EQ(ctx.stable_count, 3);
    ASSERT_TRUE(cand_ctx.pred_seed_valid);  /* 3 >= 3 */

    /* Seed BPM must be the smoothed final hr_bpm, NOT raw_selected_bpm */
    ASSERT_FLOAT_NEAR(cand_ctx.pred_seed_bpm, ctx.result.hr_bpm, 1e-6f);

    return 0;
}

static int test_fusion_init_clears_pred_seed(void)
{
    hr_fusion_ctx_t ctx;
    hr_fusion_init(&ctx);

    hr_candidate_ctx_t cand_ctx;
    hr_candidate_init(&cand_ctx);
    cand_ctx.pred_seed_valid = true;
    cand_ctx.pred_seed_bpm   = 99.0f;

    hr_candidate_result_t cands = make_all_invalid();
    set_slot_valid(&cands, 0, 80.0f, 0.9f);
    hr_params_t p = make_fusion_params(0.3f, 20.0f);

    hr_fusion_run(&ctx, &cands, HR_STATE_INIT,
                  MOTION_STATE_REST, &p, &cand_ctx);

    ASSERT_FALSE(cand_ctx.pred_seed_valid);
    return 0;
}

static int test_fusion_noninit_preserves_pred_seed(void)
{
    /*
     * In states other than INIT, when set-seed conditions are not met,
     * real code preserves the existing seed (does not clear it).
     */
    hr_source_t states[] = {
        (hr_source_t)HR_STATE_ACQUIRE,
        (hr_source_t)HR_STATE_HOLDOVER,
        (hr_source_t)HR_STATE_REACQUIRE
    };

    for (int s = 0; s < 3; s++) {
        hr_fusion_ctx_t ctx;
        hr_fusion_init(&ctx);

        hr_candidate_ctx_t cand_ctx;
        hr_candidate_init(&cand_ctx);
        cand_ctx.pred_seed_valid = true;
        cand_ctx.pred_seed_bpm   = 77.0f;

        hr_candidate_result_t cands = make_all_invalid();
        set_slot_valid(&cands, 0, 80.0f, 0.8f);
        hr_params_t p = make_fusion_params(0.3f, 20.0f);

        hr_fusion_run(&ctx, &cands, (hr_state_t)states[s],
                      MOTION_STATE_REST, &p, &cand_ctx);

        ASSERT_TRUE(ctx.result.valid);
        /* Seed should be preserved (not cleared, not overwritten) */
        ASSERT_TRUE(cand_ctx.pred_seed_valid);
        ASSERT_FLOAT_NEAR(cand_ctx.pred_seed_bpm, 77.0f, 1e-6f);
    }

    /* Also test TRACK with stable_count < threshold (no overwrite) */
    {
        hr_fusion_ctx_t ctx;
        hr_fusion_init(&ctx);

        hr_candidate_ctx_t cand_ctx;
        hr_candidate_init(&cand_ctx);
        cand_ctx.pred_seed_valid = true;
        cand_ctx.pred_seed_bpm   = 77.0f;

        hr_candidate_result_t cands = make_all_invalid();
        set_slot_valid(&cands, 0, 80.0f, 0.8f);
        hr_params_t p = make_fusion_params(0.3f, 20.0f);

        /* stable_count starts at 0, after run becomes 1 — below threshold */
        hr_fusion_run(&ctx, &cands, HR_STATE_TRACK,
                      MOTION_STATE_REST, &p, &cand_ctx);

        ASSERT_INT_EQ(ctx.stable_count, 1);
        /* Seed should still be preserved (1 < 3) */
        ASSERT_TRUE(cand_ctx.pred_seed_valid);
        ASSERT_FLOAT_NEAR(cand_ctx.pred_seed_bpm, 77.0f, 1e-6f);
    }

    return 0;
}

static int test_fusion_null_candidate_ctx_safe(void)
{
    hr_fusion_ctx_t ctx;
    hr_fusion_init(&ctx);

    hr_candidate_result_t cands = make_all_invalid();
    set_slot_valid(&cands, 0, 75.0f, 0.8f);
    hr_params_t p = make_fusion_params(0.3f, 20.0f);

    /* candidate_ctx = NULL: must not crash, fusion result still valid */
    hr_fusion_run(&ctx, &cands, HR_STATE_TRACK,
                  MOTION_STATE_REST, &p, NULL);

    ASSERT_TRUE(ctx.result.valid);
    ASSERT_TRUE(isfinite(ctx.result.hr_bpm));
    ASSERT_TRUE(ctx.result.hr_bpm > 0.0f);
    return 0;
}

/* ================================================================== */
/*  [G] Multi-cycle & robustness                                       */
/* ================================================================== */

static int test_fusion_stable_count_increments_and_resets(void)
{
    hr_fusion_ctx_t ctx;
    hr_fusion_init(&ctx);
    hr_params_t p = make_fusion_params(0.3f, 20.0f);

    hr_candidate_result_t valid_cands = make_all_invalid();
    set_slot_valid(&valid_cands, 0, 75.0f, 0.7f);
    hr_candidate_result_t invalid_cands = make_all_invalid();

    /* 3 valid cycles: stable_count 0→1→2→3 */
    hr_fusion_run(&ctx, &valid_cands, HR_STATE_TRACK,
                  MOTION_STATE_REST, &p, NULL);
    ASSERT_INT_EQ(ctx.stable_count, 1);

    hr_fusion_run(&ctx, &valid_cands, HR_STATE_TRACK,
                  MOTION_STATE_REST, &p, NULL);
    ASSERT_INT_EQ(ctx.stable_count, 2);

    hr_fusion_run(&ctx, &valid_cands, HR_STATE_TRACK,
                  MOTION_STATE_REST, &p, NULL);
    ASSERT_INT_EQ(ctx.stable_count, 3);

    /* 1 invalid cycle: resets to 0 */
    hr_fusion_run(&ctx, &invalid_cands, HR_STATE_TRACK,
                  MOTION_STATE_REST, &p, NULL);
    ASSERT_INT_EQ(ctx.stable_count, 0);

    /* Valid again: restarts from 1 */
    hr_fusion_run(&ctx, &valid_cands, HR_STATE_TRACK,
                  MOTION_STATE_REST, &p, NULL);
    ASSERT_INT_EQ(ctx.stable_count, 1);

    return 0;
}

static int test_fusion_repeated_run_no_nan(void)
{
    hr_fusion_ctx_t ctx;
    hr_fusion_init(&ctx);
    hr_params_t p = make_fusion_params(0.3f, 15.0f);

    hr_candidate_result_t cands = make_all_invalid();
    set_slot_valid(&cands, 0, 72.0f, 0.7f);
    set_slot_valid(&cands, 2, 78.0f, 0.6f);

    for (int i = 0; i < 100; i++) {
        /* Alternate between TRACK and HOLDOVER */
        hr_state_t st = (i % 5 < 3) ? HR_STATE_TRACK : HR_STATE_HOLDOVER;

        hr_fusion_run(&ctx, &cands, st, MOTION_STATE_WALK, &p, NULL);

        ASSERT_TRUE(ctx.result.valid);
        ASSERT_TRUE(isfinite(ctx.result.hr_bpm));
        ASSERT_TRUE(ctx.result.hr_bpm > 0.0f);
    }
    return 0;
}

/* ================================================================== */
/*  main — test runner                                                 */
/* ================================================================== */

int main(void)
{
    printf("==========================================\n");
    printf("  M7 Fusion & Smoothing -- Test Suite\n");
    printf("==========================================\n");

    printf("\n[A] Contract tests:\n");
    RUN_TEST(test_fusion_init_state);
    RUN_TEST(test_fusion_reset_state);
    RUN_TEST(test_fusion_basic_run_contract);
    RUN_TEST(test_fusion_invalid_args);

    printf("\n[B] INIT & all-invalid candidates:\n");
    RUN_TEST(test_fusion_init_blocks_output_and_clears_seed);
    RUN_TEST(test_fusion_all_invalid_candidates);
    RUN_TEST(test_fusion_all_invalid_preserves_prev_hr);

    printf("\n[C] Candidate ranking & history consistency:\n");
    RUN_TEST(test_fusion_selects_highest_score_without_history);
    RUN_TEST(test_fusion_history_consistency_affects_ranking);
    RUN_TEST(test_fusion_tie_breaks_by_lower_index);

    printf("\n[D] Jump limit + EMA:\n");
    RUN_TEST(test_fusion_first_valid_not_smoothed);
    RUN_TEST(test_fusion_clamp_then_ema);
    RUN_TEST(test_fusion_alpha_edge_cases);
    RUN_TEST(test_fusion_jump_threshold_zero_skips_clamp);

    printf("\n[E] State-aware behaviour:\n");
    RUN_TEST(test_fusion_holdover_biases_toward_history);
    RUN_TEST(test_fusion_motion_state_currently_unused);

    printf("\n[F] Pred seed semantics:\n");
    RUN_TEST(test_fusion_track_updates_pred_seed_after_stable);
    RUN_TEST(test_fusion_init_clears_pred_seed);
    RUN_TEST(test_fusion_noninit_preserves_pred_seed);
    RUN_TEST(test_fusion_null_candidate_ctx_safe);

    printf("\n[G] Multi-cycle & robustness:\n");
    RUN_TEST(test_fusion_stable_count_increments_and_resets);
    RUN_TEST(test_fusion_repeated_run_no_nan);

    TEST_SUMMARY();
    return TEST_EXIT_CODE();
}
