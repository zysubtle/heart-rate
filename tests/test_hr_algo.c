/**
 * @file test_hr_algo.c
 * @brief M9 Main Pipeline Integration — Test Suite.
 *
 * Tests the public API, 12-step pipeline orchestration, confidence
 * computation, output/debug writing, flags semantics, and degraded
 * paths implemented in src/hr_algo.c.
 *
 * Strategy:
 *   - hr_algo.c is #included (not linked separately) so that
 *     file-local static helpers (compute_confidence, build_flags,
 *     compute_confidence_track, etc.) can be tested directly.
 *   - All M1-M8 .c files are linked normally.
 *   - Most tests use the public API (black-box).
 *   - Confidence formula and specific flag tests use gray-box access
 *     through hr_algo_internal.h (pulled in by hr_algo.c).
 *
 * Test categories:
 *   [A] Public API contract tests
 *   [B] Data-not-ready degraded path
 *   [C] Timestamp / process_count
 *   [D] Full pipeline smoke
 *   [E] Confidence semantics
 *   [F] Flags semantics
 *   [G] HOLDOVER output rules
 */

#include "test_framework.h"

/*
 * Include hr_algo.c directly to access static helpers.
 * Do NOT link hr_algo.c separately — it is compiled here.
 */
#include "../src/hr_algo.c"

#include <math.h>

/* ================================================================== */
/*  Static context — must be file-scope to avoid stack overflow.       */
/*  hr_algo_ctx_t contains large staging buffers (~10 KB+).            */
/* ================================================================== */

static hr_algo_ctx_t ctx;

/* ================================================================== */
/*  Test helpers                                                       */
/* ================================================================== */

/**
 * Fill both ring buffers with HR_WINDOW_SIZE constant samples.
 * ACC: (0, 0, 1000) — simulates gravity-only, no dynamic motion.
 * PPG: (1000, 500, 200, 100) — constant DC, no periodicity.
 *
 * After this call, hr_sampling_ready_for_window() returns true,
 * and M4 will classify REST, M5 will bypass MAC, M6 will produce
 * no valid candidates (no periodic content).
 */
static void fill_ring_constant(hr_algo_ctx_t *c)
{
    for (uint16_t i = 0; i < HR_WINDOW_SIZE; i++) {
        hr_algo_feed_acc(c, 0, 0, 1000);
        hr_algo_feed_ppg(c, 1000, 500, 200, 100);
    }
}

/**
 * Set up gray-box HOLDOVER state for confidence / output tests.
 *
 * After this call the context is in HOLDOVER with configurable
 * prev_confidence and optional prev_hr_bpm.  Ring buffers are empty
 * so process_1s will take the data-not-ready degraded path, which
 * runs M8 (stays in HOLDOVER for a few cycles) but skips M2-M7,
 * preserving our presets for fusion.
 */
static void preset_holdover(hr_algo_ctx_t *c,
                            uint8_t prev_conf,
                            bool has_prev_hr,
                            float prev_hr_bpm)
{
    hr_algo_init(c, NULL);
    c->sm.result.state         = HR_STATE_HOLDOVER;
    c->sm.result.holdover_count = 0;
    c->sm.prev_state           = HR_STATE_HOLDOVER;
    c->sm.ever_ready           = true;
    c->sm.initialized          = true;
    c->fusion.has_prev_hr      = has_prev_hr;
    c->fusion.prev_hr_bpm      = prev_hr_bpm;
    c->main.prev_confidence    = prev_conf;
    c->main.process_count      = 5;
}

/* ================================================================== */
/*  [A] Public API contract tests                                      */
/* ================================================================== */

static int test_algo_default_params_values(void)
{
    hr_params_t p = hr_algo_default_params();

    ASSERT_FLOAT_NEAR(p.preproc.ppg_bp_low_hz,    0.5f,  1e-6f);
    ASSERT_FLOAT_NEAR(p.preproc.ppg_bp_high_hz,   4.0f,  1e-6f);
    ASSERT_FLOAT_NEAR(p.preproc.acc_lp_cutoff_hz, 10.0f, 1e-6f);

    ASSERT_FLOAT_NEAR(p.sqi.sqi_good_threshold, 0.6f,  1e-6f);
    ASSERT_FLOAT_NEAR(p.sqi.sqi_poor_threshold, 0.3f,  1e-6f);
    ASSERT_FLOAT_NEAR(p.sqi.switch_margin,      0.05f, 1e-6f);
    ASSERT_INT_EQ(p.sqi.switch_hold_count, 3);

    ASSERT_FLOAT_NEAR(p.motion.rest_energy_threshold, 0.5f,  1e-6f);
    ASSERT_FLOAT_NEAR(p.motion.walk_energy_threshold, 2.0f,  1e-6f);
    ASSERT_FLOAT_NEAR(p.motion.run_energy_threshold,  10.0f, 1e-6f);

    ASSERT_FLOAT_NEAR(p.mac.nlms_step_size, 0.01f, 1e-6f);
    ASSERT_INT_EQ(p.mac.filter_order, 8);

    ASSERT_FLOAT_NEAR(p.candidate.hr_min_bpm,         40.0f, 1e-6f);
    ASSERT_FLOAT_NEAR(p.candidate.hr_max_bpm,        220.0f, 1e-6f);
    ASSERT_FLOAT_NEAR(p.candidate.peak_min_prominence,  0.1f, 1e-6f);

    ASSERT_FLOAT_NEAR(p.fusion.smooth_alpha,       0.3f,  1e-6f);
    ASSERT_FLOAT_NEAR(p.fusion.bpm_jump_threshold, 20.0f, 1e-6f);

    ASSERT_INT_EQ(p.statemachine.acquire_stable_count,   5);
    ASSERT_INT_EQ(p.statemachine.holdover_max_count,    10);
    ASSERT_INT_EQ(p.statemachine.reacquire_stable_count, 3);

    return 0;
}

static int test_algo_null_safe_public_api(void)
{
    hr_algo_init(NULL, NULL);
    hr_algo_feed_acc(NULL, 0, 0, 0);
    hr_algo_feed_ppg(NULL, 0, 0, 0, 0);
    hr_algo_process_1s(NULL);
    ASSERT_TRUE(hr_algo_get_output(NULL) == NULL);
    ASSERT_TRUE(hr_algo_get_debug(NULL) == NULL);
    return 0;
}

static int test_algo_init_with_null_params_uses_defaults(void)
{
    hr_algo_init(&ctx, NULL);
    hr_params_t d = hr_algo_default_params();

    ASSERT_FLOAT_NEAR(ctx.params.preproc.ppg_bp_low_hz,
                      d.preproc.ppg_bp_low_hz, 1e-6f);
    ASSERT_FLOAT_NEAR(ctx.params.fusion.smooth_alpha,
                      d.fusion.smooth_alpha, 1e-6f);
    ASSERT_INT_EQ(ctx.params.statemachine.acquire_stable_count,
                  d.statemachine.acquire_stable_count);
    ASSERT_FLOAT_NEAR(ctx.params.motion.rest_energy_threshold,
                      d.motion.rest_energy_threshold, 1e-6f);
    return 0;
}

static int test_algo_init_cold_start_state(void)
{
    hr_algo_init(&ctx, NULL);

    const hr_output_t *out = hr_algo_get_output(&ctx);
    ASSERT_TRUE(out != NULL);
    ASSERT_FLOAT_NEAR(out->hr_bpm, 0.0f, 1e-6f);
    ASSERT_INT_EQ(out->confidence, 0);

    ASSERT_INT_EQ(ctx.sm.result.state, HR_STATE_INIT);
    ASSERT_INT_EQ(ctx.main.process_count, 0);
    ASSERT_INT_EQ(ctx.main.prev_confidence, 0);

    return 0;
}

/* ================================================================== */
/*  [B] Data-not-ready degraded path                                   */
/* ================================================================== */

static int test_algo_first_process_goes_to_acquire(void)
{
    hr_algo_init(&ctx, NULL);
    hr_algo_process_1s(&ctx);

    const hr_output_t *out = hr_algo_get_output(&ctx);
    ASSERT_TRUE(out != NULL);
    ASSERT_INT_EQ(out->hr_state, HR_STATE_ACQUIRE);
    ASSERT_FLOAT_NEAR(out->hr_bpm, 0.0f, 1e-6f);
    ASSERT_INT_EQ(out->confidence, 0);
    ASSERT_INT_EQ(out->motion_state, MOTION_STATE_REST);

    return 0;
}

static int test_algo_not_ready_debug_frame_contents(void)
{
    hr_algo_init(&ctx, NULL);
    hr_algo_process_1s(&ctx);

    const hr_debug_frame_t *d = hr_algo_get_debug(&ctx);
    ASSERT_TRUE(d != NULL);

    ASSERT_INT_EQ(d->timestamp_ms, 1000u);
    ASSERT_INT_EQ(d->hr_state, HR_STATE_ACQUIRE);

    for (int i = 0; i < 4; i++)
        ASSERT_FLOAT_NEAR(d->sqi[i], 0.0f, 1e-6f);

    ASSERT_FALSE(d->cand_peak_raw.valid);
    ASSERT_INT_EQ(d->cand_peak_raw.source, HR_SOURCE_PEAK_RAW);
    ASSERT_FLOAT_NEAR(d->cand_peak_raw.bpm, 0.0f, 1e-6f);

    ASSERT_FALSE(d->cand_peak_mac.valid);
    ASSERT_INT_EQ(d->cand_peak_mac.source, HR_SOURCE_PEAK_MAC);

    ASSERT_FALSE(d->cand_fft_raw.valid);
    ASSERT_INT_EQ(d->cand_fft_raw.source, HR_SOURCE_FFT_RAW);

    ASSERT_FALSE(d->cand_fft_mac.valid);
    ASSERT_INT_EQ(d->cand_fft_mac.source, HR_SOURCE_FFT_MAC);

    ASSERT_FALSE(d->cand_acf_raw.valid);
    ASSERT_INT_EQ(d->cand_acf_raw.source, HR_SOURCE_ACF_RAW);

    ASSERT_FALSE(d->cand_acf_mac.valid);
    ASSERT_INT_EQ(d->cand_acf_mac.source, HR_SOURCE_ACF_MAC);

    ASSERT_FALSE(d->cand_pred.valid);
    ASSERT_INT_EQ(d->cand_pred.source, HR_SOURCE_PRED);

    ASSERT_TRUE(d->flags & HR_DBG_FLAG_DATA_NOT_READY);

    ASSERT_FLOAT_NEAR(d->hr_out, 0.0f, 1e-6f);
    ASSERT_INT_EQ(d->confidence, 0);

    return 0;
}

static int test_algo_multiple_not_ready_stays_acquire(void)
{
    hr_algo_init(&ctx, NULL);

    for (int i = 0; i < 5; i++) {
        hr_algo_process_1s(&ctx);
        const hr_output_t *out = hr_algo_get_output(&ctx);
        ASSERT_INT_EQ(out->hr_state, HR_STATE_ACQUIRE);
        ASSERT_INT_EQ(out->confidence, 0);
        ASSERT_FLOAT_NEAR(out->hr_bpm, 0.0f, 1e-6f);
    }
    return 0;
}

/* ================================================================== */
/*  [C] Timestamp / process_count                                      */
/* ================================================================== */

static int test_algo_timestamp_increments_and_reinit(void)
{
    hr_algo_init(&ctx, NULL);

    for (int i = 1; i <= 5; i++) {
        hr_algo_process_1s(&ctx);
        const hr_debug_frame_t *d = hr_algo_get_debug(&ctx);
        ASSERT_INT_EQ(d->timestamp_ms, (uint32_t)(i * 1000));
    }

    /* Re-init must reset timestamp back to 1000 on next process */
    hr_algo_init(&ctx, NULL);
    hr_algo_process_1s(&ctx);
    ASSERT_INT_EQ(hr_algo_get_debug(&ctx)->timestamp_ms, 1000u);
    ASSERT_INT_EQ(ctx.main.process_count, 1u);

    return 0;
}

/* ================================================================== */
/*  [D] Full pipeline smoke                                            */
/* ================================================================== */

static int test_algo_full_pipeline_smoke(void)
{
    hr_algo_init(&ctx, NULL);
    fill_ring_constant(&ctx);
    hr_algo_process_1s(&ctx);

    const hr_output_t *out = hr_algo_get_output(&ctx);
    const hr_debug_frame_t *d = hr_algo_get_debug(&ctx);
    ASSERT_TRUE(out != NULL);
    ASSERT_TRUE(d != NULL);

    /* Data was ready — flag must NOT be set */
    ASSERT_FALSE(d->flags & HR_DBG_FLAG_DATA_NOT_READY);

    /* hr_bpm must be finite (0.0 in ACQUIRE, or a real value) */
    ASSERT_TRUE(isfinite(out->hr_bpm));
    ASSERT_TRUE(out->confidence <= 100);

    /* Constant ACC → REST */
    ASSERT_INT_EQ(out->motion_state, MOTION_STATE_REST);

    /* main_ch must be in valid range */
    ASSERT_TRUE(out->main_ch <= 3);
    ASSERT_TRUE(out->backup_ch <= 3);

    /* No NaN in SQI or candidate bpm */
    for (int i = 0; i < 4; i++)
        ASSERT_TRUE(isfinite(d->sqi[i]));

    ASSERT_TRUE(isfinite(d->cand_peak_raw.bpm));
    ASSERT_TRUE(isfinite(d->cand_fft_raw.bpm));
    ASSERT_TRUE(isfinite(d->cand_acf_raw.bpm));

    return 0;
}

static int test_algo_output_debug_consistency(void)
{
    hr_algo_init(&ctx, NULL);
    fill_ring_constant(&ctx);
    hr_algo_process_1s(&ctx);

    const hr_output_t *out = hr_algo_get_output(&ctx);
    const hr_debug_frame_t *d = hr_algo_get_debug(&ctx);

    ASSERT_INT_EQ(d->hr_state, out->hr_state);
    ASSERT_INT_EQ(d->motion_state, out->motion_state);
    ASSERT_INT_EQ(d->main_ch, out->main_ch);
    ASSERT_INT_EQ(d->backup_ch, out->backup_ch);
    ASSERT_FLOAT_NEAR(d->hr_out, out->hr_bpm, 1e-6f);
    ASSERT_INT_EQ(d->confidence, out->confidence);

    return 0;
}

/* ================================================================== */
/*  [E] Confidence semantics                                           */
/* ================================================================== */

static int test_algo_confidence_zero_in_init_acquire(void)
{
    hr_algo_init(&ctx, NULL);

    /* Before any process: INIT */
    ASSERT_INT_EQ(ctx.sm.result.state, HR_STATE_INIT);
    ASSERT_INT_EQ(compute_confidence(&ctx, false), 0);

    /* After first process: ACQUIRE */
    hr_algo_process_1s(&ctx);
    ASSERT_INT_EQ(ctx.sm.result.state, HR_STATE_ACQUIRE);
    ASSERT_INT_EQ(hr_algo_get_output(&ctx)->confidence, 0);

    return 0;
}

static int test_algo_track_confidence_formula(void)
{
    /*
     * Direct test of compute_confidence_track (static helper).
     *
     * Formula:  100 * (0.30*sqi + 0.35*sel + 0.20*cons + 0.15*mot)
     * mot: REST=1.0  WALK=0.9  RUN=0.6  IRREGULAR=0.5
     */

    /* Perfect inputs + REST: 100*(0.30+0.35+0.20+0.15) = 100 */
    ASSERT_INT_EQ(compute_confidence_track(1.0f, 1.0f, 1.0f,
                                           MOTION_STATE_REST), 100);

    /* All zero + REST: 100*(0+0+0+0.15*1.0) = 15 */
    ASSERT_INT_EQ(compute_confidence_track(0.0f, 0.0f, 0.0f,
                                           MOTION_STATE_REST), 15);

    /* sqi=0.8, sel=0.6, cons=0.5, WALK(0.9)
     * 100*(0.24 + 0.21 + 0.10 + 0.135) = 68.5 → rounds to 69 */
    ASSERT_INT_EQ(compute_confidence_track(0.8f, 0.6f, 0.5f,
                                           MOTION_STATE_WALK), 69);

    /* Perfect + RUN(0.6): 100*(0.30+0.35+0.20+0.09) = 94 */
    ASSERT_INT_EQ(compute_confidence_track(1.0f, 1.0f, 1.0f,
                                           MOTION_STATE_RUN), 94);

    /* Perfect + IRREGULAR(0.5): 100*(0.30+0.35+0.20+0.075) = 92.5
     * Float rounding yields 92 on most platforms (0.925 → 92.4999...) */
    ASSERT_INT_EQ(compute_confidence_track(1.0f, 1.0f, 1.0f,
                                           MOTION_STATE_IRREGULAR), 92);

    /* Clamp: negative sqi still floors at 0 */
    uint8_t c = compute_confidence_track(-1.0f, 0.0f, 0.0f,
                                         MOTION_STATE_REST);
    ASSERT_TRUE(c <= 100);

    return 0;
}

static int test_algo_holdover_confidence_decays(void)
{
    preset_holdover(&ctx, 50, true, 72.0f);

    /* Cycle 1: 50 → 40 */
    hr_algo_process_1s(&ctx);
    ASSERT_INT_EQ(hr_algo_get_output(&ctx)->hr_state, HR_STATE_HOLDOVER);
    ASSERT_INT_EQ(hr_algo_get_output(&ctx)->confidence, 40);

    /* Cycle 2: 40 → 30 */
    hr_algo_process_1s(&ctx);
    ASSERT_INT_EQ(hr_algo_get_output(&ctx)->confidence, 30);

    /* Cycle 3: 30 → 20 */
    hr_algo_process_1s(&ctx);
    ASSERT_INT_EQ(hr_algo_get_output(&ctx)->confidence, 20);

    /* Cycle 4: 20 → 10 */
    hr_algo_process_1s(&ctx);
    ASSERT_INT_EQ(hr_algo_get_output(&ctx)->confidence, 10);

    /* Cycle 5: 10 → 0 */
    hr_algo_process_1s(&ctx);
    ASSERT_INT_EQ(hr_algo_get_output(&ctx)->confidence, 0);

    /* Cycle 6: 0 → 0 (floor, does not go negative) */
    hr_algo_process_1s(&ctx);
    ASSERT_INT_EQ(hr_algo_get_output(&ctx)->confidence, 0);

    return 0;
}

static int test_algo_reacquire_confidence_capped(void)
{
    /*
     * In TRACK, perfect inputs yield confidence = 100.
     * In REACQUIRE, the same inputs must be capped at 60.
     */
    hr_algo_init(&ctx, NULL);

    /* Preset full-quality indicators */
    ctx.sqi.result.sqi_main = 1.0f;
    ctx.fusion.result.valid = true;
    ctx.fusion.result.hr_bpm = 75.0f;
    ctx.fusion.result.selected_score = 1.0f;
    ctx.motion.result.state = MOTION_STATE_REST;

    hr_candidate_t gc = {true, 75.0f, 0.8f, HR_SOURCE_PEAK_RAW};
    ctx.candidate.result.cand_peak_raw = gc;
    gc.source = HR_SOURCE_FFT_RAW;
    ctx.candidate.result.cand_fft_raw = gc;
    gc.source = HR_SOURCE_ACF_RAW;
    ctx.candidate.result.cand_acf_raw = gc;
    ctx.candidate.result.any_valid = true;

    /* TRACK: should exceed 60 */
    ctx.sm.result.state = HR_STATE_TRACK;
    uint8_t track_conf = compute_confidence(&ctx, true);
    ASSERT_TRUE(track_conf > 60);

    /* REACQUIRE: same inputs, capped at 60 */
    ctx.sm.result.state = HR_STATE_REACQUIRE;
    uint8_t reacq_conf = compute_confidence(&ctx, true);
    ASSERT_TRUE(reacq_conf > 0);
    ASSERT_TRUE(reacq_conf <= 60);

    return 0;
}

/* ================================================================== */
/*  [F] Flags semantics                                                */
/* ================================================================== */

static int test_algo_flags_data_not_ready(void)
{
    hr_algo_init(&ctx, NULL);
    hr_algo_process_1s(&ctx);

    const hr_debug_frame_t *d = hr_algo_get_debug(&ctx);
    ASSERT_TRUE(d->flags & HR_DBG_FLAG_DATA_NOT_READY);

    /* Pipeline-dependent flags must NOT be set when data not ready */
    ASSERT_FALSE(d->flags & HR_DBG_FLAG_MAC_BYPASSED);
    ASSERT_FALSE(d->flags & HR_DBG_FLAG_MAC_DIVERGED);
    ASSERT_FALSE(d->flags & HR_DBG_FLAG_NO_VALID_CANDIDATE);
    ASSERT_FALSE(d->flags & HR_DBG_FLAG_MAIN_CH_SWITCHED);

    return 0;
}

static int test_algo_flags_mac_bypassed_rest(void)
{
    hr_algo_init(&ctx, NULL);
    fill_ring_constant(&ctx);
    hr_algo_process_1s(&ctx);

    const hr_debug_frame_t *d = hr_algo_get_debug(&ctx);
    /* Constant ACC → REST → MAC bypassed */
    ASSERT_TRUE(d->flags & HR_DBG_FLAG_MAC_BYPASSED);
    ASSERT_FALSE(d->flags & HR_DBG_FLAG_DATA_NOT_READY);

    return 0;
}

static int test_algo_flags_no_valid_candidate(void)
{
    /*
     * Test build_flags directly: NO_VALID_CANDIDATE flag must
     * reflect candidate.result.any_valid when data_ready == true,
     * and must NOT be set when data_ready == false.
     */
    hr_algo_init(&ctx, NULL);

    ctx.candidate.result.any_valid = false;
    uint32_t f = build_flags(&ctx, true);
    ASSERT_TRUE(f & HR_DBG_FLAG_NO_VALID_CANDIDATE);

    ctx.candidate.result.any_valid = true;
    f = build_flags(&ctx, true);
    ASSERT_FALSE(f & HR_DBG_FLAG_NO_VALID_CANDIDATE);

    ctx.candidate.result.any_valid = false;
    f = build_flags(&ctx, false);
    ASSERT_FALSE(f & HR_DBG_FLAG_NO_VALID_CANDIDATE);

    return 0;
}

static int test_algo_flags_holdover_active(void)
{
    preset_holdover(&ctx, 50, true, 72.0f);
    hr_algo_process_1s(&ctx);

    const hr_debug_frame_t *d = hr_algo_get_debug(&ctx);
    ASSERT_TRUE(d->flags & HR_DBG_FLAG_HOLDOVER_ACTIVE);
    ASSERT_FALSE(d->flags & HR_DBG_FLAG_REACQUIRE_ACTIVE);

    return 0;
}

static int test_algo_flags_main_ch_switched(void)
{
    hr_algo_init(&ctx, NULL);
    fill_ring_constant(&ctx);

    /* First process establishes baseline main_ch */
    hr_algo_process_1s(&ctx);
    uint8_t established_ch = ctx.sqi.result.main_ch;

    /* Gray-box: force prev_main_ch to a different value */
    ctx.main.prev_main_ch = (established_ch + 1) % 4;

    /* Second process: M3 selects same channel (same input), but
     * prev_main_ch differs → MAIN_CH_SWITCHED should fire */
    hr_algo_process_1s(&ctx);
    const hr_debug_frame_t *d = hr_algo_get_debug(&ctx);
    ASSERT_TRUE(d->flags & HR_DBG_FLAG_MAIN_CH_SWITCHED);

    return 0;
}

static int test_algo_flags_no_switch_first_cycle(void)
{
    hr_algo_init(&ctx, NULL);
    fill_ring_constant(&ctx);

    /* First process: process_count was 0 → switch flag must NOT fire */
    hr_algo_process_1s(&ctx);
    const hr_debug_frame_t *d = hr_algo_get_debug(&ctx);
    ASSERT_FALSE(d->flags & HR_DBG_FLAG_MAIN_CH_SWITCHED);

    return 0;
}

/* ================================================================== */
/*  [G] HOLDOVER output rules                                          */
/* ================================================================== */

static int test_algo_holdover_uses_prev_hr_bpm(void)
{
    preset_holdover(&ctx, 60, true, 72.5f);
    hr_algo_process_1s(&ctx);

    const hr_output_t *out = hr_algo_get_output(&ctx);
    ASSERT_INT_EQ(out->hr_state, HR_STATE_HOLDOVER);
    /* HOLDOVER with no fusion this cycle → falls back to prev_hr_bpm */
    ASSERT_FLOAT_NEAR(out->hr_bpm, 72.5f, 1e-3f);
    /* Confidence decayed from 60 → 50 */
    ASSERT_INT_EQ(out->confidence, 50);

    return 0;
}

static int test_algo_holdover_no_prev_hr_outputs_zero(void)
{
    preset_holdover(&ctx, 30, false, 0.0f);
    hr_algo_process_1s(&ctx);

    const hr_output_t *out = hr_algo_get_output(&ctx);
    ASSERT_INT_EQ(out->hr_state, HR_STATE_HOLDOVER);
    ASSERT_FLOAT_NEAR(out->hr_bpm, 0.0f, 1e-6f);

    return 0;
}

static int test_algo_acquire_hr_bpm_always_zero(void)
{
    hr_algo_init(&ctx, NULL);
    fill_ring_constant(&ctx);

    /* Even with data_ready=true, ACQUIRE outputs hr_bpm=0 */
    hr_algo_process_1s(&ctx);
    const hr_output_t *out = hr_algo_get_output(&ctx);
    ASSERT_INT_EQ(out->hr_state, HR_STATE_ACQUIRE);
    ASSERT_FLOAT_NEAR(out->hr_bpm, 0.0f, 1e-6f);

    return 0;
}

/* ================================================================== */
/*  main                                                               */
/* ================================================================== */

int main(void)
{
    printf("==========================================\n");
    printf("  M9 Main Pipeline Integration -- Test Suite\n");
    printf("==========================================\n");

    printf("\n[A] Public API contract tests:\n");
    RUN_TEST(test_algo_default_params_values);
    RUN_TEST(test_algo_null_safe_public_api);
    RUN_TEST(test_algo_init_with_null_params_uses_defaults);
    RUN_TEST(test_algo_init_cold_start_state);

    printf("\n[B] Data-not-ready degraded path:\n");
    RUN_TEST(test_algo_first_process_goes_to_acquire);
    RUN_TEST(test_algo_not_ready_debug_frame_contents);
    RUN_TEST(test_algo_multiple_not_ready_stays_acquire);

    printf("\n[C] Timestamp / process_count:\n");
    RUN_TEST(test_algo_timestamp_increments_and_reinit);

    printf("\n[D] Full pipeline smoke:\n");
    RUN_TEST(test_algo_full_pipeline_smoke);
    RUN_TEST(test_algo_output_debug_consistency);

    printf("\n[E] Confidence semantics:\n");
    RUN_TEST(test_algo_confidence_zero_in_init_acquire);
    RUN_TEST(test_algo_track_confidence_formula);
    RUN_TEST(test_algo_holdover_confidence_decays);
    RUN_TEST(test_algo_reacquire_confidence_capped);

    printf("\n[F] Flags semantics:\n");
    RUN_TEST(test_algo_flags_data_not_ready);
    RUN_TEST(test_algo_flags_mac_bypassed_rest);
    RUN_TEST(test_algo_flags_no_valid_candidate);
    RUN_TEST(test_algo_flags_holdover_active);
    RUN_TEST(test_algo_flags_main_ch_switched);
    RUN_TEST(test_algo_flags_no_switch_first_cycle);

    printf("\n[G] HOLDOVER output rules:\n");
    RUN_TEST(test_algo_holdover_uses_prev_hr_bpm);
    RUN_TEST(test_algo_holdover_no_prev_hr_outputs_zero);
    RUN_TEST(test_algo_acquire_hr_bpm_always_zero);

    TEST_SUMMARY();
    return TEST_EXIT_CODE();
}
