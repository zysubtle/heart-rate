/**
 * @file test_hr_statemachine.c
 * @brief M8 State Machine — test suite.
 *
 * Tests the real implementation in src/hr_statemachine.c against:
 *   [A] Contract tests:          init / reset / invalid args / basic update
 *   [B] INIT / ACQUIRE semantics: cold-start transition, data_ready gating
 *   [C] Good / neutral / poor tier: three-tier candidate quality model
 *   [D] TRACK → HOLDOVER:        trigger condition + holdover_count
 *   [E] HOLDOVER → REACQUIRE:    signal recovery vs timeout, signal_recovered
 *   [F] REACQUIRE → TRACK:       stable-count accumulation, count isolation
 *   [G] Motion boost & fusion_hist: motion_score_boost, fusion_hist unused
 *   [H] Multi-cycle & robustness: full state cycle, repeated-run stability
 *
 * All inputs are directly constructed — no upstream M1–M7 pipeline needed.
 *
 * Test framework: tests/test_framework.h (project-local, zero deps).
 *
 * ---------------------------------------------------------------
 * REAL implementation constants mirrored for expected-value tests
 * (read from src/hr_statemachine.c, not from any public API):
 *
 *   SM_SCORE_GOOD           = 0.3f
 *   SM_SCORE_POOR           = 0.15f
 *   SM_MOTION_SCORE_BOOST   = 0.1f
 * ---------------------------------------------------------------
 */

#include "test_framework.h"
#include "hr_statemachine.h"

#include <string.h>
#include <math.h>

/* ================================================================== */
/*  Mirror of real design constants for expected-value computation.    */
/*  If M8 changes these, update here and re-verify.                   */
/* ================================================================== */

#define T_SCORE_GOOD           0.3f
#define T_SCORE_POOR           0.15f
#define T_MOTION_SCORE_BOOST   0.1f

/* ================================================================== */
/*  Test helpers — construct inputs directly, no upstream pipeline     */
/* ================================================================== */

/** Build hr_params_t with explicit statemachine and SQI fields. */
static hr_params_t make_sm_params(uint8_t acquire_cnt,
                                  uint8_t holdover_max,
                                  uint8_t reacquire_cnt,
                                  float   sqi_poor_thr)
{
    hr_params_t p;
    memset(&p, 0, sizeof(p));
    p.statemachine.acquire_stable_count   = acquire_cnt;
    p.statemachine.holdover_max_count     = holdover_max;
    p.statemachine.reacquire_stable_count = reacquire_cnt;
    p.sqi.sqi_poor_threshold              = sqi_poor_thr;
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

/** Build a candidate result where all 7 are invalid. */
static hr_candidate_result_t make_all_invalid_cands(void)
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
 * Set a specific candidate slot [0..6] to valid with given bpm/score.
 * Also sets any_valid = true.
 */
static void set_cand_valid(hr_candidate_result_t *r,
                           uint8_t slot, float bpm, float score)
{
    hr_source_t src_map[] = {
        HR_SOURCE_PEAK_RAW, HR_SOURCE_PEAK_MAC,
        HR_SOURCE_FFT_RAW,  HR_SOURCE_FFT_MAC,
        HR_SOURCE_ACF_RAW,  HR_SOURCE_ACF_MAC,
        HR_SOURCE_PRED
    };
    hr_candidate_t c;
    c.valid  = true;
    c.bpm    = bpm;
    c.score  = score;
    c.source = src_map[slot];

    switch (slot) {
    case 0: r->cand_peak_raw = c; break;
    case 1: r->cand_peak_mac = c; break;
    case 2: r->cand_fft_raw  = c; break;
    case 3: r->cand_fft_mac  = c; break;
    case 4: r->cand_acf_raw  = c; break;
    case 5: r->cand_acf_mac  = c; break;
    case 6: r->cand_pred     = c; break;
    default: break;
    }
    r->any_valid = true;
}

/** Build a "good-tier" candidate result: any_valid + high score. */
static hr_candidate_result_t make_good_cands(float score)
{
    hr_candidate_result_t r = make_all_invalid_cands();
    set_cand_valid(&r, 2, 75.0f, score);  /* fft_raw */
    return r;
}

/** Build sqi_result_t with the specified sqi_main value. */
static sqi_result_t make_sqi(float sqi_main)
{
    sqi_result_t s;
    memset(&s, 0, sizeof(s));
    s.sqi_main = sqi_main;
    s.main_ch  = 0;
    return s;
}

/** Build motion_result_t with the specified state. */
static motion_result_t make_motion(motion_state_t ms)
{
    motion_result_t m;
    memset(&m, 0, sizeof(m));
    m.state = ms;
    return m;
}

/* ================================================================== */
/*  Composite helper: drive state machine through INIT→ACQUIRE→TRACK  */
/*  so tests can start from TRACK without duplicating boilerplate.    */
/* ================================================================== */

/**
 * Drive ctx from INIT through ACQUIRE into TRACK.
 * Requires acquire_cnt good cycles after the INIT→ACQUIRE transition.
 */
static void drive_to_track(hr_sm_ctx_t *ctx, const hr_params_t *p)
{
    hr_candidate_result_t good = make_good_cands(0.8f);
    sqi_result_t          sqi  = make_sqi(0.8f);
    motion_result_t       mot  = make_motion(MOTION_STATE_REST);

    /* INIT → ACQUIRE */
    hr_sm_update(ctx, false, NULL, NULL, NULL, NULL, p);

    /* ACQUIRE → TRACK (acquire_stable_count good cycles) */
    for (uint8_t i = 0; i < p->statemachine.acquire_stable_count; i++) {
        hr_sm_update(ctx, true, &good, &sqi, &mot, NULL, p);
    }
}

/**
 * Drive ctx from TRACK into HOLDOVER by giving one poor-tier cycle.
 */
static void drive_to_holdover(hr_sm_ctx_t *ctx, const hr_params_t *p)
{
    hr_candidate_result_t bad = make_all_invalid_cands();
    sqi_result_t          sqi = make_sqi(0.1f);
    motion_result_t       mot = make_motion(MOTION_STATE_REST);
    hr_sm_update(ctx, true, &bad, &sqi, &mot, NULL, p);
}

/* ================================================================== */
/*  [A] Contract tests                                                 */
/* ================================================================== */

static int test_sm_init_state(void)
{
    hr_sm_ctx_t ctx;
    hr_sm_init(&ctx);

    ASSERT_TRUE(ctx.initialized);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_INIT);
    ASSERT_INT_EQ(ctx.prev_state, HR_STATE_INIT);
    ASSERT_FALSE(ctx.ever_ready);
    ASSERT_FALSE(ctx.result.data_ready);
    ASSERT_FALSE(ctx.result.candidate_usable);
    ASSERT_FLOAT_NEAR(ctx.result.best_candidate_score, 0.0f, 1e-9f);
    ASSERT_INT_EQ(ctx.result.acquire_good_count, 0);
    ASSERT_INT_EQ(ctx.result.holdover_count, 0);
    ASSERT_INT_EQ(ctx.result.reacquire_good_count, 0);
    ASSERT_FALSE(ctx.result.signal_recovered);
    return 0;
}

static int test_sm_reset_state(void)
{
    hr_sm_ctx_t ctx;
    hr_sm_init(&ctx);

    /* Pollute */
    ctx.result.state              = HR_STATE_TRACK;
    ctx.prev_state                = HR_STATE_ACQUIRE;
    ctx.ever_ready                = true;
    ctx.result.acquire_good_count = 5;
    ctx.result.holdover_count     = 8;
    ctx.result.signal_recovered   = true;

    hr_sm_reset(&ctx);

    ASSERT_TRUE(ctx.initialized);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_INIT);
    ASSERT_INT_EQ(ctx.prev_state, HR_STATE_INIT);
    ASSERT_FALSE(ctx.ever_ready);
    ASSERT_INT_EQ(ctx.result.acquire_good_count, 0);
    ASSERT_INT_EQ(ctx.result.holdover_count, 0);
    ASSERT_INT_EQ(ctx.result.reacquire_good_count, 0);
    ASSERT_FALSE(ctx.result.signal_recovered);
    return 0;
}

static int test_sm_invalid_args(void)
{
    hr_sm_ctx_t ctx;
    hr_sm_init(&ctx);
    hr_params_t p = make_sm_params(3, 10, 3, 0.3f);

    /* NULL ctx — must not crash */
    hr_sm_update(NULL, false, NULL, NULL, NULL, NULL, &p);

    /* NULL params — state must not change */
    hr_sm_update(&ctx, false, NULL, NULL, NULL, NULL, NULL);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_INIT);

    /* Uninitialized ctx — must not advance */
    hr_sm_ctx_t bad;
    memset(&bad, 0, sizeof(bad));
    bad.initialized = false;
    hr_sm_update(&bad, false, NULL, NULL, NULL, NULL, &p);
    ASSERT_INT_EQ(bad.result.state, HR_STATE_INIT);

    return 0;
}

/* ================================================================== */
/*  [B] INIT / ACQUIRE semantics                                       */
/* ================================================================== */

static int test_sm_init_to_acquire_on_first_update(void)
{
    hr_sm_ctx_t ctx;
    hr_sm_init(&ctx);
    hr_params_t p = make_sm_params(3, 10, 3, 0.3f);

    hr_sm_update(&ctx, false, NULL, NULL, NULL, NULL, &p);

    ASSERT_INT_EQ(ctx.prev_state, HR_STATE_INIT);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_ACQUIRE);
    ASSERT_INT_EQ(ctx.result.acquire_good_count, 0);
    ASSERT_FALSE(ctx.result.data_ready);
    return 0;
}

static int test_sm_acquire_stays_when_not_data_ready(void)
{
    hr_sm_ctx_t ctx;
    hr_sm_init(&ctx);
    hr_params_t p = make_sm_params(3, 10, 3, 0.3f);

    /* INIT → ACQUIRE */
    hr_sm_update(&ctx, false, NULL, NULL, NULL, NULL, &p);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_ACQUIRE);

    /* 5 more cycles with data_ready=false */
    for (int i = 0; i < 5; i++) {
        hr_sm_update(&ctx, false, NULL, NULL, NULL, NULL, &p);
        ASSERT_INT_EQ(ctx.result.state, HR_STATE_ACQUIRE);
        ASSERT_INT_EQ(ctx.result.acquire_good_count, 0);
    }
    ASSERT_FALSE(ctx.ever_ready);
    return 0;
}

static int test_sm_acquire_to_track_after_stable_count(void)
{
    hr_sm_ctx_t ctx;
    hr_sm_init(&ctx);
    hr_params_t p = make_sm_params(3, 10, 3, 0.3f);

    hr_candidate_result_t good = make_good_cands(0.8f);
    sqi_result_t          sqi  = make_sqi(0.8f);
    motion_result_t       mot  = make_motion(MOTION_STATE_REST);

    /* INIT → ACQUIRE */
    hr_sm_update(&ctx, false, NULL, NULL, NULL, NULL, &p);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_ACQUIRE);

    /* Cycle 1: data_ready + good → count = 1, still ACQUIRE */
    hr_sm_update(&ctx, true, &good, &sqi, &mot, NULL, &p);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_ACQUIRE);
    ASSERT_INT_EQ(ctx.result.acquire_good_count, 1);
    ASSERT_TRUE(ctx.result.candidate_usable);

    /* Cycle 2: count = 2, still ACQUIRE */
    hr_sm_update(&ctx, true, &good, &sqi, &mot, NULL, &p);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_ACQUIRE);
    ASSERT_INT_EQ(ctx.result.acquire_good_count, 2);

    /* Cycle 3: count = 3 >= acquire_stable_count → TRACK */
    hr_sm_update(&ctx, true, &good, &sqi, &mot, NULL, &p);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_TRACK);
    ASSERT_INT_EQ(ctx.result.acquire_good_count, 3);
    return 0;
}

static int test_sm_acquire_resets_count_on_bad_cycle(void)
{
    hr_sm_ctx_t ctx;
    hr_sm_init(&ctx);
    hr_params_t p = make_sm_params(3, 10, 3, 0.3f);

    hr_candidate_result_t good = make_good_cands(0.8f);
    hr_candidate_result_t bad  = make_all_invalid_cands();
    sqi_result_t          sqi  = make_sqi(0.8f);
    motion_result_t       mot  = make_motion(MOTION_STATE_REST);

    /* INIT → ACQUIRE */
    hr_sm_update(&ctx, false, NULL, NULL, NULL, NULL, &p);

    /* 2 good cycles */
    hr_sm_update(&ctx, true, &good, &sqi, &mot, NULL, &p);
    hr_sm_update(&ctx, true, &good, &sqi, &mot, NULL, &p);
    ASSERT_INT_EQ(ctx.result.acquire_good_count, 2);

    /* 1 bad cycle → count resets */
    hr_sm_update(&ctx, true, &bad, &sqi, &mot, NULL, &p);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_ACQUIRE);
    ASSERT_INT_EQ(ctx.result.acquire_good_count, 0);

    /* Must accumulate 3 fresh good cycles to enter TRACK */
    for (int i = 0; i < 3; i++) {
        hr_sm_update(&ctx, true, &good, &sqi, &mot, NULL, &p);
    }
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_TRACK);
    return 0;
}

/* ================================================================== */
/*  [C] Good / neutral / poor tier tests                               */
/*                                                                     */
/*  Real tier logic (from hr_statemachine.c):                          */
/*    good:    any_valid && best_score >= good_thr && sqi >= poor_thr  */
/*    poor:    !any_valid OR (score < 0.15 && sqi < poor_thr)          */
/*    neutral: everything in between                                   */
/* ================================================================== */

static int test_sm_good_tier(void)
{
    hr_sm_ctx_t ctx;
    hr_sm_init(&ctx);
    hr_params_t p = make_sm_params(1, 10, 1, 0.3f);

    /* score=0.5 > 0.3, sqi=0.6 > 0.3, REST → good */
    hr_candidate_result_t cands = make_good_cands(0.5f);
    sqi_result_t          sqi   = make_sqi(0.6f);
    motion_result_t       mot   = make_motion(MOTION_STATE_REST);

    /* INIT → ACQUIRE */
    hr_sm_update(&ctx, false, NULL, NULL, NULL, NULL, &p);
    /* ACQUIRE with good → should count */
    hr_sm_update(&ctx, true, &cands, &sqi, &mot, NULL, &p);

    ASSERT_TRUE(ctx.result.candidate_usable);
    ASSERT_FLOAT_NEAR(ctx.result.best_candidate_score, 0.5f, 1e-6f);
    return 0;
}

static int test_sm_poor_tier_no_valid(void)
{
    hr_sm_ctx_t ctx;
    hr_sm_init(&ctx);
    hr_params_t p = make_sm_params(1, 10, 1, 0.3f);

    drive_to_track(&ctx, &p);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_TRACK);

    /* All invalid → poor → TRACK should transition to HOLDOVER */
    hr_candidate_result_t bad = make_all_invalid_cands();
    sqi_result_t          sqi = make_sqi(0.8f);
    motion_result_t       mot = make_motion(MOTION_STATE_REST);

    hr_sm_update(&ctx, true, &bad, &sqi, &mot, NULL, &p);

    ASSERT_FALSE(ctx.result.candidate_usable);
    ASSERT_FLOAT_NEAR(ctx.result.best_candidate_score, 0.0f, 1e-9f);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_HOLDOVER);
    return 0;
}

static int test_sm_poor_tier_score_and_sqi_both_bad(void)
{
    hr_sm_ctx_t ctx;
    hr_sm_init(&ctx);
    hr_params_t p = make_sm_params(1, 10, 1, 0.3f);

    drive_to_track(&ctx, &p);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_TRACK);

    /* any_valid=true but score=0.1 < 0.15 AND sqi=0.2 < 0.3 → poor */
    hr_candidate_result_t cands = make_good_cands(0.1f);
    sqi_result_t          sqi   = make_sqi(0.2f);
    motion_result_t       mot   = make_motion(MOTION_STATE_REST);

    hr_sm_update(&ctx, true, &cands, &sqi, &mot, NULL, &p);

    ASSERT_INT_EQ(ctx.result.state, HR_STATE_HOLDOVER);
    return 0;
}

static int test_sm_neutral_tier_keeps_track(void)
{
    hr_sm_ctx_t ctx;
    hr_sm_init(&ctx);
    hr_params_t p = make_sm_params(1, 10, 1, 0.3f);

    drive_to_track(&ctx, &p);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_TRACK);

    /*
     * Neutral case 1: score=0.25 (< good=0.3 but >= poor=0.15) + sqi=0.5 (ok)
     * → not good (score too low), not poor (score >= 0.15) → neutral
     */
    hr_candidate_result_t cands = make_good_cands(0.25f);
    sqi_result_t          sqi   = make_sqi(0.5f);
    motion_result_t       mot   = make_motion(MOTION_STATE_REST);

    hr_sm_update(&ctx, true, &cands, &sqi, &mot, NULL, &p);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_TRACK);
    ASSERT_FALSE(ctx.result.candidate_usable);

    /*
     * Neutral case 2: score=0.1 (< 0.15) but sqi=0.5 (>= 0.3)
     * → not poor (because sqi is not < 0.3, poor needs BOTH bad)
     */
    cands = make_good_cands(0.1f);
    sqi   = make_sqi(0.5f);

    hr_sm_update(&ctx, true, &cands, &sqi, &mot, NULL, &p);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_TRACK);

    /*
     * Neutral case 3: score=0.5 (ok) but sqi=0.1 (< 0.3)
     * → not good (sqi too low), not poor (score >= 0.15)
     */
    cands = make_good_cands(0.5f);
    sqi   = make_sqi(0.1f);

    hr_sm_update(&ctx, true, &cands, &sqi, &mot, NULL, &p);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_TRACK);

    return 0;
}

/* ================================================================== */
/*  [D] TRACK → HOLDOVER                                               */
/* ================================================================== */

static int test_sm_track_to_holdover_on_poor(void)
{
    hr_sm_ctx_t ctx;
    hr_sm_init(&ctx);
    hr_params_t p = make_sm_params(1, 10, 1, 0.3f);

    drive_to_track(&ctx, &p);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_TRACK);

    drive_to_holdover(&ctx, &p);

    ASSERT_INT_EQ(ctx.result.state, HR_STATE_HOLDOVER);
    ASSERT_INT_EQ(ctx.result.holdover_count, 0);
    ASSERT_INT_EQ(ctx.prev_state, HR_STATE_TRACK);
    return 0;
}

static int test_sm_track_to_holdover_on_data_loss(void)
{
    hr_sm_ctx_t ctx;
    hr_sm_init(&ctx);
    hr_params_t p = make_sm_params(1, 10, 1, 0.3f);

    drive_to_track(&ctx, &p);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_TRACK);

    /* data_ready=false while in TRACK → HOLDOVER */
    hr_sm_update(&ctx, false, NULL, NULL, NULL, NULL, &p);

    ASSERT_INT_EQ(ctx.result.state, HR_STATE_HOLDOVER);
    ASSERT_INT_EQ(ctx.result.holdover_count, 0);
    return 0;
}

static int test_sm_holdover_count_increments(void)
{
    hr_sm_ctx_t ctx;
    hr_sm_init(&ctx);
    hr_params_t p = make_sm_params(1, 20, 1, 0.3f);

    drive_to_track(&ctx, &p);
    drive_to_holdover(&ctx, &p);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_HOLDOVER);
    ASSERT_INT_EQ(ctx.result.holdover_count, 0);

    /* Stay in HOLDOVER with poor input — count should increment each cycle */
    hr_candidate_result_t bad = make_all_invalid_cands();
    sqi_result_t          sqi = make_sqi(0.1f);
    motion_result_t       mot = make_motion(MOTION_STATE_REST);

    for (int i = 1; i <= 5; i++) {
        hr_sm_update(&ctx, true, &bad, &sqi, &mot, NULL, &p);
        ASSERT_INT_EQ(ctx.result.state, HR_STATE_HOLDOVER);
        ASSERT_INT_EQ(ctx.result.holdover_count, (uint8_t)i);
    }
    return 0;
}

/* ================================================================== */
/*  [E] HOLDOVER → REACQUIRE                                           */
/* ================================================================== */

static int test_sm_holdover_to_reacquire_on_recovery(void)
{
    hr_sm_ctx_t ctx;
    hr_sm_init(&ctx);
    hr_params_t p = make_sm_params(1, 20, 3, 0.3f);

    drive_to_track(&ctx, &p);
    drive_to_holdover(&ctx, &p);

    /* One poor cycle to increment holdover_count */
    hr_candidate_result_t bad = make_all_invalid_cands();
    sqi_result_t          sqi_bad = make_sqi(0.1f);
    motion_result_t       mot     = make_motion(MOTION_STATE_REST);
    hr_sm_update(&ctx, true, &bad, &sqi_bad, &mot, NULL, &p);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_HOLDOVER);
    ASSERT_INT_EQ(ctx.result.holdover_count, 1);

    /* Signal recovery: valid + score >= 0.3 + sqi >= 0.3 */
    hr_candidate_result_t good = make_good_cands(0.6f);
    sqi_result_t          sqi_ok = make_sqi(0.7f);
    hr_sm_update(&ctx, true, &good, &sqi_ok, &mot, NULL, &p);

    ASSERT_INT_EQ(ctx.result.state, HR_STATE_REACQUIRE);
    ASSERT_INT_EQ(ctx.result.reacquire_good_count, 1);
    ASSERT_TRUE(ctx.result.signal_recovered);
    ASSERT_INT_EQ(ctx.prev_state, HR_STATE_HOLDOVER);
    return 0;
}

static int test_sm_holdover_to_reacquire_on_timeout(void)
{
    hr_sm_ctx_t ctx;
    hr_sm_init(&ctx);
    hr_params_t p = make_sm_params(1, 5, 3, 0.3f);

    drive_to_track(&ctx, &p);
    drive_to_holdover(&ctx, &p);
    ASSERT_INT_EQ(ctx.result.holdover_count, 0);

    /* Stay in HOLDOVER for 5 cycles (holdover_max_count=5) */
    hr_candidate_result_t bad = make_all_invalid_cands();
    sqi_result_t          sqi = make_sqi(0.1f);
    motion_result_t       mot = make_motion(MOTION_STATE_REST);

    for (int i = 1; i <= 4; i++) {
        hr_sm_update(&ctx, true, &bad, &sqi, &mot, NULL, &p);
        ASSERT_INT_EQ(ctx.result.state, HR_STATE_HOLDOVER);
        ASSERT_INT_EQ(ctx.result.holdover_count, (uint8_t)i);
    }

    /* Cycle 5: holdover_count reaches 5 → REACQUIRE via timeout */
    hr_sm_update(&ctx, true, &bad, &sqi, &mot, NULL, &p);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_REACQUIRE);
    ASSERT_INT_EQ(ctx.result.holdover_count, 5);
    ASSERT_INT_EQ(ctx.result.reacquire_good_count, 0);
    ASSERT_FALSE(ctx.result.signal_recovered);
    return 0;
}

static int test_sm_signal_recovered_only_on_recovery_transition(void)
{
    hr_sm_ctx_t ctx;
    hr_sm_init(&ctx);
    hr_params_t p = make_sm_params(1, 20, 3, 0.3f);

    hr_candidate_result_t good = make_good_cands(0.8f);
    sqi_result_t          sqi  = make_sqi(0.8f);
    motion_result_t       mot  = make_motion(MOTION_STATE_REST);

    /* INIT → ACQUIRE */
    hr_sm_update(&ctx, false, NULL, NULL, NULL, NULL, &p);
    ASSERT_FALSE(ctx.result.signal_recovered);

    /* ACQUIRE → TRACK */
    hr_sm_update(&ctx, true, &good, &sqi, &mot, NULL, &p);
    ASSERT_FALSE(ctx.result.signal_recovered);

    /* TRACK → HOLDOVER */
    drive_to_holdover(&ctx, &p);
    ASSERT_FALSE(ctx.result.signal_recovered);

    /* HOLDOVER + recovery → REACQUIRE: this is the ONLY transition
     * where signal_recovered should be true */
    hr_sm_update(&ctx, true, &good, &sqi, &mot, NULL, &p);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_REACQUIRE);
    ASSERT_TRUE(ctx.result.signal_recovered);

    /* Next cycle in REACQUIRE — signal_recovered must be false */
    hr_sm_update(&ctx, true, &good, &sqi, &mot, NULL, &p);
    ASSERT_FALSE(ctx.result.signal_recovered);
    return 0;
}

static int test_sm_holdover_count_preserved_after_exit(void)
{
    hr_sm_ctx_t ctx;
    hr_sm_init(&ctx);
    hr_params_t p = make_sm_params(1, 20, 1, 0.3f);

    drive_to_track(&ctx, &p);
    drive_to_holdover(&ctx, &p);

    /* 3 cycles in HOLDOVER */
    hr_candidate_result_t bad = make_all_invalid_cands();
    sqi_result_t          sqi = make_sqi(0.1f);
    motion_result_t       mot = make_motion(MOTION_STATE_REST);
    for (int i = 0; i < 3; i++) {
        hr_sm_update(&ctx, true, &bad, &sqi, &mot, NULL, &p);
    }
    ASSERT_INT_EQ(ctx.result.holdover_count, 3);

    /* Recovery → REACQUIRE */
    hr_candidate_result_t good = make_good_cands(0.8f);
    sqi_result_t          sqi_ok = make_sqi(0.8f);
    hr_sm_update(&ctx, true, &good, &sqi_ok, &mot, NULL, &p);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_REACQUIRE);

    /* holdover_count must still be readable (preserved, not cleared) */
    ASSERT_INT_EQ(ctx.result.holdover_count, 4);

    /* Continue into TRACK (reacquire_stable_count=1 → immediate) */
    hr_sm_update(&ctx, true, &good, &sqi_ok, &mot, NULL, &p);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_TRACK);

    /* holdover_count still preserved for M9's O7 reference */
    ASSERT_TRUE(ctx.result.holdover_count >= 4);
    return 0;
}

/* ================================================================== */
/*  [F] REACQUIRE → TRACK                                              */
/* ================================================================== */

static int test_sm_reacquire_to_track_after_stable_count(void)
{
    hr_sm_ctx_t ctx;
    hr_sm_init(&ctx);
    hr_params_t p = make_sm_params(1, 5, 3, 0.3f);

    drive_to_track(&ctx, &p);
    drive_to_holdover(&ctx, &p);

    /* Timeout into REACQUIRE */
    hr_candidate_result_t bad = make_all_invalid_cands();
    sqi_result_t          sqi_bad = make_sqi(0.1f);
    motion_result_t       mot     = make_motion(MOTION_STATE_REST);
    for (int i = 0; i < 5; i++) {
        hr_sm_update(&ctx, true, &bad, &sqi_bad, &mot, NULL, &p);
    }
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_REACQUIRE);
    ASSERT_INT_EQ(ctx.result.reacquire_good_count, 0);

    /* Now give 3 good cycles (reacquire_stable_count=3) */
    hr_candidate_result_t good = make_good_cands(0.8f);
    sqi_result_t          sqi  = make_sqi(0.8f);

    hr_sm_update(&ctx, true, &good, &sqi, &mot, NULL, &p);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_REACQUIRE);
    ASSERT_INT_EQ(ctx.result.reacquire_good_count, 1);

    hr_sm_update(&ctx, true, &good, &sqi, &mot, NULL, &p);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_REACQUIRE);
    ASSERT_INT_EQ(ctx.result.reacquire_good_count, 2);

    hr_sm_update(&ctx, true, &good, &sqi, &mot, NULL, &p);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_TRACK);
    ASSERT_INT_EQ(ctx.result.reacquire_good_count, 3);
    return 0;
}

static int test_sm_reacquire_count_resets_on_bad_cycle(void)
{
    hr_sm_ctx_t ctx;
    hr_sm_init(&ctx);
    hr_params_t p = make_sm_params(1, 5, 3, 0.3f);

    drive_to_track(&ctx, &p);
    drive_to_holdover(&ctx, &p);

    /* Timeout into REACQUIRE */
    hr_candidate_result_t bad = make_all_invalid_cands();
    sqi_result_t          sqi_bad = make_sqi(0.1f);
    motion_result_t       mot     = make_motion(MOTION_STATE_REST);
    for (int i = 0; i < 5; i++) {
        hr_sm_update(&ctx, true, &bad, &sqi_bad, &mot, NULL, &p);
    }
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_REACQUIRE);

    /* 2 good cycles */
    hr_candidate_result_t good = make_good_cands(0.8f);
    sqi_result_t          sqi  = make_sqi(0.8f);
    hr_sm_update(&ctx, true, &good, &sqi, &mot, NULL, &p);
    hr_sm_update(&ctx, true, &good, &sqi, &mot, NULL, &p);
    ASSERT_INT_EQ(ctx.result.reacquire_good_count, 2);

    /* 1 bad cycle → count resets, stay REACQUIRE */
    hr_sm_update(&ctx, true, &bad, &sqi_bad, &mot, NULL, &p);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_REACQUIRE);
    ASSERT_INT_EQ(ctx.result.reacquire_good_count, 0);

    /* Need fresh 3 good cycles */
    for (int i = 0; i < 3; i++) {
        hr_sm_update(&ctx, true, &good, &sqi, &mot, NULL, &p);
    }
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_TRACK);
    return 0;
}

static int test_sm_acquire_reacquire_counts_independent(void)
{
    hr_sm_ctx_t ctx;
    hr_sm_init(&ctx);
    hr_params_t p = make_sm_params(2, 5, 3, 0.3f);

    hr_candidate_result_t good = make_good_cands(0.8f);
    sqi_result_t          sqi  = make_sqi(0.8f);
    motion_result_t       mot  = make_motion(MOTION_STATE_REST);

    /* INIT → ACQUIRE, accumulate 2 good → TRACK */
    hr_sm_update(&ctx, false, NULL, NULL, NULL, NULL, &p);
    hr_sm_update(&ctx, true, &good, &sqi, &mot, NULL, &p);
    hr_sm_update(&ctx, true, &good, &sqi, &mot, NULL, &p);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_TRACK);
    uint8_t acq_count = ctx.result.acquire_good_count;

    /* TRACK → HOLDOVER → (timeout) → REACQUIRE */
    drive_to_holdover(&ctx, &p);
    hr_candidate_result_t bad = make_all_invalid_cands();
    sqi_result_t          sqi_bad = make_sqi(0.1f);
    for (int i = 0; i < 5; i++) {
        hr_sm_update(&ctx, true, &bad, &sqi_bad, &mot, NULL, &p);
    }
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_REACQUIRE);

    /* reacquire_good_count starts at 0, independent of acquire_good_count */
    ASSERT_INT_EQ(ctx.result.reacquire_good_count, 0);

    /* Accumulate 1 good in REACQUIRE — should not be confused with acq */
    hr_sm_update(&ctx, true, &good, &sqi, &mot, NULL, &p);
    ASSERT_INT_EQ(ctx.result.reacquire_good_count, 1);
    /* acquire_good_count is a separate counter from a different state epoch */
    (void)acq_count;
    return 0;
}

/* ================================================================== */
/*  [G] Motion boost & fusion_hist                                     */
/* ================================================================== */

static int test_sm_motion_boost_raises_good_threshold(void)
{
    hr_params_t p = make_sm_params(1, 10, 1, 0.3f);

    /*
     * score = 0.35: above SM_SCORE_GOOD (0.3) but below 0.3+0.1=0.4
     * sqi = 0.5: above poor_threshold (0.3)
     *
     * REST: good_thr = 0.3 → 0.35 >= 0.3 → usable = true
     * RUN:  good_thr = 0.4 → 0.35 < 0.4  → usable = false
     */
    hr_candidate_result_t cands = make_good_cands(0.35f);
    sqi_result_t          sqi   = make_sqi(0.5f);

    /* REST → should be usable */
    {
        hr_sm_ctx_t ctx;
        hr_sm_init(&ctx);
        motion_result_t mot = make_motion(MOTION_STATE_REST);
        hr_sm_update(&ctx, false, NULL, NULL, NULL, NULL, &p);
        hr_sm_update(&ctx, true, &cands, &sqi, &mot, NULL, &p);
        ASSERT_TRUE(ctx.result.candidate_usable);
    }

    /* WALK → should be usable (no boost) */
    {
        hr_sm_ctx_t ctx;
        hr_sm_init(&ctx);
        motion_result_t mot = make_motion(MOTION_STATE_WALK);
        hr_sm_update(&ctx, false, NULL, NULL, NULL, NULL, &p);
        hr_sm_update(&ctx, true, &cands, &sqi, &mot, NULL, &p);
        ASSERT_TRUE(ctx.result.candidate_usable);
    }

    /* RUN → should NOT be usable (score < 0.4) */
    {
        hr_sm_ctx_t ctx;
        hr_sm_init(&ctx);
        motion_result_t mot = make_motion(MOTION_STATE_RUN);
        hr_sm_update(&ctx, false, NULL, NULL, NULL, NULL, &p);
        hr_sm_update(&ctx, true, &cands, &sqi, &mot, NULL, &p);
        ASSERT_FALSE(ctx.result.candidate_usable);
    }

    /* IRREGULAR → should NOT be usable (score < 0.4) */
    {
        hr_sm_ctx_t ctx;
        hr_sm_init(&ctx);
        motion_result_t mot = make_motion(MOTION_STATE_IRREGULAR);
        hr_sm_update(&ctx, false, NULL, NULL, NULL, NULL, &p);
        hr_sm_update(&ctx, true, &cands, &sqi, &mot, NULL, &p);
        ASSERT_FALSE(ctx.result.candidate_usable);
    }

    return 0;
}

static int test_sm_fusion_hist_currently_unused(void)
{
    hr_params_t p = make_sm_params(1, 10, 1, 0.3f);

    hr_candidate_result_t good = make_good_cands(0.8f);
    sqi_result_t          sqi  = make_sqi(0.8f);
    motion_result_t       mot  = make_motion(MOTION_STATE_REST);

    /* Run A: fusion_hist = NULL */
    hr_sm_ctx_t ctxA;
    hr_sm_init(&ctxA);
    hr_sm_update(&ctxA, false, NULL, NULL, NULL, NULL, &p);
    hr_sm_update(&ctxA, true, &good, &sqi, &mot, NULL, &p);

    /* Run B: fusion_hist = populated context */
    hr_fusion_ctx_t fh;
    memset(&fh, 0, sizeof(fh));
    fh.has_prev_hr  = true;
    fh.prev_hr_bpm  = 120.0f;
    fh.stable_count = 10;
    fh.initialized  = true;

    hr_sm_ctx_t ctxB;
    hr_sm_init(&ctxB);
    hr_sm_update(&ctxB, false, NULL, NULL, NULL, NULL, &p);
    hr_sm_update(&ctxB, true, &good, &sqi, &mot, &fh, &p);

    /* All outputs must be identical — fusion_hist is (void)-ed */
    ASSERT_INT_EQ(ctxA.result.state, ctxB.result.state);
    ASSERT_TRUE(ctxA.result.candidate_usable == ctxB.result.candidate_usable);
    ASSERT_FLOAT_NEAR(ctxA.result.best_candidate_score,
                      ctxB.result.best_candidate_score, 1e-9f);
    ASSERT_INT_EQ(ctxA.result.acquire_good_count,
                  ctxB.result.acquire_good_count);
    ASSERT_TRUE(ctxA.result.signal_recovered == ctxB.result.signal_recovered);
    return 0;
}

/* ================================================================== */
/*  [H] Multi-cycle & robustness                                       */
/* ================================================================== */

static int test_sm_full_state_cycle(void)
{
    hr_sm_ctx_t ctx;
    hr_sm_init(&ctx);
    hr_params_t p = make_sm_params(2, 3, 2, 0.3f);

    hr_candidate_result_t good = make_good_cands(0.8f);
    hr_candidate_result_t bad  = make_all_invalid_cands();
    sqi_result_t sqi_ok  = make_sqi(0.8f);
    sqi_result_t sqi_bad = make_sqi(0.1f);
    motion_result_t mot  = make_motion(MOTION_STATE_REST);

    /* 1: INIT → ACQUIRE */
    hr_sm_update(&ctx, false, NULL, NULL, NULL, NULL, &p);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_ACQUIRE);

    /* 2-3: ACQUIRE + 2 good → TRACK */
    hr_sm_update(&ctx, true, &good, &sqi_ok, &mot, NULL, &p);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_ACQUIRE);
    hr_sm_update(&ctx, true, &good, &sqi_ok, &mot, NULL, &p);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_TRACK);

    /* 4: TRACK + poor → HOLDOVER */
    hr_sm_update(&ctx, true, &bad, &sqi_bad, &mot, NULL, &p);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_HOLDOVER);

    /* 5-7: HOLDOVER for 3 cycles → timeout → REACQUIRE */
    for (int i = 0; i < 3; i++) {
        hr_sm_update(&ctx, true, &bad, &sqi_bad, &mot, NULL, &p);
    }
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_REACQUIRE);
    ASSERT_INT_EQ(ctx.result.holdover_count, 3);
    ASSERT_FALSE(ctx.result.signal_recovered);

    /* 8-9: REACQUIRE + 2 good → TRACK */
    hr_sm_update(&ctx, true, &good, &sqi_ok, &mot, NULL, &p);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_REACQUIRE);
    hr_sm_update(&ctx, true, &good, &sqi_ok, &mot, NULL, &p);
    ASSERT_INT_EQ(ctx.result.state, HR_STATE_TRACK);

    return 0;
}

static int test_sm_repeated_run_stability(void)
{
    hr_sm_ctx_t ctx;
    hr_sm_init(&ctx);
    hr_params_t p = make_sm_params(3, 10, 3, 0.3f);

    hr_candidate_result_t good = make_good_cands(0.7f);
    hr_candidate_result_t bad  = make_all_invalid_cands();
    sqi_result_t sqi_ok  = make_sqi(0.7f);
    sqi_result_t sqi_bad = make_sqi(0.1f);
    motion_result_t mot  = make_motion(MOTION_STATE_WALK);

    /* INIT → ACQUIRE */
    hr_sm_update(&ctx, false, NULL, NULL, NULL, NULL, &p);

    /* Run 200 cycles with alternating patterns */
    for (int i = 0; i < 200; i++) {
        bool use_good = (i % 7) < 5;
        if (use_good) {
            hr_sm_update(&ctx, true, &good, &sqi_ok, &mot, NULL, &p);
        } else {
            hr_sm_update(&ctx, true, &bad, &sqi_bad, &mot, NULL, &p);
        }

        /* Verify state is always a valid enum value */
        ASSERT_TRUE(ctx.result.state >= HR_STATE_INIT &&
                    ctx.result.state <= HR_STATE_REACQUIRE);
        ASSERT_TRUE(isfinite(ctx.result.best_candidate_score));
        ASSERT_TRUE(ctx.result.best_candidate_score >= 0.0f);
        ASSERT_TRUE(ctx.result.best_candidate_score <= 1.0f);
    }
    return 0;
}

static int test_sm_best_score_from_multiple_candidates(void)
{
    hr_sm_ctx_t ctx;
    hr_sm_init(&ctx);
    hr_params_t p = make_sm_params(1, 10, 1, 0.3f);

    hr_candidate_result_t cands = make_all_invalid_cands();
    set_cand_valid(&cands, 0, 70.0f, 0.2f);   /* peak_raw: 0.2 */
    set_cand_valid(&cands, 2, 80.0f, 0.55f);  /* fft_raw:  0.55 */
    set_cand_valid(&cands, 4, 75.0f, 0.4f);   /* acf_raw:  0.4 */

    sqi_result_t    sqi = make_sqi(0.8f);
    motion_result_t mot = make_motion(MOTION_STATE_REST);

    hr_sm_update(&ctx, false, NULL, NULL, NULL, NULL, &p);
    hr_sm_update(&ctx, true, &cands, &sqi, &mot, NULL, &p);

    /* best_candidate_score should be the max among valid: 0.55 */
    ASSERT_FLOAT_NEAR(ctx.result.best_candidate_score, 0.55f, 1e-6f);
    ASSERT_TRUE(ctx.result.candidate_usable);
    return 0;
}

/* ================================================================== */
/*  main — test runner                                                 */
/* ================================================================== */

int main(void)
{
    printf("==========================================\n");
    printf("  M8 State Machine -- Test Suite\n");
    printf("==========================================\n");

    printf("\n[A] Contract tests:\n");
    RUN_TEST(test_sm_init_state);
    RUN_TEST(test_sm_reset_state);
    RUN_TEST(test_sm_invalid_args);

    printf("\n[B] INIT / ACQUIRE semantics:\n");
    RUN_TEST(test_sm_init_to_acquire_on_first_update);
    RUN_TEST(test_sm_acquire_stays_when_not_data_ready);
    RUN_TEST(test_sm_acquire_to_track_after_stable_count);
    RUN_TEST(test_sm_acquire_resets_count_on_bad_cycle);

    printf("\n[C] Good / neutral / poor tier:\n");
    RUN_TEST(test_sm_good_tier);
    RUN_TEST(test_sm_poor_tier_no_valid);
    RUN_TEST(test_sm_poor_tier_score_and_sqi_both_bad);
    RUN_TEST(test_sm_neutral_tier_keeps_track);

    printf("\n[D] TRACK -> HOLDOVER:\n");
    RUN_TEST(test_sm_track_to_holdover_on_poor);
    RUN_TEST(test_sm_track_to_holdover_on_data_loss);
    RUN_TEST(test_sm_holdover_count_increments);

    printf("\n[E] HOLDOVER -> REACQUIRE:\n");
    RUN_TEST(test_sm_holdover_to_reacquire_on_recovery);
    RUN_TEST(test_sm_holdover_to_reacquire_on_timeout);
    RUN_TEST(test_sm_signal_recovered_only_on_recovery_transition);
    RUN_TEST(test_sm_holdover_count_preserved_after_exit);

    printf("\n[F] REACQUIRE -> TRACK:\n");
    RUN_TEST(test_sm_reacquire_to_track_after_stable_count);
    RUN_TEST(test_sm_reacquire_count_resets_on_bad_cycle);
    RUN_TEST(test_sm_acquire_reacquire_counts_independent);

    printf("\n[G] Motion boost & fusion_hist:\n");
    RUN_TEST(test_sm_motion_boost_raises_good_threshold);
    RUN_TEST(test_sm_fusion_hist_currently_unused);

    printf("\n[H] Multi-cycle & robustness:\n");
    RUN_TEST(test_sm_full_state_cycle);
    RUN_TEST(test_sm_repeated_run_stability);
    RUN_TEST(test_sm_best_score_from_multiple_candidates);

    TEST_SUMMARY();
    return TEST_EXIT_CODE();
}
