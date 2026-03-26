/**
 * @file test_hr_candidate.c
 * @brief M6 Candidate Estimation — test suite.
 *
 * Tests the real implementation in src/hr_candidate.c against:
 *   [A] Contract tests:        init / reset / basic run
 *   [B] Invalid candidate & source semantics
 *   [C] raw / mac dual-branch semantics
 *   [D] Pred seed inject / clear semantics
 *   [E] Constructed periodic signal — basic viability
 *   [F] Score modulation (SQI & motion)
 *   [G] main_ch fallback & boundary conditions
 *
 * Constructed signals are deterministic and self-contained — no
 * external data files, no random seeds, no upstream M1–M5 pipeline.
 *
 * Test framework: tests/test_framework.h (project-local, zero deps).
 */

#include "test_framework.h"
#include "hr_candidate.h"

#include <string.h>
#include <math.h>

/* ================================================================== */
/*  Shared test helpers                                                */
/* ================================================================== */

#define FS          25.0f
#define N           HR_WINDOW_SIZE     /* 200 */
#define TEST_PI     3.14159265358979f

/** Build a minimal hr_params_t with sensible candidate defaults. */
static hr_params_t make_test_params(void)
{
    hr_params_t p;
    memset(&p, 0, sizeof(p));
    p.candidate.hr_min_bpm          = 40.0f;
    p.candidate.hr_max_bpm          = 220.0f;
    p.candidate.peak_min_prominence = 0.0f;
    return p;
}

/** Build a zeroed ppg_preproc_out_t. */
static ppg_preproc_out_t make_zero_ppg(void)
{
    ppg_preproc_out_t ppg;
    memset(&ppg, 0, sizeof(ppg));
    return ppg;
}

/** Build a zeroed ppg_mac_out_t. */
static ppg_mac_out_t make_zero_mac_out(void)
{
    ppg_mac_out_t m;
    memset(&m, 0, sizeof(m));
    return m;
}

/** Build a mac_result_t with valid = false. */
static mac_result_t make_mac_invalid(void)
{
    mac_result_t r;
    memset(&r, 0, sizeof(r));
    r.valid   = false;
    r.bypassed = true;
    return r;
}

/** Build a mac_result_t with valid = true. */
static mac_result_t make_mac_valid(void)
{
    mac_result_t r;
    memset(&r, 0, sizeof(r));
    r.valid        = true;
    r.main_ch_used = 0;
    return r;
}

/** Build default sqi_result_t: main_ch=0, sqi_main=0.8. */
static sqi_result_t make_sqi(uint8_t main_ch, float sqi_main)
{
    sqi_result_t s;
    memset(&s, 0, sizeof(s));
    s.main_ch  = main_ch;
    s.sqi_main = sqi_main;
    return s;
}

/** Build default motion_result_t. */
static motion_result_t make_motion(motion_state_t state)
{
    motion_result_t m;
    memset(&m, 0, sizeof(m));
    m.state = state;
    return m;
}

/**
 * Fill one PPG channel with a sine wave at the given BPM.
 * Amplitude 1.0, DC offset 0.0 (bandpass-like).
 */
static void fill_ppg_sine(ppg_preproc_out_t *ppg, uint8_t ch,
                          float bpm)
{
    float freq_hz = bpm / 60.0f;
    for (int i = 0; i < N; i++) {
        ppg->data[i][ch] = sinf(2.0f * TEST_PI * freq_hz
                                * (float)i / FS);
    }
}

/** Fill mac_out->data with a sine wave at the given BPM. */
static void fill_mac_sine(ppg_mac_out_t *mac_out, float bpm)
{
    float freq_hz = bpm / 60.0f;
    for (int i = 0; i < N; i++) {
        mac_out->data[i] = sinf(2.0f * TEST_PI * freq_hz
                                * (float)i / FS);
    }
}

/** Check that a candidate is strictly invalid per frozen contract. */
static int assert_cand_invalid(const hr_candidate_t *c,
                               hr_source_t expected_src)
{
    ASSERT_FALSE(c->valid);
    ASSERT_FLOAT_NEAR(c->bpm,   0.0f, 1e-9f);
    ASSERT_FLOAT_NEAR(c->score, 0.0f, 1e-9f);
    ASSERT_INT_EQ(c->source, expected_src);
    return 0;
}

/** Check that a candidate has a valid score in [0,1] and no NaN. */
static int assert_cand_score_range(const hr_candidate_t *c)
{
    if (c->valid) {
        ASSERT_TRUE(c->score >= 0.0f);
        ASSERT_TRUE(c->score <= 1.0f);
        ASSERT_TRUE(isfinite(c->bpm));
        ASSERT_TRUE(isfinite(c->score));
    }
    return 0;
}

/* Static ctx must live at file scope — too large for stack. */
static hr_candidate_ctx_t g_ctx;

/* ================================================================== */
/*  [A] Contract tests                                                 */
/* ================================================================== */

static int test_candidate_init_state(void)
{
    hr_candidate_init(&g_ctx);

    ASSERT_TRUE(g_ctx.initialized);
    ASSERT_FALSE(g_ctx.pred_seed_valid);
    ASSERT_FLOAT_NEAR(g_ctx.pred_seed_bpm, 0.0f, 1e-9f);
    ASSERT_FALSE(g_ctx.result.any_valid);

    /* Every candidate must be zeroed / invalid after init */
    const hr_candidate_t *c[7] = {
        &g_ctx.result.cand_peak_raw, &g_ctx.result.cand_peak_mac,
        &g_ctx.result.cand_fft_raw,  &g_ctx.result.cand_fft_mac,
        &g_ctx.result.cand_acf_raw,  &g_ctx.result.cand_acf_mac,
        &g_ctx.result.cand_pred
    };
    for (int i = 0; i < 7; i++) {
        ASSERT_FALSE(c[i]->valid);
        ASSERT_FLOAT_NEAR(c[i]->bpm,   0.0f, 1e-9f);
        ASSERT_FLOAT_NEAR(c[i]->score, 0.0f, 1e-9f);
    }
    return 0;
}

static int test_candidate_reset_state(void)
{
    hr_candidate_init(&g_ctx);

    /* Dirty the context */
    hr_candidate_set_pred_seed(&g_ctx, 80.0f);
    g_ctx.result.any_valid = true;

    hr_candidate_reset(&g_ctx);

    ASSERT_TRUE(g_ctx.initialized);
    ASSERT_FALSE(g_ctx.pred_seed_valid);
    ASSERT_FLOAT_NEAR(g_ctx.pred_seed_bpm, 0.0f, 1e-9f);
    ASSERT_FALSE(g_ctx.result.any_valid);
    return 0;
}

static int test_candidate_basic_run_contract(void)
{
    hr_candidate_init(&g_ctx);

    ppg_preproc_out_t ppg = make_zero_ppg();
    fill_ppg_sine(&ppg, 0, 75.0f);

    ppg_mac_out_t  mac_out  = make_zero_mac_out();
    mac_result_t   mac_r    = make_mac_invalid();
    sqi_result_t   sqi      = make_sqi(0, 0.8f);
    motion_result_t motion  = make_motion(MOTION_STATE_REST);
    hr_params_t    params   = make_test_params();

    hr_candidate_run(&g_ctx, &ppg, &mac_r, &mac_out,
                     &sqi, &motion, N, &params);

    /* 7 candidates exist and have defined fields */
    const hr_candidate_t *c[7] = {
        &g_ctx.result.cand_peak_raw, &g_ctx.result.cand_peak_mac,
        &g_ctx.result.cand_fft_raw,  &g_ctx.result.cand_fft_mac,
        &g_ctx.result.cand_acf_raw,  &g_ctx.result.cand_acf_mac,
        &g_ctx.result.cand_pred
    };
    for (int i = 0; i < 7; i++) {
        ASSERT_TRUE(isfinite(c[i]->bpm));
        ASSERT_TRUE(isfinite(c[i]->score));
        if (!c[i]->valid) {
            ASSERT_FLOAT_NEAR(c[i]->bpm,   0.0f, 1e-9f);
            ASSERT_FLOAT_NEAR(c[i]->score, 0.0f, 1e-9f);
        }
    }
    return 0;
}

/* ================================================================== */
/*  [B] Invalid candidate & source semantics                           */
/* ================================================================== */

static int test_candidate_invalid_fields_and_source(void)
{
    hr_candidate_init(&g_ctx);

    ppg_preproc_out_t ppg = make_zero_ppg();
    /* All-zero signal → should make most methods produce invalid */

    ppg_mac_out_t mac_out = make_zero_mac_out();
    mac_result_t  mac_r   = make_mac_invalid();
    sqi_result_t  sqi     = make_sqi(0, 0.5f);
    motion_result_t motion = make_motion(MOTION_STATE_REST);
    hr_params_t   params  = make_test_params();

    hr_candidate_run(&g_ctx, &ppg, &mac_r, &mac_out,
                     &sqi, &motion, N, &params);

    /* mac candidates must be invalid with correct source */
    if (assert_cand_invalid(&g_ctx.result.cand_peak_mac,
                            HR_SOURCE_PEAK_MAC)) return 1;
    if (assert_cand_invalid(&g_ctx.result.cand_fft_mac,
                            HR_SOURCE_FFT_MAC)) return 1;
    if (assert_cand_invalid(&g_ctx.result.cand_acf_mac,
                            HR_SOURCE_ACF_MAC)) return 1;

    /* pred must be invalid (no seed) */
    if (assert_cand_invalid(&g_ctx.result.cand_pred,
                            HR_SOURCE_PRED)) return 1;

    /* raw candidates on zero signal should also be invalid */
    ASSERT_INT_EQ(g_ctx.result.cand_peak_raw.source, HR_SOURCE_PEAK_RAW);
    ASSERT_INT_EQ(g_ctx.result.cand_fft_raw.source,  HR_SOURCE_FFT_RAW);
    ASSERT_INT_EQ(g_ctx.result.cand_acf_raw.source,  HR_SOURCE_ACF_RAW);

    return 0;
}

static int test_candidate_any_valid_false_when_all_invalid(void)
{
    hr_candidate_init(&g_ctx);

    ppg_preproc_out_t ppg = make_zero_ppg();
    ppg_mac_out_t mac_out = make_zero_mac_out();
    mac_result_t  mac_r   = make_mac_invalid();
    sqi_result_t  sqi     = make_sqi(0, 0.5f);
    motion_result_t motion = make_motion(MOTION_STATE_REST);
    hr_params_t   params  = make_test_params();

    hr_candidate_run(&g_ctx, &ppg, &mac_r, &mac_out,
                     &sqi, &motion, N, &params);

    /* All-zero raw + mac invalid + no pred seed → everything invalid */
    ASSERT_FALSE(g_ctx.result.any_valid);
    return 0;
}

static int test_candidate_any_valid_true_when_one_valid(void)
{
    hr_candidate_init(&g_ctx);

    ppg_preproc_out_t ppg = make_zero_ppg();
    fill_ppg_sine(&ppg, 0, 75.0f);

    ppg_mac_out_t  mac_out = make_zero_mac_out();
    mac_result_t   mac_r   = make_mac_invalid();
    sqi_result_t   sqi     = make_sqi(0, 0.8f);
    motion_result_t motion = make_motion(MOTION_STATE_REST);
    hr_params_t    params  = make_test_params();

    hr_candidate_run(&g_ctx, &ppg, &mac_r, &mac_out,
                     &sqi, &motion, N, &params);

    /* At least FFT or ACF should pick up the 75 BPM sine */
    ASSERT_TRUE(g_ctx.result.any_valid);
    return 0;
}

/* ================================================================== */
/*  [C] raw / mac dual-branch semantics                                */
/* ================================================================== */

static int test_candidate_raw_branch_runs_when_mac_invalid(void)
{
    hr_candidate_init(&g_ctx);

    ppg_preproc_out_t ppg = make_zero_ppg();
    fill_ppg_sine(&ppg, 0, 90.0f);

    ppg_mac_out_t mac_out = make_zero_mac_out();
    fill_mac_sine(&mac_out, 90.0f);   /* waveform present but invalid */

    mac_result_t   mac_r  = make_mac_invalid();   /* valid = false */
    sqi_result_t   sqi    = make_sqi(0, 0.9f);
    motion_result_t motion = make_motion(MOTION_STATE_REST);
    hr_params_t    params = make_test_params();

    hr_candidate_run(&g_ctx, &ppg, &mac_r, &mac_out,
                     &sqi, &motion, N, &params);

    /* Raw branch should have attempted: at least one raw valid */
    ASSERT_TRUE(g_ctx.result.cand_fft_raw.valid  ||
                g_ctx.result.cand_acf_raw.valid  ||
                g_ctx.result.cand_peak_raw.valid);

    /* All mac candidates MUST be invalid regardless of mac_out data */
    ASSERT_FALSE(g_ctx.result.cand_peak_mac.valid);
    ASSERT_FALSE(g_ctx.result.cand_fft_mac.valid);
    ASSERT_FALSE(g_ctx.result.cand_acf_mac.valid);

    return 0;
}

static int test_candidate_mac_branch_disabled_when_not_valid(void)
{
    hr_candidate_init(&g_ctx);

    ppg_preproc_out_t ppg = make_zero_ppg();
    fill_ppg_sine(&ppg, 0, 75.0f);

    ppg_mac_out_t mac_out = make_zero_mac_out();
    fill_mac_sine(&mac_out, 75.0f);

    mac_result_t mac_r;
    memset(&mac_r, 0, sizeof(mac_r));
    mac_r.valid    = false;
    mac_r.bypassed = false;
    mac_r.diverged = true;    /* diverged still means invalid */

    sqi_result_t   sqi    = make_sqi(0, 0.8f);
    motion_result_t motion = make_motion(MOTION_STATE_WALK);
    hr_params_t    params = make_test_params();

    hr_candidate_run(&g_ctx, &ppg, &mac_r, &mac_out,
                     &sqi, &motion, N, &params);

    /* MAC branch must remain off */
    ASSERT_FALSE(g_ctx.result.cand_peak_mac.valid);
    ASSERT_FALSE(g_ctx.result.cand_fft_mac.valid);
    ASSERT_FALSE(g_ctx.result.cand_acf_mac.valid);

    /* source tags still set correctly */
    ASSERT_INT_EQ(g_ctx.result.cand_peak_mac.source, HR_SOURCE_PEAK_MAC);
    ASSERT_INT_EQ(g_ctx.result.cand_fft_mac.source,  HR_SOURCE_FFT_MAC);
    ASSERT_INT_EQ(g_ctx.result.cand_acf_mac.source,  HR_SOURCE_ACF_MAC);

    return 0;
}

static int test_candidate_mac_branch_runs_when_valid(void)
{
    hr_candidate_init(&g_ctx);

    ppg_preproc_out_t ppg = make_zero_ppg();
    fill_ppg_sine(&ppg, 0, 90.0f);

    ppg_mac_out_t mac_out = make_zero_mac_out();
    fill_mac_sine(&mac_out, 90.0f);

    mac_result_t   mac_r  = make_mac_valid();
    sqi_result_t   sqi    = make_sqi(0, 0.9f);
    motion_result_t motion = make_motion(MOTION_STATE_WALK);
    hr_params_t    params = make_test_params();

    hr_candidate_run(&g_ctx, &ppg, &mac_r, &mac_out,
                     &sqi, &motion, N, &params);

    /* At least one mac candidate should now be valid */
    ASSERT_TRUE(g_ctx.result.cand_fft_mac.valid  ||
                g_ctx.result.cand_acf_mac.valid  ||
                g_ctx.result.cand_peak_mac.valid);

    /* Source tags must reflect mac */
    if (g_ctx.result.cand_fft_mac.valid)
        ASSERT_INT_EQ(g_ctx.result.cand_fft_mac.source, HR_SOURCE_FFT_MAC);
    if (g_ctx.result.cand_acf_mac.valid)
        ASSERT_INT_EQ(g_ctx.result.cand_acf_mac.source, HR_SOURCE_ACF_MAC);
    if (g_ctx.result.cand_peak_mac.valid)
        ASSERT_INT_EQ(g_ctx.result.cand_peak_mac.source,HR_SOURCE_PEAK_MAC);

    return 0;
}

static int test_candidate_raw_mac_source_no_cross(void)
{
    hr_candidate_init(&g_ctx);

    ppg_preproc_out_t ppg = make_zero_ppg();
    fill_ppg_sine(&ppg, 0, 80.0f);

    ppg_mac_out_t mac_out = make_zero_mac_out();
    fill_mac_sine(&mac_out, 100.0f);

    mac_result_t   mac_r  = make_mac_valid();
    sqi_result_t   sqi    = make_sqi(0, 0.9f);
    motion_result_t motion = make_motion(MOTION_STATE_REST);
    hr_params_t    params = make_test_params();

    hr_candidate_run(&g_ctx, &ppg, &mac_r, &mac_out,
                     &sqi, &motion, N, &params);

    /* Raw sources must be raw enum values */
    ASSERT_INT_EQ(g_ctx.result.cand_peak_raw.source, HR_SOURCE_PEAK_RAW);
    ASSERT_INT_EQ(g_ctx.result.cand_fft_raw.source,  HR_SOURCE_FFT_RAW);
    ASSERT_INT_EQ(g_ctx.result.cand_acf_raw.source,  HR_SOURCE_ACF_RAW);

    /* Mac sources must be mac enum values */
    ASSERT_INT_EQ(g_ctx.result.cand_peak_mac.source, HR_SOURCE_PEAK_MAC);
    ASSERT_INT_EQ(g_ctx.result.cand_fft_mac.source,  HR_SOURCE_FFT_MAC);
    ASSERT_INT_EQ(g_ctx.result.cand_acf_mac.source,  HR_SOURCE_ACF_MAC);

    ASSERT_INT_EQ(g_ctx.result.cand_pred.source, HR_SOURCE_PRED);

    return 0;
}

/* ================================================================== */
/*  [D] Pred seed semantics                                            */
/* ================================================================== */

static int test_candidate_pred_default_invalid(void)
{
    hr_candidate_init(&g_ctx);

    ppg_preproc_out_t ppg = make_zero_ppg();
    fill_ppg_sine(&ppg, 0, 75.0f);
    ppg_mac_out_t  mac_out = make_zero_mac_out();
    mac_result_t   mac_r   = make_mac_invalid();
    sqi_result_t   sqi     = make_sqi(0, 0.8f);
    motion_result_t motion = make_motion(MOTION_STATE_REST);
    hr_params_t    params  = make_test_params();

    hr_candidate_run(&g_ctx, &ppg, &mac_r, &mac_out,
                     &sqi, &motion, N, &params);

    ASSERT_FALSE(g_ctx.result.cand_pred.valid);
    ASSERT_FLOAT_NEAR(g_ctx.result.cand_pred.bpm,   0.0f, 1e-9f);
    ASSERT_FLOAT_NEAR(g_ctx.result.cand_pred.score, 0.0f, 1e-9f);
    ASSERT_INT_EQ(g_ctx.result.cand_pred.source, HR_SOURCE_PRED);

    return 0;
}

static int test_candidate_pred_seed_set_and_clear(void)
{
    hr_candidate_init(&g_ctx);

    ppg_preproc_out_t ppg = make_zero_ppg();
    fill_ppg_sine(&ppg, 0, 75.0f);
    ppg_mac_out_t  mac_out = make_zero_mac_out();
    mac_result_t   mac_r   = make_mac_invalid();
    sqi_result_t   sqi     = make_sqi(0, 0.8f);
    motion_result_t motion = make_motion(MOTION_STATE_REST);
    hr_params_t    params  = make_test_params();

    /* --- Set seed and run --- */
    hr_candidate_set_pred_seed(&g_ctx, 72.0f);

    hr_candidate_run(&g_ctx, &ppg, &mac_r, &mac_out,
                     &sqi, &motion, N, &params);

    ASSERT_TRUE(g_ctx.result.cand_pred.valid);
    ASSERT_FLOAT_NEAR(g_ctx.result.cand_pred.bpm, 72.0f, 1e-3f);
    ASSERT_TRUE(g_ctx.result.cand_pred.score > 0.0f);
    ASSERT_TRUE(g_ctx.result.cand_pred.score <= 1.0f);
    ASSERT_INT_EQ(g_ctx.result.cand_pred.source, HR_SOURCE_PRED);

    /* --- Clear seed and run again --- */
    hr_candidate_clear_pred_seed(&g_ctx);

    hr_candidate_run(&g_ctx, &ppg, &mac_r, &mac_out,
                     &sqi, &motion, N, &params);

    ASSERT_FALSE(g_ctx.result.cand_pred.valid);
    ASSERT_FLOAT_NEAR(g_ctx.result.cand_pred.bpm,   0.0f, 1e-9f);
    ASSERT_FLOAT_NEAR(g_ctx.result.cand_pred.score, 0.0f, 1e-9f);

    return 0;
}

static int test_candidate_pred_seed_out_of_range(void)
{
    hr_candidate_init(&g_ctx);

    ppg_preproc_out_t ppg = make_zero_ppg();
    fill_ppg_sine(&ppg, 0, 75.0f);
    ppg_mac_out_t  mac_out = make_zero_mac_out();
    mac_result_t   mac_r   = make_mac_invalid();
    sqi_result_t   sqi     = make_sqi(0, 0.8f);
    motion_result_t motion = make_motion(MOTION_STATE_REST);
    hr_params_t    params  = make_test_params();

    /* Seed below hr_min_bpm (40) */
    hr_candidate_set_pred_seed(&g_ctx, 20.0f);
    hr_candidate_run(&g_ctx, &ppg, &mac_r, &mac_out,
                     &sqi, &motion, N, &params);

    /* make_candidate does range check → should be invalid */
    ASSERT_FALSE(g_ctx.result.cand_pred.valid);

    /* Seed above hr_max_bpm (220) */
    hr_candidate_set_pred_seed(&g_ctx, 250.0f);
    hr_candidate_run(&g_ctx, &ppg, &mac_r, &mac_out,
                     &sqi, &motion, N, &params);

    ASSERT_FALSE(g_ctx.result.cand_pred.valid);

    return 0;
}

/* ================================================================== */
/*  [E] Constructed periodic signal — basic viability                  */
/* ================================================================== */

/**
 * Helper: run with a periodic raw signal, return the result.
 * Verifies at least one raw method produces a valid candidate
 * and the BPM is within tolerance.
 */
static int run_periodic_raw_check(float target_bpm, float bpm_tol)
{
    hr_candidate_init(&g_ctx);

    ppg_preproc_out_t ppg = make_zero_ppg();
    fill_ppg_sine(&ppg, 0, target_bpm);

    ppg_mac_out_t  mac_out = make_zero_mac_out();
    mac_result_t   mac_r   = make_mac_invalid();
    sqi_result_t   sqi     = make_sqi(0, 1.0f);
    motion_result_t motion = make_motion(MOTION_STATE_REST);
    hr_params_t    params  = make_test_params();

    hr_candidate_run(&g_ctx, &ppg, &mac_r, &mac_out,
                     &sqi, &motion, N, &params);

    /* At least one raw method should produce a valid candidate */
    int any_raw_valid = g_ctx.result.cand_peak_raw.valid ||
                        g_ctx.result.cand_fft_raw.valid  ||
                        g_ctx.result.cand_acf_raw.valid;
    ASSERT_TRUE(any_raw_valid);

    /* Check BPM accuracy for each valid raw candidate */
    const hr_candidate_t *raw[3] = {
        &g_ctx.result.cand_peak_raw,
        &g_ctx.result.cand_fft_raw,
        &g_ctx.result.cand_acf_raw
    };
    for (int i = 0; i < 3; i++) {
        if (raw[i]->valid) {
            ASSERT_TRUE(raw[i]->bpm >= target_bpm - bpm_tol);
            ASSERT_TRUE(raw[i]->bpm <= target_bpm + bpm_tol);
            if (assert_cand_score_range(raw[i])) return 1;
        }
    }
    return 0;
}

static int test_candidate_periodic_raw_75bpm(void)
{
    /* 75 BPM = 1.25 Hz.  DFT resolution ~5.9 BPM. */
    return run_periodic_raw_check(75.0f, 8.0f);
}

static int test_candidate_periodic_raw_120bpm(void)
{
    /* 120 BPM = 2.0 Hz. */
    return run_periodic_raw_check(120.0f, 8.0f);
}

static int test_candidate_periodic_mac_signal(void)
{
    hr_candidate_init(&g_ctx);

    ppg_preproc_out_t ppg = make_zero_ppg();
    fill_ppg_sine(&ppg, 0, 60.0f);

    ppg_mac_out_t mac_out = make_zero_mac_out();
    fill_mac_sine(&mac_out, 90.0f);

    mac_result_t   mac_r  = make_mac_valid();
    sqi_result_t   sqi    = make_sqi(0, 0.9f);
    motion_result_t motion = make_motion(MOTION_STATE_WALK);
    hr_params_t    params = make_test_params();

    hr_candidate_run(&g_ctx, &ppg, &mac_r, &mac_out,
                     &sqi, &motion, N, &params);

    /* At least one mac candidate should be valid near 90 BPM */
    int any_mac_valid = g_ctx.result.cand_peak_mac.valid ||
                        g_ctx.result.cand_fft_mac.valid  ||
                        g_ctx.result.cand_acf_mac.valid;
    ASSERT_TRUE(any_mac_valid);

    const hr_candidate_t *mc[3] = {
        &g_ctx.result.cand_peak_mac,
        &g_ctx.result.cand_fft_mac,
        &g_ctx.result.cand_acf_mac
    };
    for (int i = 0; i < 3; i++) {
        if (mc[i]->valid) {
            ASSERT_TRUE(mc[i]->bpm >= 82.0f && mc[i]->bpm <= 98.0f);
            if (assert_cand_score_range(mc[i])) return 1;
        }
    }
    return 0;
}

static int test_candidate_nonperiodic_low_or_invalid(void)
{
    hr_candidate_init(&g_ctx);

    ppg_preproc_out_t ppg = make_zero_ppg();
    /* Deterministic pseudo-noise: linear ramp with wrap — not periodic */
    for (int i = 0; i < N; i++) {
        float x = (float)(i * 137 % 200) / 200.0f - 0.5f;
        ppg.data[i][0] = x * 0.01f;
    }

    ppg_mac_out_t  mac_out = make_zero_mac_out();
    mac_result_t   mac_r   = make_mac_invalid();
    sqi_result_t   sqi     = make_sqi(0, 0.5f);
    motion_result_t motion = make_motion(MOTION_STATE_IRREGULAR);
    hr_params_t    params  = make_test_params();

    hr_candidate_run(&g_ctx, &ppg, &mac_r, &mac_out,
                     &sqi, &motion, N, &params);

    /* For any candidate that is valid, score should be low */
    const hr_candidate_t *c[3] = {
        &g_ctx.result.cand_peak_raw,
        &g_ctx.result.cand_fft_raw,
        &g_ctx.result.cand_acf_raw
    };
    int high_score_count = 0;
    for (int i = 0; i < 3; i++) {
        if (c[i]->valid && c[i]->score > 0.6f) {
            high_score_count++;
        }
    }
    /* Not all three should report high confidence on noise */
    ASSERT_TRUE(high_score_count < 3);

    /* No NaN / Inf */
    for (int i = 0; i < 3; i++) {
        ASSERT_TRUE(isfinite(c[i]->bpm));
        ASSERT_TRUE(isfinite(c[i]->score));
    }
    return 0;
}

/* ================================================================== */
/*  [F] Score modulation (SQI & motion)                                */
/* ================================================================== */

static int test_candidate_sqi_score_modulation(void)
{
    /* Same signal, different sqi_main → score difference */
    float scores_low_sqi[3];
    float scores_high_sqi[3];

    for (int pass = 0; pass < 2; pass++) {
        hr_candidate_init(&g_ctx);

        ppg_preproc_out_t ppg = make_zero_ppg();
        fill_ppg_sine(&ppg, 0, 80.0f);

        ppg_mac_out_t  mac_out = make_zero_mac_out();
        mac_result_t   mac_r   = make_mac_invalid();
        motion_result_t motion = make_motion(MOTION_STATE_REST);
        hr_params_t    params  = make_test_params();

        float sqi_val = (pass == 0) ? 0.0f : 1.0f;
        sqi_result_t sqi = make_sqi(0, sqi_val);

        hr_candidate_run(&g_ctx, &ppg, &mac_r, &mac_out,
                         &sqi, &motion, N, &params);

        float *dst = (pass == 0) ? scores_low_sqi : scores_high_sqi;
        dst[0] = g_ctx.result.cand_peak_raw.score;
        dst[1] = g_ctx.result.cand_fft_raw.score;
        dst[2] = g_ctx.result.cand_acf_raw.score;
    }

    /* High SQI should yield >= scores compared to low SQI */
    for (int i = 0; i < 3; i++) {
        ASSERT_TRUE(scores_high_sqi[i] >= scores_low_sqi[i] - 1e-6f);
    }

    return 0;
}

static int test_candidate_peak_raw_motion_penalty(void)
{
    float peak_score_rest;
    float peak_score_run;
    float fft_score_rest;
    float fft_score_run;

    for (int pass = 0; pass < 2; pass++) {
        hr_candidate_init(&g_ctx);

        ppg_preproc_out_t ppg = make_zero_ppg();
        fill_ppg_sine(&ppg, 0, 80.0f);

        ppg_mac_out_t  mac_out = make_zero_mac_out();
        mac_result_t   mac_r   = make_mac_invalid();
        sqi_result_t   sqi     = make_sqi(0, 1.0f);
        hr_params_t    params  = make_test_params();

        motion_state_t ms = (pass == 0) ? MOTION_STATE_REST
                                        : MOTION_STATE_RUN;
        motion_result_t motion = make_motion(ms);

        hr_candidate_run(&g_ctx, &ppg, &mac_r, &mac_out,
                         &sqi, &motion, N, &params);

        if (pass == 0) {
            peak_score_rest = g_ctx.result.cand_peak_raw.score;
            fft_score_rest  = g_ctx.result.cand_fft_raw.score;
        } else {
            peak_score_run  = g_ctx.result.cand_peak_raw.score;
            fft_score_run   = g_ctx.result.cand_fft_raw.score;
        }
    }

    /* PEAK_RAW gets extra penalty in RUN: score_run < score_rest
     * (only when both are valid and nonzero) */
    if (peak_score_rest > 0.01f && peak_score_run > 0.0f) {
        ASSERT_TRUE(peak_score_run < peak_score_rest + 1e-6f);
    }

    /* FFT_RAW should NOT have the peak-specific penalty.
     * Both passes have sqi=1.0 → factor=1.0.  Scores equal. */
    if (fft_score_rest > 0.01f) {
        ASSERT_FLOAT_NEAR(fft_score_rest, fft_score_run, 1e-5f);
    }

    return 0;
}

static int test_candidate_score_range_all(void)
{
    hr_candidate_init(&g_ctx);

    ppg_preproc_out_t ppg = make_zero_ppg();
    fill_ppg_sine(&ppg, 0, 100.0f);

    ppg_mac_out_t mac_out = make_zero_mac_out();
    fill_mac_sine(&mac_out, 100.0f);

    mac_result_t   mac_r  = make_mac_valid();
    sqi_result_t   sqi    = make_sqi(0, 0.5f);
    motion_result_t motion = make_motion(MOTION_STATE_WALK);
    hr_params_t    params = make_test_params();

    hr_candidate_set_pred_seed(&g_ctx, 100.0f);

    hr_candidate_run(&g_ctx, &ppg, &mac_r, &mac_out,
                     &sqi, &motion, N, &params);

    const hr_candidate_t *c[7] = {
        &g_ctx.result.cand_peak_raw, &g_ctx.result.cand_peak_mac,
        &g_ctx.result.cand_fft_raw,  &g_ctx.result.cand_fft_mac,
        &g_ctx.result.cand_acf_raw,  &g_ctx.result.cand_acf_mac,
        &g_ctx.result.cand_pred
    };
    for (int i = 0; i < 7; i++) {
        ASSERT_TRUE(c[i]->score >= 0.0f);
        ASSERT_TRUE(c[i]->score <= 1.0f);
        ASSERT_TRUE(isfinite(c[i]->score));
        ASSERT_TRUE(isfinite(c[i]->bpm));
        if (!c[i]->valid) {
            ASSERT_FLOAT_NEAR(c[i]->bpm,   0.0f, 1e-9f);
            ASSERT_FLOAT_NEAR(c[i]->score, 0.0f, 1e-9f);
        }
    }
    return 0;
}

/* ================================================================== */
/*  [G] main_ch fallback & boundary conditions                         */
/* ================================================================== */

static int test_candidate_main_ch_fallback_to_zero(void)
{
    hr_candidate_init(&g_ctx);

    ppg_preproc_out_t ppg = make_zero_ppg();
    /* Put a strong periodic signal on ch0 only */
    fill_ppg_sine(&ppg, 0, 80.0f);

    ppg_mac_out_t  mac_out = make_zero_mac_out();
    mac_result_t   mac_r   = make_mac_invalid();
    motion_result_t motion = make_motion(MOTION_STATE_REST);
    hr_params_t    params  = make_test_params();

    /* main_ch = 7, which is > 3 → real code falls back to 0 */
    sqi_result_t sqi = make_sqi(7, 0.8f);

    hr_candidate_run(&g_ctx, &ppg, &mac_r, &mac_out,
                     &sqi, &motion, N, &params);

    /* Should have used ch0 (where the sine lives) — at least one valid */
    ASSERT_TRUE(g_ctx.result.cand_fft_raw.valid  ||
                g_ctx.result.cand_acf_raw.valid  ||
                g_ctx.result.cand_peak_raw.valid);

    /* Verify BPM is near 80 for at least one valid candidate */
    const hr_candidate_t *raw[3] = {
        &g_ctx.result.cand_peak_raw,
        &g_ctx.result.cand_fft_raw,
        &g_ctx.result.cand_acf_raw
    };
    int bpm_near_80 = 0;
    for (int i = 0; i < 3; i++) {
        if (raw[i]->valid && raw[i]->bpm >= 72.0f && raw[i]->bpm <= 88.0f)
            bpm_near_80++;
    }
    ASSERT_TRUE(bpm_near_80 > 0);

    return 0;
}

static int test_candidate_invalid_args(void)
{
    hr_candidate_init(&g_ctx);

    ppg_preproc_out_t ppg = make_zero_ppg();
    ppg_mac_out_t  mac_out = make_zero_mac_out();
    mac_result_t   mac_r   = make_mac_invalid();
    sqi_result_t   sqi     = make_sqi(0, 0.8f);
    motion_result_t motion = make_motion(MOTION_STATE_REST);
    hr_params_t    params  = make_test_params();

    /* n == 0 */
    hr_candidate_run(&g_ctx, &ppg, &mac_r, &mac_out,
                     &sqi, &motion, 0, &params);
    ASSERT_FALSE(g_ctx.result.any_valid);
    ASSERT_FALSE(g_ctx.result.cand_fft_raw.valid);

    /* n > HR_WINDOW_SIZE */
    hr_candidate_init(&g_ctx);
    hr_candidate_run(&g_ctx, &ppg, &mac_r, &mac_out,
                     &sqi, &motion, HR_WINDOW_SIZE + 1, &params);
    ASSERT_FALSE(g_ctx.result.any_valid);

    /* NULL ppg_in */
    hr_candidate_init(&g_ctx);
    hr_candidate_run(&g_ctx, NULL, &mac_r, &mac_out,
                     &sqi, &motion, N, &params);
    ASSERT_FALSE(g_ctx.result.any_valid);

    /* NULL params */
    hr_candidate_init(&g_ctx);
    hr_candidate_run(&g_ctx, &ppg, &mac_r, &mac_out,
                     &sqi, &motion, N, NULL);
    ASSERT_FALSE(g_ctx.result.any_valid);

    return 0;
}

static int test_candidate_n_zero_all_invalid(void)
{
    hr_candidate_init(&g_ctx);

    ppg_preproc_out_t ppg = make_zero_ppg();
    ppg_mac_out_t  mac_out = make_zero_mac_out();
    mac_result_t   mac_r   = make_mac_invalid();
    sqi_result_t   sqi     = make_sqi(0, 0.8f);
    motion_result_t motion = make_motion(MOTION_STATE_REST);
    hr_params_t    params  = make_test_params();

    hr_candidate_run(&g_ctx, &ppg, &mac_r, &mac_out,
                     &sqi, &motion, 0, &params);

    const hr_candidate_t *c[7] = {
        &g_ctx.result.cand_peak_raw, &g_ctx.result.cand_peak_mac,
        &g_ctx.result.cand_fft_raw,  &g_ctx.result.cand_fft_mac,
        &g_ctx.result.cand_acf_raw,  &g_ctx.result.cand_acf_mac,
        &g_ctx.result.cand_pred
    };
    for (int i = 0; i < 7; i++) {
        ASSERT_FALSE(c[i]->valid);
        ASSERT_FLOAT_NEAR(c[i]->bpm,   0.0f, 1e-9f);
        ASSERT_FLOAT_NEAR(c[i]->score, 0.0f, 1e-9f);
    }
    ASSERT_FALSE(g_ctx.result.any_valid);
    return 0;
}

static int test_candidate_repeated_run_stability(void)
{
    hr_candidate_init(&g_ctx);

    ppg_preproc_out_t ppg = make_zero_ppg();
    fill_ppg_sine(&ppg, 0, 75.0f);
    ppg_mac_out_t  mac_out = make_zero_mac_out();
    mac_result_t   mac_r   = make_mac_invalid();
    sqi_result_t   sqi     = make_sqi(0, 0.9f);
    motion_result_t motion = make_motion(MOTION_STATE_REST);
    hr_params_t    params  = make_test_params();

    /* Run 5 consecutive cycles with identical input */
    float fft_bpm_first = 0.0f;
    for (int cycle = 0; cycle < 5; cycle++) {
        hr_candidate_run(&g_ctx, &ppg, &mac_r, &mac_out,
                         &sqi, &motion, N, &params);

        ASSERT_TRUE(isfinite(g_ctx.result.cand_fft_raw.bpm));
        ASSERT_TRUE(isfinite(g_ctx.result.cand_fft_raw.score));

        if (cycle == 0 && g_ctx.result.cand_fft_raw.valid) {
            fft_bpm_first = g_ctx.result.cand_fft_raw.bpm;
        }

        /* Deterministic input → same output each cycle */
        if (g_ctx.result.cand_fft_raw.valid && fft_bpm_first > 0.0f) {
            ASSERT_FLOAT_NEAR(g_ctx.result.cand_fft_raw.bpm,
                              fft_bpm_first, 1e-3f);
        }
    }
    return 0;
}

/* ================================================================== */
/*  main                                                               */
/* ================================================================== */

int main(void)
{
    printf("==========================================\n");
    printf("  M6 Candidate Estimation -- Test Suite\n");
    printf("==========================================\n\n");

    printf("[A] Contract tests:\n");
    RUN_TEST(test_candidate_init_state);
    RUN_TEST(test_candidate_reset_state);
    RUN_TEST(test_candidate_basic_run_contract);

    printf("\n[B] Invalid candidate & source semantics:\n");
    RUN_TEST(test_candidate_invalid_fields_and_source);
    RUN_TEST(test_candidate_any_valid_false_when_all_invalid);
    RUN_TEST(test_candidate_any_valid_true_when_one_valid);

    printf("\n[C] raw / mac dual-branch semantics:\n");
    RUN_TEST(test_candidate_raw_branch_runs_when_mac_invalid);
    RUN_TEST(test_candidate_mac_branch_disabled_when_not_valid);
    RUN_TEST(test_candidate_mac_branch_runs_when_valid);
    RUN_TEST(test_candidate_raw_mac_source_no_cross);

    printf("\n[D] Pred seed semantics:\n");
    RUN_TEST(test_candidate_pred_default_invalid);
    RUN_TEST(test_candidate_pred_seed_set_and_clear);
    RUN_TEST(test_candidate_pred_seed_out_of_range);

    printf("\n[E] Constructed periodic signal -- basic viability:\n");
    RUN_TEST(test_candidate_periodic_raw_75bpm);
    RUN_TEST(test_candidate_periodic_raw_120bpm);
    RUN_TEST(test_candidate_periodic_mac_signal);
    RUN_TEST(test_candidate_nonperiodic_low_or_invalid);

    printf("\n[F] Score modulation (SQI & motion):\n");
    RUN_TEST(test_candidate_sqi_score_modulation);
    RUN_TEST(test_candidate_peak_raw_motion_penalty);
    RUN_TEST(test_candidate_score_range_all);

    printf("\n[G] main_ch fallback & boundary conditions:\n");
    RUN_TEST(test_candidate_main_ch_fallback_to_zero);
    RUN_TEST(test_candidate_invalid_args);
    RUN_TEST(test_candidate_n_zero_all_invalid);
    RUN_TEST(test_candidate_repeated_run_stability);

    TEST_SUMMARY();
    return TEST_EXIT_CODE();
}
