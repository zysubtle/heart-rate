/**
 * @file test_hr_mac.c
 * @brief M5 MAC (Motion Artifact Cancellation) — test suite.
 *
 * Tests cover:
 *   [A] Contract: init, reset, invalid args, output validity
 *   [B] REST / bypass / invalid: REST bypass, main_ch OOB, order=0, step<=0
 *   [C] Dynamic reference de-mean: constant ACC invariance, DC offset invariance
 *   [D] Constructed artifact cancellation: correlated artifact, uncorrelated
 *   [E] raw/mac dual-branch semantics: bypass copies raw, success differs raw
 *   [F] Multi-cycle: weight persistence, convergence over repeated windows
 *   [G] Divergence / stability: large step, all-zero, MAX_ORDER clamping
 *
 * All test signals are deterministic and constructed directly as
 * ppg_preproc_out_t / acc_preproc_out_t (bypassing M1–M4), keeping
 * tests self-contained.
 */

#include "test_framework.h"
#include "hr_mac.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define FS  25.0f   /* sampling rate, Hz */

/* ================================================================== */
/*  Test helpers                                                       */
/* ================================================================== */

static hr_params_t make_mac_params(float step, uint16_t order)
{
    hr_params_t p;
    memset(&p, 0, sizeof(p));
    p.mac.nlms_step_size = step;
    p.mac.filter_order   = order;
    return p;
}

static sqi_result_t make_sqi(uint8_t main_ch)
{
    sqi_result_t s;
    memset(&s, 0, sizeof(s));
    s.main_ch   = main_ch;
    s.backup_ch = main_ch;
    s.sqi_main  = 0.8f;
    return s;
}

static motion_result_t make_motion(motion_state_t state)
{
    motion_result_t m;
    memset(&m, 0, sizeof(m));
    m.state = state;
    return m;
}

static bool all_finite(const float *arr, int n)
{
    for (int i = 0; i < n; i++) {
        if (!isfinite(arr[i])) return false;
    }
    return true;
}

static float compute_energy(const float *arr, int n)
{
    float sum = 0.0f;
    for (int i = 0; i < n; i++) sum += arr[i] * arr[i];
    return sum / (float)n;
}

static float compute_mse(const float *a, const float *b, int n)
{
    float sum = 0.0f;
    for (int i = 0; i < n; i++) {
        float d = a[i] - b[i];
        sum += d * d;
    }
    return sum / (float)n;
}

/** Fill all 4 PPG channels with zero. */
static void gen_ppg_zero(ppg_preproc_out_t *p)
{
    memset(p, 0, sizeof(*p));
}

/** Fill all 3 ACC axes with zero. */
static void gen_acc_zero(acc_preproc_out_t *a)
{
    memset(a, 0, sizeof(*a));
}

/** Set one PPG channel to a sine wave; other channels to zero. */
static void gen_ppg_sine(ppg_preproc_out_t *p, int ch,
                         float freq_hz, float amp)
{
    gen_ppg_zero(p);
    for (int i = 0; i < HR_WINDOW_SIZE; i++) {
        p->data[i][ch] = amp * sinf(2.0f * (float)M_PI * freq_hz
                                    * (float)i / FS);
    }
}

/**
 * Set one ACC axis to sine + DC offset; other axes to dc_other.
 * Useful for constructing dynamic reference with controllable DC.
 */
static void gen_acc_axis_sine(acc_preproc_out_t *a, int axis,
                              float freq_hz, float amp, float dc,
                              float dc_other)
{
    for (int i = 0; i < HR_WINDOW_SIZE; i++) {
        float dyn = amp * sinf(2.0f * (float)M_PI * freq_hz
                               * (float)i / FS);
        for (int ax = 0; ax < 3; ax++) {
            a->data[i][ax] = (ax == axis) ? (dc + dyn) : dc_other;
        }
    }
}

/* ================================================================== */
/*  [A] Contract tests                                                 */
/* ================================================================== */

static int test_mac_init_state(void)
{
    hr_params_t params = make_mac_params(0.01f, 8);
    hr_mac_ctx_t ctx;
    hr_mac_init(&ctx, &params);

    ASSERT_TRUE(ctx.initialized);
    ASSERT_INT_EQ(ctx.effective_order, 8);
    ASSERT_INT_EQ(ctx.total_taps, 24);

    for (int k = 0; k < HR_MAC_MAX_TAPS; k++) {
        ASSERT_FLOAT_NEAR(ctx.weights[k], 0.0f, 1e-12f);
        ASSERT_FLOAT_NEAR(ctx.delay_line[k], 0.0f, 1e-12f);
    }
    for (int i = 0; i < HR_WINDOW_SIZE; i++) {
        ASSERT_FLOAT_NEAR(ctx.out.data[i], 0.0f, 1e-12f);
    }
    ASSERT_FALSE(ctx.result.valid);
    ASSERT_FALSE(ctx.result.bypassed);
    ASSERT_FALSE(ctx.result.diverged);
    return 0;
}

static int test_mac_reset_state(void)
{
    hr_params_t params = make_mac_params(0.05f, 8);
    hr_mac_ctx_t ctx;
    hr_mac_init(&ctx, &params);

    /* Run once to mutate weights */
    ppg_preproc_out_t ppg;
    acc_preproc_out_t acc;
    gen_ppg_sine(&ppg, 0, 1.5f, 100.0f);
    gen_acc_axis_sine(&acc, 0, 2.0f, 500.0f, 0.0f, 0.0f);

    sqi_result_t sqi       = make_sqi(0);
    motion_result_t motion = make_motion(MOTION_STATE_WALK);
    hr_mac_run(&ctx, &ppg, &acc, &sqi, &motion, HR_WINDOW_SIZE, &params);

    /* Weights should be non-zero after run */
    float wnorm = 0.0f;
    for (int k = 0; k < (int)ctx.total_taps; k++)
        wnorm += ctx.weights[k] * ctx.weights[k];
    ASSERT_TRUE(wnorm > 1e-12f);

    /* Reset and verify clean state */
    hr_mac_reset(&ctx, &params);
    ASSERT_TRUE(ctx.initialized);
    ASSERT_INT_EQ(ctx.effective_order, 8);
    ASSERT_INT_EQ(ctx.total_taps, 24);
    for (int k = 0; k < (int)ctx.total_taps; k++) {
        ASSERT_FLOAT_NEAR(ctx.weights[k], 0.0f, 1e-12f);
    }
    ASSERT_FALSE(ctx.result.valid);
    return 0;
}

static int test_mac_invalid_args(void)
{
    hr_params_t params = make_mac_params(0.01f, 8);
    hr_mac_ctx_t ctx;
    hr_mac_init(&ctx, &params);

    ppg_preproc_out_t ppg;
    acc_preproc_out_t acc;
    gen_ppg_zero(&ppg);
    gen_acc_zero(&acc);
    sqi_result_t sqi       = make_sqi(0);
    motion_result_t motion = make_motion(MOTION_STATE_WALK);

    /* NULL ctx — should not crash */
    hr_mac_run(NULL, &ppg, &acc, &sqi, &motion, HR_WINDOW_SIZE, &params);

    /* NULL ppg_in */
    hr_mac_run(&ctx, NULL, &acc, &sqi, &motion, HR_WINDOW_SIZE, &params);
    ASSERT_FALSE(ctx.result.valid);
    ASSERT_FALSE(ctx.result.bypassed);

    /* n = 0 */
    hr_mac_run(&ctx, &ppg, &acc, &sqi, &motion, 0, &params);
    ASSERT_FALSE(ctx.result.valid);

    /* n > HR_WINDOW_SIZE */
    hr_mac_run(&ctx, &ppg, &acc, &sqi, &motion, HR_WINDOW_SIZE + 1, &params);
    ASSERT_FALSE(ctx.result.valid);

    /* Not initialized */
    hr_mac_ctx_t ctx2;
    memset(&ctx2, 0, sizeof(ctx2));
    ctx2.initialized = false;
    hr_mac_run(&ctx2, &ppg, &acc, &sqi, &motion, HR_WINDOW_SIZE, &params);
    ASSERT_FALSE(ctx2.result.valid);

    return 0;
}

static int test_mac_output_no_nan(void)
{
    hr_params_t params = make_mac_params(0.01f, 8);
    hr_mac_ctx_t ctx;
    hr_mac_init(&ctx, &params);

    ppg_preproc_out_t ppg;
    acc_preproc_out_t acc;
    gen_ppg_sine(&ppg, 1, 1.5f, 100.0f);
    gen_acc_axis_sine(&acc, 0, 2.0f, 300.0f, 100.0f, 50.0f);

    sqi_result_t sqi       = make_sqi(1);
    motion_result_t motion = make_motion(MOTION_STATE_WALK);
    hr_mac_run(&ctx, &ppg, &acc, &sqi, &motion, HR_WINDOW_SIZE, &params);

    ASSERT_TRUE(all_finite(ctx.out.data, HR_WINDOW_SIZE));
    ASSERT_TRUE(isfinite(ctx.result.input_energy));
    ASSERT_TRUE(isfinite(ctx.result.output_energy));
    ASSERT_TRUE(isfinite(ctx.result.artifact_est_energy));
    return 0;
}

/* ================================================================== */
/*  [B] REST / bypass / invalid tests                                  */
/* ================================================================== */

static int test_mac_rest_bypass(void)
{
    hr_params_t params = make_mac_params(0.01f, 8);
    hr_mac_ctx_t ctx;
    hr_mac_init(&ctx, &params);

    ppg_preproc_out_t ppg;
    acc_preproc_out_t acc;
    gen_ppg_sine(&ppg, 0, 1.5f, 100.0f);
    gen_acc_axis_sine(&acc, 0, 2.0f, 300.0f, 0.0f, 0.0f);

    sqi_result_t sqi       = make_sqi(0);
    motion_result_t motion = make_motion(MOTION_STATE_REST);
    hr_mac_run(&ctx, &ppg, &acc, &sqi, &motion, HR_WINDOW_SIZE, &params);

    ASSERT_FALSE(ctx.result.valid);
    ASSERT_TRUE(ctx.result.bypassed);
    ASSERT_FALSE(ctx.result.diverged);
    ASSERT_INT_EQ(ctx.result.main_ch_used, 0);

    /* Bypass energy statistics: current impl sets all to 0 */
    ASSERT_FLOAT_NEAR(ctx.result.input_energy, 0.0f, 1e-12f);
    ASSERT_FLOAT_NEAR(ctx.result.output_energy, 0.0f, 1e-12f);
    ASSERT_FLOAT_NEAR(ctx.result.artifact_est_energy, 0.0f, 1e-12f);
    return 0;
}

static int test_mac_invalid_main_ch(void)
{
    hr_params_t params = make_mac_params(0.01f, 8);
    hr_mac_ctx_t ctx;
    hr_mac_init(&ctx, &params);

    ppg_preproc_out_t ppg;
    acc_preproc_out_t acc;
    gen_ppg_sine(&ppg, 0, 1.5f, 100.0f);
    gen_acc_zero(&acc);

    sqi_result_t sqi       = make_sqi(5);   /* out of bounds */
    motion_result_t motion = make_motion(MOTION_STATE_WALK);
    hr_mac_run(&ctx, &ppg, &acc, &sqi, &motion, HR_WINDOW_SIZE, &params);

    ASSERT_FALSE(ctx.result.valid);
    ASSERT_FALSE(ctx.result.bypassed);
    ASSERT_FALSE(ctx.result.diverged);
    /* True impl: main_ch_used records the requested (invalid) value */
    ASSERT_INT_EQ(ctx.result.main_ch_used, 5);

    /* True impl: out.data contains a copy of ch0 as fallback */
    for (int i = 0; i < HR_WINDOW_SIZE; i++) {
        ASSERT_FLOAT_NEAR(ctx.out.data[i], ppg.data[i][0], 1e-12f);
    }
    return 0;
}

static int test_mac_zero_order_bypass(void)
{
    hr_params_t params = make_mac_params(0.01f, 0);
    hr_mac_ctx_t ctx;
    hr_mac_init(&ctx, &params);

    ASSERT_INT_EQ(ctx.effective_order, 0);
    ASSERT_INT_EQ(ctx.total_taps, 0);

    ppg_preproc_out_t ppg;
    acc_preproc_out_t acc;
    gen_ppg_sine(&ppg, 0, 1.5f, 100.0f);
    gen_acc_zero(&acc);

    sqi_result_t sqi       = make_sqi(0);
    motion_result_t motion = make_motion(MOTION_STATE_WALK);
    hr_mac_run(&ctx, &ppg, &acc, &sqi, &motion, HR_WINDOW_SIZE, &params);

    ASSERT_FALSE(ctx.result.valid);
    ASSERT_TRUE(ctx.result.bypassed);
    ASSERT_FALSE(ctx.result.diverged);
    ASSERT_TRUE(all_finite(ctx.out.data, HR_WINDOW_SIZE));
    return 0;
}

static int test_mac_zero_step_bypass(void)
{
    hr_params_t params = make_mac_params(0.0f, 8);
    hr_mac_ctx_t ctx;
    hr_mac_init(&ctx, &params);

    ppg_preproc_out_t ppg;
    acc_preproc_out_t acc;
    gen_ppg_sine(&ppg, 0, 1.5f, 100.0f);
    gen_acc_zero(&acc);

    sqi_result_t sqi       = make_sqi(0);
    motion_result_t motion = make_motion(MOTION_STATE_WALK);
    hr_mac_run(&ctx, &ppg, &acc, &sqi, &motion, HR_WINDOW_SIZE, &params);

    ASSERT_FALSE(ctx.result.valid);
    ASSERT_TRUE(ctx.result.bypassed);
    ASSERT_FALSE(ctx.result.diverged);
    return 0;
}

static int test_mac_negative_step_bypass(void)
{
    hr_params_t params = make_mac_params(-0.5f, 8);
    hr_mac_ctx_t ctx;
    hr_mac_init(&ctx, &params);

    ppg_preproc_out_t ppg;
    acc_preproc_out_t acc;
    gen_ppg_sine(&ppg, 0, 1.5f, 100.0f);
    gen_acc_zero(&acc);

    sqi_result_t sqi       = make_sqi(0);
    motion_result_t motion = make_motion(MOTION_STATE_WALK);
    hr_mac_run(&ctx, &ppg, &acc, &sqi, &motion, HR_WINDOW_SIZE, &params);

    ASSERT_FALSE(ctx.result.valid);
    ASSERT_TRUE(ctx.result.bypassed);
    ASSERT_FALSE(ctx.result.diverged);
    ASSERT_TRUE(all_finite(ctx.out.data, HR_WINDOW_SIZE));
    return 0;
}

/* ================================================================== */
/*  [C] Dynamic reference de-mean tests                                */
/* ================================================================== */

/**
 * Constant ACC (pure DC, no dynamic) should produce no artifact estimate.
 * After de-meaning, the reference is all-zero, so y[n]=0 and e[n]=d[n].
 * Output must equal raw input exactly (within float precision).
 */
static int test_mac_constant_acc_no_false_artifact(void)
{
    hr_params_t params = make_mac_params(0.05f, 8);
    hr_mac_ctx_t ctx;
    hr_mac_init(&ctx, &params);

    ppg_preproc_out_t ppg;
    acc_preproc_out_t acc;
    gen_ppg_sine(&ppg, 0, 1.5f, 100.0f);

    /* All-constant ACC: large DC, no dynamic component */
    for (int i = 0; i < HR_WINDOW_SIZE; i++) {
        acc.data[i][0] = 1000.0f;
        acc.data[i][1] = -500.0f;
        acc.data[i][2] = 200.0f;
    }

    sqi_result_t sqi       = make_sqi(0);
    motion_result_t motion = make_motion(MOTION_STATE_WALK);
    hr_mac_run(&ctx, &ppg, &acc, &sqi, &motion, HR_WINDOW_SIZE, &params);

    ASSERT_TRUE(ctx.result.valid);
    ASSERT_FALSE(ctx.result.diverged);

    /* Output must equal raw input: no artifact to cancel */
    for (int i = 0; i < HR_WINDOW_SIZE; i++) {
        ASSERT_FLOAT_NEAR(ctx.out.data[i], ppg.data[i][0], 1e-6f);
    }
    /* artifact_est_energy should be zero (y[n]=0 for all n) */
    ASSERT_FLOAT_NEAR(ctx.result.artifact_est_energy, 0.0f, 1e-12f);
    return 0;
}

/**
 * Same dynamic ACC + different DC offsets must produce nearly identical
 * MAC output, proving that per-axis de-meaning works correctly.
 */
static int test_mac_dc_offset_invariance(void)
{
    const float step  = 0.05f;
    const uint16_t order = 8;
    hr_params_t params = make_mac_params(step, order);

    /* Common PPG: sine at 1.5 Hz corrupted by ACC-correlated artifact */
    ppg_preproc_out_t ppg;
    gen_ppg_zero(&ppg);
    for (int i = 0; i < HR_WINDOW_SIZE; i++) {
        float t  = (float)i / FS;
        float dyn_x = 500.0f * sinf(2.0f * (float)M_PI * 2.0f * t);
        ppg.data[i][0] = 100.0f * sinf(2.0f * (float)M_PI * 1.5f * t)
                         + 0.5f * dyn_x;
    }

    sqi_result_t sqi       = make_sqi(0);
    motion_result_t motion = make_motion(MOTION_STATE_WALK);

    /* --- Run 1: dynamic ACC with zero DC --- */
    hr_mac_ctx_t ctx1;
    hr_mac_init(&ctx1, &params);
    acc_preproc_out_t acc1;
    gen_acc_axis_sine(&acc1, 0, 2.0f, 500.0f, 0.0f, 0.0f);
    hr_mac_run(&ctx1, &ppg, &acc1, &sqi, &motion, HR_WINDOW_SIZE, &params);

    /* --- Run 2: same dynamic ACC with large DC offset --- */
    hr_mac_ctx_t ctx2;
    hr_mac_init(&ctx2, &params);
    acc_preproc_out_t acc2;
    gen_acc_axis_sine(&acc2, 0, 2.0f, 500.0f, 5000.0f, 2000.0f);
    hr_mac_run(&ctx2, &ppg, &acc2, &sqi, &motion, HR_WINDOW_SIZE, &params);

    /* Both runs should succeed */
    ASSERT_TRUE(ctx1.result.valid);
    ASSERT_TRUE(ctx2.result.valid);

    /* Outputs should be nearly identical */
    float mse = compute_mse(ctx1.out.data, ctx2.out.data, HR_WINDOW_SIZE);
    float ref_energy = compute_energy(ctx1.out.data, HR_WINDOW_SIZE);
    ASSERT_TRUE(ref_energy > 0.0f);
    /* MSE between outputs < 1% of output energy */
    ASSERT_TRUE(mse < 0.01f * ref_energy);
    return 0;
}

/* ================================================================== */
/*  [D] Constructed artifact cancellation tests                        */
/* ================================================================== */

/**
 * Construct PPG = clean_heart + acc_correlated_artifact.
 * After multiple MAC cycles, output should be closer to clean_heart
 * than the corrupted input was.
 */
static int test_mac_correlated_artifact_basic(void)
{
    hr_params_t params = make_mac_params(0.1f, 8);
    hr_mac_ctx_t ctx;
    hr_mac_init(&ctx, &params);

    float clean[HR_WINDOW_SIZE];
    ppg_preproc_out_t ppg;
    acc_preproc_out_t acc;
    gen_ppg_zero(&ppg);
    gen_acc_zero(&acc);

    for (int i = 0; i < HR_WINDOW_SIZE; i++) {
        float t = (float)i / FS;
        clean[i] = 100.0f * sinf(2.0f * (float)M_PI * 1.5f * t);
        float acc_dyn = 500.0f * sinf(2.0f * (float)M_PI * 2.0f * t);
        float artifact = 0.5f * acc_dyn;
        ppg.data[i][0] = clean[i] + artifact;
        acc.data[i][0] = acc_dyn;
    }

    sqi_result_t sqi       = make_sqi(0);
    motion_result_t motion = make_motion(MOTION_STATE_WALK);

    float mse_before = compute_mse(ppg.data[0], clean, HR_WINDOW_SIZE);

    /* Run several cycles to allow convergence (weights persist) */
    for (int cycle = 0; cycle < 10; cycle++) {
        hr_mac_run(&ctx, &ppg, &acc, &sqi, &motion,
                   HR_WINDOW_SIZE, &params);
    }

    ASSERT_TRUE(ctx.result.valid);
    ASSERT_FALSE(ctx.result.diverged);

    float mse_after = compute_mse(ctx.out.data, clean, HR_WINDOW_SIZE);

    /* After convergence, MAC output should be closer to clean signal.
     * Require at least 50% MSE reduction — conservative for NLMS. */
    ASSERT_TRUE(mse_after < mse_before * 0.5f);
    return 0;
}

/**
 * Disturbance uncorrelated with ACC must not be catastrophically
 * amplified.  The filter cannot remove uncorrelated noise but
 * must not blow it up either.
 */
static int test_mac_uncorrelated_no_amplification(void)
{
    hr_params_t params = make_mac_params(0.01f, 8);
    hr_mac_ctx_t ctx;
    hr_mac_init(&ctx, &params);

    ppg_preproc_out_t ppg;
    acc_preproc_out_t acc;
    gen_ppg_zero(&ppg);

    /* PPG: signal at 1.5 Hz + uncorrelated disturbance at 3.7 Hz */
    for (int i = 0; i < HR_WINDOW_SIZE; i++) {
        float t = (float)i / FS;
        ppg.data[i][0] = 100.0f * sinf(2.0f * (float)M_PI * 1.5f * t)
                        +  50.0f * sinf(2.0f * (float)M_PI * 3.7f * t);
    }
    /* ACC: dynamics at 2.0 Hz — uncorrelated with the 3.7 Hz noise */
    gen_acc_axis_sine(&acc, 0, 2.0f, 500.0f, 0.0f, 0.0f);

    sqi_result_t sqi       = make_sqi(0);
    motion_result_t motion = make_motion(MOTION_STATE_WALK);

    float input_energy = compute_energy(ppg.data[0], HR_WINDOW_SIZE);
    hr_mac_run(&ctx, &ppg, &acc, &sqi, &motion, HR_WINDOW_SIZE, &params);

    ASSERT_TRUE(all_finite(ctx.out.data, HR_WINDOW_SIZE));
    ASSERT_FALSE(ctx.result.diverged);

    float output_energy = compute_energy(ctx.out.data, HR_WINDOW_SIZE);
    /*
     * NLMS with uncorrelated reference has inherent "excess MSE":
     * finite-sample adaptation can increase energy moderately.
     * The code's divergence threshold is 10×; here we verify no
     * catastrophic amplification by using a 5× limit.
     */
    ASSERT_TRUE(output_energy < 5.0f * input_energy);
    return 0;
}

/* ================================================================== */
/*  [E] raw/mac dual-branch semantics tests                            */
/* ================================================================== */

/**
 * In bypass (REST), out.data must be an exact copy of raw main_ch.
 */
static int test_mac_bypass_out_equals_raw(void)
{
    hr_params_t params = make_mac_params(0.01f, 8);
    hr_mac_ctx_t ctx;
    hr_mac_init(&ctx, &params);

    ppg_preproc_out_t ppg;
    gen_ppg_zero(&ppg);
    /* Fill main_ch=2 with distinct values */
    for (int i = 0; i < HR_WINDOW_SIZE; i++) {
        ppg.data[i][2] = (float)(i * 7 + 13);
    }

    acc_preproc_out_t acc;
    gen_acc_zero(&acc);

    sqi_result_t sqi       = make_sqi(2);
    motion_result_t motion = make_motion(MOTION_STATE_REST);
    hr_mac_run(&ctx, &ppg, &acc, &sqi, &motion, HR_WINDOW_SIZE, &params);

    ASSERT_FALSE(ctx.result.valid);
    ASSERT_TRUE(ctx.result.bypassed);

    for (int i = 0; i < HR_WINDOW_SIZE; i++) {
        ASSERT_FLOAT_NEAR(ctx.out.data[i], ppg.data[i][2], 1e-12f);
    }
    return 0;
}

/**
 * After successful MAC with correlated artifact, out.data must differ
 * from raw input — demonstrating the filter actually did something.
 */
static int test_mac_success_out_differs_raw(void)
{
    hr_params_t params = make_mac_params(0.1f, 8);
    hr_mac_ctx_t ctx;
    hr_mac_init(&ctx, &params);

    ppg_preproc_out_t ppg;
    acc_preproc_out_t acc;
    gen_ppg_zero(&ppg);
    gen_acc_zero(&acc);

    for (int i = 0; i < HR_WINDOW_SIZE; i++) {
        float t = (float)i / FS;
        float acc_dyn = 500.0f * sinf(2.0f * (float)M_PI * 2.0f * t);
        ppg.data[i][0] = 100.0f * sinf(2.0f * (float)M_PI * 1.5f * t)
                        + 0.5f * acc_dyn;
        acc.data[i][0] = acc_dyn;
    }

    sqi_result_t sqi       = make_sqi(0);
    motion_result_t motion = make_motion(MOTION_STATE_WALK);

    /* Multiple cycles for the filter to adapt */
    for (int c = 0; c < 5; c++) {
        hr_mac_run(&ctx, &ppg, &acc, &sqi, &motion,
                   HR_WINDOW_SIZE, &params);
    }

    ASSERT_TRUE(ctx.result.valid);

    /* Output should measurably differ from raw input */
    float diff_energy = compute_mse(ctx.out.data, ppg.data[0],
                                    HR_WINDOW_SIZE);
    ASSERT_TRUE(diff_energy > 1.0f);

    /* artifact_est_energy should be non-trivial */
    ASSERT_TRUE(ctx.result.artifact_est_energy > 1.0f);
    return 0;
}

/* ================================================================== */
/*  [F] Multi-cycle weight persistence tests                           */
/* ================================================================== */

/**
 * After a successful run, weights must be non-zero and must survive
 * into the next run (no implicit reset between calls).
 */
static int test_mac_weights_persist_across_runs(void)
{
    hr_params_t params = make_mac_params(0.05f, 8);
    hr_mac_ctx_t ctx;
    hr_mac_init(&ctx, &params);

    ppg_preproc_out_t ppg;
    acc_preproc_out_t acc;
    gen_ppg_sine(&ppg, 0, 1.5f, 100.0f);
    gen_acc_axis_sine(&acc, 0, 2.0f, 500.0f, 0.0f, 0.0f);

    sqi_result_t sqi       = make_sqi(0);
    motion_result_t motion = make_motion(MOTION_STATE_WALK);

    /* Run 1 */
    hr_mac_run(&ctx, &ppg, &acc, &sqi, &motion, HR_WINDOW_SIZE, &params);
    ASSERT_TRUE(ctx.result.valid);

    float wnorm1 = 0.0f;
    for (int k = 0; k < (int)ctx.total_taps; k++)
        wnorm1 += ctx.weights[k] * ctx.weights[k];
    ASSERT_TRUE(wnorm1 > 1e-12f);

    /* Run 2 — weights should NOT be reset */
    float weights_after_run1[HR_MAC_MAX_TAPS];
    memcpy(weights_after_run1, ctx.weights, sizeof(weights_after_run1));

    hr_mac_run(&ctx, &ppg, &acc, &sqi, &motion, HR_WINDOW_SIZE, &params);

    /* Weights should have changed (further adaptation) but not reset */
    float wnorm2 = 0.0f;
    for (int k = 0; k < (int)ctx.total_taps; k++)
        wnorm2 += ctx.weights[k] * ctx.weights[k];
    ASSERT_TRUE(wnorm2 > 1e-12f);

    /* At least some weight should have changed between runs */
    bool any_changed = false;
    for (int k = 0; k < (int)ctx.total_taps; k++) {
        if (fabsf(ctx.weights[k] - weights_after_run1[k]) > 1e-12f) {
            any_changed = true;
            break;
        }
    }
    ASSERT_TRUE(any_changed);
    return 0;
}

/**
 * Repeated MAC runs on a correlated-artifact window should show that
 * output error (vs clean) does not increase — the persistent weights
 * allow progressive convergence.
 */
static int test_mac_multi_cycle_improves(void)
{
    hr_params_t params = make_mac_params(0.05f, 8);
    hr_mac_ctx_t ctx;
    hr_mac_init(&ctx, &params);

    float clean[HR_WINDOW_SIZE];
    ppg_preproc_out_t ppg;
    acc_preproc_out_t acc;
    gen_ppg_zero(&ppg);
    gen_acc_zero(&acc);

    for (int i = 0; i < HR_WINDOW_SIZE; i++) {
        float t = (float)i / FS;
        clean[i] = 100.0f * sinf(2.0f * (float)M_PI * 1.5f * t);
        float dyn = 500.0f * sinf(2.0f * (float)M_PI * 2.0f * t);
        ppg.data[i][0] = clean[i] + 0.5f * dyn;
        acc.data[i][0] = dyn;
    }

    sqi_result_t sqi       = make_sqi(0);
    motion_result_t motion = make_motion(MOTION_STATE_WALK);

    float mse_cycle1 = 0.0f;
    float mse_cycle5 = 0.0f;

    for (int c = 1; c <= 5; c++) {
        hr_mac_run(&ctx, &ppg, &acc, &sqi, &motion,
                   HR_WINDOW_SIZE, &params);
        ASSERT_TRUE(ctx.result.valid);

        float mse = compute_mse(ctx.out.data, clean, HR_WINDOW_SIZE);
        if (c == 1) mse_cycle1 = mse;
        if (c == 5) mse_cycle5 = mse;
    }

    /* Cycle 5 error should not be worse than cycle 1 */
    ASSERT_TRUE(mse_cycle5 <= mse_cycle1 * 1.05f);
    return 0;
}

/* ================================================================== */
/*  [G] Divergence / stability tests                                   */
/* ================================================================== */

/**
 * Very large step size (mu >> 2, beyond NLMS stability limit) should
 * trigger divergence detection or at least not crash / produce NaN
 * in the result.
 */
static int test_mac_large_step_stability(void)
{
    hr_params_t params = make_mac_params(100.0f, 8);
    hr_mac_ctx_t ctx;
    hr_mac_init(&ctx, &params);

    ppg_preproc_out_t ppg;
    acc_preproc_out_t acc;
    gen_ppg_sine(&ppg, 0, 1.5f, 100.0f);
    gen_acc_axis_sine(&acc, 0, 2.0f, 500.0f, 0.0f, 0.0f);

    sqi_result_t sqi       = make_sqi(0);
    motion_result_t motion = make_motion(MOTION_STATE_WALK);
    hr_mac_run(&ctx, &ppg, &acc, &sqi, &motion, HR_WINDOW_SIZE, &params);

    /* Must NOT crash.  Should diverge with such aggressive step. */
    ASSERT_FALSE(ctx.result.valid);
    ASSERT_TRUE(ctx.result.diverged);

    /* Output should be safe (raw copy fallback) */
    ASSERT_TRUE(all_finite(ctx.out.data, HR_WINDOW_SIZE));
    ASSERT_TRUE(isfinite(ctx.result.input_energy));

    /* Divergence resets weights for recovery */
    for (int k = 0; k < (int)ctx.total_taps; k++) {
        ASSERT_FLOAT_NEAR(ctx.weights[k], 0.0f, 1e-12f);
    }
    return 0;
}

/**
 * All-zero inputs (PPG and ACC both zero) should be numerically
 * stable: no NaN/Inf, no divergence, well-defined result.
 */
static int test_mac_all_zero_stable(void)
{
    hr_params_t params = make_mac_params(0.01f, 8);
    hr_mac_ctx_t ctx;
    hr_mac_init(&ctx, &params);

    ppg_preproc_out_t ppg;
    acc_preproc_out_t acc;
    gen_ppg_zero(&ppg);
    gen_acc_zero(&acc);

    sqi_result_t sqi       = make_sqi(0);
    motion_result_t motion = make_motion(MOTION_STATE_WALK);
    hr_mac_run(&ctx, &ppg, &acc, &sqi, &motion, HR_WINDOW_SIZE, &params);

    ASSERT_TRUE(all_finite(ctx.out.data, HR_WINDOW_SIZE));
    ASSERT_FALSE(ctx.result.diverged);

    for (int i = 0; i < HR_WINDOW_SIZE; i++) {
        ASSERT_FLOAT_NEAR(ctx.out.data[i], 0.0f, 1e-12f);
    }
    ASSERT_FLOAT_NEAR(ctx.result.input_energy, 0.0f, 1e-12f);
    ASSERT_FLOAT_NEAR(ctx.result.output_energy, 0.0f, 1e-12f);
    return 0;
}

/**
 * filter_order exceeding HR_MAC_MAX_ORDER must be clamped,
 * not cause buffer overrun.
 */
static int test_mac_max_order_clamping(void)
{
    hr_params_t params = make_mac_params(0.01f, 100);
    hr_mac_ctx_t ctx;
    hr_mac_init(&ctx, &params);

    ASSERT_INT_EQ(ctx.effective_order, HR_MAC_MAX_ORDER);
    ASSERT_INT_EQ(ctx.total_taps, 3 * HR_MAC_MAX_ORDER);

    /* Run should work with the clamped order */
    ppg_preproc_out_t ppg;
    acc_preproc_out_t acc;
    gen_ppg_sine(&ppg, 0, 1.5f, 100.0f);
    gen_acc_axis_sine(&acc, 0, 2.0f, 300.0f, 0.0f, 0.0f);

    sqi_result_t sqi       = make_sqi(0);
    motion_result_t motion = make_motion(MOTION_STATE_WALK);
    hr_mac_run(&ctx, &ppg, &acc, &sqi, &motion, HR_WINDOW_SIZE, &params);

    ASSERT_TRUE(all_finite(ctx.out.data, HR_WINDOW_SIZE));
    ASSERT_FALSE(ctx.result.diverged);
    return 0;
}

/* ================================================================== */
/*  Main                                                               */
/* ================================================================== */

int main(void)
{
    printf("\n==========================================\n");
    printf("  M5 MAC (Motion Artifact Cancellation) -- Test Suite\n");
    printf("==========================================\n\n");

    printf("[A] Contract tests:\n");
    RUN_TEST(test_mac_init_state);
    RUN_TEST(test_mac_reset_state);
    RUN_TEST(test_mac_invalid_args);
    RUN_TEST(test_mac_output_no_nan);

    printf("\n[B] REST / bypass / invalid:\n");
    RUN_TEST(test_mac_rest_bypass);
    RUN_TEST(test_mac_invalid_main_ch);
    RUN_TEST(test_mac_zero_order_bypass);
    RUN_TEST(test_mac_zero_step_bypass);
    RUN_TEST(test_mac_negative_step_bypass);

    printf("\n[C] Dynamic reference de-mean:\n");
    RUN_TEST(test_mac_constant_acc_no_false_artifact);
    RUN_TEST(test_mac_dc_offset_invariance);

    printf("\n[D] Constructed artifact cancellation:\n");
    RUN_TEST(test_mac_correlated_artifact_basic);
    RUN_TEST(test_mac_uncorrelated_no_amplification);

    printf("\n[E] raw/mac dual-branch semantics:\n");
    RUN_TEST(test_mac_bypass_out_equals_raw);
    RUN_TEST(test_mac_success_out_differs_raw);

    printf("\n[F] Multi-cycle weight persistence:\n");
    RUN_TEST(test_mac_weights_persist_across_runs);
    RUN_TEST(test_mac_multi_cycle_improves);

    printf("\n[G] Divergence / stability:\n");
    RUN_TEST(test_mac_large_step_stability);
    RUN_TEST(test_mac_all_zero_stable);
    RUN_TEST(test_mac_max_order_clamping);

    TEST_SUMMARY();
    return TEST_EXIT_CODE();
}
