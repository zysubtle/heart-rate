/**
 * @file test_hr_motion.c
 * @brief M4 Motion State Detection -- test suite.
 *
 * Tests cover:
 *   [A] Contract: init, reset, invalid args, output validity
 *   [B] DC removal & energy: constant input, DC invariance, zero input
 *   [C] Classification: walk / run / irregular / rest / threshold influence
 *   [D] Feature interpretability: periodicity, frequency, periodic_motion
 *   [E] Multi-cycle: initialized, prev_state, stability, reset recovery
 *
 * All test signals are deterministic and constructed directly as
 * acc_preproc_out_t (bypassing M1/M2), keeping tests self-contained.
 *
 * Signal construction note:
 *   A single-axis sine at frequency f produces magnitude |sin| whose
 *   fundamental period is T/2 (frequency 2f).  Test frequencies are
 *   chosen accordingly so that the ACF-detected frequency lands in
 *   the desired band.
 *     Walk target: ACF freq ~1.8 Hz  =>  input sine at 0.9 Hz
 *     Run  target: ACF freq ~3.1 Hz  =>  input sine at 1.5 Hz
 */

#include "test_framework.h"
#include "hr_motion.h"

#include <math.h>
#include <string.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ================================================================== */
/*  Test helpers                                                       */
/* ================================================================== */

/**
 * Standard test parameters.
 * Thresholds chosen so that constructed signals land clearly in the
 * expected classification zones:
 *   REST:  energy < 0.5
 *   WALK:  energy in [2.0, 10.0), periodic, walk-band freq
 *   RUN:   energy >= 10.0, periodic, run-band freq
 *   IRREGULAR: non-REST but aperiodic or freq mismatch
 */
static hr_params_t make_test_params(void)
{
    hr_params_t p;
    memset(&p, 0, sizeof(p));
    p.motion.rest_energy_threshold = 0.5f;
    p.motion.walk_energy_threshold = 2.0f;
    p.motion.run_energy_threshold  = 10.0f;
    return p;
}

/* --- Deterministic PRNG (LCG, fixed seed) --- */

static uint32_t _prng_state;

static void prng_seed(uint32_t seed)
{
    _prng_state = seed;
}

static float prng_float(float lo, float hi)
{
    _prng_state = _prng_state * 1103515245u + 12345u;
    float t = (float)((_prng_state >> 16) & 0x7FFF) / 32767.0f;
    return lo + t * (hi - lo);
}

/* --- Signal generators --- */

static void gen_constant(acc_preproc_out_t *out,
                         float x, float y, float z)
{
    for (int i = 0; i < HR_WINDOW_SIZE; i++) {
        out->data[i][0] = x;
        out->data[i][1] = y;
        out->data[i][2] = z;
    }
}

/*
 * Periodic signal on z-axis: z = dc_z + amp * sin(2*pi*freq*i/fs).
 * x and y are constant DC values.
 *
 * A single-axis sine at frequency f produces magnitude |sin| with
 * ACF fundamental at 2f.  Callers choose freq_z_hz such that the
 * detected frequency (2 * freq_z_hz) lands in the desired band.
 */
static void gen_periodic_z(acc_preproc_out_t *out,
                           float dc_x, float dc_y, float dc_z,
                           float amp_z, float freq_z_hz)
{
    const float fs = 25.0f;
    for (int i = 0; i < HR_WINDOW_SIZE; i++) {
        out->data[i][0] = dc_x;
        out->data[i][1] = dc_y;
        out->data[i][2] = dc_z + amp_z *
            sinf(2.0f * (float)M_PI * freq_z_hz * (float)i / fs);
    }
}

/* Random non-periodic signal on z-axis (deterministic, fixed seed) */
static void gen_irregular_z(acc_preproc_out_t *out,
                            float dc_x, float dc_y, float dc_z,
                            float amp, uint32_t seed)
{
    prng_seed(seed);
    for (int i = 0; i < HR_WINDOW_SIZE; i++) {
        out->data[i][0] = dc_x;
        out->data[i][1] = dc_y;
        out->data[i][2] = dc_z + prng_float(-amp, amp);
    }
}

/* --- Numeric validators --- */

static int is_finite_f(float v)
{
    return (v == v) && (v - v == 0.0f);
}

static int is_valid_state(motion_state_t s)
{
    return s == MOTION_STATE_REST ||
           s == MOTION_STATE_WALK ||
           s == MOTION_STATE_RUN  ||
           s == MOTION_STATE_IRREGULAR;
}

/* ================================================================== */
/*  [A] Contract tests                                                 */
/* ================================================================== */

static int test_motion_init_state(void)
{
    hr_motion_ctx_t ctx;
    hr_motion_init(&ctx);

    ASSERT_INT_EQ(ctx.result.state, MOTION_STATE_REST);
    ASSERT_FLOAT_NEAR(ctx.result.dynamic_energy,   0.0f, 1e-6f);
    ASSERT_FLOAT_NEAR(ctx.result.dominant_freq_hz,  0.0f, 1e-6f);
    ASSERT_FLOAT_NEAR(ctx.result.periodicity,       0.0f, 1e-6f);
    ASSERT_FALSE(ctx.result.periodic_motion);
    ASSERT_INT_EQ(ctx.prev_state, MOTION_STATE_REST);
    ASSERT_FALSE(ctx.initialized);

    return 0;
}

static int test_motion_reset_state(void)
{
    hr_motion_ctx_t ctx;
    hr_motion_init(&ctx);

    static acc_preproc_out_t acc;
    gen_periodic_z(&acc, 0.0f, 0.0f, 9800.0f, 3.0f, 0.9f);
    hr_params_t p = make_test_params();
    hr_motion_run(&ctx, &acc, HR_WINDOW_SIZE, &p);

    /* Dirty the state, then reset */
    hr_motion_reset(&ctx);

    ASSERT_INT_EQ(ctx.result.state, MOTION_STATE_REST);
    ASSERT_FLOAT_NEAR(ctx.result.dynamic_energy,   0.0f, 1e-6f);
    ASSERT_FLOAT_NEAR(ctx.result.dominant_freq_hz,  0.0f, 1e-6f);
    ASSERT_FLOAT_NEAR(ctx.result.periodicity,       0.0f, 1e-6f);
    ASSERT_FALSE(ctx.result.periodic_motion);
    ASSERT_INT_EQ(ctx.prev_state, MOTION_STATE_REST);
    ASSERT_FALSE(ctx.initialized);

    return 0;
}

static int test_motion_invalid_args(void)
{
    hr_motion_ctx_t ctx;
    hr_motion_init(&ctx);
    static acc_preproc_out_t acc;
    gen_constant(&acc, 0.0f, 0.0f, 0.0f);
    hr_params_t p = make_test_params();

    /* Each invalid call must not crash and must not set initialized */
    hr_motion_run(NULL, &acc, HR_WINDOW_SIZE, &p);
    hr_motion_run(&ctx, NULL, HR_WINDOW_SIZE, &p);
    hr_motion_run(&ctx, &acc, HR_WINDOW_SIZE, NULL);
    hr_motion_run(&ctx, &acc, 0, &p);
    hr_motion_run(&ctx, &acc, HR_WINDOW_SIZE + 1, &p);
    ASSERT_FALSE(ctx.initialized);

    /* NULL init/reset must not crash */
    hr_motion_init(NULL);
    hr_motion_reset(NULL);

    return 0;
}

static int test_motion_output_range(void)
{
    hr_motion_ctx_t ctx;
    hr_motion_init(&ctx);
    static acc_preproc_out_t acc;
    gen_periodic_z(&acc, 100.0f, 200.0f, 9800.0f, 3.0f, 0.9f);
    hr_params_t p = make_test_params();
    hr_motion_run(&ctx, &acc, HR_WINDOW_SIZE, &p);

    ASSERT_TRUE(is_valid_state(ctx.result.state));
    ASSERT_TRUE(is_finite_f(ctx.result.dynamic_energy));
    ASSERT_TRUE(is_finite_f(ctx.result.dominant_freq_hz));
    ASSERT_TRUE(is_finite_f(ctx.result.periodicity));
    ASSERT_TRUE(ctx.result.dynamic_energy >= 0.0f);
    ASSERT_TRUE(ctx.result.periodicity >= 0.0f);
    ASSERT_TRUE(ctx.result.periodicity <= 1.0f);
    ASSERT_TRUE(ctx.result.dominant_freq_hz >= 0.0f);

    return 0;
}

/* ================================================================== */
/*  [B] DC removal & energy semantics                                  */
/* ================================================================== */

/**
 * Constant input simulating pure gravity DC offset.
 * After per-axis mean subtraction, dynamic energy must be ~0 => REST.
 * One-vote-veto: constant input classified as WALK/RUN is a hard fail.
 */
static int test_constant_input_rest(void)
{
    hr_motion_ctx_t ctx;
    hr_motion_init(&ctx);
    static acc_preproc_out_t acc;
    hr_params_t p = make_test_params();

    gen_constant(&acc, 100.0f, 200.0f, 9800.0f);
    hr_motion_run(&ctx, &acc, HR_WINDOW_SIZE, &p);

    ASSERT_FLOAT_NEAR(ctx.result.dynamic_energy, 0.0f, 1e-6f);
    ASSERT_INT_EQ(ctx.result.state, MOTION_STATE_REST);

    return 0;
}

static int test_zero_input_rest(void)
{
    hr_motion_ctx_t ctx;
    hr_motion_init(&ctx);
    static acc_preproc_out_t acc;
    hr_params_t p = make_test_params();

    gen_constant(&acc, 0.0f, 0.0f, 0.0f);
    hr_motion_run(&ctx, &acc, HR_WINDOW_SIZE, &p);

    ASSERT_FLOAT_NEAR(ctx.result.dynamic_energy, 0.0f, 1e-6f);
    ASSERT_INT_EQ(ctx.result.state, MOTION_STATE_REST);
    ASSERT_FLOAT_NEAR(ctx.result.dominant_freq_hz, 0.0f, 1e-6f);

    return 0;
}

/**
 * DC offset invariance: identical dynamic component with different DC.
 * One-vote-veto: if DC offset changes the classification, mean removal
 * is broken and gravity will pollute all downstream logic.
 */
static int test_dc_offset_invariance(void)
{
    hr_params_t p = make_test_params();
    static acc_preproc_out_t acc_a, acc_b;

    gen_periodic_z(&acc_a, 100.0f, 200.0f, 9800.0f, 3.0f, 0.9f);
    gen_periodic_z(&acc_b,   0.0f,   0.0f,    0.0f, 3.0f, 0.9f);

    hr_motion_ctx_t ctx_a, ctx_b;
    hr_motion_init(&ctx_a);
    hr_motion_init(&ctx_b);

    hr_motion_run(&ctx_a, &acc_a, HR_WINDOW_SIZE, &p);
    hr_motion_run(&ctx_b, &acc_b, HR_WINDOW_SIZE, &p);

    ASSERT_FLOAT_NEAR(ctx_a.result.dynamic_energy,
                      ctx_b.result.dynamic_energy, 0.01f);
    ASSERT_INT_EQ(ctx_a.result.state, ctx_b.result.state);
    ASSERT_FLOAT_NEAR(ctx_a.result.periodicity,
                      ctx_b.result.periodicity, 0.02f);

    return 0;
}

/* ================================================================== */
/*  [C] Constructed window classification                              */
/* ================================================================== */

/**
 * Walk-like signal: 0.9 Hz sine on z-axis, amplitude 3.0.
 *   dynamic_energy ~ 3^2/2 = 4.5  (above walk=2, below run=10)
 *   |sin| magnitude: ACF peak at lag ~14 => freq ~1.79 Hz (walk band)
 */
static int test_walk_like_window(void)
{
    hr_motion_ctx_t ctx;
    hr_motion_init(&ctx);
    static acc_preproc_out_t acc;
    hr_params_t p = make_test_params();

    gen_periodic_z(&acc, 0.0f, 0.0f, 9800.0f, 3.0f, 0.9f);
    hr_motion_run(&ctx, &acc, HR_WINDOW_SIZE, &p);

    ASSERT_TRUE(ctx.result.dynamic_energy > p.motion.walk_energy_threshold);
    ASSERT_TRUE(ctx.result.dynamic_energy < p.motion.run_energy_threshold);
    ASSERT_TRUE(ctx.result.periodicity > 0.3f);
    ASSERT_TRUE(ctx.result.periodic_motion);
    ASSERT_TRUE(ctx.result.dominant_freq_hz >= 1.2f);
    ASSERT_TRUE(ctx.result.dominant_freq_hz <= 2.5f);
    ASSERT_INT_EQ(ctx.result.state, MOTION_STATE_WALK);

    return 0;
}

/**
 * Run-like signal: 1.5 Hz sine on z-axis, amplitude 6.0.
 *   dynamic_energy ~ 6^2/2 = 18  (above run=10)
 *   |sin| magnitude: ACF peak at lag ~8 => freq ~3.125 Hz (run band)
 */
static int test_run_like_window(void)
{
    hr_motion_ctx_t ctx;
    hr_motion_init(&ctx);
    static acc_preproc_out_t acc;
    hr_params_t p = make_test_params();

    gen_periodic_z(&acc, 0.0f, 0.0f, 9800.0f, 6.0f, 1.5f);
    hr_motion_run(&ctx, &acc, HR_WINDOW_SIZE, &p);

    ASSERT_TRUE(ctx.result.dynamic_energy > p.motion.run_energy_threshold);
    ASSERT_TRUE(ctx.result.periodicity > 0.3f);
    ASSERT_TRUE(ctx.result.periodic_motion);
    ASSERT_TRUE(ctx.result.dominant_freq_hz >= 2.5f);
    ASSERT_INT_EQ(ctx.result.state, MOTION_STATE_RUN);

    return 0;
}

/**
 * Irregular signal: random non-periodic, medium energy.
 *   Uniform random +/-3.0 on z => energy ~ 3.0 (above rest=0.5)
 *   No periodic structure => periodicity < 0.3
 */
static int test_irregular_high_energy(void)
{
    hr_motion_ctx_t ctx;
    hr_motion_init(&ctx);
    static acc_preproc_out_t acc;
    hr_params_t p = make_test_params();

    gen_irregular_z(&acc, 0.0f, 0.0f, 9800.0f, 3.0f, 42u);
    hr_motion_run(&ctx, &acc, HR_WINDOW_SIZE, &p);

    ASSERT_TRUE(ctx.result.dynamic_energy > p.motion.rest_energy_threshold);
    ASSERT_TRUE(ctx.result.periodicity < 0.3f);
    ASSERT_FALSE(ctx.result.periodic_motion);
    ASSERT_INT_EQ(ctx.result.state, MOTION_STATE_IRREGULAR);

    return 0;
}

/**
 * Low-energy rest: tiny perturbation well below rest threshold.
 *   amplitude 0.01 => energy ~ 5e-5 << 0.5
 */
static int test_rest_low_energy(void)
{
    hr_motion_ctx_t ctx;
    hr_motion_init(&ctx);
    static acc_preproc_out_t acc;
    hr_params_t p = make_test_params();

    gen_periodic_z(&acc, 100.0f, 200.0f, 9800.0f, 0.01f, 1.5f);
    hr_motion_run(&ctx, &acc, HR_WINDOW_SIZE, &p);

    ASSERT_TRUE(ctx.result.dynamic_energy < p.motion.rest_energy_threshold);
    ASSERT_INT_EQ(ctx.result.state, MOTION_STATE_REST);

    return 0;
}

/**
 * Verify that changing energy thresholds directly affects classification.
 * Same walk-like signal, three parameter settings:
 *   1. Normal thresholds   => WALK
 *   2. rest >> energy      => REST  (energy absorbed by rest zone)
 *   3. walk,run >> energy  => IRREGULAR  (periodic but below walk energy)
 */
static int test_threshold_influence(void)
{
    hr_motion_ctx_t ctx;
    static acc_preproc_out_t acc;
    hr_params_t p = make_test_params();

    gen_periodic_z(&acc, 0.0f, 0.0f, 0.0f, 3.0f, 0.9f);

    /* Case 1: normal thresholds => WALK */
    hr_motion_init(&ctx);
    hr_motion_run(&ctx, &acc, HR_WINDOW_SIZE, &p);
    ASSERT_INT_EQ(ctx.result.state, MOTION_STATE_WALK);

    /* Case 2: rest threshold above signal energy => REST */
    hr_params_t p_rest = p;
    p_rest.motion.rest_energy_threshold = 100.0f;
    hr_motion_init(&ctx);
    hr_motion_run(&ctx, &acc, HR_WINDOW_SIZE, &p_rest);
    ASSERT_INT_EQ(ctx.result.state, MOTION_STATE_REST);

    /* Case 3: walk & run thresholds far above energy => IRREGULAR
     * (periodic but doesn't meet walk/run energy requirements) */
    hr_params_t p_high = p;
    p_high.motion.walk_energy_threshold = 100.0f;
    p_high.motion.run_energy_threshold  = 200.0f;
    hr_motion_init(&ctx);
    hr_motion_run(&ctx, &acc, HR_WINDOW_SIZE, &p_high);
    ASSERT_INT_EQ(ctx.result.state, MOTION_STATE_IRREGULAR);

    return 0;
}

/* ================================================================== */
/*  [D] Feature interpretability                                       */
/* ================================================================== */

/** Periodic signal must produce higher periodicity than random noise. */
static int test_periodicity_higher_for_periodic(void)
{
    hr_params_t p = make_test_params();
    static acc_preproc_out_t acc_periodic, acc_random;

    gen_periodic_z(&acc_periodic, 0.0f, 0.0f, 0.0f, 3.0f, 0.9f);
    gen_irregular_z(&acc_random,  0.0f, 0.0f, 0.0f, 3.0f, 42u);

    hr_motion_ctx_t ctx_p, ctx_r;
    hr_motion_init(&ctx_p);
    hr_motion_init(&ctx_r);

    hr_motion_run(&ctx_p, &acc_periodic, HR_WINDOW_SIZE, &p);
    hr_motion_run(&ctx_r, &acc_random,   HR_WINDOW_SIZE, &p);

    ASSERT_TRUE(ctx_p.result.periodicity > ctx_r.result.periodicity);
    ASSERT_TRUE(ctx_p.result.periodicity > 0.3f);
    ASSERT_TRUE(ctx_r.result.periodicity < 0.3f);

    return 0;
}

/**
 * Walk signal dominant frequency must be lower than run signal.
 *   Walk: 0.9 Hz input => |sin| freq ~1.79 Hz
 *   Run:  1.5 Hz input => |sin| freq ~3.125 Hz
 * One-vote-veto: if walk freq >= run freq, the frequency estimation
 * is fundamentally broken.
 */
static int test_frequency_walk_vs_run(void)
{
    hr_params_t p = make_test_params();
    static acc_preproc_out_t acc_walk, acc_run;

    gen_periodic_z(&acc_walk, 0.0f, 0.0f, 0.0f, 3.0f, 0.9f);
    gen_periodic_z(&acc_run,  0.0f, 0.0f, 0.0f, 6.0f, 1.5f);

    hr_motion_ctx_t ctx_w, ctx_r;
    hr_motion_init(&ctx_w);
    hr_motion_init(&ctx_r);

    hr_motion_run(&ctx_w, &acc_walk, HR_WINDOW_SIZE, &p);
    hr_motion_run(&ctx_r, &acc_run,  HR_WINDOW_SIZE, &p);

    ASSERT_TRUE(ctx_w.result.dominant_freq_hz > 0.0f);
    ASSERT_TRUE(ctx_r.result.dominant_freq_hz > 0.0f);
    ASSERT_TRUE(ctx_w.result.dominant_freq_hz < ctx_r.result.dominant_freq_hz);

    return 0;
}

/**
 * periodic_motion flag must agree with periodicity vs internal threshold.
 * V1 threshold is 0.3 (design constant MOTION_PERIODICITY_THRESHOLD).
 */
static int test_periodic_flag_consistency(void)
{
    hr_params_t p = make_test_params();
    static acc_preproc_out_t acc;
    hr_motion_ctx_t ctx;

    /* Periodic signal: periodicity > 0.3 => periodic_motion = true */
    hr_motion_init(&ctx);
    gen_periodic_z(&acc, 0.0f, 0.0f, 0.0f, 3.0f, 0.9f);
    hr_motion_run(&ctx, &acc, HR_WINDOW_SIZE, &p);
    ASSERT_TRUE(ctx.result.periodicity >= 0.3f);
    ASSERT_TRUE(ctx.result.periodic_motion);

    /* Random signal: periodicity < 0.3 => periodic_motion = false */
    hr_motion_init(&ctx);
    gen_irregular_z(&acc, 0.0f, 0.0f, 0.0f, 3.0f, 42u);
    hr_motion_run(&ctx, &acc, HR_WINDOW_SIZE, &p);
    ASSERT_TRUE(ctx.result.periodicity < 0.3f);
    ASSERT_FALSE(ctx.result.periodic_motion);

    /* Constant signal: near-zero periodicity => false */
    hr_motion_init(&ctx);
    gen_constant(&acc, 100.0f, 200.0f, 9800.0f);
    hr_motion_run(&ctx, &acc, HR_WINDOW_SIZE, &p);
    ASSERT_FLOAT_NEAR(ctx.result.periodicity, 0.0f, 1e-6f);
    ASSERT_FALSE(ctx.result.periodic_motion);

    return 0;
}

/**
 * dominant_freq_hz must be 0 when periodicity is below threshold
 * or signal has near-zero dynamic energy (flatline / constant).
 */
static int test_freq_zero_when_aperiodic(void)
{
    hr_params_t p = make_test_params();
    static acc_preproc_out_t acc;
    hr_motion_ctx_t ctx;

    /* Constant input => freq 0 */
    hr_motion_init(&ctx);
    gen_constant(&acc, 100.0f, 200.0f, 9800.0f);
    hr_motion_run(&ctx, &acc, HR_WINDOW_SIZE, &p);
    ASSERT_FLOAT_NEAR(ctx.result.dominant_freq_hz, 0.0f, 1e-6f);

    /* Random noise (periodicity < threshold) => freq 0 */
    hr_motion_init(&ctx);
    gen_irregular_z(&acc, 0.0f, 0.0f, 0.0f, 3.0f, 42u);
    hr_motion_run(&ctx, &acc, HR_WINDOW_SIZE, &p);
    ASSERT_FLOAT_NEAR(ctx.result.dominant_freq_hz, 0.0f, 1e-6f);

    return 0;
}

/* ================================================================== */
/*  [E] Multi-cycle & state tracking                                   */
/* ================================================================== */

static int test_initialized_flag(void)
{
    hr_motion_ctx_t ctx;
    hr_motion_init(&ctx);
    static acc_preproc_out_t acc;
    hr_params_t p = make_test_params();

    ASSERT_FALSE(ctx.initialized);

    gen_constant(&acc, 100.0f, 200.0f, 9800.0f);
    hr_motion_run(&ctx, &acc, HR_WINDOW_SIZE, &p);

    ASSERT_TRUE(ctx.initialized);

    return 0;
}

/**
 * prev_state is updated to the current cycle's state after each run.
 * V1 does NOT use prev_state in classification (no cross-cycle
 * hysteresis); this test verifies the bookkeeping is correct.
 */
static int test_prev_state_updates(void)
{
    hr_motion_ctx_t ctx;
    hr_motion_init(&ctx);
    static acc_preproc_out_t acc;
    hr_params_t p = make_test_params();

    /* Cycle 1: REST */
    gen_constant(&acc, 100.0f, 200.0f, 9800.0f);
    hr_motion_run(&ctx, &acc, HR_WINDOW_SIZE, &p);
    ASSERT_INT_EQ(ctx.result.state, MOTION_STATE_REST);
    ASSERT_INT_EQ(ctx.prev_state,   MOTION_STATE_REST);

    /* Cycle 2: WALK */
    gen_periodic_z(&acc, 0.0f, 0.0f, 9800.0f, 3.0f, 0.9f);
    hr_motion_run(&ctx, &acc, HR_WINDOW_SIZE, &p);
    ASSERT_INT_EQ(ctx.result.state, MOTION_STATE_WALK);
    ASSERT_INT_EQ(ctx.prev_state,   MOTION_STATE_WALK);

    /* Cycle 3: IRREGULAR */
    gen_irregular_z(&acc, 0.0f, 0.0f, 9800.0f, 3.0f, 42u);
    hr_motion_run(&ctx, &acc, HR_WINDOW_SIZE, &p);
    ASSERT_INT_EQ(ctx.result.state, MOTION_STATE_IRREGULAR);
    ASSERT_INT_EQ(ctx.prev_state,   MOTION_STATE_IRREGULAR);

    return 0;
}

/**
 * Feeding the same window repeatedly must produce identical results
 * every cycle (V1 is purely per-window, no cross-cycle state influence).
 */
static int test_repeat_run_stability(void)
{
    hr_motion_ctx_t ctx;
    hr_motion_init(&ctx);
    static acc_preproc_out_t acc;
    hr_params_t p = make_test_params();

    gen_periodic_z(&acc, 0.0f, 0.0f, 9800.0f, 3.0f, 0.9f);

    motion_state_t first_state = MOTION_STATE_REST;
    float first_energy = 0.0f;

    for (int cycle = 0; cycle < 5; cycle++) {
        hr_motion_run(&ctx, &acc, HR_WINDOW_SIZE, &p);
        if (cycle == 0) {
            first_state  = ctx.result.state;
            first_energy = ctx.result.dynamic_energy;
        } else {
            ASSERT_INT_EQ(ctx.result.state, first_state);
            ASSERT_FLOAT_NEAR(ctx.result.dynamic_energy, first_energy, 1e-6f);
        }
    }

    return 0;
}

/**
 * Reset clears all state; subsequent run behaves like cold start.
 * No residual state from previous cycles should leak through.
 */
static int test_reset_clears_state(void)
{
    hr_motion_ctx_t ctx;
    hr_motion_init(&ctx);
    static acc_preproc_out_t acc;
    hr_params_t p = make_test_params();

    /* Run with walk signal */
    gen_periodic_z(&acc, 0.0f, 0.0f, 9800.0f, 3.0f, 0.9f);
    hr_motion_run(&ctx, &acc, HR_WINDOW_SIZE, &p);
    ASSERT_TRUE(ctx.initialized);
    ASSERT_INT_EQ(ctx.result.state, MOTION_STATE_WALK);

    /* Reset */
    hr_motion_reset(&ctx);
    ASSERT_FALSE(ctx.initialized);
    ASSERT_INT_EQ(ctx.prev_state,   MOTION_STATE_REST);
    ASSERT_INT_EQ(ctx.result.state, MOTION_STATE_REST);
    ASSERT_FLOAT_NEAR(ctx.result.dynamic_energy, 0.0f, 1e-6f);

    /* Run with constant => should behave like fresh start */
    gen_constant(&acc, 100.0f, 200.0f, 9800.0f);
    hr_motion_run(&ctx, &acc, HR_WINDOW_SIZE, &p);
    ASSERT_INT_EQ(ctx.result.state, MOTION_STATE_REST);
    ASSERT_TRUE(ctx.initialized);

    return 0;
}

/* ================================================================== */
/*  main                                                               */
/* ================================================================== */

int main(void)
{
    printf("==========================================\n");
    printf("  M4 Motion State Detection -- Test Suite\n");
    printf("==========================================\n\n");

    printf("[A] Contract tests:\n");
    RUN_TEST(test_motion_init_state);
    RUN_TEST(test_motion_reset_state);
    RUN_TEST(test_motion_invalid_args);
    RUN_TEST(test_motion_output_range);

    printf("\n[B] DC removal & energy semantics:\n");
    RUN_TEST(test_constant_input_rest);
    RUN_TEST(test_zero_input_rest);
    RUN_TEST(test_dc_offset_invariance);

    printf("\n[C] Constructed window classification:\n");
    RUN_TEST(test_walk_like_window);
    RUN_TEST(test_run_like_window);
    RUN_TEST(test_irregular_high_energy);
    RUN_TEST(test_rest_low_energy);
    RUN_TEST(test_threshold_influence);

    printf("\n[D] Feature interpretability:\n");
    RUN_TEST(test_periodicity_higher_for_periodic);
    RUN_TEST(test_frequency_walk_vs_run);
    RUN_TEST(test_periodic_flag_consistency);
    RUN_TEST(test_freq_zero_when_aperiodic);

    printf("\n[E] Multi-cycle & state tracking:\n");
    RUN_TEST(test_initialized_flag);
    RUN_TEST(test_prev_state_updates);
    RUN_TEST(test_repeat_run_stability);
    RUN_TEST(test_reset_clears_state);

    TEST_SUMMARY();
    return TEST_EXIT_CODE();
}
