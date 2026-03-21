/**
 * @file test_hr_sqi.c
 * @brief M3 SQI & Channel Selection — regression test suite.
 *
 * Tests cover:
 *   [A] Contract tests        — init / reset / invalid args
 *   [B] Output range          — SQI ∈ [0,1], main_ch/backup_ch ∈ [0,3]
 *   [C] Signal discrimination — flatline low, periodic high, usable flag
 *   [D] Channel selection     — main/backup, all-bad, permutation
 *   [E] Channel independence  — changing one ch doesn't affect others
 *   [F] Hysteresis (O11)      — margin blocking, sustained switch,
 *                                unusable immediate switch, counter reset
 *   [G] Stability             — multi-cycle, large-value robustness
 *
 * Build & run:
 *   make -C tests run
 * or manually:
 *   cc -std=c11 -Wall -Wextra -Iinclude -Isrc \
 *      tests/test_hr_sqi.c src/hr_sqi.c -lm -o test_hr_sqi
 *   ./test_hr_sqi
 */

#include "test_framework.h"
#include "hr_sqi.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ================================================================== */
/*  Static buffers — kept off the stack (ppg_preproc_out_t ≈ 3.2 KB)  */
/* ================================================================== */

static hr_sqi_ctx_t      g_sqi;
static ppg_preproc_out_t  g_ppg;

/* ================================================================== */
/*  Helper: default params with explicit SQI fields                    */
/* ================================================================== */

static hr_params_t make_default_sqi_params(void)
{
    hr_params_t p;
    memset(&p, 0, sizeof(p));
    p.sqi.sqi_good_threshold = 0.6f;
    p.sqi.sqi_poor_threshold = 0.3f;
    p.sqi.switch_margin      = 0.05f;
    p.sqi.switch_hold_count  = 3;
    return p;
}

/* ================================================================== */
/*  Signal construction helpers                                        */
/*                                                                     */
/*  All operate on a single channel of g_ppg.  Deterministic and       */
/*  reproducible — no external random sources.                         */
/* ================================================================== */

static void fill_channel_sine(ppg_preproc_out_t *ppg, int ch,
                               float freq_hz, uint16_t n)
{
    for (uint16_t i = 0; i < n; i++) {
        ppg->data[i][ch] = sinf(2.0f * (float)M_PI * freq_hz
                                * (float)i / 25.0f);
    }
}

static void fill_channel_zero(ppg_preproc_out_t *ppg, int ch, uint16_t n)
{
    for (uint16_t i = 0; i < n; i++) {
        ppg->data[i][ch] = 0.0f;
    }
}

static void fill_channel_const(ppg_preproc_out_t *ppg, int ch,
                                float value, uint16_t n)
{
    for (uint16_t i = 0; i < n; i++) {
        ppg->data[i][ch] = value;
    }
}

/** Deterministic pseudo-noise via Numerical Recipes LCG.  Values ∈ [-1, 1). */
static void fill_channel_noise(ppg_preproc_out_t *ppg, int ch,
                                uint32_t seed, uint16_t n)
{
    uint32_t s = seed;
    for (uint16_t i = 0; i < n; i++) {
        s = s * 1664525u + 1013904223u;
        float u = (float)(s >> 16) / 65536.0f;
        ppg->data[i][ch] = 2.0f * u - 1.0f;
    }
}

static void fill_all_zero(ppg_preproc_out_t *ppg, uint16_t n)
{
    for (int ch = 0; ch < 4; ch++)
        fill_channel_zero(ppg, ch, n);
}

/* ================================================================== */
/*  [A] Contract tests                                                 */
/* ================================================================== */

static int test_sqi_init_state(void)
{
    hr_sqi_init(&g_sqi);

    ASSERT_FALSE(g_sqi.initialized);
    ASSERT_INT_EQ(g_sqi.result.main_ch, 0);
    ASSERT_INT_EQ(g_sqi.result.backup_ch, 0);
    ASSERT_FLOAT_NEAR(g_sqi.result.sqi_main, 0.0f, 1e-9f);

    for (int ch = 0; ch < 4; ch++) {
        ASSERT_FLOAT_NEAR(g_sqi.result.ch[ch].total, 0.0f, 1e-9f);
        ASSERT_FLOAT_NEAR(g_sqi.result.ch[ch].periodicity, 0.0f, 1e-9f);
        ASSERT_FLOAT_NEAR(g_sqi.result.ch[ch].peak_regularity, 0.0f, 1e-9f);
        ASSERT_FALSE(g_sqi.result.ch[ch].usable);
    }

    ASSERT_INT_EQ(g_sqi.switch_counter, 0);
    return 0;
}

static int test_sqi_reset_state(void)
{
    hr_params_t p = make_default_sqi_params();

    hr_sqi_init(&g_sqi);
    fill_channel_sine(&g_ppg, 0, 1.0f, HR_WINDOW_SIZE);
    fill_all_zero(&g_ppg, HR_WINDOW_SIZE);
    fill_channel_sine(&g_ppg, 0, 1.0f, HR_WINDOW_SIZE);
    hr_sqi_run(&g_sqi, &g_ppg, HR_WINDOW_SIZE, &p);

    ASSERT_TRUE(g_sqi.initialized);

    hr_sqi_reset(&g_sqi);

    ASSERT_FALSE(g_sqi.initialized);
    ASSERT_INT_EQ(g_sqi.switch_counter, 0);
    ASSERT_FLOAT_NEAR(g_sqi.result.sqi_main, 0.0f, 1e-9f);

    for (int ch = 0; ch < 4; ch++) {
        ASSERT_FLOAT_NEAR(g_sqi.result.ch[ch].total, 0.0f, 1e-9f);
    }
    return 0;
}

static int test_sqi_invalid_args(void)
{
    hr_params_t p = make_default_sqi_params();

    hr_sqi_init(&g_sqi);
    fill_channel_sine(&g_ppg, 0, 1.0f, HR_WINDOW_SIZE);

    hr_sqi_run(NULL, &g_ppg, HR_WINDOW_SIZE, &p);
    hr_sqi_run(&g_sqi, NULL, HR_WINDOW_SIZE, &p);
    hr_sqi_run(&g_sqi, &g_ppg, HR_WINDOW_SIZE, NULL);
    hr_sqi_run(&g_sqi, &g_ppg, 0, &p);
    hr_sqi_run(&g_sqi, &g_ppg, HR_WINDOW_SIZE + 1, &p);

    ASSERT_FALSE(g_sqi.initialized);

    hr_sqi_init(NULL);
    hr_sqi_reset(NULL);

    return 0;
}

/* ================================================================== */
/*  [B] Output range & consistency                                     */
/* ================================================================== */

static int test_sqi_output_range(void)
{
    hr_params_t p = make_default_sqi_params();
    hr_sqi_init(&g_sqi);

    fill_channel_sine(&g_ppg, 0, 1.0f, HR_WINDOW_SIZE);
    fill_channel_zero(&g_ppg, 1, HR_WINDOW_SIZE);
    fill_channel_noise(&g_ppg, 2, 42u, HR_WINDOW_SIZE);
    fill_channel_const(&g_ppg, 3, 1000.0f, HR_WINDOW_SIZE);

    hr_sqi_run(&g_sqi, &g_ppg, HR_WINDOW_SIZE, &p);

    for (int ch = 0; ch < 4; ch++) {
        ASSERT_TRUE(g_sqi.result.ch[ch].total >= 0.0f);
        ASSERT_TRUE(g_sqi.result.ch[ch].total <= 1.0f);
        ASSERT_TRUE(g_sqi.result.ch[ch].periodicity >= 0.0f);
        ASSERT_TRUE(g_sqi.result.ch[ch].periodicity <= 1.0f);
        ASSERT_TRUE(g_sqi.result.ch[ch].peak_regularity >= 0.0f);
        ASSERT_TRUE(g_sqi.result.ch[ch].peak_regularity <= 1.0f);
    }

    ASSERT_TRUE(g_sqi.result.main_ch <= 3);
    ASSERT_TRUE(g_sqi.result.backup_ch <= 3);
    ASSERT_TRUE(g_sqi.result.sqi_main >= 0.0f);
    ASSERT_TRUE(g_sqi.result.sqi_main <= 1.0f);

    return 0;
}

static int test_sqi_main_equals_channel_total(void)
{
    hr_params_t p = make_default_sqi_params();
    hr_sqi_init(&g_sqi);

    fill_channel_sine(&g_ppg, 0, 1.0f, HR_WINDOW_SIZE);
    fill_channel_noise(&g_ppg, 1, 99u, HR_WINDOW_SIZE);
    fill_channel_zero(&g_ppg, 2, HR_WINDOW_SIZE);
    fill_channel_sine(&g_ppg, 3, 2.0f, HR_WINDOW_SIZE);

    hr_sqi_run(&g_sqi, &g_ppg, HR_WINDOW_SIZE, &p);

    uint8_t mc = g_sqi.result.main_ch;
    ASSERT_FLOAT_NEAR(g_sqi.result.sqi_main,
                      g_sqi.result.ch[mc].total, 1e-9f);
    return 0;
}

/* ================================================================== */
/*  [C] Signal quality discrimination                                  */
/* ================================================================== */

static int test_flatline_scores_low(void)
{
    hr_params_t p = make_default_sqi_params();

    /* All-zero flatline */
    hr_sqi_init(&g_sqi);
    fill_all_zero(&g_ppg, HR_WINDOW_SIZE);
    hr_sqi_run(&g_sqi, &g_ppg, HR_WINDOW_SIZE, &p);

    for (int ch = 0; ch < 4; ch++) {
        ASSERT_FLOAT_NEAR(g_sqi.result.ch[ch].total, 0.0f, 1e-6f);
        ASSERT_FLOAT_NEAR(g_sqi.result.ch[ch].periodicity, 0.0f, 1e-6f);
        ASSERT_FLOAT_NEAR(g_sqi.result.ch[ch].peak_regularity, 0.0f, 1e-6f);
        ASSERT_FALSE(g_sqi.result.ch[ch].usable);
    }

    /* Constant non-zero value should also score zero */
    hr_sqi_init(&g_sqi);
    for (int ch = 0; ch < 4; ch++)
        fill_channel_const(&g_ppg, ch, 42.0f, HR_WINDOW_SIZE);
    hr_sqi_run(&g_sqi, &g_ppg, HR_WINDOW_SIZE, &p);

    for (int ch = 0; ch < 4; ch++) {
        ASSERT_FLOAT_NEAR(g_sqi.result.ch[ch].total, 0.0f, 1e-6f);
    }

    return 0;
}

static int test_periodic_scores_high(void)
{
    hr_params_t p = make_default_sqi_params();
    hr_sqi_init(&g_sqi);

    /*
     * 1 Hz sine at 25 Hz: period = 25 samples, well within ACF lag
     * range [7, 38].  Expected: periodicity > 0.7, peak_regularity > 0.8,
     * total > 0.8.
     */
    fill_channel_sine(&g_ppg, 0, 1.0f, HR_WINDOW_SIZE);
    fill_channel_zero(&g_ppg, 1, HR_WINDOW_SIZE);
    fill_channel_zero(&g_ppg, 2, HR_WINDOW_SIZE);
    fill_channel_zero(&g_ppg, 3, HR_WINDOW_SIZE);

    hr_sqi_run(&g_sqi, &g_ppg, HR_WINDOW_SIZE, &p);

    ASSERT_TRUE(g_sqi.result.ch[0].periodicity > 0.7f);
    ASSERT_TRUE(g_sqi.result.ch[0].peak_regularity > 0.8f);
    ASSERT_TRUE(g_sqi.result.ch[0].total > 0.8f);
    ASSERT_TRUE(g_sqi.result.ch[0].usable);

    /* Sine at 2 Hz (period=12.5, lag≈12-13, within [7,38]) — also high */
    hr_sqi_init(&g_sqi);
    fill_channel_sine(&g_ppg, 0, 2.0f, HR_WINDOW_SIZE);
    hr_sqi_run(&g_sqi, &g_ppg, HR_WINDOW_SIZE, &p);

    ASSERT_TRUE(g_sqi.result.ch[0].periodicity > 0.7f);
    ASSERT_TRUE(g_sqi.result.ch[0].total > 0.8f);

    /* Periodic >> flatline */
    ASSERT_TRUE(g_sqi.result.ch[0].total > g_sqi.result.ch[1].total + 0.5f);

    return 0;
}

static int test_usable_threshold(void)
{
    hr_params_t p = make_default_sqi_params();
    p.sqi.sqi_poor_threshold = 0.5f;

    hr_sqi_init(&g_sqi);
    fill_channel_sine(&g_ppg, 0, 1.0f, HR_WINDOW_SIZE);
    fill_channel_zero(&g_ppg, 1, HR_WINDOW_SIZE);
    fill_channel_zero(&g_ppg, 2, HR_WINDOW_SIZE);
    fill_channel_zero(&g_ppg, 3, HR_WINDOW_SIZE);

    hr_sqi_run(&g_sqi, &g_ppg, HR_WINDOW_SIZE, &p);

    /* ch0 sine (total ~0.9) should be usable with threshold 0.5 */
    ASSERT_TRUE(g_sqi.result.ch[0].usable);
    /* ch1 flat (total = 0) should NOT be usable */
    ASSERT_FALSE(g_sqi.result.ch[1].usable);

    /* Lower threshold: everything with SQI >= 0 is usable */
    p.sqi.sqi_poor_threshold = 0.0f;
    hr_sqi_init(&g_sqi);
    hr_sqi_run(&g_sqi, &g_ppg, HR_WINDOW_SIZE, &p);

    for (int ch = 0; ch < 4; ch++) {
        ASSERT_TRUE(g_sqi.result.ch[ch].usable);
    }

    return 0;
}

/* ================================================================== */
/*  [D] Channel selection                                              */
/* ================================================================== */

static int test_select_main_and_backup(void)
{
    hr_params_t p = make_default_sqi_params();
    hr_sqi_init(&g_sqi);

    /*
     * ch0 and ch1: identical 1 Hz sine (both usable, tied SQI).
     * ch2 and ch3: flat (SQI = 0, not usable).
     * Tie-breaking: ch0 wins (lower index), backup = ch1.
     */
    fill_channel_sine(&g_ppg, 0, 1.0f, HR_WINDOW_SIZE);
    fill_channel_sine(&g_ppg, 1, 1.0f, HR_WINDOW_SIZE);
    fill_channel_zero(&g_ppg, 2, HR_WINDOW_SIZE);
    fill_channel_zero(&g_ppg, 3, HR_WINDOW_SIZE);

    hr_sqi_run(&g_sqi, &g_ppg, HR_WINDOW_SIZE, &p);

    ASSERT_INT_EQ(g_sqi.result.main_ch, 0);
    ASSERT_INT_EQ(g_sqi.result.backup_ch, 1);
    ASSERT_TRUE(g_sqi.result.backup_ch != g_sqi.result.main_ch);

    /* Only one good channel → backup == main (no independent backup) */
    hr_sqi_init(&g_sqi);
    fill_channel_sine(&g_ppg, 0, 1.0f, HR_WINDOW_SIZE);
    fill_channel_zero(&g_ppg, 1, HR_WINDOW_SIZE);
    fill_channel_zero(&g_ppg, 2, HR_WINDOW_SIZE);
    fill_channel_zero(&g_ppg, 3, HR_WINDOW_SIZE);

    hr_sqi_run(&g_sqi, &g_ppg, HR_WINDOW_SIZE, &p);

    ASSERT_INT_EQ(g_sqi.result.main_ch, 0);
    ASSERT_INT_EQ(g_sqi.result.backup_ch, g_sqi.result.main_ch);

    return 0;
}

static int test_all_bad_channels(void)
{
    hr_params_t p = make_default_sqi_params();
    hr_sqi_init(&g_sqi);

    fill_all_zero(&g_ppg, HR_WINDOW_SIZE);
    hr_sqi_run(&g_sqi, &g_ppg, HR_WINDOW_SIZE, &p);

    ASSERT_TRUE(g_sqi.result.main_ch <= 3);
    ASSERT_INT_EQ(g_sqi.result.backup_ch, g_sqi.result.main_ch);

    for (int ch = 0; ch < 4; ch++) {
        ASSERT_FLOAT_NEAR(g_sqi.result.ch[ch].total, 0.0f, 1e-6f);
        ASSERT_FALSE(g_sqi.result.ch[ch].usable);
    }

    ASSERT_FLOAT_NEAR(g_sqi.result.sqi_main, 0.0f, 1e-6f);

    return 0;
}

static int test_channel_permutation(void)
{
    hr_params_t p = make_default_sqi_params();

    /*
     * Put the only good signal on each channel index in turn.
     * main_ch must follow the good channel.
     */
    for (int best = 0; best < 4; best++) {
        hr_sqi_init(&g_sqi);
        for (int c = 0; c < 4; c++) {
            if (c == best)
                fill_channel_sine(&g_ppg, c, 1.0f, HR_WINDOW_SIZE);
            else
                fill_channel_zero(&g_ppg, c, HR_WINDOW_SIZE);
        }
        hr_sqi_run(&g_sqi, &g_ppg, HR_WINDOW_SIZE, &p);
        ASSERT_INT_EQ(g_sqi.result.main_ch, best);
    }
    return 0;
}

/* ================================================================== */
/*  [E] Channel independence                                           */
/* ================================================================== */

static int test_channel_independence(void)
{
    hr_params_t p = make_default_sqi_params();
    float ref_total[4];

    /*
     * Run 1: all channels = 1 Hz sine.
     * Record each channel's total SQI.
     */
    hr_sqi_init(&g_sqi);
    for (int ch = 0; ch < 4; ch++)
        fill_channel_sine(&g_ppg, ch, 1.0f, HR_WINDOW_SIZE);
    hr_sqi_run(&g_sqi, &g_ppg, HR_WINDOW_SIZE, &p);
    for (int ch = 0; ch < 4; ch++)
        ref_total[ch] = g_sqi.result.ch[ch].total;

    /*
     * Run 2: change ch2 to noise, keep ch0/ch1/ch3 as sine.
     * ch0, ch1, ch3 totals must be unchanged; ch2 must differ.
     */
    hr_sqi_init(&g_sqi);
    fill_channel_noise(&g_ppg, 2, 12345u, HR_WINDOW_SIZE);
    hr_sqi_run(&g_sqi, &g_ppg, HR_WINDOW_SIZE, &p);

    ASSERT_FLOAT_NEAR(g_sqi.result.ch[0].total, ref_total[0], 1e-6f);
    ASSERT_FLOAT_NEAR(g_sqi.result.ch[1].total, ref_total[1], 1e-6f);
    ASSERT_FLOAT_NEAR(g_sqi.result.ch[3].total, ref_total[3], 1e-6f);

    float diff2 = g_sqi.result.ch[2].total - ref_total[2];
    if (diff2 < 0) diff2 = -diff2;
    ASSERT_TRUE(diff2 > 0.1f);

    return 0;
}

/* ================================================================== */
/*  [F] Hysteresis (O11)                                               */
/* ================================================================== */

static int test_no_switch_insufficient_margin(void)
{
    hr_params_t p = make_default_sqi_params();
    p.sqi.switch_margin     = 2.0f;   /* impossibly high */
    p.sqi.sqi_poor_threshold = 0.0f;  /* everything stays usable */
    p.sqi.switch_hold_count = 1;

    hr_sqi_init(&g_sqi);

    /* Cycle 1: ch0 = good → main = 0 */
    fill_channel_sine(&g_ppg, 0, 1.0f, HR_WINDOW_SIZE);
    fill_channel_zero(&g_ppg, 1, HR_WINDOW_SIZE);
    fill_channel_zero(&g_ppg, 2, HR_WINDOW_SIZE);
    fill_channel_zero(&g_ppg, 3, HR_WINDOW_SIZE);
    hr_sqi_run(&g_sqi, &g_ppg, HR_WINDOW_SIZE, &p);
    ASSERT_INT_EQ(g_sqi.result.main_ch, 0);

    /* Cycle 2: ch1 = best, ch0 = flat (SQI=0 but usable at threshold 0) */
    fill_channel_zero(&g_ppg, 0, HR_WINDOW_SIZE);
    fill_channel_sine(&g_ppg, 1, 1.0f, HR_WINDOW_SIZE);
    hr_sqi_run(&g_sqi, &g_ppg, HR_WINDOW_SIZE, &p);

    /* margin=2.0: ch1 SQI (~0.9) < ch0 SQI (0) + 2.0 → Rule 3 → keep */
    ASSERT_INT_EQ(g_sqi.result.main_ch, 0);

    /* Cycle 3: still the same — must not switch */
    hr_sqi_run(&g_sqi, &g_ppg, HR_WINDOW_SIZE, &p);
    ASSERT_INT_EQ(g_sqi.result.main_ch, 0);

    return 0;
}

static int test_switch_after_sustained_advantage(void)
{
    hr_params_t p = make_default_sqi_params();
    p.sqi.switch_margin      = 0.05f;
    p.sqi.sqi_poor_threshold = 0.0f;   /* ch0 stays usable even at SQI=0 */
    p.sqi.switch_hold_count  = 3;

    hr_sqi_init(&g_sqi);

    /* Cycle 1: ch0 = good → main = 0 */
    fill_channel_sine(&g_ppg, 0, 1.0f, HR_WINDOW_SIZE);
    fill_channel_zero(&g_ppg, 1, HR_WINDOW_SIZE);
    fill_channel_zero(&g_ppg, 2, HR_WINDOW_SIZE);
    fill_channel_zero(&g_ppg, 3, HR_WINDOW_SIZE);
    hr_sqi_run(&g_sqi, &g_ppg, HR_WINDOW_SIZE, &p);
    ASSERT_INT_EQ(g_sqi.result.main_ch, 0);

    /* Cycles 2-4: ch1 clearly better, ch0 = flat */
    fill_channel_zero(&g_ppg, 0, HR_WINDOW_SIZE);
    fill_channel_sine(&g_ppg, 1, 1.0f, HR_WINDOW_SIZE);

    /* Cycle 2: counter = 1, no switch yet */
    hr_sqi_run(&g_sqi, &g_ppg, HR_WINDOW_SIZE, &p);
    ASSERT_INT_EQ(g_sqi.result.main_ch, 0);

    /* Cycle 3: counter = 2, no switch yet */
    hr_sqi_run(&g_sqi, &g_ppg, HR_WINDOW_SIZE, &p);
    ASSERT_INT_EQ(g_sqi.result.main_ch, 0);

    /* Cycle 4: counter = 3 >= hold_count → SWITCH */
    hr_sqi_run(&g_sqi, &g_ppg, HR_WINDOW_SIZE, &p);
    ASSERT_INT_EQ(g_sqi.result.main_ch, 1);

    return 0;
}

static int test_immediate_switch_when_unusable(void)
{
    hr_params_t p = make_default_sqi_params();
    /* default poor_threshold = 0.3: ch0 SQI=0 → unusable */

    hr_sqi_init(&g_sqi);

    /* Cycle 1: ch0 = good → main = 0 */
    fill_channel_sine(&g_ppg, 0, 1.0f, HR_WINDOW_SIZE);
    fill_channel_zero(&g_ppg, 1, HR_WINDOW_SIZE);
    fill_channel_zero(&g_ppg, 2, HR_WINDOW_SIZE);
    fill_channel_zero(&g_ppg, 3, HR_WINDOW_SIZE);
    hr_sqi_run(&g_sqi, &g_ppg, HR_WINDOW_SIZE, &p);
    ASSERT_INT_EQ(g_sqi.result.main_ch, 0);

    /* Cycle 2: ch0 = flat (unusable), ch1 = good → immediate switch */
    fill_channel_zero(&g_ppg, 0, HR_WINDOW_SIZE);
    fill_channel_sine(&g_ppg, 1, 1.0f, HR_WINDOW_SIZE);
    hr_sqi_run(&g_sqi, &g_ppg, HR_WINDOW_SIZE, &p);
    ASSERT_INT_EQ(g_sqi.result.main_ch, 1);

    return 0;
}

static int test_hysteresis_counter_resets(void)
{
    hr_params_t p = make_default_sqi_params();
    p.sqi.switch_margin      = 0.05f;
    p.sqi.sqi_poor_threshold = 0.0f;
    p.sqi.switch_hold_count  = 3;

    hr_sqi_init(&g_sqi);

    /* Cycle 1: ch0 = good → main = 0 */
    fill_channel_sine(&g_ppg, 0, 1.0f, HR_WINDOW_SIZE);
    fill_channel_zero(&g_ppg, 1, HR_WINDOW_SIZE);
    fill_channel_zero(&g_ppg, 2, HR_WINDOW_SIZE);
    fill_channel_zero(&g_ppg, 3, HR_WINDOW_SIZE);
    hr_sqi_run(&g_sqi, &g_ppg, HR_WINDOW_SIZE, &p);
    ASSERT_INT_EQ(g_sqi.result.main_ch, 0);

    /* Cycle 2: ch1 = best → candidate=ch1, counter=1 */
    fill_channel_zero(&g_ppg, 0, HR_WINDOW_SIZE);
    fill_channel_sine(&g_ppg, 1, 1.0f, HR_WINDOW_SIZE);
    fill_channel_zero(&g_ppg, 2, HR_WINDOW_SIZE);
    hr_sqi_run(&g_sqi, &g_ppg, HR_WINDOW_SIZE, &p);
    ASSERT_INT_EQ(g_sqi.result.main_ch, 0);

    /* Cycle 3: ch2 = best (ch1 drops out) → candidate RESETS to ch2, counter=1 */
    fill_channel_zero(&g_ppg, 1, HR_WINDOW_SIZE);
    fill_channel_sine(&g_ppg, 2, 1.0f, HR_WINDOW_SIZE);
    hr_sqi_run(&g_sqi, &g_ppg, HR_WINDOW_SIZE, &p);
    ASSERT_INT_EQ(g_sqi.result.main_ch, 0);

    /* Cycle 4: ch2 still best → counter=2, not enough */
    hr_sqi_run(&g_sqi, &g_ppg, HR_WINDOW_SIZE, &p);
    ASSERT_INT_EQ(g_sqi.result.main_ch, 0);

    /* Cycle 5: ch2 still best → counter=3 → SWITCH */
    hr_sqi_run(&g_sqi, &g_ppg, HR_WINDOW_SIZE, &p);
    ASSERT_INT_EQ(g_sqi.result.main_ch, 2);

    return 0;
}

/* ================================================================== */
/*  [G] Multi-cycle stability & robustness                             */
/* ================================================================== */

static int test_multi_cycle_stability(void)
{
    hr_params_t p = make_default_sqi_params();
    hr_sqi_init(&g_sqi);

    fill_channel_sine(&g_ppg, 0, 1.0f, HR_WINDOW_SIZE);
    fill_channel_zero(&g_ppg, 1, HR_WINDOW_SIZE);
    fill_channel_zero(&g_ppg, 2, HR_WINDOW_SIZE);
    fill_channel_zero(&g_ppg, 3, HR_WINDOW_SIZE);

    for (int cycle = 0; cycle < 20; cycle++) {
        hr_sqi_run(&g_sqi, &g_ppg, HR_WINDOW_SIZE, &p);
        ASSERT_INT_EQ(g_sqi.result.main_ch, 0);
        ASSERT_TRUE(g_sqi.result.ch[0].total > 0.8f);
        ASSERT_TRUE(g_sqi.result.sqi_main >= 0.0f);
        ASSERT_TRUE(g_sqi.result.sqi_main <= 1.0f);
    }
    return 0;
}

static int test_large_values_no_overflow(void)
{
    hr_params_t p = make_default_sqi_params();
    hr_sqi_init(&g_sqi);

    for (int ch = 0; ch < 4; ch++) {
        for (uint16_t i = 0; i < HR_WINDOW_SIZE; i++) {
            g_ppg.data[i][ch] = 1e6f * sinf(2.0f * (float)M_PI
                                             * 1.0f * (float)i / 25.0f);
        }
    }
    hr_sqi_run(&g_sqi, &g_ppg, HR_WINDOW_SIZE, &p);

    for (int ch = 0; ch < 4; ch++) {
        float t = g_sqi.result.ch[ch].total;
        ASSERT_TRUE(t >= 0.0f && t <= 1.0f);
        ASSERT_TRUE(t == t);  /* not NaN: NaN != NaN */
    }
    return 0;
}

/* ================================================================== */
/*  main                                                               */
/* ================================================================== */

int main(void)
{
    printf("==========================================\n");
    printf("  M3 SQI & Channel Selection — Test Suite\n");
    printf("==========================================\n\n");

    printf("[A] Contract tests:\n");
    RUN_TEST(test_sqi_init_state);
    RUN_TEST(test_sqi_reset_state);
    RUN_TEST(test_sqi_invalid_args);

    printf("\n[B] Output range & consistency:\n");
    RUN_TEST(test_sqi_output_range);
    RUN_TEST(test_sqi_main_equals_channel_total);

    printf("\n[C] Signal quality discrimination:\n");
    RUN_TEST(test_flatline_scores_low);
    RUN_TEST(test_periodic_scores_high);
    RUN_TEST(test_usable_threshold);

    printf("\n[D] Channel selection:\n");
    RUN_TEST(test_select_main_and_backup);
    RUN_TEST(test_all_bad_channels);
    RUN_TEST(test_channel_permutation);

    printf("\n[E] Channel independence:\n");
    RUN_TEST(test_channel_independence);

    printf("\n[F] Hysteresis (O11):\n");
    RUN_TEST(test_no_switch_insufficient_margin);
    RUN_TEST(test_switch_after_sustained_advantage);
    RUN_TEST(test_immediate_switch_when_unusable);
    RUN_TEST(test_hysteresis_counter_resets);

    printf("\n[G] Multi-cycle stability & robustness:\n");
    RUN_TEST(test_multi_cycle_stability);
    RUN_TEST(test_large_values_no_overflow);

    TEST_SUMMARY();
    return TEST_EXIT_CODE();
}
