/**
 * @file test_hr_preproc.c
 * @brief M2 Preprocessing — regression test suite.
 *
 * Tests cover:
 *   [A] Contract tests        — init / reset / initialized flag
 *   [B] DC suppression        — constant PPG input → near-zero output
 *   [C] Passband preservation — PPG sine at 1.5 Hz retained
 *   [D] Stopband attenuation  — PPG sine at 0.1 Hz attenuated
 *   [E] ACC lowpass basic     — ACC sine within passband preserved
 *   [F] Output shape          — input 200 pts → output 200 pts
 *   [G] Channel independence  — different per-channel input → independent output
 *   [H] Robustness            — all-zero, large values → no NaN/Inf
 *
 * Build & run:
 *   make -C tests run
 * or manually:
 *   cc -std=c11 -Wall -Wextra -Iinclude -Isrc \
 *      tests/test_hr_preproc.c src/hr_preproc.c src/hr_sampling.c \
 *      -lm -o test_hr_preproc
 *   ./test_hr_preproc
 */

#include "test_framework.h"
#include "hr_preproc.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ================================================================== */
/*  Static buffers — kept off the stack                                */
/* ================================================================== */

static hr_preproc_ctx_t g_preproc;
static acc_sample_t     g_acc_in[HR_WINDOW_SIZE];
static ppg_sample_t     g_ppg_in[HR_WINDOW_SIZE];

/* ================================================================== */
/*  Helper: default params matching architecture defaults              */
/* ================================================================== */

static hr_params_t make_default_params(void)
{
    hr_params_t p;
    memset(&p, 0, sizeof(p));
    p.preproc.ppg_bp_low_hz  = 0.5f;
    p.preproc.ppg_bp_high_hz = 4.0f;
    p.preproc.acc_lp_cutoff_hz = 10.0f;
    return p;
}

/* ================================================================== */
/*  Helper: fill PPG with per-channel sine wave                        */
/* ================================================================== */

static void fill_ppg_sine(ppg_sample_t *buf, uint16_t n,
                          float freq_hz, float amplitude,
                          float dc_offset)
{
    for (uint16_t i = 0; i < n; i++) {
        float val = dc_offset +
                    amplitude * sinf(2.0f * (float)M_PI * freq_hz *
                                     (float)i / HR_PREPROC_SAMPLE_RATE_HZ);
        int32_t ival = (int32_t)val;
        for (int ch = 0; ch < 4; ch++) {
            buf[i].ch[ch] = ival;
        }
    }
}

/* ================================================================== */
/*  Helper: compute RMS of a float array                               */
/* ================================================================== */

static float rms_float(const float *arr, int len)
{
    double sum = 0.0;
    for (int i = 0; i < len; i++) {
        sum += (double)arr[i] * (double)arr[i];
    }
    return (float)sqrt(sum / len);
}

/* ================================================================== */
/*  Helper: check no NaN or Inf in a 2D float region                   */
/* ================================================================== */

static int check_finite(const float *data, int count)
{
    for (int i = 0; i < count; i++) {
        if (isnan(data[i]) || isinf(data[i])) return 0;
    }
    return 1;
}

/* ================================================================== */
/*  [A] Contract tests                                                 */
/* ================================================================== */

static int test_init_sets_initialized(void)
{
    hr_params_t p = make_default_params();
    memset(&g_preproc, 0xFF, sizeof(g_preproc));
    hr_preproc_init(&g_preproc, &p);

    ASSERT_TRUE(g_preproc.initialized);
    return 0;
}

static int test_init_null_ctx(void)
{
    hr_params_t p = make_default_params();
    hr_preproc_init(NULL, &p);
    return 0;
}

static int test_init_null_params(void)
{
    hr_preproc_init(&g_preproc, NULL);
    return 0;
}

static int test_reset_is_equivalent_to_init(void)
{
    hr_params_t p = make_default_params();
    hr_preproc_init(&g_preproc, &p);

    memset(g_acc_in, 0, sizeof(g_acc_in));
    memset(g_ppg_in, 0, sizeof(g_ppg_in));
    hr_preproc_run(&g_preproc, g_acc_in, g_ppg_in, HR_WINDOW_SIZE);

    hr_preproc_reset(&g_preproc, &p);
    ASSERT_TRUE(g_preproc.initialized);

    ASSERT_FLOAT_NEAR(g_preproc.ppg_hp_state[0].w1, 0.0f, 1e-6f);
    ASSERT_FLOAT_NEAR(g_preproc.ppg_hp_state[0].w2, 0.0f, 1e-6f);

    return 0;
}

/* ================================================================== */
/*  [B] DC suppression: constant PPG → near-zero output (steady state) */
/* ================================================================== */

static int test_ppg_dc_suppression(void)
{
    hr_params_t p = make_default_params();
    hr_preproc_init(&g_preproc, &p);

    for (uint16_t i = 0; i < HR_WINDOW_SIZE; i++) {
        g_ppg_in[i].ch[0] = 500000;
        g_ppg_in[i].ch[1] = 300000;
        g_ppg_in[i].ch[2] = 100000;
        g_ppg_in[i].ch[3] = 800000;
    }
    memset(g_acc_in, 0, sizeof(g_acc_in));

    hr_preproc_run(&g_preproc, g_acc_in, g_ppg_in, HR_WINDOW_SIZE);

    /*
     * After HP filtering, a constant input should converge to ~0.
     * Check the last 100 samples (well past the transient).
     */
    for (int ch = 0; ch < 4; ch++) {
        float tail_rms = 0.0f;
        for (int i = 100; i < HR_WINDOW_SIZE; i++) {
            tail_rms += g_preproc.ppg_out.data[i][ch] *
                        g_preproc.ppg_out.data[i][ch];
        }
        tail_rms = sqrtf(tail_rms / 100.0f);
        ASSERT_TRUE(tail_rms < 50.0f);
    }

    return 0;
}

/* ================================================================== */
/*  [C] Passband preservation: 1.5 Hz sine (in 0.5–4.0 Hz band)       */
/* ================================================================== */

static int test_ppg_passband_sine(void)
{
    hr_params_t p = make_default_params();
    hr_preproc_init(&g_preproc, &p);

    float freq = 1.5f;
    float amp  = 10000.0f;
    float dc   = 500000.0f;
    fill_ppg_sine(g_ppg_in, HR_WINDOW_SIZE, freq, amp, dc);
    memset(g_acc_in, 0, sizeof(g_acc_in));

    hr_preproc_run(&g_preproc, g_acc_in, g_ppg_in, HR_WINDOW_SIZE);

    /*
     * In passband, the output should retain significant amplitude.
     * Check RMS of last 150 samples (past transient) on ch0.
     * A 10000-amplitude sine has RMS ~7071.  After filtering in
     * passband, expect at least 50% retention.
     */
    float out_buf[150];
    for (int i = 0; i < 150; i++) {
        out_buf[i] = g_preproc.ppg_out.data[50 + i][0];
    }
    float out_rms = rms_float(out_buf, 150);

    ASSERT_TRUE(out_rms > 3000.0f);

    return 0;
}

/* ================================================================== */
/*  [D] Stopband attenuation: 0.1 Hz sine (below 0.5 Hz HP cutoff)    */
/* ================================================================== */

static int test_ppg_stopband_low_freq(void)
{
    hr_params_t p = make_default_params();
    hr_preproc_init(&g_preproc, &p);

    float freq = 0.1f;
    float amp  = 10000.0f;
    float dc   = 500000.0f;
    fill_ppg_sine(g_ppg_in, HR_WINDOW_SIZE, freq, amp, dc);
    memset(g_acc_in, 0, sizeof(g_acc_in));

    hr_preproc_run(&g_preproc, g_acc_in, g_ppg_in, HR_WINDOW_SIZE);

    /*
     * 0.1 Hz is well below the HP cutoff of 0.5 Hz.
     * The output should be significantly attenuated.
     * Check RMS of last 100 samples on ch0.
     */
    float out_buf[100];
    for (int i = 0; i < 100; i++) {
        out_buf[i] = g_preproc.ppg_out.data[100 + i][0];
    }
    float out_rms = rms_float(out_buf, 100);
    float in_rms  = amp / sqrtf(2.0f);

    ASSERT_TRUE(out_rms < in_rms * 0.5f);

    return 0;
}

/* ================================================================== */
/*  [E] ACC lowpass: sine within passband (5 Hz, cutoff 10 Hz)         */
/* ================================================================== */

static int test_acc_lowpass_passband(void)
{
    hr_params_t p = make_default_params();
    hr_preproc_init(&g_preproc, &p);

    float freq = 5.0f;
    float amp  = 1000.0f;

    for (uint16_t i = 0; i < HR_WINDOW_SIZE; i++) {
        float val = amp * sinf(2.0f * (float)M_PI * freq *
                               (float)i / HR_PREPROC_SAMPLE_RATE_HZ);
        g_acc_in[i].x = (int16_t)val;
        g_acc_in[i].y = 0;
        g_acc_in[i].z = 0;
    }
    memset(g_ppg_in, 0, sizeof(g_ppg_in));

    hr_preproc_run(&g_preproc, g_acc_in, g_ppg_in, HR_WINDOW_SIZE);

    float out_buf[150];
    for (int i = 0; i < 150; i++) {
        out_buf[i] = g_preproc.acc_out.data[50 + i][0];
    }
    float out_rms = rms_float(out_buf, 150);

    ASSERT_TRUE(out_rms > 300.0f);

    return 0;
}

/* ================================================================== */
/*  [F] Output shape: 200 in → 200 out, values are finite              */
/* ================================================================== */

static int test_output_shape_200(void)
{
    hr_params_t p = make_default_params();
    hr_preproc_init(&g_preproc, &p);

    fill_ppg_sine(g_ppg_in, HR_WINDOW_SIZE, 2.0f, 5000.0f, 100000.0f);
    for (uint16_t i = 0; i < HR_WINDOW_SIZE; i++) {
        g_acc_in[i].x = (int16_t)(i % 100);
        g_acc_in[i].y = (int16_t)(200 - i % 100);
        g_acc_in[i].z = (int16_t)(i * 3 % 500);
    }

    hr_preproc_run(&g_preproc, g_acc_in, g_ppg_in, HR_WINDOW_SIZE);

    ASSERT_TRUE(check_finite(&g_preproc.ppg_out.data[0][0],
                             HR_WINDOW_SIZE * 4));
    ASSERT_TRUE(check_finite(&g_preproc.acc_out.data[0][0],
                             HR_WINDOW_SIZE * 3));

    return 0;
}

/* ================================================================== */
/*  [G] Channel independence: only ch1 has signal, others zero         */
/* ================================================================== */

static int test_ppg_channel_independence(void)
{
    hr_params_t p = make_default_params();
    hr_preproc_init(&g_preproc, &p);

    for (uint16_t i = 0; i < HR_WINDOW_SIZE; i++) {
        float val = 10000.0f * sinf(2.0f * (float)M_PI * 2.0f *
                                    (float)i / HR_PREPROC_SAMPLE_RATE_HZ);
        g_ppg_in[i].ch[0] = 0;
        g_ppg_in[i].ch[1] = (int32_t)val;
        g_ppg_in[i].ch[2] = 0;
        g_ppg_in[i].ch[3] = 0;
    }
    memset(g_acc_in, 0, sizeof(g_acc_in));

    hr_preproc_run(&g_preproc, g_acc_in, g_ppg_in, HR_WINDOW_SIZE);

    float rms_ch1 = rms_float(&g_preproc.ppg_out.data[50][1], 150);

    float rms_ch0 = rms_float(&g_preproc.ppg_out.data[50][0], 150);
    float rms_ch2 = rms_float(&g_preproc.ppg_out.data[50][2], 150);
    float rms_ch3 = rms_float(&g_preproc.ppg_out.data[50][3], 150);

    /*
     * NOTE: rms_float reads from strided memory here (stride=4 floats
     * per sample), but we pass channel-0 pointer with count 150.
     * This actually reads ppg_out.data[50][0], data[50][1], data[50][2]...
     * which is mixed channels.  Fix: extract per-channel.
     */

    /* Extract per-channel data properly */
    float ch_buf[4][150];
    for (int i = 0; i < 150; i++) {
        ch_buf[0][i] = g_preproc.ppg_out.data[50 + i][0];
        ch_buf[1][i] = g_preproc.ppg_out.data[50 + i][1];
        ch_buf[2][i] = g_preproc.ppg_out.data[50 + i][2];
        ch_buf[3][i] = g_preproc.ppg_out.data[50 + i][3];
    }

    rms_ch0 = rms_float(ch_buf[0], 150);
    rms_ch1 = rms_float(ch_buf[1], 150);
    rms_ch2 = rms_float(ch_buf[2], 150);
    rms_ch3 = rms_float(ch_buf[3], 150);

    ASSERT_TRUE(rms_ch1 > 1000.0f);
    ASSERT_TRUE(rms_ch0 < 10.0f);
    ASSERT_TRUE(rms_ch2 < 10.0f);
    ASSERT_TRUE(rms_ch3 < 10.0f);

    return 0;
}

static int test_acc_axis_independence(void)
{
    hr_params_t p = make_default_params();
    hr_preproc_init(&g_preproc, &p);

    for (uint16_t i = 0; i < HR_WINDOW_SIZE; i++) {
        float val = 1000.0f * sinf(2.0f * (float)M_PI * 3.0f *
                                   (float)i / HR_PREPROC_SAMPLE_RATE_HZ);
        g_acc_in[i].x = 0;
        g_acc_in[i].y = (int16_t)val;
        g_acc_in[i].z = 0;
    }
    memset(g_ppg_in, 0, sizeof(g_ppg_in));

    hr_preproc_run(&g_preproc, g_acc_in, g_ppg_in, HR_WINDOW_SIZE);

    float ax_buf[3][150];
    for (int i = 0; i < 150; i++) {
        ax_buf[0][i] = g_preproc.acc_out.data[50 + i][0];
        ax_buf[1][i] = g_preproc.acc_out.data[50 + i][1];
        ax_buf[2][i] = g_preproc.acc_out.data[50 + i][2];
    }

    float rms_x = rms_float(ax_buf[0], 150);
    float rms_y = rms_float(ax_buf[1], 150);
    float rms_z = rms_float(ax_buf[2], 150);

    ASSERT_TRUE(rms_y > 300.0f);
    ASSERT_TRUE(rms_x < 1.0f);
    ASSERT_TRUE(rms_z < 1.0f);

    return 0;
}

/* ================================================================== */
/*  [H] Robustness tests                                               */
/* ================================================================== */

static int test_all_zero_input(void)
{
    hr_params_t p = make_default_params();
    hr_preproc_init(&g_preproc, &p);

    memset(g_acc_in, 0, sizeof(g_acc_in));
    memset(g_ppg_in, 0, sizeof(g_ppg_in));

    hr_preproc_run(&g_preproc, g_acc_in, g_ppg_in, HR_WINDOW_SIZE);

    ASSERT_TRUE(check_finite(&g_preproc.ppg_out.data[0][0],
                             HR_WINDOW_SIZE * 4));
    ASSERT_TRUE(check_finite(&g_preproc.acc_out.data[0][0],
                             HR_WINDOW_SIZE * 3));

    for (int i = 0; i < HR_WINDOW_SIZE; i++) {
        for (int ch = 0; ch < 4; ch++) {
            ASSERT_FLOAT_NEAR(g_preproc.ppg_out.data[i][ch], 0.0f, 1e-6f);
        }
        for (int ax = 0; ax < 3; ax++) {
            ASSERT_FLOAT_NEAR(g_preproc.acc_out.data[i][ax], 0.0f, 1e-6f);
        }
    }

    return 0;
}

static int test_large_ppg_values_no_nan(void)
{
    hr_params_t p = make_default_params();
    hr_preproc_init(&g_preproc, &p);

    for (uint16_t i = 0; i < HR_WINDOW_SIZE; i++) {
        g_ppg_in[i].ch[0] = 2000000000;
        g_ppg_in[i].ch[1] = -2000000000;
        g_ppg_in[i].ch[2] = 2000000000;
        g_ppg_in[i].ch[3] = -2000000000;
    }
    memset(g_acc_in, 0, sizeof(g_acc_in));

    hr_preproc_run(&g_preproc, g_acc_in, g_ppg_in, HR_WINDOW_SIZE);

    ASSERT_TRUE(check_finite(&g_preproc.ppg_out.data[0][0],
                             HR_WINDOW_SIZE * 4));

    return 0;
}

static int test_large_acc_values_no_nan(void)
{
    hr_params_t p = make_default_params();
    hr_preproc_init(&g_preproc, &p);

    for (uint16_t i = 0; i < HR_WINDOW_SIZE; i++) {
        g_acc_in[i].x = 32767;
        g_acc_in[i].y = -32768;
        g_acc_in[i].z = 32767;
    }
    memset(g_ppg_in, 0, sizeof(g_ppg_in));

    hr_preproc_run(&g_preproc, g_acc_in, g_ppg_in, HR_WINDOW_SIZE);

    ASSERT_TRUE(check_finite(&g_preproc.acc_out.data[0][0],
                             HR_WINDOW_SIZE * 3));

    return 0;
}

static int test_run_without_init(void)
{
    hr_preproc_ctx_t uninit;
    memset(&uninit, 0, sizeof(uninit));

    memset(g_acc_in, 0, sizeof(g_acc_in));
    memset(g_ppg_in, 0, sizeof(g_ppg_in));

    hr_preproc_run(&uninit, g_acc_in, g_ppg_in, HR_WINDOW_SIZE);

    return 0;
}

static int test_run_null_inputs(void)
{
    hr_params_t p = make_default_params();
    hr_preproc_init(&g_preproc, &p);

    hr_preproc_run(&g_preproc, NULL, g_ppg_in, HR_WINDOW_SIZE);
    hr_preproc_run(&g_preproc, g_acc_in, NULL, HR_WINDOW_SIZE);
    hr_preproc_run(NULL, g_acc_in, g_ppg_in, HR_WINDOW_SIZE);
    hr_preproc_run(&g_preproc, g_acc_in, g_ppg_in, 0);

    return 0;
}

static int test_run_n_exceeds_window(void)
{
    hr_params_t p = make_default_params();
    hr_preproc_init(&g_preproc, &p);

    memset(g_acc_in, 0, sizeof(g_acc_in));
    memset(g_ppg_in, 0, sizeof(g_ppg_in));

    hr_preproc_run(&g_preproc, g_acc_in, g_ppg_in, HR_WINDOW_SIZE + 1);

    return 0;
}

/* ================================================================== */
/*  [I] ACC DC preservation                                            */
/* ================================================================== */

static int test_acc_dc_preserved(void)
{
    hr_params_t p = make_default_params();
    hr_preproc_init(&g_preproc, &p);

    for (uint16_t i = 0; i < HR_WINDOW_SIZE; i++) {
        g_acc_in[i].x = 1000;
        g_acc_in[i].y = -500;
        g_acc_in[i].z = 2000;
    }
    memset(g_ppg_in, 0, sizeof(g_ppg_in));

    hr_preproc_run(&g_preproc, g_acc_in, g_ppg_in, HR_WINDOW_SIZE);

    /*
     * ACC LP preserves DC.  After settling, the output should be
     * close to the DC input value.
     */
    for (int i = 150; i < HR_WINDOW_SIZE; i++) {
        ASSERT_FLOAT_NEAR(g_preproc.acc_out.data[i][0], 1000.0f, 5.0f);
        ASSERT_FLOAT_NEAR(g_preproc.acc_out.data[i][1], -500.0f, 5.0f);
        ASSERT_FLOAT_NEAR(g_preproc.acc_out.data[i][2], 2000.0f, 5.0f);
    }

    return 0;
}

/* ================================================================== */
/*  main — test runner                                                 */
/* ================================================================== */

int main(void)
{
    printf("==========================================\n");
    printf("  M2 Preprocessing — Test Suite\n");
    printf("==========================================\n\n");

    printf("[A] Contract tests:\n");
    RUN_TEST(test_init_sets_initialized);
    RUN_TEST(test_init_null_ctx);
    RUN_TEST(test_init_null_params);
    RUN_TEST(test_reset_is_equivalent_to_init);

    printf("\n[B] DC suppression:\n");
    RUN_TEST(test_ppg_dc_suppression);

    printf("\n[C] Passband preservation:\n");
    RUN_TEST(test_ppg_passband_sine);

    printf("\n[D] Stopband attenuation:\n");
    RUN_TEST(test_ppg_stopband_low_freq);

    printf("\n[E] ACC lowpass:\n");
    RUN_TEST(test_acc_lowpass_passband);

    printf("\n[F] Output shape:\n");
    RUN_TEST(test_output_shape_200);

    printf("\n[G] Channel independence:\n");
    RUN_TEST(test_ppg_channel_independence);
    RUN_TEST(test_acc_axis_independence);

    printf("\n[H] Robustness:\n");
    RUN_TEST(test_all_zero_input);
    RUN_TEST(test_large_ppg_values_no_nan);
    RUN_TEST(test_large_acc_values_no_nan);
    RUN_TEST(test_run_without_init);
    RUN_TEST(test_run_null_inputs);
    RUN_TEST(test_run_n_exceeds_window);

    printf("\n[I] ACC DC preservation:\n");
    RUN_TEST(test_acc_dc_preserved);

    TEST_SUMMARY();
    return TEST_EXIT_CODE();
}
