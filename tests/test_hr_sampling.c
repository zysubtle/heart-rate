/**
 * @file test_hr_sampling.c
 * @brief M1 Sampling & Buffering — regression test suite.
 *
 * Tests cover:
 *   [A] Contract tests      — init / reset / ready / export contracts
 *   [B] Constructed data     — identifiable data write, overwrite, order
 *   [C] Misaligned ACC/PPG  — independent feed, min-count ready rule
 *   [D] Simulated replay    — 25 Hz per-second window sliding
 *   [E] Robustness          — large feed, repeated reset, invalid args
 *
 * Build & run:
 *   make -C tests run
 * or manually:
 *   cc -std=c11 -Wall -Wextra -Iinclude -Isrc \
 *      tests/test_hr_sampling.c src/hr_sampling.c -o test_hr_sampling
 *   ./test_hr_sampling
 */

#include "test_framework.h"
#include "hr_sampling.h"
#include <string.h>

/* ================================================================== */
/*  Static buffers — kept off the stack                                */
/* ================================================================== */

static hr_sampling_ctx_t g_ctx;
static acc_sample_t      g_acc_out[HR_RING_CAPACITY];
static ppg_sample_t      g_ppg_out[HR_RING_CAPACITY];

/* ================================================================== */
/*  Helper: push N identifiable ACC + PPG samples starting at offset  */
/*                                                                     */
/*  ACC sample i:  x = i,  y = 10000+i,  z = 20000+i                  */
/*  PPG sample i:  ch[0..3] = 100000+i, 200000+i, 300000+i, 400000+i  */
/* ================================================================== */

static void push_pair(hr_sampling_ctx_t *ctx, int start, int count)
{
    for (int i = start; i < start + count; i++) {
        hr_sampling_push_acc(ctx,
                             (int16_t)i,
                             (int16_t)(10000 + i),
                             (int16_t)(20000 + i));
        hr_sampling_push_ppg(ctx,
                             100000 + i, 200000 + i,
                             300000 + i, 400000 + i);
    }
}

/* ================================================================== */
/*  [A] Contract tests                                                 */
/* ================================================================== */

static int test_init_state(void)
{
    hr_sampling_init(&g_ctx);

    ASSERT_INT_EQ(hr_sampling_get_acc_count(&g_ctx), 0);
    ASSERT_INT_EQ(hr_sampling_get_ppg_count(&g_ctx), 0);
    ASSERT_FALSE(hr_sampling_ready_for_window(&g_ctx));
    ASSERT_FALSE(hr_sampling_export_recent_acc(&g_ctx, g_acc_out, HR_WINDOW_SIZE));
    ASSERT_FALSE(hr_sampling_export_recent_ppg(&g_ctx, g_ppg_out, HR_WINDOW_SIZE));

    return 0;
}

static int test_reset_state(void)
{
    hr_sampling_init(&g_ctx);
    push_pair(&g_ctx, 1000, HR_WINDOW_SIZE);
    ASSERT_TRUE(hr_sampling_ready_for_window(&g_ctx));

    hr_sampling_reset(&g_ctx);

    ASSERT_INT_EQ(hr_sampling_get_acc_count(&g_ctx), 0);
    ASSERT_INT_EQ(hr_sampling_get_ppg_count(&g_ctx), 0);
    ASSERT_FALSE(hr_sampling_ready_for_window(&g_ctx));
    ASSERT_FALSE(hr_sampling_export_recent_acc(&g_ctx, g_acc_out, HR_WINDOW_SIZE));

    /* Refill with different data — verify no residue from pre-reset */
    push_pair(&g_ctx, 2000, HR_WINDOW_SIZE);
    ASSERT_TRUE(hr_sampling_export_recent_acc(&g_ctx, g_acc_out, HR_WINDOW_SIZE));
    ASSERT_INT_EQ(g_acc_out[0].x, 2000);
    ASSERT_INT_EQ(g_acc_out[HR_WINDOW_SIZE - 1].x, 2000 + HR_WINDOW_SIZE - 1);

    return 0;
}

static int test_not_ready_before_200(void)
{
    hr_sampling_init(&g_ctx);
    push_pair(&g_ctx, 0, HR_WINDOW_SIZE - 1);   /* 199 points */

    ASSERT_INT_EQ(hr_sampling_get_acc_count(&g_ctx), HR_WINDOW_SIZE - 1);
    ASSERT_INT_EQ(hr_sampling_get_ppg_count(&g_ctx), HR_WINDOW_SIZE - 1);
    ASSERT_FALSE(hr_sampling_ready_for_window(&g_ctx));
    ASSERT_FALSE(hr_sampling_export_recent_acc(&g_ctx, g_acc_out, HR_WINDOW_SIZE));
    ASSERT_FALSE(hr_sampling_export_recent_ppg(&g_ctx, g_ppg_out, HR_WINDOW_SIZE));

    return 0;
}

static int test_ready_at_200(void)
{
    hr_sampling_init(&g_ctx);
    push_pair(&g_ctx, 0, HR_WINDOW_SIZE);        /* exactly 200 */

    ASSERT_INT_EQ(hr_sampling_get_acc_count(&g_ctx), HR_WINDOW_SIZE);
    ASSERT_INT_EQ(hr_sampling_get_ppg_count(&g_ctx), HR_WINDOW_SIZE);
    ASSERT_TRUE(hr_sampling_ready_for_window(&g_ctx));
    ASSERT_TRUE(hr_sampling_export_recent_acc(&g_ctx, g_acc_out, HR_WINDOW_SIZE));
    ASSERT_TRUE(hr_sampling_export_recent_ppg(&g_ctx, g_ppg_out, HR_WINDOW_SIZE));

    return 0;
}

static int test_export_fail_when_insufficient(void)
{
    hr_sampling_init(&g_ctx);
    push_pair(&g_ctx, 0, 100);

    /* 100 < 200 → export 200 must fail */
    ASSERT_FALSE(hr_sampling_export_recent_acc(&g_ctx, g_acc_out, HR_WINDOW_SIZE));
    ASSERT_FALSE(hr_sampling_export_recent_ppg(&g_ctx, g_ppg_out, HR_WINDOW_SIZE));

    /* But export <= 100 must succeed */
    ASSERT_TRUE(hr_sampling_export_recent_acc(&g_ctx, g_acc_out, 100));
    ASSERT_TRUE(hr_sampling_export_recent_ppg(&g_ctx, g_ppg_out, 100));

    /* 101 still too many */
    ASSERT_FALSE(hr_sampling_export_recent_acc(&g_ctx, g_acc_out, 101));

    return 0;
}

/* ================================================================== */
/*  [B] Constructed data tests                                         */
/* ================================================================== */

static int test_export_recent_200_order(void)
{
    hr_sampling_init(&g_ctx);
    push_pair(&g_ctx, 0, HR_WINDOW_SIZE);

    ASSERT_TRUE(hr_sampling_export_recent_acc(&g_ctx, g_acc_out, HR_WINDOW_SIZE));
    ASSERT_TRUE(hr_sampling_export_recent_ppg(&g_ctx, g_ppg_out, HR_WINDOW_SIZE));

    for (int i = 0; i < HR_WINDOW_SIZE; i++) {
        /* ACC oldest→newest must be 0,1,...,199 */
        ASSERT_INT_EQ(g_acc_out[i].x,  i);
        ASSERT_INT_EQ(g_acc_out[i].y,  10000 + i);
        ASSERT_INT_EQ(g_acc_out[i].z,  20000 + i);

        /* PPG oldest→newest, all 4 channels */
        ASSERT_INT_EQ(g_ppg_out[i].ch[0], 100000 + i);
        ASSERT_INT_EQ(g_ppg_out[i].ch[1], 200000 + i);
        ASSERT_INT_EQ(g_ppg_out[i].ch[2], 300000 + i);
        ASSERT_INT_EQ(g_ppg_out[i].ch[3], 400000 + i);
    }

    return 0;
}

static int test_ring_overwrite_recent_window(void)
{
    hr_sampling_init(&g_ctx);

    /* Push 300 samples → ring wraps at 256, recent 200 = [100..299] */
    const int total = 300;
    push_pair(&g_ctx, 0, total);

    ASSERT_INT_EQ(hr_sampling_get_acc_count(&g_ctx), HR_RING_CAPACITY);
    ASSERT_TRUE(hr_sampling_ready_for_window(&g_ctx));

    ASSERT_TRUE(hr_sampling_export_recent_acc(&g_ctx, g_acc_out, HR_WINDOW_SIZE));
    ASSERT_TRUE(hr_sampling_export_recent_ppg(&g_ctx, g_ppg_out, HR_WINDOW_SIZE));

    const int first = total - HR_WINDOW_SIZE;   /* 100 */
    for (int i = 0; i < HR_WINDOW_SIZE; i++) {
        ASSERT_INT_EQ(g_acc_out[i].x,  first + i);
        ASSERT_INT_EQ(g_acc_out[i].y,  10000 + first + i);
        ASSERT_INT_EQ(g_acc_out[i].z,  20000 + first + i);

        ASSERT_INT_EQ(g_ppg_out[i].ch[0], 100000 + first + i);
        ASSERT_INT_EQ(g_ppg_out[i].ch[1], 200000 + first + i);
        ASSERT_INT_EQ(g_ppg_out[i].ch[2], 300000 + first + i);
        ASSERT_INT_EQ(g_ppg_out[i].ch[3], 400000 + first + i);
    }

    return 0;
}

/* ================================================================== */
/*  [C] Misaligned ACC / PPG tests                                     */
/* ================================================================== */

static int test_acc_ppg_misaligned_not_ready(void)
{
    hr_sampling_init(&g_ctx);

    for (int i = 0; i < 220; i++)
        hr_sampling_push_acc(&g_ctx, (int16_t)i, 0, 0);
    for (int i = 0; i < 180; i++)
        hr_sampling_push_ppg(&g_ctx, i, 0, 0, 0);

    ASSERT_INT_EQ(hr_sampling_get_acc_count(&g_ctx), 220);
    ASSERT_INT_EQ(hr_sampling_get_ppg_count(&g_ctx), 180);

    /* min(220, 180) = 180 < 200 → not ready */
    ASSERT_FALSE(hr_sampling_ready_for_window(&g_ctx));

    return 0;
}

static int test_acc_ppg_misaligned_then_ready(void)
{
    hr_sampling_init(&g_ctx);

    for (int i = 0; i < 220; i++)
        hr_sampling_push_acc(&g_ctx, (int16_t)i, 0, 0);
    for (int i = 0; i < 180; i++)
        hr_sampling_push_ppg(&g_ctx, i, 0, 0, 0);

    ASSERT_FALSE(hr_sampling_ready_for_window(&g_ctx));

    /* Top up PPG to 200 */
    for (int i = 180; i < 200; i++)
        hr_sampling_push_ppg(&g_ctx, i, 0, 0, 0);

    ASSERT_INT_EQ(hr_sampling_get_ppg_count(&g_ctx), 200);
    ASSERT_TRUE(hr_sampling_ready_for_window(&g_ctx));

    return 0;
}

/* ================================================================== */
/*  [D] Simulated replay — 25 Hz, verify window slides by 25 / sec    */
/*                                                                     */
/*  NOTE: No real ACC/PPG data files found in repo.  This test uses    */
/*        identifiable synthetic data fed at 25 samples per "second".  */
/* ================================================================== */

static int test_replay_window_slide_by_25(void)
{
    hr_sampling_init(&g_ctx);

    int sample_id = 0;

    /* Simulate 12 seconds @ 25 Hz → 300 total samples */
    for (int sec = 1; sec <= 12; sec++) {
        for (int j = 0; j < 25; j++) {
            hr_sampling_push_acc(&g_ctx,
                                 (int16_t)sample_id,
                                 (int16_t)(10000 + sample_id),
                                 (int16_t)(20000 + sample_id));
            hr_sampling_push_ppg(&g_ctx,
                                 100000 + sample_id,
                                 200000 + sample_id,
                                 300000 + sample_id,
                                 400000 + sample_id);
            sample_id++;
        }

        int total_fed = sec * 25;

        if (total_fed < HR_WINDOW_SIZE) {
            /* Seconds 1–7 (25..175 samples): must NOT be ready */
            ASSERT_FALSE(hr_sampling_ready_for_window(&g_ctx));
        } else {
            /* Second 8+ (200..300 samples): must be ready */
            ASSERT_TRUE(hr_sampling_ready_for_window(&g_ctx));
            ASSERT_TRUE(hr_sampling_export_recent_acc(
                             &g_ctx, g_acc_out, HR_WINDOW_SIZE));

            int expect_oldest = total_fed - HR_WINDOW_SIZE;
            int expect_newest = total_fed - 1;

            ASSERT_INT_EQ(g_acc_out[0].x,                    expect_oldest);
            ASSERT_INT_EQ(g_acc_out[HR_WINDOW_SIZE - 1].x,  expect_newest);

            /* Full sequence continuity check */
            for (int k = 0; k < HR_WINDOW_SIZE; k++) {
                ASSERT_INT_EQ(g_acc_out[k].x, expect_oldest + k);
            }
        }
    }

    /*
     * Verify window slide step between consecutive seconds:
     *   sec  8 → oldest =   0, newest = 199
     *   sec  9 → oldest =  25, newest = 224   (slid by 25)
     *   sec 10 → oldest =  50, newest = 249   (slid by 25)
     *   sec 11 → oldest =  75, newest = 274   (slid by 25)
     *   sec 12 → oldest = 100, newest = 299   (slid by 25)
     * (Already verified in the loop above; the pattern confirms
     *  the window slides exactly 25 samples per second.)
     */

    return 0;
}

/* ================================================================== */
/*  [E] Robustness tests                                               */
/* ================================================================== */

static int test_large_continuous_feed_stability(void)
{
    hr_sampling_init(&g_ctx);

    const int total = 2000;
    push_pair(&g_ctx, 0, total);

    /* Count caps at ring capacity */
    ASSERT_INT_EQ(hr_sampling_get_acc_count(&g_ctx), HR_RING_CAPACITY);
    ASSERT_INT_EQ(hr_sampling_get_ppg_count(&g_ctx), HR_RING_CAPACITY);
    ASSERT_TRUE(hr_sampling_ready_for_window(&g_ctx));

    /* Export most recent 200 */
    ASSERT_TRUE(hr_sampling_export_recent_acc(&g_ctx, g_acc_out, HR_WINDOW_SIZE));
    int win_start = total - HR_WINDOW_SIZE;   /* 1800 */
    ASSERT_INT_EQ(g_acc_out[0].x,                    win_start);
    ASSERT_INT_EQ(g_acc_out[HR_WINDOW_SIZE - 1].x,  total - 1);

    /* Export full capacity (256) */
    ASSERT_TRUE(hr_sampling_export_recent_acc(&g_ctx, g_acc_out, HR_RING_CAPACITY));
    int full_start = total - HR_RING_CAPACITY; /* 1744 */
    for (int i = 0; i < HR_RING_CAPACITY; i++) {
        ASSERT_INT_EQ(g_acc_out[i].x, full_start + i);
    }

    return 0;
}

static int test_repeated_reset_and_refill(void)
{
    for (int round = 0; round < 5; round++) {
        hr_sampling_reset(&g_ctx);

        ASSERT_INT_EQ(hr_sampling_get_acc_count(&g_ctx), 0);
        ASSERT_INT_EQ(hr_sampling_get_ppg_count(&g_ctx), 0);
        ASSERT_FALSE(hr_sampling_ready_for_window(&g_ctx));

        int base = round * 1000;
        push_pair(&g_ctx, base, HR_WINDOW_SIZE);

        ASSERT_TRUE(hr_sampling_ready_for_window(&g_ctx));
        ASSERT_TRUE(hr_sampling_export_recent_acc(&g_ctx, g_acc_out, HR_WINDOW_SIZE));
        ASSERT_INT_EQ(g_acc_out[0].x,                    base);
        ASSERT_INT_EQ(g_acc_out[HR_WINDOW_SIZE - 1].x,  base + HR_WINDOW_SIZE - 1);

        /* PPG consistency in same round */
        ASSERT_TRUE(hr_sampling_export_recent_ppg(&g_ctx, g_ppg_out, HR_WINDOW_SIZE));
        ASSERT_INT_EQ(g_ppg_out[0].ch[0],                    100000 + base);
        ASSERT_INT_EQ(g_ppg_out[HR_WINDOW_SIZE - 1].ch[3],  400000 + base + HR_WINDOW_SIZE - 1);
    }

    return 0;
}

static int test_invalid_args(void)
{
    hr_sampling_init(&g_ctx);
    push_pair(&g_ctx, 0, HR_WINDOW_SIZE);

    /* NULL ctx — must not crash, must return safe defaults / false */
    ASSERT_FALSE(hr_sampling_ready_for_window(NULL));
    ASSERT_INT_EQ(hr_sampling_get_acc_count(NULL), 0);
    ASSERT_INT_EQ(hr_sampling_get_ppg_count(NULL), 0);
    ASSERT_FALSE(hr_sampling_export_recent_acc(NULL, g_acc_out, HR_WINDOW_SIZE));
    ASSERT_FALSE(hr_sampling_export_recent_ppg(NULL, g_ppg_out, HR_WINDOW_SIZE));

    /* NULL output buffer */
    ASSERT_FALSE(hr_sampling_export_recent_acc(&g_ctx, NULL, HR_WINDOW_SIZE));
    ASSERT_FALSE(hr_sampling_export_recent_ppg(&g_ctx, NULL, HR_WINDOW_SIZE));

    /* n = 0 */
    ASSERT_FALSE(hr_sampling_export_recent_acc(&g_ctx, g_acc_out, 0));
    ASSERT_FALSE(hr_sampling_export_recent_ppg(&g_ctx, g_ppg_out, 0));

    /* n > capacity */
    ASSERT_FALSE(hr_sampling_export_recent_acc(&g_ctx, g_acc_out, HR_RING_CAPACITY + 1));
    ASSERT_FALSE(hr_sampling_export_recent_ppg(&g_ctx, g_ppg_out, HR_RING_CAPACITY + 1));

    /* push / init / reset with NULL ctx — must not crash */
    hr_sampling_push_acc(NULL, 0, 0, 0);
    hr_sampling_push_ppg(NULL, 0, 0, 0, 0);
    hr_sampling_init(NULL);
    hr_sampling_reset(NULL);

    return 0;
}

/* ================================================================== */
/*  main — test runner                                                 */
/* ================================================================== */

int main(void)
{
    printf("==========================================\n");
    printf("  M1 Sampling & Buffering — Test Suite\n");
    printf("==========================================\n\n");

    printf("[A] Contract tests:\n");
    RUN_TEST(test_init_state);
    RUN_TEST(test_reset_state);
    RUN_TEST(test_not_ready_before_200);
    RUN_TEST(test_ready_at_200);
    RUN_TEST(test_export_fail_when_insufficient);

    printf("\n[B] Constructed data tests:\n");
    RUN_TEST(test_export_recent_200_order);
    RUN_TEST(test_ring_overwrite_recent_window);

    printf("\n[C] Misaligned ACC/PPG tests:\n");
    RUN_TEST(test_acc_ppg_misaligned_not_ready);
    RUN_TEST(test_acc_ppg_misaligned_then_ready);

    printf("\n[D] Simulated replay test:\n");
    RUN_TEST(test_replay_window_slide_by_25);

    printf("\n[E] Robustness tests:\n");
    RUN_TEST(test_large_continuous_feed_stability);
    RUN_TEST(test_repeated_reset_and_refill);
    RUN_TEST(test_invalid_args);

    TEST_SUMMARY();
    return TEST_EXIT_CODE();
}
