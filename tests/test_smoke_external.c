/**
 * @file test_smoke_external.c
 * @brief External API smoke test (Review B).
 *
 * This file simulates an external caller who only includes public
 * headers.  It verifies that the public API surface is sufficient
 * to complete a full init-feed-process-get lifecycle WITHOUT access
 * to any src/ internal headers.
 *
 * Key validation:
 *   - hr_algo_ctx_sizeof() provides allocation size
 *   - All 7+1 public functions compile and link
 *   - NULL safety works
 *   - Full lifecycle produces non-NULL output pointers
 */

#include "hr_algo_api.h"
#include "test_framework.h"
#include <string.h>
#include <stdint.h>

/* ================================================================== */
/*  Static ctx buffer — allocated using hr_algo_ctx_sizeof()           */
/*  This is the EXACT pattern an external MCU caller would use.        */
/*  No access to hr_algo_internal.h or sizeof(struct hr_algo_ctx).     */
/* ================================================================== */

#define CTX_BUF_SIZE  (128 * 1024)
static uint8_t g_ctx_buf[CTX_BUF_SIZE] __attribute__((aligned(8)));

/* ================================================================== */
/*  Tests                                                              */
/* ================================================================== */

static int test_ctx_sizeof_positive(void)
{
    size_t sz = hr_algo_ctx_sizeof();
    ASSERT_TRUE(sz > 0);
    ASSERT_TRUE(sz <= CTX_BUF_SIZE);
    return 0;
}

static int test_null_safety(void)
{
    hr_params_t p = hr_algo_default_params();
    hr_algo_init(NULL, &p);
    hr_algo_feed_acc(NULL, 1, 2, 3);
    hr_algo_feed_ppg(NULL, 100, 200, 300, 400);
    hr_algo_process_1s(NULL);
    ASSERT_TRUE(hr_algo_get_output(NULL) == NULL);
    ASSERT_TRUE(hr_algo_get_debug(NULL) == NULL);
    return 0;
}

static int test_full_lifecycle(void)
{
    size_t sz = hr_algo_ctx_sizeof();
    ASSERT_TRUE(sz <= CTX_BUF_SIZE);

    hr_algo_ctx_t *ctx = (hr_algo_ctx_t *)g_ctx_buf;
    memset(g_ctx_buf, 0, sz);

    hr_params_t p = hr_algo_default_params();
    hr_algo_init(ctx, &p);

    /* Feed 8 seconds of data (200 samples at 25 Hz) */
    for (int i = 0; i < 200; i++) {
        hr_algo_feed_acc(ctx, (int16_t)(i % 100), 0, 0);
        hr_algo_feed_ppg(ctx, 10000 + i, 10000 + i,
                         10000 + i, 10000 + i);
    }

    hr_algo_process_1s(ctx);

    const hr_output_t *out = hr_algo_get_output(ctx);
    ASSERT_TRUE(out != NULL);

    const hr_debug_frame_t *dbg = hr_algo_get_debug(ctx);
    ASSERT_TRUE(dbg != NULL);

    /* Verify output fields are valid enum values */
    ASSERT_TRUE(out->hr_state >= HR_STATE_INIT &&
                out->hr_state <= HR_STATE_REACQUIRE);
    ASSERT_TRUE(out->motion_state >= MOTION_STATE_REST &&
                out->motion_state <= MOTION_STATE_IRREGULAR);
    ASSERT_TRUE(out->main_ch <= 3);
    ASSERT_TRUE(out->confidence <= 100);

    /* Debug frame produced */
    ASSERT_TRUE(dbg->timestamp_ms > 0);
    ASSERT_TRUE(dbg->hr_state == out->hr_state);
    ASSERT_TRUE(dbg->confidence == out->confidence);

    return 0;
}

static int test_default_params_sensible(void)
{
    hr_params_t p = hr_algo_default_params();
    ASSERT_TRUE(p.candidate.hr_min_bpm > 0.0f);
    ASSERT_TRUE(p.candidate.hr_max_bpm > p.candidate.hr_min_bpm);
    ASSERT_TRUE(p.preproc.ppg_bp_low_hz > 0.0f);
    ASSERT_TRUE(p.preproc.ppg_bp_high_hz > p.preproc.ppg_bp_low_hz);
    ASSERT_TRUE(p.fusion.smooth_alpha > 0.0f);
    ASSERT_TRUE(p.fusion.smooth_alpha <= 1.0f);
    return 0;
}

/* ================================================================== */
/*  main                                                               */
/* ================================================================== */

int main(void)
{
    printf("==========================================\n");
    printf("  External API Smoke Test (Review B)\n");
    printf("==========================================\n\n");

    RUN_TEST(test_ctx_sizeof_positive);
    RUN_TEST(test_null_safety);
    RUN_TEST(test_full_lifecycle);
    RUN_TEST(test_default_params_sensible);

    TEST_SUMMARY();
    return TEST_EXIT_CODE();
}
