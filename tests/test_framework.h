/**
 * @file test_framework.h
 * @brief Minimal self-contained test framework for heart-rate project.
 *
 * No external dependencies.  Each test function returns 0 (pass) or 1 (fail).
 * ASSERT macros print location on failure and force an early return 1.
 * RUN_TEST collects pass/fail counts; TEST_SUMMARY prints the report.
 *
 * Usage:
 *   static int test_foo(void) {
 *       ASSERT_TRUE(1 + 1 == 2);
 *       ASSERT_INT_EQ(42, 42);
 *       return 0;
 *   }
 *   int main(void) {
 *       RUN_TEST(test_foo);
 *       TEST_SUMMARY();
 *       return TEST_EXIT_CODE();
 *   }
 */

#ifndef TEST_FRAMEWORK_H
#define TEST_FRAMEWORK_H

#include <stdio.h>

static int _tf_run  = 0;
static int _tf_pass = 0;
static int _tf_fail = 0;

/* ------------------------------------------------------------------ */
/*  Assertion macros                                                   */
/* ------------------------------------------------------------------ */

#define ASSERT_TRUE(cond) do {                                         \
    if (!(cond)) {                                                     \
        printf("    ASSERT FAIL [%s:%d]: %s\n",                        \
               __FILE__, __LINE__, #cond);                             \
        return 1;                                                      \
    }                                                                  \
} while (0)

#define ASSERT_FALSE(cond) ASSERT_TRUE(!(cond))

#define ASSERT_FLOAT_NEAR(a, b, tol) do {                              \
    double _fa = (double)(a);                                          \
    double _fb = (double)(b);                                          \
    double _diff = (_fa > _fb) ? (_fa - _fb) : (_fb - _fa);           \
    if (_diff > (double)(tol)) {                                       \
        printf("    ASSERT FAIL [%s:%d]: |%s - %s| <= %s  "           \
               "(%.8g vs %.8g, diff %.8g)\n",                          \
               __FILE__, __LINE__, #a, #b, #tol, _fa, _fb, _diff);    \
        return 1;                                                      \
    }                                                                  \
} while (0)

#define ASSERT_INT_EQ(a, b) do {                                       \
    long long _va = (long long)(a);                                    \
    long long _vb = (long long)(b);                                    \
    if (_va != _vb) {                                                  \
        printf("    ASSERT FAIL [%s:%d]: %s == %s  (%lld != %lld)\n", \
               __FILE__, __LINE__, #a, #b, _va, _vb);                 \
        return 1;                                                      \
    }                                                                  \
} while (0)

/* ------------------------------------------------------------------ */
/*  Test runner macros                                                 */
/* ------------------------------------------------------------------ */

#define RUN_TEST(fn) do {                                              \
    _tf_run++;                                                         \
    if ((fn)() == 0) {                                                 \
        _tf_pass++;                                                    \
        printf("  [PASS] %s\n", #fn);                                 \
    } else {                                                           \
        _tf_fail++;                                                    \
        printf("  [FAIL] %s\n", #fn);                                 \
    }                                                                  \
} while (0)

#define TEST_SUMMARY() do {                                            \
    printf("\n========================================\n");            \
    printf("  Total: %d   Pass: %d   Fail: %d\n",                     \
           _tf_run, _tf_pass, _tf_fail);                               \
    printf("========================================\n");              \
    printf("  Result: %s\n",                                           \
           _tf_fail == 0 ? "ALL PASSED" : "FAILED");                   \
    printf("========================================\n");              \
} while (0)

#define TEST_EXIT_CODE() (_tf_fail == 0 ? 0 : 1)

#endif /* TEST_FRAMEWORK_H */
