/**
 * @file hr_sampling.c
 * @brief M1 Sampling & Buffering — implementation.
 *
 * Manages ACC / PPG ring buffers for raw sample caching.
 * Provides controlled export of the most recent N samples
 * in chronological order (oldest → newest) for downstream processing.
 *
 * Key invariants:
 *  - Buffer full → new sample overwrites the oldest; "most recent N"
 *    semantics remain stable after wrap-around.
 *  - Export order is always chronological (oldest first).
 *  - Export fails explicitly (returns false) when data is insufficient;
 *    no partial / silent-fail export is performed.
 *  - No dynamic memory allocation.
 *  - No large arrays on the stack.
 */

#include "hr_sampling.h"
#include <string.h>

/* ================================================================== */
/*  Initialization                                                     */
/* ================================================================== */

void hr_sampling_init(hr_sampling_ctx_t *ctx)
{
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
}

void hr_sampling_reset(hr_sampling_ctx_t *ctx)
{
    hr_sampling_init(ctx);
}

/* ================================================================== */
/*  Sample push                                                        */
/*                                                                     */
/*  Write at head, advance head with bitmask, bump count up to cap.    */
/*  When count == HR_RING_CAPACITY the oldest sample is silently       */
/*  overwritten — this is the intended circular-buffer behaviour.       */
/* ================================================================== */

void hr_sampling_push_acc(hr_sampling_ctx_t *ctx,
                          int16_t ax, int16_t ay, int16_t az)
{
    if (!ctx) return;

    acc_ring_t *ring = &ctx->acc;
    ring->buf[ring->head].x = ax;
    ring->buf[ring->head].y = ay;
    ring->buf[ring->head].z = az;

    ring->head = (ring->head + 1) & HR_RING_MASK;
    if (ring->count < HR_RING_CAPACITY) {
        ring->count++;
    }
}

void hr_sampling_push_ppg(hr_sampling_ctx_t *ctx,
                          int32_t ch0, int32_t ch1,
                          int32_t ch2, int32_t ch3)
{
    if (!ctx) return;

    ppg_ring_t *ring = &ctx->ppg;
    ring->buf[ring->head].ch[0] = ch0;
    ring->buf[ring->head].ch[1] = ch1;
    ring->buf[ring->head].ch[2] = ch2;
    ring->buf[ring->head].ch[3] = ch3;

    ring->head = (ring->head + 1) & HR_RING_MASK;
    if (ring->count < HR_RING_CAPACITY) {
        ring->count++;
    }
}

/* ================================================================== */
/*  Status queries                                                     */
/* ================================================================== */

bool hr_sampling_ready_for_window(const hr_sampling_ctx_t *ctx)
{
    if (!ctx) return false;

    uint16_t min_count = ctx->acc.count < ctx->ppg.count
                         ? ctx->acc.count
                         : ctx->ppg.count;
    return min_count >= HR_WINDOW_SIZE;
}

uint16_t hr_sampling_get_acc_count(const hr_sampling_ctx_t *ctx)
{
    if (!ctx) return 0;
    return ctx->acc.count;
}

uint16_t hr_sampling_get_ppg_count(const hr_sampling_ctx_t *ctx)
{
    if (!ctx) return 0;
    return ctx->ppg.count;
}

/* ================================================================== */
/*  Window export (internal helper)                                    */
/*                                                                     */
/*  Linearises the most recent n samples from a circular buffer into   */
/*  a caller-provided contiguous array, ordered oldest → newest.       */
/*                                                                     */
/*  Uses at most two memcpy calls (split at the physical wrap point)   */
/*  instead of per-element copy for better cache / DMA efficiency.     */
/* ================================================================== */

bool hr_sampling_export_recent_acc(const hr_sampling_ctx_t *ctx,
                                   acc_sample_t *out, uint16_t n)
{
    if (!ctx || !out || n == 0 || n > HR_RING_CAPACITY) return false;
    if (ctx->acc.count < n) return false;

    const acc_ring_t *ring = &ctx->acc;
    uint16_t start = (ring->head + HR_RING_CAPACITY - n) & HR_RING_MASK;
    uint16_t first = HR_RING_CAPACITY - start;

    if (first >= n) {
        memcpy(out, &ring->buf[start], n * sizeof(acc_sample_t));
    } else {
        memcpy(out, &ring->buf[start], first * sizeof(acc_sample_t));
        memcpy(&out[first], &ring->buf[0],
               (n - first) * sizeof(acc_sample_t));
    }
    return true;
}

bool hr_sampling_export_recent_ppg(const hr_sampling_ctx_t *ctx,
                                   ppg_sample_t *out, uint16_t n)
{
    if (!ctx || !out || n == 0 || n > HR_RING_CAPACITY) return false;
    if (ctx->ppg.count < n) return false;

    const ppg_ring_t *ring = &ctx->ppg;
    uint16_t start = (ring->head + HR_RING_CAPACITY - n) & HR_RING_MASK;
    uint16_t first = HR_RING_CAPACITY - start;

    if (first >= n) {
        memcpy(out, &ring->buf[start], n * sizeof(ppg_sample_t));
    } else {
        memcpy(out, &ring->buf[start], first * sizeof(ppg_sample_t));
        memcpy(&out[first], &ring->buf[0],
               (n - first) * sizeof(ppg_sample_t));
    }
    return true;
}
