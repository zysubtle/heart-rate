/**
 * @file hr_sampling.h
 * @brief M1 Sampling & Buffering — internal API.
 *
 * This header is internal to src/ and MUST NOT be exposed as public API.
 * Downstream modules MUST NOT directly access ring buffer internals
 * (head / count / buf); use the provided export functions instead.
 */

#ifndef HR_SAMPLING_H
#define HR_SAMPLING_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/*  Constants                                                          */
/* ------------------------------------------------------------------ */

/*
 * Ring buffer capacity: 256 samples (power of 2).
 *  - Bitmask indexing (& 0xFF) avoids expensive modulo on MCU
 *  - 56-sample headroom beyond the required 200-sample window
 *  - ACC ring: 256 * 6  = 1 536 bytes
 *  - PPG ring: 256 * 16 = 4 096 bytes
 *  - Total ≈ 5.6 KB, acceptable for Apollo 3.5 class MCU
 */
#define HR_RING_CAPACITY    256
#define HR_RING_MASK        (HR_RING_CAPACITY - 1)

/* Minimum samples required for a valid processing window (8 s @ 25 Hz). */
#define HR_WINDOW_SIZE      200

/* ------------------------------------------------------------------ */
/*  Sample types                                                       */
/* ------------------------------------------------------------------ */

/** ACC sample: 3-axis, int16_t, per input specification (25 Hz). */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} acc_sample_t;

/** PPG sample: 4-channel, int32_t, per input specification (25 Hz). */
typedef struct {
    int32_t ch[4];
} ppg_sample_t;

/* ------------------------------------------------------------------ */
/*  Ring buffer types                                                  */
/*  Internal layout — downstream must not touch head / count / buf.   */
/* ------------------------------------------------------------------ */

typedef struct {
    acc_sample_t buf[HR_RING_CAPACITY];
    uint16_t     head;      /* Next write position [0, HR_RING_CAPACITY) */
    uint16_t     count;     /* Valid samples, capped at HR_RING_CAPACITY */
} acc_ring_t;

typedef struct {
    ppg_sample_t buf[HR_RING_CAPACITY];
    uint16_t     head;
    uint16_t     count;
} ppg_ring_t;

/* ------------------------------------------------------------------ */
/*  Sampling context                                                   */
/*  Aggregates both ring buffers. Intended to be embedded inside       */
/*  hr_algo_ctx_t (defined in hr_algo_internal.h).                     */
/* ------------------------------------------------------------------ */

typedef struct {
    acc_ring_t  acc;
    ppg_ring_t  ppg;
} hr_sampling_ctx_t;

/* ================================================================== */
/*  Internal API                                                       */
/* ================================================================== */

/* --- A. Initialization / Reset ------------------------------------ */

/** Initialize sampling context; clears all ring buffers to zero. */
void hr_sampling_init(hr_sampling_ctx_t *ctx);

/** Reset sampling context to cold-start state (semantically same as init). */
void hr_sampling_reset(hr_sampling_ctx_t *ctx);

/* --- B. Sample push ----------------------------------------------- */

/** Push one ACC sample into the ring buffer. */
void hr_sampling_push_acc(hr_sampling_ctx_t *ctx,
                          int16_t ax, int16_t ay, int16_t az);

/** Push one PPG sample (4 channels) into the ring buffer. */
void hr_sampling_push_ppg(hr_sampling_ctx_t *ctx,
                          int32_t ch0, int32_t ch1,
                          int32_t ch2, int32_t ch3);

/* --- C. Status queries -------------------------------------------- */

/**
 * Check if both ACC and PPG have at least HR_WINDOW_SIZE valid samples.
 * Rule: min(acc.count, ppg.count) >= HR_WINDOW_SIZE.
 */
bool hr_sampling_ready_for_window(const hr_sampling_ctx_t *ctx);

/** Return current valid ACC sample count [0, HR_RING_CAPACITY]. */
uint16_t hr_sampling_get_acc_count(const hr_sampling_ctx_t *ctx);

/** Return current valid PPG sample count [0, HR_RING_CAPACITY]. */
uint16_t hr_sampling_get_ppg_count(const hr_sampling_ctx_t *ctx);

/* --- D. Window export --------------------------------------------- */

/**
 * Export the most recent @p n ACC samples into caller-provided buffer.
 *
 * @param ctx   Sampling context (must not be NULL)
 * @param out   Output buffer, must hold at least @p n elements
 * @param n     Number of samples to export (typically HR_WINDOW_SIZE)
 * @return      true  — success, @p out filled oldest → newest
 *              false — data insufficient (count < n), or invalid params
 *                      (NULL pointers, n == 0, n > HR_RING_CAPACITY).
 *                      On failure @p out is NOT modified.
 */
bool hr_sampling_export_recent_acc(const hr_sampling_ctx_t *ctx,
                                   acc_sample_t *out, uint16_t n);

/**
 * Export the most recent @p n PPG samples into caller-provided buffer.
 * Same semantics as hr_sampling_export_recent_acc.
 */
bool hr_sampling_export_recent_ppg(const hr_sampling_ctx_t *ctx,
                                   ppg_sample_t *out, uint16_t n);

#ifdef __cplusplus
}
#endif

#endif /* HR_SAMPLING_H */
