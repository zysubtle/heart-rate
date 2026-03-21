/**
 * @file hr_sqi.h
 * @brief M3 SQI & Channel Selection — internal API.
 *
 * This header is internal to src/ and MUST NOT be exposed as public API.
 *
 * M3 receives the preprocessed PPG window from M2 (200 points x 4 channels,
 * float, oldest-to-newest) and produces:
 *   - Per-channel SQI (4 values, normalized to [0,1])
 *   - main_ch:   primary PPG channel for downstream M5/M6
 *   - backup_ch:  secondary channel; equals main_ch when no independent backup
 *   - sqi_main:   total SQI of the selected main_ch
 *
 * SQI model (first version):
 *   total = 0.6 * periodicity + 0.4 * peak_regularity
 *     periodicity:      max normalized ACF peak in valid HR lag range
 *     peak_regularity:  inter-peak interval consistency (1 - CV)
 *
 * Responsibilities:
 *   - Compute per-channel SQI from two sub-indicators
 *   - Rank channels and select main_ch / backup_ch with hysteresis (O11)
 *
 * NOT responsible for: motion detection, MAC, candidate estimation,
 * fusion, state machine, confidence, or final HR output.
 *
 * Architecture doc: Section 2.1 M3, Section 5 Step 4, Section 6.2,
 * O2 (SQI sub-indicators), O11 (main_ch switch hysteresis).
 */

#ifndef HR_SQI_H
#define HR_SQI_H

#include "hr_preproc.h"
#include "hr_algo_params.h"

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/*  Per-channel SQI result                                             */
/* ------------------------------------------------------------------ */

typedef struct {
    float total;            /* Combined SQI [0.0, 1.0]                      */
    float periodicity;      /* ACF-based periodicity sub-score [0.0, 1.0]   */
    float peak_regularity;  /* Inter-peak interval regularity  [0.0, 1.0]   */
    bool  usable;           /* true if total >= sqi_poor_threshold           */
} sqi_ch_result_t;

/* ------------------------------------------------------------------ */
/*  Overall SQI output (one per processing cycle)                      */
/* ------------------------------------------------------------------ */

typedef struct {
    sqi_ch_result_t ch[4];  /* Per-channel results, indexed [0..3]          */
    uint8_t main_ch;        /* Primary channel [0..3]                       */
    uint8_t backup_ch;      /* Backup channel [0..3]; == main_ch => none    */
    float   sqi_main;       /* == ch[main_ch].total                         */
} sqi_result_t;

/* ------------------------------------------------------------------ */
/*  M3 sub-context                                                     */
/*                                                                     */
/*  Embedded inside hr_algo_ctx_t.  No dynamic memory.                 */
/*  Carries cross-cycle hysteresis state for main_ch switching (O11).  */
/* ------------------------------------------------------------------ */

typedef struct {
    sqi_result_t result;         /* Current cycle output                    */

    /* --- Hysteresis state (O11) --- */
    uint8_t prev_main_ch;        /* Main channel from previous cycle        */
    uint8_t switch_candidate;    /* Channel being considered for switch     */
    uint8_t switch_counter;      /* Consecutive cycles candidate was better */
    bool    initialized;         /* false until first successful run        */
} hr_sqi_ctx_t;

/* ================================================================== */
/*  Internal API                                                       */
/* ================================================================== */

/**
 * Initialize M3 sub-context.
 *
 * Clears all results and hysteresis state.  Sets initialized = false;
 * the first hr_sqi_run() call will select main_ch purely by highest
 * SQI without hysteresis.
 */
void hr_sqi_init(hr_sqi_ctx_t *ctx);

/**
 * Run M3: compute per-channel SQI and select main/backup channels.
 *
 * @param ctx      M3 sub-context (must have been initialized)
 * @param ppg_in   Preprocessed PPG window from M2 (n x 4 channels)
 * @param n        Number of samples (typically HR_WINDOW_SIZE = 200)
 * @param params   Algorithm parameters (reads params->sqi.*)
 *
 * On return:
 *   ctx->result.ch[0..3]   per-channel SQI
 *   ctx->result.main_ch    selected primary channel
 *   ctx->result.backup_ch  selected backup channel
 *   ctx->result.sqi_main   main channel's total SQI
 *   Hysteresis state updated for next cycle.
 *
 * Results are stored in ctx->result only.  M3 does NOT write to
 * hr_output_t or hr_debug_frame_t; that is M9's responsibility.
 */
void hr_sqi_run(hr_sqi_ctx_t *ctx,
                const ppg_preproc_out_t *ppg_in,
                uint16_t n,
                const hr_params_t *params);

/**
 * Reset M3 sub-context to cold-start state.
 * Semantically equivalent to hr_sqi_init().
 */
void hr_sqi_reset(hr_sqi_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* HR_SQI_H */
