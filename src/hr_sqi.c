/**
 * @file hr_sqi.c
 * @brief M3 SQI & Channel Selection — implementation.
 *
 * Computes per-channel Signal Quality Index from M2 preprocessed PPG
 * and selects main/backup channels with hysteresis.
 *
 * SQI model (first version):
 *   total = W_PERIODICITY * periodicity + W_PEAK_REG * peak_regularity
 *
 *   periodicity:     max normalized ACF peak in valid HR lag range
 *   peak_regularity: 1 - CV(inter-peak intervals), clamped to [0,1]
 *
 * Channel selection:
 *   main_ch  = highest total SQI (with hysteresis — O11)
 *   backup_ch = second highest usable channel (or main_ch if none)
 *
 * Hysteresis (O11):
 *   Switch only when a candidate channel exceeds current main_ch
 *   by switch_margin for switch_hold_count consecutive cycles.
 *   Immediate switch if current main_ch becomes unusable.
 *
 * No dynamic memory.  No large arrays on stack.
 */

#include "hr_sqi.h"

#include <string.h>
#include <math.h>

/* ================================================================== */
/*  First-version design constants                                     */
/*                                                                     */
/*  These are intentionally fixed for V1.  If future tuning requires   */
/*  parameterization, promote to hr_params_t.sqi.                      */
/* ================================================================== */

/*
 * Physiological HR range mapped to ACF lag at 25 Hz.
 *   220 BPM => period 60/220 = 0.273 s => lag = ceil(25*0.273) = 7
 *    40 BPM => period 60/40  = 1.5   s => lag = ceil(25*1.5)   = 38
 */
#define SQI_ACF_LAG_MIN      7
#define SQI_ACF_LAG_MAX      38

/*
 * Sub-indicator combination weights.
 * Periodicity is weighted higher: ACF is more robust to amplitude
 * variation than simple peak detection.
 */
#define SQI_W_PERIODICITY    0.6f
#define SQI_W_PEAK_REG       0.4f

/*
 * Peak regularity: coefficient of variation (CV) ceiling.
 * CV >= this value yields peak_regularity = 0.
 * Stable PPG typically has CV < 0.1; noisy signals ~0.3-0.5.
 */
#define SQI_CV_CEILING       0.5f

/*
 * Max peaks tracked during peak detection.
 * 200 samples / 7 min-lag ~ 28 peaks physiologically;
 * 50 gives headroom.  Stack: 50 * sizeof(uint16_t) = 100 bytes.
 */
#define SQI_MAX_PEAKS        50

/* Minimum energy threshold for ACF denominator (acf0) */
#define SQI_ENERGY_FLOOR     1e-10f

/* ================================================================== */
/*  Helper: periodicity score for one channel (ACF-based)              */
/* ================================================================== */

/**
 * Max normalized ACF peak in the valid HR lag range [LAG_MIN, LAG_MAX].
 *
 * Accesses channel data via stride: data[i][ch].
 * Returns 0 for flatline / near-zero-energy signals.
 * Result clamped to [0.0, 1.0].
 */
static float compute_periodicity(const float (*data)[4],
                                 uint16_t n, int ch)
{
    float sum = 0.0f;
    for (uint16_t i = 0; i < n; i++) {
        sum += data[i][ch];
    }
    float mean = sum / (float)n;

    float acf0 = 0.0f;
    for (uint16_t i = 0; i < n; i++) {
        float d = data[i][ch] - mean;
        acf0 += d * d;
    }

    if (acf0 < SQI_ENERGY_FLOOR) {
        return 0.0f;
    }

    int lag_max = SQI_ACF_LAG_MAX;
    if (lag_max >= (int)n) {
        lag_max = (int)n - 1;
    }

    float best = 0.0f;
    for (int lag = SQI_ACF_LAG_MIN; lag <= lag_max; lag++) {
        float acf_lag = 0.0f;
        uint16_t limit = n - (uint16_t)lag;
        for (uint16_t i = 0; i < limit; i++) {
            acf_lag += (data[i][ch] - mean) * (data[i + lag][ch] - mean);
        }
        float nacf = acf_lag / acf0;
        if (nacf > best) {
            best = nacf;
        }
    }

    if (best < 0.0f) best = 0.0f;
    if (best > 1.0f) best = 1.0f;
    return best;
}

/* ================================================================== */
/*  Helper: peak regularity score for one channel                      */
/* ================================================================== */

/**
 * Detect local maxima above signal mean, compute inter-peak interval
 * CV, and return regularity score = 1 - CV/CV_CEILING, clamped [0,1].
 *
 * Returns 0 if fewer than 2 valid inter-peak intervals exist.
 */
static float compute_peak_regularity(const float (*data)[4],
                                     uint16_t n, int ch)
{
    if (n < 3) return 0.0f;

    float sum = 0.0f;
    for (uint16_t i = 0; i < n; i++) {
        sum += data[i][ch];
    }
    float mean = sum / (float)n;

    uint16_t peaks[SQI_MAX_PEAKS];
    int peak_count = 0;

    for (uint16_t i = 1; i < n - 1 && peak_count < SQI_MAX_PEAKS; i++) {
        float v = data[i][ch];
        if (v > data[i - 1][ch] && v > data[i + 1][ch] && v > mean) {
            peaks[peak_count++] = i;
        }
    }

    if (peak_count < 2) return 0.0f;

    /*
     * Keep only inter-peak intervals >= SQI_ACF_LAG_MIN
     * to filter sub-harmonic / noise-induced close peaks.
     */
    float intervals[SQI_MAX_PEAKS];
    int ivl_count = 0;

    for (int k = 1; k < peak_count; k++) {
        int gap = (int)peaks[k] - (int)peaks[k - 1];
        if (gap >= SQI_ACF_LAG_MIN) {
            intervals[ivl_count++] = (float)gap;
        }
    }

    if (ivl_count < 2) return 0.0f;

    float ivl_mean = 0.0f;
    for (int k = 0; k < ivl_count; k++) {
        ivl_mean += intervals[k];
    }
    ivl_mean /= (float)ivl_count;

    float ivl_var = 0.0f;
    for (int k = 0; k < ivl_count; k++) {
        float d = intervals[k] - ivl_mean;
        ivl_var += d * d;
    }
    ivl_var /= (float)ivl_count;

    float ivl_std = (ivl_var > 0.0f) ? sqrtf(ivl_var) : 0.0f;
    float cv = (ivl_mean > 1e-6f) ? (ivl_std / ivl_mean) : 1.0f;

    float score = 1.0f - cv / SQI_CV_CEILING;
    if (score < 0.0f) score = 0.0f;
    if (score > 1.0f) score = 1.0f;
    return score;
}

/* ================================================================== */
/*  Helper: compute total SQI for one channel                          */
/* ================================================================== */

static void compute_channel_sqi(const float (*data)[4],
                                uint16_t n, int ch,
                                float poor_threshold,
                                sqi_ch_result_t *out)
{
    out->periodicity     = compute_periodicity(data, n, ch);
    out->peak_regularity = compute_peak_regularity(data, n, ch);

    out->total = SQI_W_PERIODICITY * out->periodicity
               + SQI_W_PEAK_REG   * out->peak_regularity;

    if (out->total < 0.0f) out->total = 0.0f;
    if (out->total > 1.0f) out->total = 1.0f;

    out->usable = (out->total >= poor_threshold);
}

/* ================================================================== */
/*  Helper: channel selection with hysteresis (O11)                    */
/*                                                                     */
/*  Hysteresis rules:                                                  */
/*    1. First cycle: no hysteresis; pick best channel directly.       */
/*    2. Current main_ch unusable: immediate switch to best channel.   */
/*    3. Candidate > current + margin for hold_count cycles: switch.   */
/*    4. Otherwise: keep current main_ch.                              */
/*                                                                     */
/*  backup_ch: best usable channel != main_ch, or main_ch if none.    */
/* ================================================================== */

static void select_channels(hr_sqi_ctx_t *ctx, const hr_params_t *params)
{
    sqi_result_t *r = &ctx->result;
    float   margin = params->sqi.switch_margin;
    uint8_t hold   = params->sqi.switch_hold_count;

    /* --- Find best and second-best channels by total SQI --- */
    uint8_t best_ch    = 0;
    uint8_t second_ch  = 0;
    float   best_sqi   = r->ch[0].total;
    float   second_sqi = -1.0f;

    for (uint8_t c = 1; c < 4; c++) {
        if (r->ch[c].total > best_sqi) {
            second_ch  = best_ch;
            second_sqi = best_sqi;
            best_ch    = c;
            best_sqi   = r->ch[c].total;
        } else if (r->ch[c].total > second_sqi) {
            second_ch  = c;
            second_sqi = r->ch[c].total;
        }
    }

    (void)second_ch;
    (void)second_sqi;

    /* --- First cycle: bypass hysteresis, take best directly --- */
    if (!ctx->initialized) {
        r->main_ch = best_ch;
        ctx->prev_main_ch     = best_ch;
        ctx->switch_candidate = best_ch;
        ctx->switch_counter   = 0;
        ctx->initialized      = true;
    } else {
        uint8_t current = ctx->prev_main_ch;
        float   cur_sqi = r->ch[current].total;

        if (!r->ch[current].usable) {
            /* Rule 1: current main_ch unusable → immediate switch */
            r->main_ch            = best_ch;
            ctx->switch_candidate = best_ch;
            ctx->switch_counter   = 0;
        } else if (best_ch != current &&
                   best_sqi > cur_sqi + margin) {
            /* Rule 2: challenger exceeds current + margin */
            if (ctx->switch_candidate == best_ch) {
                ctx->switch_counter++;
            } else {
                ctx->switch_candidate = best_ch;
                ctx->switch_counter   = 1;
            }

            if (hold == 0 || ctx->switch_counter >= hold) {
                r->main_ch            = best_ch;
                ctx->switch_candidate = best_ch;
                ctx->switch_counter   = 0;
            } else {
                r->main_ch = current;
            }
        } else {
            /* Rule 3: no strong challenger → keep current */
            r->main_ch            = current;
            ctx->switch_candidate = current;
            ctx->switch_counter   = 0;
        }

        ctx->prev_main_ch = r->main_ch;
    }

    /* --- Backup: best usable channel != main_ch --- */
    r->backup_ch = r->main_ch;
    for (uint8_t c = 0; c < 4; c++) {
        if (c != r->main_ch && r->ch[c].usable) {
            if (r->backup_ch == r->main_ch ||
                r->ch[c].total > r->ch[r->backup_ch].total) {
                r->backup_ch = c;
            }
        }
    }

    r->sqi_main = r->ch[r->main_ch].total;
}

/* ================================================================== */
/*  Public API                                                         */
/* ================================================================== */

void hr_sqi_init(hr_sqi_ctx_t *ctx)
{
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
}

void hr_sqi_run(hr_sqi_ctx_t *ctx,
                const ppg_preproc_out_t *ppg_in,
                uint16_t n,
                const hr_params_t *params)
{
    if (!ctx || !ppg_in || !params) return;
    if (n == 0 || n > HR_WINDOW_SIZE) return;

    float poor_thr = params->sqi.sqi_poor_threshold;

    for (int ch = 0; ch < 4; ch++) {
        compute_channel_sqi(ppg_in->data, n, ch, poor_thr,
                            &ctx->result.ch[ch]);
    }

    select_channels(ctx, params);
}

void hr_sqi_reset(hr_sqi_ctx_t *ctx)
{
    hr_sqi_init(ctx);
}
