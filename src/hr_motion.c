/**
 * @file hr_motion.c
 * @brief M4 Motion State Detection -- implementation.
 *
 * Extracts motion features from M2 preprocessed ACC window and
 * classifies motion state (REST / WALK / RUN / IRREGULAR).
 *
 * Feature extraction pipeline:
 *   1. Per-axis mean removal (window-level gravity estimate)
 *   2. Dynamic acceleration magnitude:  mag_dyn[i] = ||acc[i] - mean||
 *   3. Dynamic energy:  mean( ||acc[i] - mean||^2 )  over the window
 *   4. ACF on zero-mean mag_dyn  ->  periodicity score + dominant freq
 *
 * Classification rules (V1, rule-based, conservative):
 *   REST:      energy < rest_energy_threshold
 *   RUN:       energy >= run_threshold  AND  periodic  AND  freq in run band
 *   WALK:      energy >= walk_threshold AND  periodic  AND  freq in walk band
 *   IRREGULAR: fallback for all other non-REST states
 *
 * No dynamic memory.  No large arrays on the stack.  All intermediate
 * buffers live in hr_motion_ctx_t (embedded in hr_algo_ctx_t).
 */

#include "hr_motion.h"

#include <string.h>
#include <math.h>

/* ================================================================== */
/*  V1 design constants                                                */
/*                                                                     */
/*  Each constant documents its semantic meaning below.  If field      */
/*  tuning shows a constant needs per-device or per-population         */
/*  adjustment, promote it to hr_params_t.motion at that time.         */
/* ================================================================== */

/*
 * Activity frequency range for ACF periodicity search.
 *
 * Human locomotion typically spans 0.8 - 4.0 Hz:
 *   Slow walk  ~1.4 Hz    Brisk walk ~2.0 Hz
 *   Slow run   ~2.5 Hz    Fast run   ~3.5 Hz
 *
 * At 25 Hz sampling:
 *   lag_min = floor(25 / 4.0) = 6      (upper freq bound -> shorter lag)
 *   lag_max = floor(25 / 0.8) = 31     (lower freq bound -> longer lag)
 *
 * Future parameterisation: promote to hr_params_t.motion if device
 * placement or target sport requires a different frequency range.
 */
#define MOTION_ACF_LAG_MIN    6
#define MOTION_ACF_LAG_MAX    31

/*
 * Periodicity threshold: minimum normalised ACF peak to classify
 * motion as periodic.  Below this, motion is IRREGULAR regardless
 * of energy.
 *
 * Empirical basis:
 *   - Steady walking/running ACF peak: typically 0.5 - 0.8
 *   - Random arm motion:               typically < 0.2
 *   - 0.3 provides conservative separation with margin
 *
 * Future parameterisation: promote to hr_params_t.motion.periodicity_threshold
 * if per-device calibration is required.
 */
#define MOTION_PERIODICITY_THRESHOLD  0.3f

/*
 * Walking frequency band: 1.2 - 2.5 Hz.
 * Covers slow walk (~1.4 Hz) through brisk walk (~2.2 Hz) with margin.
 *
 * Running frequency band: >= 2.5 Hz (open upper end, capped by lag search).
 * Running step frequency typically 2.5 - 3.8 Hz.
 *
 * Future parameterisation: promote to hr_params_t.motion.walk_freq_min_hz etc.
 * if device placement (wrist vs. chest) shifts the frequency mapping.
 */
#define MOTION_WALK_FREQ_MIN_HZ  1.2f
#define MOTION_WALK_FREQ_MAX_HZ  2.5f
#define MOTION_RUN_FREQ_MIN_HZ   2.5f

/*
 * Energy floor for ACF denominator (acf[0]).
 * Signals with total power below this are treated as flatline;
 * periodicity is forced to 0 to avoid division-by-near-zero.
 */
#define MOTION_ENERGY_FLOOR  1e-10f

/* ================================================================== */
/*  Internal helpers (static, file scope)                              */
/* ================================================================== */

/**
 * Compute dynamic acceleration magnitude buffer and dynamic energy.
 *
 * For each sample i in [0, n):
 *   dx = acc[i][0] - mean_x
 *   dy = acc[i][1] - mean_y
 *   dz = acc[i][2] - mean_z
 *   mag_out[i] = sqrt(dx^2 + dy^2 + dz^2)
 *   energy_sum += dx^2 + dy^2 + dz^2
 *
 * Returns dynamic_energy = energy_sum / n.
 *
 * The per-axis mean subtraction removes the gravity (DC) component
 * that M2 intentionally preserves in the ACC low-pass output.
 */
static float compute_dynamic_features(const float (*acc_data)[3],
                                      uint16_t n,
                                      float *mag_out)
{
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    for (uint16_t i = 0; i < n; i++) {
        sum_x += acc_data[i][0];
        sum_y += acc_data[i][1];
        sum_z += acc_data[i][2];
    }
    float inv_n  = 1.0f / (float)n;
    float mean_x = sum_x * inv_n;
    float mean_y = sum_y * inv_n;
    float mean_z = sum_z * inv_n;

    float energy_sum = 0.0f;
    for (uint16_t i = 0; i < n; i++) {
        float dx = acc_data[i][0] - mean_x;
        float dy = acc_data[i][1] - mean_y;
        float dz = acc_data[i][2] - mean_z;
        float e  = dx * dx + dy * dy + dz * dz;
        energy_sum += e;
        mag_out[i] = sqrtf(e);
    }

    return energy_sum * inv_n;
}

/**
 * Compute periodicity and dominant frequency via ACF on dynamic
 * magnitude signal.
 *
 * Steps:
 *   1. Mean-subtract mag_buf in place (zero-mean for proper ACF)
 *   2. Compute acf[0] (signal power); bail out if below energy floor
 *   3. For each lag in [LAG_MIN, LAG_MAX]: normalised ACF = acf[lag]/acf[0]
 *   4. Peak normalised ACF  ->  periodicity;  peak lag  ->  dominant_freq
 *
 * dominant_freq_hz is only set when periodicity >= PERIODICITY_THRESHOLD.
 * Otherwise it stays 0.0, signalling "unreliable / not applicable".
 *
 * NOTE: mag_buf is modified in place (mean subtracted).  Its content
 * is undefined after this function returns.
 */
static void compute_acf_features(float *mag_buf,
                                 uint16_t n,
                                 float sample_rate_hz,
                                 float *out_periodicity,
                                 float *out_freq_hz)
{
    *out_periodicity = 0.0f;
    *out_freq_hz     = 0.0f;

    if (n <= (uint16_t)MOTION_ACF_LAG_MAX) {
        return;
    }

    /* Mean-subtract in place */
    float sum = 0.0f;
    for (uint16_t i = 0; i < n; i++) {
        sum += mag_buf[i];
    }
    float mean = sum / (float)n;
    for (uint16_t i = 0; i < n; i++) {
        mag_buf[i] -= mean;
    }

    /* ACF at lag 0 (total signal variance * n) */
    float acf0 = 0.0f;
    for (uint16_t i = 0; i < n; i++) {
        acf0 += mag_buf[i] * mag_buf[i];
    }
    if (acf0 < MOTION_ENERGY_FLOOR) {
        return;   /* flatline / near-zero dynamic signal */
    }

    /* Search for max normalised ACF peak in activity lag range */
    float best_nacf = 0.0f;
    int   best_lag  = 0;

    int lag_max = MOTION_ACF_LAG_MAX;
    if (lag_max >= (int)n) {
        lag_max = (int)n - 1;
    }

    for (int lag = MOTION_ACF_LAG_MIN; lag <= lag_max; lag++) {
        float acf_val = 0.0f;
        uint16_t limit = n - (uint16_t)lag;
        for (uint16_t i = 0; i < limit; i++) {
            acf_val += mag_buf[i] * mag_buf[i + lag];
        }
        float nacf = acf_val / acf0;
        if (nacf > best_nacf) {
            best_nacf = nacf;
            best_lag  = lag;
        }
    }

    /* Clamp to [0, 1] */
    if (best_nacf < 0.0f) best_nacf = 0.0f;
    if (best_nacf > 1.0f) best_nacf = 1.0f;

    *out_periodicity = best_nacf;

    /*
     * Derive dominant frequency only when periodicity is meaningful.
     * For aperiodic / low-periodicity signals the ACF peak lag is
     * unreliable — report 0 Hz to avoid downstream mis-use.
     */
    if (best_lag > 0 && best_nacf >= MOTION_PERIODICITY_THRESHOLD) {
        *out_freq_hz = sample_rate_hz / (float)best_lag;
    }
}

/**
 * Classify motion state from extracted features.
 *
 * Decision tree (V1, conservative):
 *
 *   1. energy < rest_threshold
 *        -> REST
 *
 *   2. periodicity < PERIODICITY_THRESHOLD
 *        -> IRREGULAR   (active but aperiodic / random)
 *
 *   3. energy >= run_threshold  AND  freq >= RUN_FREQ_MIN
 *        -> RUN
 *
 *   4. energy >= walk_threshold AND  freq in [WALK_FREQ_MIN, WALK_FREQ_MAX]
 *        -> WALK
 *
 *   5. fallback
 *        -> IRREGULAR   (periodic but energy/freq combo does not match
 *                        any stable locomotion pattern)
 *
 * Design principle: ambiguous cases default to IRREGULAR rather than
 * risk a false WALK/RUN classification.
 */
static motion_state_t classify_motion_state(float energy,
                                            float periodicity,
                                            float freq_hz,
                                            const hr_params_t *params)
{
    /* Rule 1: REST */
    if (energy < params->motion.rest_energy_threshold) {
        return MOTION_STATE_REST;
    }

    /* Rule 2: active but aperiodic */
    if (periodicity < MOTION_PERIODICITY_THRESHOLD) {
        return MOTION_STATE_IRREGULAR;
    }

    /* Rule 3: RUN — high energy + periodic + high-frequency cadence */
    if (energy >= params->motion.run_energy_threshold &&
        freq_hz >= MOTION_RUN_FREQ_MIN_HZ) {
        return MOTION_STATE_RUN;
    }

    /* Rule 4: WALK — moderate energy + periodic + walk-band cadence */
    if (energy >= params->motion.walk_energy_threshold &&
        freq_hz >= MOTION_WALK_FREQ_MIN_HZ &&
        freq_hz <= MOTION_WALK_FREQ_MAX_HZ) {
        return MOTION_STATE_WALK;
    }

    /* Rule 5: periodic motion that doesn't fit WALK or RUN pattern */
    return MOTION_STATE_IRREGULAR;
}

/* ================================================================== */
/*  Public (internal) API                                              */
/* ================================================================== */

void hr_motion_init(hr_motion_ctx_t *ctx)
{
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
    /* result.state defaults to MOTION_STATE_REST (== 0) */
    /* prev_state  defaults to MOTION_STATE_REST (== 0) */
    /* initialized defaults to false                    */
}

void hr_motion_run(hr_motion_ctx_t *ctx,
                   const acc_preproc_out_t *acc_in,
                   uint16_t n,
                   const hr_params_t *params)
{
    if (!ctx || !acc_in || !params) return;
    if (n == 0 || n > HR_WINDOW_SIZE) return;

    /* --- Step 1: dynamic magnitude + energy ----------------------- */
    float energy = compute_dynamic_features(acc_in->data, n,
                                            ctx->mag_dyn_buf);

    /* --- Step 2: periodicity + dominant frequency (ACF-based) ----- */
    /*     NOTE: compute_acf_features modifies mag_dyn_buf in place   */
    /*     (mean-subtraction).  Buffer content is scratch after this. */
    float periodicity = 0.0f;
    float freq_hz     = 0.0f;
    compute_acf_features(ctx->mag_dyn_buf, n,
                         HR_PREPROC_SAMPLE_RATE_HZ,
                         &periodicity, &freq_hz);

    /* --- Step 3: classify ----------------------------------------- */
    motion_state_t state = classify_motion_state(energy, periodicity,
                                                 freq_hz, params);

    /* --- Step 4: store results ------------------------------------ */
    ctx->result.state            = state;
    ctx->result.dynamic_energy   = energy;
    ctx->result.dominant_freq_hz = freq_hz;
    ctx->result.periodicity      = periodicity;
    ctx->result.periodic_motion  = (periodicity >= MOTION_PERIODICITY_THRESHOLD);

    /* --- Step 5: update cross-cycle state ------------------------- */
    ctx->prev_state  = state;
    ctx->initialized = true;
}

void hr_motion_reset(hr_motion_ctx_t *ctx)
{
    hr_motion_init(ctx);
}
