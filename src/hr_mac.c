/**
 * @file hr_mac.c
 * @brief M5 Motion Artifact Cancellation — implementation.
 *
 * First-version MAC using multi-input NLMS adaptive filter.
 *
 * Signal model:
 *   d[n] = s[n] + v[n]
 *   where s[n] is the PPG heart-rate component and v[n] is motion artifact.
 *   3-axis ACC (de-meaned) serves as a correlated reference for v[n].
 *
 * NLMS adaptive filter:
 *   Reference vector u[n] = [ax_dyn(n..n-L+1), ay_dyn(n..n-L+1), az_dyn(n..n-L+1)]
 *   Filter output:     y[n] = w^T u[n]            (artifact estimate)
 *   Error signal:      e[n] = d[n] - y[n]         (cleaned PPG = MAC output)
 *   Weight update:     w += mu * e[n] * u[n] / (eps + ||u[n]||^2)
 *
 * Weight persistence:
 *   Weights persist across calls to allow convergence over multiple windows.
 *   Delay lines are reset per call because M2 is block-stateless — the
 *   same physical samples may yield slightly different preprocessed values
 *   between overlapping windows.
 *
 * Dynamic reference construction:
 *   M2 ACC output preserves DC (gravity).  M5 removes it by subtracting
 *   the per-axis window mean before feeding ACC into the NLMS reference.
 *   This ensures the filter models dynamic motion artifacts, not gravity.
 *
 * No dynamic memory.  No large arrays on the stack.
 */

#include "hr_mac.h"

#include <string.h>
#include <math.h>

/* ================================================================== */
/*  V1 internal design constants                                       */
/*                                                                     */
/*  These are stability/safety parameters, not algorithm tuning knobs. */
/*  If future tuning requires per-device adjustment, promote to        */
/*  hr_params_t.mac at that time.                                      */
/* ================================================================== */

/*
 * NLMS regularisation epsilon.
 * Prevents division by zero when reference signal power is near zero
 * (e.g., nearly motionless but not classified as REST).
 * Chosen well above float epsilon (~1e-7) but small enough not to
 * impede convergence.
 */
#define MAC_NLMS_EPS  1e-6f

/*
 * Divergence: maximum allowed ratio of output energy to input energy.
 * If mean(e^2) > RATIO * mean(d^2), the filter is amplifying rather
 * than cancelling — declare diverged.
 * Value 10 allows moderate transient overshoot while catching runaway.
 */
#define MAC_DIVERGE_ENERGY_RATIO  10.0f

/*
 * Divergence: maximum allowed weight vector L2 norm (squared).
 * Catches slow weight drift even when the output looks temporarily sane.
 */
#define MAC_WEIGHT_NORM_LIMIT  1e6f

/*
 * Minimum input signal energy (mean d^2) below which energy-ratio
 * divergence check is skipped.  Avoids false divergence on near-silent
 * PPG signals where any tiny output could exceed the ratio.
 */
#define MAC_ENERGY_FLOOR  1e-10f

/* ================================================================== */
/*  Internal helpers                                                   */
/* ================================================================== */

/** Portable finite check for a single float. */
static inline bool mac_is_finite(float x)
{
    return isfinite(x) != 0;
}

/**
 * Copy the raw main-channel PPG waveform into the MAC output buffer.
 *
 * Used for bypass, invalid, and divergence fallback cases so that
 * out.data always contains readable data.  IMPORTANT: even when raw
 * is copied here, result.valid == false tells M6 that this is NOT a
 * useful MAC result — it must rely on the raw branch only.
 */
static void mac_copy_raw_to_output(hr_mac_ctx_t *ctx,
                                   const ppg_preproc_out_t *ppg_in,
                                   uint8_t ch,
                                   uint16_t n)
{
    for (uint16_t i = 0; i < n; i++) {
        ctx->out.data[i] = ppg_in->data[i][ch];
    }
    for (uint16_t i = n; i < HR_WINDOW_SIZE; i++) {
        ctx->out.data[i] = 0.0f;
    }
}

/** Fill result struct for the bypass case (intentional skip). */
static void mac_set_bypass(mac_result_t *r, uint8_t ch)
{
    r->valid               = false;
    r->bypassed            = true;
    r->diverged            = false;
    r->main_ch_used        = ch;
    r->input_energy        = 0.0f;
    r->output_energy       = 0.0f;
    r->artifact_est_energy = 0.0f;
}

/** Fill result struct for the invalid-input case. */
static void mac_set_invalid(mac_result_t *r, uint8_t ch)
{
    r->valid               = false;
    r->bypassed            = false;
    r->diverged            = false;
    r->main_ch_used        = ch;
    r->input_energy        = 0.0f;
    r->output_energy       = 0.0f;
    r->artifact_est_energy = 0.0f;
}

/* ================================================================== */
/*  Core NLMS processing (static, called only from hr_mac_run)         */
/*                                                                     */
/*  Returns true on success, false if divergence was detected.         */
/*  On divergence the caller is responsible for cleanup.               */
/* ================================================================== */

static bool mac_run_nlms(hr_mac_ctx_t *ctx,
                         const ppg_preproc_out_t *ppg_in,
                         const acc_preproc_out_t *acc_in,
                         uint8_t main_ch,
                         uint16_t n,
                         float mu,
                         float *out_sum_d2,
                         float *out_sum_e2,
                         float *out_sum_y2)
{
    const uint16_t L     = ctx->effective_order;
    const uint16_t total = ctx->total_taps;

    /* --- Compute per-axis ACC means for dynamic reference ---------- */
    float mean_acc[3] = {0.0f, 0.0f, 0.0f};
    for (uint16_t i = 0; i < n; i++) {
        mean_acc[0] += acc_in->data[i][0];
        mean_acc[1] += acc_in->data[i][1];
        mean_acc[2] += acc_in->data[i][2];
    }
    float inv_n = 1.0f / (float)n;
    mean_acc[0] *= inv_n;
    mean_acc[1] *= inv_n;
    mean_acc[2] *= inv_n;

    /* --- Reset delay lines (weights persist across calls) ---------- */
    memset(ctx->delay_line, 0, sizeof(float) * (size_t)total);

    float sum_d2 = 0.0f;
    float sum_e2 = 0.0f;
    float sum_y2 = 0.0f;

    /* --- Sample-by-sample NLMS ------------------------------------ */
    for (uint16_t i = 0; i < n; i++) {

        /* Target: main-channel preprocessed PPG */
        float d = ppg_in->data[i][main_ch];

        /* Dynamic ACC reference (gravity removed) */
        float ref[3];
        ref[0] = acc_in->data[i][0] - mean_acc[0];
        ref[1] = acc_in->data[i][1] - mean_acc[1];
        ref[2] = acc_in->data[i][2] - mean_acc[2];

        /* Shift delay lines and insert new reference values.
         * Layout per axis a: delay_line[a*L + 0] = newest,
         *                    delay_line[a*L + L-1] = oldest. */
        for (int ax = 0; ax < 3; ax++) {
            uint16_t base = (uint16_t)(ax * L);
            for (uint16_t t = L - 1; t > 0; t--) {
                ctx->delay_line[base + t] = ctx->delay_line[base + t - 1];
            }
            ctx->delay_line[base] = ref[ax];
        }

        /* Filter output y = w^T u  and  reference power ||u||^2 */
        float y       = 0.0f;
        float norm_sq = 0.0f;
        for (uint16_t k = 0; k < total; k++) {
            y       += ctx->weights[k] * ctx->delay_line[k];
            norm_sq += ctx->delay_line[k] * ctx->delay_line[k];
        }

        /* Error signal (cleaned PPG) */
        float e = d - y;

        /* NaN/Inf check on critical values */
        if (!mac_is_finite(y) || !mac_is_finite(e)) {
            return false;
        }

        ctx->out.data[i] = e;

        sum_d2 += d * d;
        sum_e2 += e * e;
        sum_y2 += y * y;

        /* NLMS weight update */
        float denom = MAC_NLMS_EPS + norm_sq;
        float step  = mu * e / denom;

        if (!mac_is_finite(step)) {
            return false;
        }

        for (uint16_t k = 0; k < total; k++) {
            ctx->weights[k] += step * ctx->delay_line[k];
        }
    }

    /* Zero-fill beyond n (guard for n < HR_WINDOW_SIZE) */
    for (uint16_t i = n; i < HR_WINDOW_SIZE; i++) {
        ctx->out.data[i] = 0.0f;
    }

    *out_sum_d2 = sum_d2;
    *out_sum_e2 = sum_e2;
    *out_sum_y2 = sum_y2;
    return true;
}

/* ================================================================== */
/*  Post-NLMS divergence checks                                        */
/*                                                                     */
/*  Returns true if divergence is detected (filter unusable).          */
/* ================================================================== */

static bool mac_check_divergence(const hr_mac_ctx_t *ctx,
                                 float sum_d2, float sum_e2,
                                 float inv_n)
{
    /* Check weight vector for NaN/Inf and excessive norm */
    float wnorm_sq = 0.0f;
    for (uint16_t k = 0; k < ctx->total_taps; k++) {
        if (!mac_is_finite(ctx->weights[k])) {
            return true;
        }
        wnorm_sq += ctx->weights[k] * ctx->weights[k];
    }
    if (wnorm_sq > MAC_WEIGHT_NORM_LIMIT) {
        return true;
    }

    /* Energy-ratio check (skip when input is near-silent) */
    float input_e  = sum_d2 * inv_n;
    float output_e = sum_e2 * inv_n;
    if (input_e > MAC_ENERGY_FLOOR &&
        output_e > MAC_DIVERGE_ENERGY_RATIO * input_e) {
        return true;
    }

    return false;
}

/* ================================================================== */
/*  Public (internal) API                                              */
/* ================================================================== */

void hr_mac_init(hr_mac_ctx_t *ctx, const hr_params_t *params)
{
    if (!ctx || !params) return;

    memset(ctx, 0, sizeof(*ctx));

    uint16_t order = params->mac.filter_order;
    if (order > HR_MAC_MAX_ORDER) {
        order = HR_MAC_MAX_ORDER;
    }
    ctx->effective_order = order;
    ctx->total_taps      = (uint16_t)(3u * order);
    ctx->initialized     = true;
}

void hr_mac_run(hr_mac_ctx_t *ctx,
                const ppg_preproc_out_t *ppg_in,
                const acc_preproc_out_t *acc_in,
                const sqi_result_t *sqi_in,
                const motion_result_t *motion_in,
                uint16_t n,
                const hr_params_t *params)
{
    /* --- Input validation ----------------------------------------- */
    if (!ctx || !ctx->initialized) {
        if (ctx) mac_set_invalid(&ctx->result, 0);
        return;
    }
    if (!ppg_in || !acc_in || !sqi_in || !motion_in || !params) {
        mac_set_invalid(&ctx->result, 0);
        return;
    }
    if (n == 0 || n > HR_WINDOW_SIZE) {
        mac_set_invalid(&ctx->result, sqi_in->main_ch);
        return;
    }

    uint8_t main_ch = sqi_in->main_ch;

    /* --- Guard: main_ch must be in [0, 3] ------------------------- */
    if (main_ch > 3) {
        mac_set_invalid(&ctx->result, main_ch);
        mac_copy_raw_to_output(ctx, ppg_in, 0, n);
        return;
    }

    /* --- REST bypass: do not run MAC on clean signal --------------- */
    /*     Running NLMS on a resting signal would only add noise.     */
    if (motion_in->state == MOTION_STATE_REST) {
        mac_set_bypass(&ctx->result, main_ch);
        mac_copy_raw_to_output(ctx, ppg_in, main_ch, n);
        return;
    }

    /* --- Guard: filter must have positive order and step size ------ */
    if (ctx->effective_order == 0 || ctx->total_taps == 0 ||
        params->mac.nlms_step_size <= 0.0f) {
        mac_set_bypass(&ctx->result, main_ch);
        mac_copy_raw_to_output(ctx, ppg_in, main_ch, n);
        return;
    }

    /* --- Run NLMS ------------------------------------------------- */
    float sum_d2 = 0.0f, sum_e2 = 0.0f, sum_y2 = 0.0f;
    float mu = params->mac.nlms_step_size;

    bool ok = mac_run_nlms(ctx, ppg_in, acc_in, main_ch, n, mu,
                           &sum_d2, &sum_e2, &sum_y2);

    float inv_n = 1.0f / (float)n;

    /* --- Divergence handling -------------------------------------- */
    if (!ok || mac_check_divergence(ctx, sum_d2, sum_e2, inv_n)) {
        /* Reset weights to allow recovery on next call */
        memset(ctx->weights, 0, sizeof(float) * (size_t)ctx->total_taps);
        memset(ctx->delay_line, 0, sizeof(float) * (size_t)ctx->total_taps);

        mac_copy_raw_to_output(ctx, ppg_in, main_ch, n);

        ctx->result.valid               = false;
        ctx->result.bypassed            = false;
        ctx->result.diverged            = true;
        ctx->result.main_ch_used        = main_ch;
        ctx->result.input_energy        = sum_d2 * inv_n;
        ctx->result.output_energy       = 0.0f;
        ctx->result.artifact_est_energy = 0.0f;
        return;
    }

    /* --- Success -------------------------------------------------- */
    ctx->result.valid               = true;
    ctx->result.bypassed            = false;
    ctx->result.diverged            = false;
    ctx->result.main_ch_used        = main_ch;
    ctx->result.input_energy        = sum_d2 * inv_n;
    ctx->result.output_energy       = sum_e2 * inv_n;
    ctx->result.artifact_est_energy = sum_y2 * inv_n;
}

void hr_mac_reset(hr_mac_ctx_t *ctx, const hr_params_t *params)
{
    hr_mac_init(ctx, params);
}
