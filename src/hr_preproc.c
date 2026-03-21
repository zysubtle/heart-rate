/**
 * @file hr_preproc.c
 * @brief M2 Preprocessing — implementation.
 *
 * Filters ACC and PPG raw integer windows into float outputs suitable
 * for downstream SQI (M3), motion detection (M4), and MAC (M5).
 *
 * PPG path (per channel, independent):
 *   int32_t → float → 2nd-order Butterworth HP → 2nd-order Butterworth LP
 *   HP removes DC / baseline drift.  LP removes high-frequency noise.
 *
 * ACC path (per axis, independent):
 *   int16_t → float → 2nd-order Butterworth LP
 *   LP smooths; DC (gravity) is intentionally preserved.
 *
 * Filter design: bilinear-transform Butterworth, coefficients computed
 * once at init from hr_params_t.preproc fields.
 *
 * Processing mode: block-stateless with DC warm-up.  Each call to
 * hr_preproc_run() re-initialises filter states from the first sample
 * of the block, then processes all n samples sequentially.
 *
 * No dynamic memory.  No large arrays on the stack.
 */

#include "hr_preproc.h"
#include <string.h>
#include <math.h>

/* ================================================================== */
/*  Biquad primitives                                                  */
/* ================================================================== */

/**
 * Process one sample through a biquad in Direct Form II Transposed.
 *
 *   y = b0*x + w1
 *   w1 = b1*x - a1*y + w2
 *   w2 = b2*x - a2*y
 */
static inline float biquad_df2t_process(const biquad_coeff_t *c,
                                        biquad_state_t *s,
                                        float x)
{
    float y  = c->b0 * x + s->w1;
    s->w1    = c->b1 * x - c->a1 * y + s->w2;
    s->w2    = c->b2 * x - c->a2 * y;
    return y;
}

/* ================================================================== */
/*  Butterworth coefficient design via bilinear transform              */
/*                                                                     */
/*  2nd-order Butterworth Q = 1/sqrt(2).                               */
/*  K = tan(pi * fc / fs)   (frequency pre-warping).                   */
/*                                                                     */
/*  If fc is out of range (<=0 or >= Nyquist), the filter degrades     */
/*  to all-pass (b0=1, others=0) to guarantee numerical stability.     */
/* ================================================================== */

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define BUTTERWORTH_Q  0.70710678118654752f   /* 1/sqrt(2) */

static void biquad_design_lpf(biquad_coeff_t *c, float fc_hz, float fs_hz)
{
    if (fc_hz <= 0.0f || fc_hz >= fs_hz * 0.5f || fs_hz <= 0.0f) {
        c->b0 = 1.0f; c->b1 = 0.0f; c->b2 = 0.0f;
        c->a1 = 0.0f; c->a2 = 0.0f;
        return;
    }

    float K    = tanf((float)M_PI * fc_hz / fs_hz);
    float K2   = K * K;
    float KoQ  = K / BUTTERWORTH_Q;
    float norm = 1.0f / (1.0f + KoQ + K2);

    c->b0 = K2 * norm;
    c->b1 = 2.0f * c->b0;
    c->b2 = c->b0;
    c->a1 = 2.0f * (K2 - 1.0f) * norm;
    c->a2 = (1.0f - KoQ + K2) * norm;
}

static void biquad_design_hpf(biquad_coeff_t *c, float fc_hz, float fs_hz)
{
    if (fc_hz <= 0.0f || fc_hz >= fs_hz * 0.5f || fs_hz <= 0.0f) {
        c->b0 = 1.0f; c->b1 = 0.0f; c->b2 = 0.0f;
        c->a1 = 0.0f; c->a2 = 0.0f;
        return;
    }

    float K    = tanf((float)M_PI * fc_hz / fs_hz);
    float K2   = K * K;
    float KoQ  = K / BUTTERWORTH_Q;
    float norm = 1.0f / (1.0f + KoQ + K2);

    c->b0 =  norm;
    c->b1 = -2.0f * norm;
    c->b2 =  norm;
    c->a1 =  2.0f * (K2 - 1.0f) * norm;
    c->a2 =  (1.0f - KoQ + K2) * norm;
}

/* ================================================================== */
/*  DC warm-up: set DF2T delay elements to the steady-state values     */
/*  that correspond to a constant DC input of dc_val.                  */
/*                                                                     */
/*  For HP: steady-state output y_ss = 0 (DC rejected).               */
/*    w2_ss = b2 * dc_val                                              */
/*    w1_ss = (b1 + b2) * dc_val                                      */
/*                                                                     */
/*  For LP: steady-state output y_ss = G * dc_val where G = DC gain.  */
/*    G     = (b0 + b1 + b2) / (1 + a1 + a2)                          */
/*    w2_ss = (b2 - a2 * G) * dc_val                                  */
/*    w1_ss = (b1 - a1 * G) * dc_val + w2_ss                          */
/* ================================================================== */

static void biquad_state_init_dc_hp(biquad_state_t *s,
                                    const biquad_coeff_t *c,
                                    float dc_val)
{
    s->w2 = c->b2 * dc_val;
    s->w1 = (c->b1 + c->b2) * dc_val;
}

static void biquad_state_init_dc_lp(biquad_state_t *s,
                                    const biquad_coeff_t *c,
                                    float dc_val)
{
    float denom = 1.0f + c->a1 + c->a2;
    float G;
    if (fabsf(denom) < 1e-12f) {
        G = 1.0f;
    } else {
        G = (c->b0 + c->b1 + c->b2) / denom;
    }
    float y_ss = G * dc_val;
    s->w2 = (c->b2 - c->a2 * y_ss / (dc_val != 0.0f ? dc_val : 1.0f)) * dc_val;
    s->w1 = (c->b1 * dc_val - c->a1 * y_ss) + s->w2;
}

/* ================================================================== */
/*  PPG preprocessing: per-channel HP + LP cascade                     */
/* ================================================================== */

static void hr_preproc_run_ppg(hr_preproc_ctx_t *ctx,
                               const ppg_sample_t *ppg_in,
                               uint16_t n)
{
    for (int ch = 0; ch < 4; ch++) {
        biquad_state_t *hp_s = &ctx->ppg_hp_state[ch];
        biquad_state_t *lp_s = &ctx->ppg_lp_state[ch];

        float first_val = (float)ppg_in[0].ch[ch];
        biquad_state_init_dc_hp(hp_s, &ctx->ppg_hp_coeff, first_val);
        lp_s->w1 = 0.0f;
        lp_s->w2 = 0.0f;

        for (uint16_t i = 0; i < n; i++) {
            float x = (float)ppg_in[i].ch[ch];
            float hp_out = biquad_df2t_process(&ctx->ppg_hp_coeff, hp_s, x);
            float lp_out = biquad_df2t_process(&ctx->ppg_lp_coeff, lp_s, hp_out);
            ctx->ppg_out.data[i][ch] = lp_out;
        }
    }
}

/* ================================================================== */
/*  ACC preprocessing: per-axis LP                                     */
/* ================================================================== */

static void hr_preproc_run_acc(hr_preproc_ctx_t *ctx,
                               const acc_sample_t *acc_in,
                               uint16_t n)
{
    for (uint16_t i = 0; i < n; i++) {
        float axes[3];
        axes[0] = (float)acc_in[i].x;
        axes[1] = (float)acc_in[i].y;
        axes[2] = (float)acc_in[i].z;

        if (i == 0) {
            for (int ax = 0; ax < 3; ax++) {
                biquad_state_init_dc_lp(&ctx->acc_lp_state[ax],
                                        &ctx->acc_lp_coeff,
                                        axes[ax]);
            }
        }

        for (int ax = 0; ax < 3; ax++) {
            ctx->acc_out.data[i][ax] =
                biquad_df2t_process(&ctx->acc_lp_coeff,
                                    &ctx->acc_lp_state[ax],
                                    axes[ax]);
        }
    }
}

/* ================================================================== */
/*  Public API                                                         */
/* ================================================================== */

void hr_preproc_init(hr_preproc_ctx_t *ctx, const hr_params_t *params)
{
    if (!ctx || !params) return;

    memset(ctx, 0, sizeof(*ctx));

    biquad_design_hpf(&ctx->ppg_hp_coeff,
                      params->preproc.ppg_bp_low_hz,
                      HR_PREPROC_SAMPLE_RATE_HZ);

    biquad_design_lpf(&ctx->ppg_lp_coeff,
                      params->preproc.ppg_bp_high_hz,
                      HR_PREPROC_SAMPLE_RATE_HZ);

    biquad_design_lpf(&ctx->acc_lp_coeff,
                      params->preproc.acc_lp_cutoff_hz,
                      HR_PREPROC_SAMPLE_RATE_HZ);

    ctx->initialized = true;
}

void hr_preproc_run(hr_preproc_ctx_t *ctx,
                    const acc_sample_t *acc_in,
                    const ppg_sample_t *ppg_in,
                    uint16_t n)
{
    if (!ctx || !ctx->initialized) return;
    if (!acc_in || !ppg_in || n == 0 || n > HR_WINDOW_SIZE) return;

    hr_preproc_run_ppg(ctx, ppg_in, n);
    hr_preproc_run_acc(ctx, acc_in, n);
}

void hr_preproc_reset(hr_preproc_ctx_t *ctx, const hr_params_t *params)
{
    hr_preproc_init(ctx, params);
}
