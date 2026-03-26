/**
 * @file hr_candidate.c
 * @brief M6 Candidate Estimation — implementation.
 *
 * Generates 7 heart-rate candidates per processing cycle:
 *   - 3 from raw branch (peak, FFT, ACF)
 *   - 3 from mac branch (peak, FFT, ACF) — only when mac is valid
 *   - 1 pred candidate from historical seed
 *
 * Signal processing:
 *   - Peak:  local-maxima detection with topographic prominence,
 *            IBI median → BPM, score from IBI consistency.
 *   - FFT:   Goertzel selective DFT on HR-band bins (O1: 256 zero-pad),
 *            dominant bin → BPM, score from spectral concentration.
 *   - ACF:   normalised autocorrelation in valid HR lag range,
 *            best lag → BPM, score = normalised ACF peak.
 *   - Pred:  seed injected by M7/M8; first version defaults to invalid.
 *
 * No dynamic memory.  No large arrays on the stack.
 * All scratch buffers live in hr_candidate_ctx_t.
 */

#include "hr_candidate.h"

#include <string.h>
#include <math.h>

/* ================================================================== */
/*  V1 internal design constants                                       */
/*                                                                     */
/*  Stability / safety parameters.  If future tuning requires          */
/*  per-device adjustment, promote to hr_params_t.candidate.           */
/* ================================================================== */

#define HR_CAND_PI  3.14159265358979323846f

#define HR_CAND_FS  HR_PREPROC_SAMPLE_RATE_HZ   /* 25.0 Hz */

/* Max peaks tracked during peak detection.  200 / 6 ≈ 33 peaks
 * physiologically; 50 gives headroom.  Stack: 50 * 2 = 100 bytes. */
#define HR_CAND_MAX_PEAKS  50

/* ACF / DFT energy floor to avoid division by near-zero. */
#define HR_CAND_ENERGY_FLOOR  1e-10f

/* IBI coefficient-of-variation ceiling for peak score.
 * CV >= this → consistency factor = 0. */
#define HR_CAND_IBI_CV_CEILING  0.4f

/* SQI score modulation: score *= (BASE + SCALE * sqi_main).
 * At sqi_main=1.0 → factor=1.0.  At sqi_main=0.0 → factor=0.7. */
#define HR_CAND_SQI_MOD_BASE   0.7f
#define HR_CAND_SQI_MOD_SCALE  0.3f

/* Peak-raw motion penalty in RUN / IRREGULAR states.
 * Motion artifacts corrupt time-domain peaks more than frequency
 * methods; a mild penalty lets M7 prefer FFT/ACF/MAC candidates. */
#define HR_CAND_PEAK_MOTION_PENALTY  0.85f

/* Pred candidate score: conservative constant reflecting that pred
 * is a fallback, not a primary estimator.  M7 may weigh it higher
 * when in TRACK state with high historical consistency. */
#define HR_CAND_PRED_SCORE  0.3f

/* ================================================================== */
/*  Helper: invalid candidate                                          */
/* ================================================================== */

static hr_candidate_t make_invalid(hr_source_t source)
{
    hr_candidate_t c;
    c.valid  = false;
    c.bpm    = 0.0f;
    c.score  = 0.0f;
    c.source = source;
    return c;
}

/* ================================================================== */
/*  Helper: valid candidate with range and sanity check                */
/* ================================================================== */

static hr_candidate_t make_candidate(float bpm, float score,
                                     hr_source_t source,
                                     float hr_min, float hr_max)
{
    if (!isfinite(bpm) || !isfinite(score) ||
        bpm < hr_min   || bpm > hr_max) {
        return make_invalid(source);
    }
    hr_candidate_t c;
    c.valid  = true;
    c.bpm    = bpm;
    c.score  = (score < 0.0f) ? 0.0f : (score > 1.0f) ? 1.0f : score;
    c.source = source;
    return c;
}

/* ================================================================== */
/*  Helper: set all 7 candidates to invalid                            */
/* ================================================================== */

static void set_all_invalid(hr_candidate_result_t *r)
{
    r->cand_peak_raw = make_invalid(HR_SOURCE_PEAK_RAW);
    r->cand_peak_mac = make_invalid(HR_SOURCE_PEAK_MAC);
    r->cand_fft_raw  = make_invalid(HR_SOURCE_FFT_RAW);
    r->cand_fft_mac  = make_invalid(HR_SOURCE_FFT_MAC);
    r->cand_acf_raw  = make_invalid(HR_SOURCE_ACF_RAW);
    r->cand_acf_mac  = make_invalid(HR_SOURCE_ACF_MAC);
    r->cand_pred     = make_invalid(HR_SOURCE_PRED);
    r->any_valid     = false;
}

/* ================================================================== */
/*  1D signal extraction with mean removal                             */
/*                                                                     */
/*  Populates ctx->work_buf with the mean-subtracted 1D signal.        */
/*  Mean removal is essential for proper ACF normalisation and to       */
/*  avoid DC leakage in the Goertzel DFT.                              */
/* ================================================================== */

static void extract_raw_branch(hr_candidate_ctx_t *ctx,
                               const ppg_preproc_out_t *ppg_in,
                               uint8_t main_ch,
                               uint16_t n)
{
    float sum = 0.0f;
    for (uint16_t i = 0; i < n; i++) {
        ctx->work_buf[i] = ppg_in->data[i][main_ch];
        sum += ctx->work_buf[i];
    }
    float mean = sum / (float)n;
    for (uint16_t i = 0; i < n; i++) {
        ctx->work_buf[i] -= mean;
    }
}

static void extract_mac_branch(hr_candidate_ctx_t *ctx,
                               const ppg_mac_out_t *mac_out,
                               uint16_t n)
{
    float sum = 0.0f;
    for (uint16_t i = 0; i < n; i++) {
        ctx->work_buf[i] = mac_out->data[i];
        sum += ctx->work_buf[i];
    }
    float mean = sum / (float)n;
    for (uint16_t i = 0; i < n; i++) {
        ctx->work_buf[i] -= mean;
    }
}

/* ================================================================== */
/*  Peak candidate estimation                                          */
/*                                                                     */
/*  Algorithm:                                                         */
/*    1. Find all local maxima in the mean-subtracted 1D signal        */
/*    2. Compute topographic prominence for each peak                  */
/*    3. Filter by peak_min_prominence parameter                       */
/*    4. Compute inter-beat intervals (IBI)                            */
/*    5. Keep IBIs within valid HR range                               */
/*    6. BPM = 60 * fs / median(valid IBIs)                            */
/*    7. Score from IBI consistency (1 - CV) and IBI count             */
/*                                                                     */
/*  Stack usage: ~400 bytes (raw_peaks + peaks + ibis arrays).         */
/* ================================================================== */

static hr_candidate_t estimate_peak(const float *sig, uint16_t n,
                                    float hr_min, float hr_max,
                                    float min_prom,
                                    hr_source_t source)
{
    if (n < 3) return make_invalid(source);

    /* Valid IBI range derived from HR bounds */
    uint16_t min_ibi = (uint16_t)floorf(HR_CAND_FS * 60.0f / hr_max);
    uint16_t max_ibi = (uint16_t)ceilf(HR_CAND_FS * 60.0f / hr_min);
    if (min_ibi < 2) min_ibi = 2;
    if (max_ibi >= n) max_ibi = n - 1;

    /* --- Pass 1: find all local maxima ----------------------------- */
    uint16_t raw_peaks[HR_CAND_MAX_PEAKS];
    int raw_count = 0;

    for (uint16_t i = 1; i < n - 1 && raw_count < HR_CAND_MAX_PEAKS; i++) {
        if (sig[i] > sig[i - 1] && sig[i] > sig[i + 1]) {
            raw_peaks[raw_count++] = i;
        }
    }

    if (raw_count < 2) return make_invalid(source);

    /* --- Pass 2: prominence filter --------------------------------- */
    uint16_t peaks[HR_CAND_MAX_PEAKS];
    int peak_count = 0;

    for (int p = 0; p < raw_count; p++) {
        uint16_t idx = raw_peaks[p];

        /* Left valley: min between previous peak (or edge) and here */
        uint16_t left_bound = (p > 0) ? raw_peaks[p - 1] : 0;
        float left_min = sig[idx];
        for (uint16_t j = left_bound; j < idx; j++) {
            if (sig[j] < left_min) left_min = sig[j];
        }

        /* Right valley: min between here and next peak (or edge) */
        uint16_t right_bound = (p < raw_count - 1)
                               ? raw_peaks[p + 1] : (n - 1);
        float right_min = sig[idx];
        for (uint16_t j = idx + 1; j <= right_bound; j++) {
            if (sig[j] < right_min) right_min = sig[j];
        }

        /* Topographic prominence: height above the higher valley */
        float ref  = (left_min > right_min) ? left_min : right_min;
        float prom = sig[idx] - ref;

        if (prom >= min_prom && peak_count < HR_CAND_MAX_PEAKS) {
            peaks[peak_count++] = idx;
        }
    }

    if (peak_count < 2) return make_invalid(source);

    /* --- Compute IBIs in valid HR range ---------------------------- */
    float ibis[HR_CAND_MAX_PEAKS];
    int ibi_count = 0;

    for (int k = 1; k < peak_count; k++) {
        uint16_t gap = peaks[k] - peaks[k - 1];
        if (gap >= min_ibi && gap <= max_ibi) {
            if (ibi_count < HR_CAND_MAX_PEAKS)
                ibis[ibi_count++] = (float)gap;
        }
    }

    if (ibi_count < 1) return make_invalid(source);

    /* --- Median IBI via insertion sort ------------------------------ */
    for (int i = 1; i < ibi_count; i++) {
        float key = ibis[i];
        int j = i - 1;
        while (j >= 0 && ibis[j] > key) {
            ibis[j + 1] = ibis[j];
            j--;
        }
        ibis[j + 1] = key;
    }
    float median_ibi = ibis[ibi_count / 2];

    /* --- BPM from median IBI --------------------------------------- */
    if (median_ibi < 1.0f) return make_invalid(source);
    float bpm = HR_CAND_FS * 60.0f / median_ibi;

    /* --- Score: IBI consistency + count factor --------------------- */
    float ibi_mean = 0.0f;
    for (int k = 0; k < ibi_count; k++) ibi_mean += ibis[k];
    ibi_mean /= (float)ibi_count;

    float ibi_var = 0.0f;
    for (int k = 0; k < ibi_count; k++) {
        float d = ibis[k] - ibi_mean;
        ibi_var += d * d;
    }
    ibi_var /= (float)ibi_count;
    float ibi_std = (ibi_var > 0.0f) ? sqrtf(ibi_var) : 0.0f;
    float cv = (ibi_mean > 1e-6f) ? (ibi_std / ibi_mean) : 1.0f;

    float score_cv = 1.0f - cv / HR_CAND_IBI_CV_CEILING;
    if (score_cv < 0.0f) score_cv = 0.0f;

    float score_count = 1.0f;
    if (ibi_count < 2) score_count = 0.4f;
    else if (ibi_count < 4) score_count = 0.7f;

    float score = score_cv * score_count;

    return make_candidate(bpm, score, source, hr_min, hr_max);
}

/* ================================================================== */
/*  FFT candidate estimation (selective DFT via Goertzel)              */
/*                                                                     */
/*  Goertzel computes |X[k]|^2 for a single DFT bin k without         */
/*  requiring the full FFT butterfly.  Only HR-band bins are           */
/*  evaluated, giving O(B * N) where B ≈ 31 bins, N ≈ 256.            */
/*                                                                     */
/*  Zero-padding from 200 to 256 is implicit: the Goertzel recursion   */
/*  processes 200 actual data samples, then continues with zero input  */
/*  for the remaining 56 points.  No 256-element buffer needed.        */
/* ================================================================== */

/**
 * Goertzel: compute |X[k]|^2 for one frequency bin.
 *
 * @param x         Input signal (mean-subtracted)
 * @param data_len  Actual data length (200)
 * @param k         DFT bin index
 * @param nfft      Total DFT length (256, includes zero-padding)
 * @return          Squared magnitude of X[k]
 */
static float goertzel_power(const float *x, uint16_t data_len,
                            uint16_t k, uint16_t nfft)
{
    float omega = 2.0f * HR_CAND_PI * (float)k / (float)nfft;
    float coeff = 2.0f * cosf(omega);
    float s1 = 0.0f, s2 = 0.0f;

    for (uint16_t i = 0; i < data_len; i++) {
        float s0 = x[i] + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }

    /* Implicit zero-padding: x[n] = 0 for n >= data_len */
    for (uint16_t i = data_len; i < nfft; i++) {
        float s0 = coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }

    return s1 * s1 + s2 * s2 - coeff * s1 * s2;
}

static hr_candidate_t estimate_fft(const float *sig, uint16_t n,
                                   float hr_min, float hr_max,
                                   hr_source_t source)
{
    const uint16_t nfft = HR_CAND_FFT_LEN;

    /* Bin range for HR frequency band */
    uint16_t bin_min = (uint16_t)ceilf(
        hr_min / 60.0f * (float)nfft / HR_CAND_FS);
    uint16_t bin_max = (uint16_t)floorf(
        hr_max / 60.0f * (float)nfft / HR_CAND_FS);

    if (bin_min < 1) bin_min = 1;
    if (bin_max >= nfft / 2) bin_max = nfft / 2 - 1;
    if (bin_min > bin_max) return make_invalid(source);

    /* Evaluate power at each HR-band bin */
    float dom_power    = 0.0f;
    uint16_t dom_bin   = bin_min;
    float total_power  = 0.0f;

    for (uint16_t k = bin_min; k <= bin_max; k++) {
        float p = goertzel_power(sig, n, k, nfft);
        total_power += p;
        if (p > dom_power) {
            dom_power = p;
            dom_bin   = k;
        }
    }

    if (!isfinite(total_power) || total_power < HR_CAND_ENERGY_FLOOR ||
        dom_power < HR_CAND_ENERGY_FLOOR) {
        return make_invalid(source);
    }

    /* Dominant bin → BPM */
    float freq_hz = (float)dom_bin * HR_CAND_FS / (float)nfft;
    float bpm     = freq_hz * 60.0f;

    /* Score: spectral concentration (dominant / total HR-band energy).
     * A clean single-tone signal concentrates most energy in 1-2 bins,
     * giving ratio ~0.3-0.8.  Noisy signals spread to ~0.03. */
    float score = dom_power / total_power;

    return make_candidate(bpm, score, source, hr_min, hr_max);
}

/* ================================================================== */
/*  ACF candidate estimation                                           */
/*                                                                     */
/*  Normalised autocorrelation in the lag range corresponding to       */
/*  [hr_min_bpm, hr_max_bpm].  Best lag peak → BPM.                   */
/*  Score = normalised ACF peak value (already in [0, 1]).             */
/* ================================================================== */

static hr_candidate_t estimate_acf(const float *sig, uint16_t n,
                                   float hr_min, float hr_max,
                                   hr_source_t source)
{
    /* Lag range from HR bounds.
     * Higher BPM → shorter period → smaller lag. */
    uint16_t lag_min = (uint16_t)floorf(HR_CAND_FS * 60.0f / hr_max);
    uint16_t lag_max = (uint16_t)ceilf(HR_CAND_FS * 60.0f / hr_min);

    if (lag_min < 2) lag_min = 2;
    if (lag_max >= n) lag_max = n - 1;
    if (lag_min > lag_max) return make_invalid(source);

    /* ACF at lag 0 (signal energy, already mean-subtracted) */
    float acf0 = 0.0f;
    for (uint16_t i = 0; i < n; i++) {
        acf0 += sig[i] * sig[i];
    }
    if (acf0 < HR_CAND_ENERGY_FLOOR) {
        return make_invalid(source);
    }

    /* Search for max normalised ACF peak in HR lag range */
    float best_nacf   = -1.0f;
    uint16_t best_lag = lag_min;

    for (uint16_t lag = lag_min; lag <= lag_max; lag++) {
        float acf_val = 0.0f;
        uint16_t limit = n - lag;
        for (uint16_t i = 0; i < limit; i++) {
            acf_val += sig[i] * sig[i + lag];
        }
        float nacf = acf_val / acf0;
        if (nacf > best_nacf) {
            best_nacf = nacf;
            best_lag  = lag;
        }
    }

    if (best_nacf <= 0.0f || best_lag < 1) {
        return make_invalid(source);
    }

    /* Lag → BPM */
    float bpm = HR_CAND_FS * 60.0f / (float)best_lag;

    /* Score = normalised ACF peak, clamped to [0, 1] */
    float score = best_nacf;
    if (score > 1.0f) score = 1.0f;

    return make_candidate(bpm, score, source, hr_min, hr_max);
}

/* ================================================================== */
/*  Pred candidate estimation                                          */
/*                                                                     */
/*  Pred seed is injected by M7/M8.  If no valid seed exists,          */
/*  cand_pred is invalid.  Never hardcodes a BPM constant.             */
/* ================================================================== */

static hr_candidate_t estimate_pred(const hr_candidate_ctx_t *ctx,
                                    float hr_min, float hr_max)
{
    if (!ctx->pred_seed_valid || ctx->pred_seed_bpm <= 0.0f) {
        return make_invalid(HR_SOURCE_PRED);
    }

    return make_candidate(ctx->pred_seed_bpm, HR_CAND_PRED_SCORE,
                          HR_SOURCE_PRED, hr_min, hr_max);
}

/* ================================================================== */
/*  Score modulation by SQI and motion state                           */
/*                                                                     */
/*  Lightweight adjustment — keeps M6 from becoming a complex rule     */
/*  engine.  M7 is the proper owner of cross-method arbitration.       */
/* ================================================================== */

static void modulate_score(hr_candidate_t *c,
                           float sqi_main,
                           motion_state_t ms,
                           bool is_peak_raw)
{
    if (!c->valid) return;

    float factor = HR_CAND_SQI_MOD_BASE + HR_CAND_SQI_MOD_SCALE * sqi_main;

    if (is_peak_raw &&
        (ms == MOTION_STATE_RUN || ms == MOTION_STATE_IRREGULAR)) {
        factor *= HR_CAND_PEAK_MOTION_PENALTY;
    }

    c->score *= factor;
    if (c->score > 1.0f) c->score = 1.0f;
    if (c->score < 0.0f) c->score = 0.0f;
}

/* ================================================================== */
/*  Public (internal) API                                              */
/* ================================================================== */

void hr_candidate_init(hr_candidate_ctx_t *ctx)
{
    if (!ctx) return;
    memset(ctx, 0, sizeof(*ctx));
    ctx->initialized = true;
}

void hr_candidate_run(hr_candidate_ctx_t *ctx,
                      const ppg_preproc_out_t *ppg_in,
                      const mac_result_t *mac_result,
                      const ppg_mac_out_t *mac_out,
                      const sqi_result_t *sqi_in,
                      const motion_result_t *motion_in,
                      uint16_t n,
                      const hr_params_t *params)
{
    if (!ctx || !ctx->initialized) return;

    if (!ppg_in || !mac_result || !mac_out ||
        !sqi_in || !motion_in  || !params  ||
        n == 0   || n > HR_WINDOW_SIZE) {
        set_all_invalid(&ctx->result);
        return;
    }

    const float hr_min   = params->candidate.hr_min_bpm;
    const float hr_max   = params->candidate.hr_max_bpm;
    const float min_prom = params->candidate.peak_min_prominence;
    uint8_t main_ch      = sqi_in->main_ch;
    const float sqi_main = sqi_in->sqi_main;
    const motion_state_t mstate = motion_in->state;

    if (main_ch > 3) main_ch = 0;

    /* ================================================================ */
    /*  RAW branch — always attempted                                   */
    /* ================================================================ */
    extract_raw_branch(ctx, ppg_in, main_ch, n);

    ctx->result.cand_peak_raw = estimate_peak(ctx->work_buf, n,
                                              hr_min, hr_max, min_prom,
                                              HR_SOURCE_PEAK_RAW);
    ctx->result.cand_fft_raw  = estimate_fft(ctx->work_buf, n,
                                             hr_min, hr_max,
                                             HR_SOURCE_FFT_RAW);
    ctx->result.cand_acf_raw  = estimate_acf(ctx->work_buf, n,
                                             hr_min, hr_max,
                                             HR_SOURCE_ACF_RAW);

    modulate_score(&ctx->result.cand_peak_raw, sqi_main, mstate, true);
    modulate_score(&ctx->result.cand_fft_raw,  sqi_main, mstate, false);
    modulate_score(&ctx->result.cand_acf_raw,  sqi_main, mstate, false);

    /* ================================================================ */
    /*  MAC branch — only when mac_result->valid == true                */
    /*                                                                  */
    /*  Even if mac_out->data contains a raw copy (bypass/diverged),    */
    /*  it MUST NOT be treated as valid MAC output.                     */
    /* ================================================================ */
    if (mac_result->valid) {
        extract_mac_branch(ctx, mac_out, n);

        ctx->result.cand_peak_mac = estimate_peak(ctx->work_buf, n,
                                                   hr_min, hr_max,
                                                   min_prom,
                                                   HR_SOURCE_PEAK_MAC);
        ctx->result.cand_fft_mac  = estimate_fft(ctx->work_buf, n,
                                                  hr_min, hr_max,
                                                  HR_SOURCE_FFT_MAC);
        ctx->result.cand_acf_mac  = estimate_acf(ctx->work_buf, n,
                                                  hr_min, hr_max,
                                                  HR_SOURCE_ACF_MAC);

        modulate_score(&ctx->result.cand_peak_mac, sqi_main, mstate, false);
        modulate_score(&ctx->result.cand_fft_mac,  sqi_main, mstate, false);
        modulate_score(&ctx->result.cand_acf_mac,  sqi_main, mstate, false);
    } else {
        ctx->result.cand_peak_mac = make_invalid(HR_SOURCE_PEAK_MAC);
        ctx->result.cand_fft_mac  = make_invalid(HR_SOURCE_FFT_MAC);
        ctx->result.cand_acf_mac  = make_invalid(HR_SOURCE_ACF_MAC);
    }

    /* ================================================================ */
    /*  Pred candidate                                                  */
    /* ================================================================ */
    ctx->result.cand_pred = estimate_pred(ctx, hr_min, hr_max);

    /* ================================================================ */
    /*  Aggregate validity flag                                         */
    /* ================================================================ */
    ctx->result.any_valid =
        ctx->result.cand_peak_raw.valid ||
        ctx->result.cand_peak_mac.valid ||
        ctx->result.cand_fft_raw.valid  ||
        ctx->result.cand_fft_mac.valid  ||
        ctx->result.cand_acf_raw.valid  ||
        ctx->result.cand_acf_mac.valid  ||
        ctx->result.cand_pred.valid;
}

void hr_candidate_reset(hr_candidate_ctx_t *ctx)
{
    hr_candidate_init(ctx);
}

void hr_candidate_set_pred_seed(hr_candidate_ctx_t *ctx, float bpm)
{
    if (!ctx) return;
    ctx->pred_seed_valid = true;
    ctx->pred_seed_bpm   = bpm;
}

void hr_candidate_clear_pred_seed(hr_candidate_ctx_t *ctx)
{
    if (!ctx) return;
    ctx->pred_seed_valid = false;
    ctx->pred_seed_bpm   = 0.0f;
}
