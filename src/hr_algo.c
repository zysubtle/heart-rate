/**
 * @file hr_algo.c
 * @brief M9 Main Pipeline Integration + Output/Debug Writing.
 *
 * Implements all public API functions declared in hr_algo_api.h.
 * Orchestrates the 12-step processing pipeline (M1-M8) per cycle.
 *
 * M9 is the sole writer of:
 *   - confidence (uint8_t [0..100])
 *   - hr_output_t (complete per-cycle output)
 *   - hr_debug_frame_t (complete per-cycle debug snapshot)
 *
 * M9 does NOT:
 *   - Compute SQI, motion, MAC, candidates, fusion, or state transitions
 *   - Allocate dynamic memory
 *   - Place large arrays on the stack
 *
 * Timestamp strategy (V1):
 *   timestamp_ms = process_count * 1000
 *   This is "processing cadence time", not sensor physical time.
 *   When real-time clock input becomes available, replace with actual
 *   wall-clock time at that point.
 */

#include "hr_algo_api.h"
#include "hr_algo_internal.h"

#include <string.h>
#include <math.h>

/* ================================================================== */
/*  Debug flags bit layout (internal to M9, O13 first allocation)      */
/*                                                                     */
/*  Bit assignments are append-only: once a bit is allocated, its      */
/*  semantic meaning must not change.  New flags may be appended to    */
/*  higher bits in future modules / iterations.                        */
/* ================================================================== */

#define HR_DBG_FLAG_DATA_NOT_READY      (1u << 0)
#define HR_DBG_FLAG_MAC_BYPASSED        (1u << 1)
#define HR_DBG_FLAG_MAC_DIVERGED        (1u << 2)
#define HR_DBG_FLAG_NO_VALID_CANDIDATE  (1u << 3)
#define HR_DBG_FLAG_HOLDOVER_ACTIVE     (1u << 4)
#define HR_DBG_FLAG_REACQUIRE_ACTIVE    (1u << 5)
#define HR_DBG_FLAG_MAIN_CH_SWITCHED    (1u << 6)
#define HR_DBG_FLAG_SIGNAL_RECOVERED    (1u << 7)

/* ================================================================== */
/*  Confidence computation constants (V1, O3 + O7)                     */
/*                                                                     */
/*  TRACK formula (weighted linear combination):                       */
/*    raw = 100 * (W_SQI*sqi + W_SEL*sel_score + W_CONS*consistency    */
/*                 + W_MOT*motion_factor)                              */
/*    confidence = clamp(round(raw), 0, 100)                           */
/*                                                                     */
/*  All inputs are normalised to [0.0, 1.0] before weighting.         */
/*  Weights sum to 1.0 so that perfect inputs yield confidence = 100.  */
/*                                                                     */
/*  HOLDOVER (O7): confidence = max(prev_confidence - DECAY, 0)        */
/*  REACQUIRE:     same as TRACK but capped at REACQUIRE_CAP.          */
/*  INIT/ACQUIRE:  confidence = 0.                                     */
/* ================================================================== */

#define CONF_W_SQI   0.30f
#define CONF_W_SEL   0.35f
#define CONF_W_CONS  0.20f
#define CONF_W_MOT   0.15f

/* BPM tolerance for candidate consistency: valid candidates within
 * this window of final hr_bpm count as "agreeing". */
#define CONF_CONSISTENCY_BPM_WINDOW  8.0f

#define CONF_HOLDOVER_DECAY          10
#define CONF_REACQUIRE_CAP           60

/* ================================================================== */
/*  Static helpers: confidence                                         */
/* ================================================================== */

/**
 * Motion-state-dependent confidence factor [0.0, 1.0].
 *
 * REST/WALK produce higher confidence (less artifact contamination).
 * RUN/IRREGULAR reduce confidence (more artifacts, noisier estimates).
 */
static float motion_confidence_factor(motion_state_t ms)
{
    switch (ms) {
    case MOTION_STATE_REST:      return 1.0f;
    case MOTION_STATE_WALK:      return 0.9f;
    case MOTION_STATE_RUN:       return 0.6f;
    case MOTION_STATE_IRREGULAR: return 0.5f;
    default:                     return 0.5f;
    }
}

/**
 * Candidate consistency: proportion of valid candidates whose BPM
 * falls within CONF_CONSISTENCY_BPM_WINDOW of ref_bpm.
 *
 * Rationale: when multiple independent methods agree on a similar
 * BPM, the estimate is more trustworthy.
 *
 * Returns [0.0, 1.0].  Returns 0.0 when ref_bpm <= 0 or no valid
 * candidate exists.
 */
static float compute_candidate_consistency(
    const hr_candidate_result_t *cand, float ref_bpm)
{
    if (!cand || ref_bpm <= 0.0f) return 0.0f;

    const hr_candidate_t *arr[7] = {
        &cand->cand_peak_raw, &cand->cand_peak_mac,
        &cand->cand_fft_raw,  &cand->cand_fft_mac,
        &cand->cand_acf_raw,  &cand->cand_acf_mac,
        &cand->cand_pred
    };

    int valid_count = 0;
    int agree_count = 0;
    for (int i = 0; i < 7; i++) {
        if (arr[i]->valid) {
            valid_count++;
            if (fabsf(arr[i]->bpm - ref_bpm) <=
                CONF_CONSISTENCY_BPM_WINDOW) {
                agree_count++;
            }
        }
    }

    if (valid_count == 0) return 0.0f;
    return (float)agree_count / (float)valid_count;
}

/**
 * TRACK-mode confidence: weighted linear combination.
 * All inputs should be in [0.0, 1.0]; output is uint8_t [0..100].
 */
static uint8_t compute_confidence_track(float sqi_main,
                                        float selected_score,
                                        float consistency,
                                        motion_state_t motion)
{
    float raw = 100.0f * (
        CONF_W_SQI  * sqi_main +
        CONF_W_SEL  * selected_score +
        CONF_W_CONS * consistency +
        CONF_W_MOT  * motion_confidence_factor(motion)
    );

    if (raw < 0.0f)     raw = 0.0f;
    if (raw > 100.0f)   raw = 100.0f;
    if (!isfinite(raw))  raw = 0.0f;

    return (uint8_t)(raw + 0.5f);
}

/**
 * Compute per-cycle confidence.
 *
 * Called at step 10, after M8 (step 8) has determined hr_state.
 * Reads hr_state but does NOT write it — causal constraint satisfied.
 */
static uint8_t compute_confidence(const hr_algo_ctx_t *ctx,
                                  bool data_ready)
{
    hr_state_t state = ctx->sm.result.state;

    switch (state) {

    case HR_STATE_INIT:
    case HR_STATE_ACQUIRE:
        return 0;

    case HR_STATE_TRACK: {
        float ref_bpm    = ctx->fusion.result.valid
                         ? ctx->fusion.result.hr_bpm : 0.0f;
        float sel_score  = ctx->fusion.result.valid
                         ? ctx->fusion.result.selected_score : 0.0f;
        float sqi        = data_ready
                         ? ctx->sqi.result.sqi_main : 0.0f;
        float consistency = data_ready
                          ? compute_candidate_consistency(
                                &ctx->candidate.result, ref_bpm)
                          : 0.0f;
        motion_state_t ms = data_ready
                          ? ctx->motion.result.state
                          : MOTION_STATE_REST;

        return compute_confidence_track(sqi, sel_score, consistency, ms);
    }

    case HR_STATE_HOLDOVER: {
        int prev = (int)ctx->main.prev_confidence;
        int val  = prev - CONF_HOLDOVER_DECAY;
        return (uint8_t)(val > 0 ? val : 0);
    }

    case HR_STATE_REACQUIRE: {
        float ref_bpm    = ctx->fusion.result.valid
                         ? ctx->fusion.result.hr_bpm : 0.0f;
        float sel_score  = ctx->fusion.result.valid
                         ? ctx->fusion.result.selected_score : 0.0f;
        float sqi        = data_ready
                         ? ctx->sqi.result.sqi_main : 0.0f;
        float consistency = data_ready
                          ? compute_candidate_consistency(
                                &ctx->candidate.result, ref_bpm)
                          : 0.0f;
        motion_state_t ms = data_ready
                          ? ctx->motion.result.state
                          : MOTION_STATE_REST;

        uint8_t conf = compute_confidence_track(
                           sqi, sel_score, consistency, ms);
        if (conf > CONF_REACQUIRE_CAP)
            conf = CONF_REACQUIRE_CAP;
        return conf;
    }

    default:
        return 0;
    }
}

/* ================================================================== */
/*  Static helpers: output / debug / flags                             */
/* ================================================================== */

/**
 * Fill hr_output_t (step 11).
 *
 * hr_bpm rules:
 *   INIT / ACQUIRE:  0.0f (no meaningful output yet)
 *   TRACK / REACQUIRE / HOLDOVER:
 *     - If M7 produced a valid result this cycle → fusion.result.hr_bpm
 *     - Else if M7 has historical output → fusion.prev_hr_bpm
 *     - Else → 0.0f
 *
 * full_pipeline: true when M2-M7 all executed this cycle (data_ready).
 * When false, fusion.result may be stale from a prior cycle;
 * only prev_hr_bpm is a trustworthy fallback.
 */
static void fill_output(hr_algo_ctx_t *ctx,
                        uint8_t confidence,
                        bool full_pipeline)
{
    hr_output_t *out   = &ctx->output;
    hr_state_t   state = ctx->sm.result.state;

    out->hr_state     = state;
    out->motion_state = ctx->motion.result.state;
    out->main_ch      = ctx->sqi.result.main_ch;
    out->backup_ch    = ctx->sqi.result.backup_ch;
    out->sqi_main     = ctx->sqi.result.sqi_main;
    out->confidence   = confidence;

    switch (state) {
    case HR_STATE_INIT:
    case HR_STATE_ACQUIRE:
        out->hr_bpm = 0.0f;
        break;

    case HR_STATE_TRACK:
    case HR_STATE_REACQUIRE:
    case HR_STATE_HOLDOVER:
        if (full_pipeline && ctx->fusion.result.valid) {
            out->hr_bpm = ctx->fusion.result.hr_bpm;
        } else if (ctx->fusion.has_prev_hr) {
            out->hr_bpm = ctx->fusion.prev_hr_bpm;
        } else {
            out->hr_bpm = 0.0f;
        }
        break;

    default:
        out->hr_bpm = 0.0f;
        break;
    }
}

/**
 * Build debug flags from current module states.
 *
 * Each flag is derived from actual module results, never fabricated.
 * data_ready gates flags that depend on M2-M7 results.
 */
static uint32_t build_flags(const hr_algo_ctx_t *ctx, bool data_ready)
{
    uint32_t flags = 0;

    if (!data_ready)
        flags |= HR_DBG_FLAG_DATA_NOT_READY;

    if (data_ready && ctx->mac.result.bypassed)
        flags |= HR_DBG_FLAG_MAC_BYPASSED;

    if (data_ready && ctx->mac.result.diverged)
        flags |= HR_DBG_FLAG_MAC_DIVERGED;

    if (data_ready && !ctx->candidate.result.any_valid)
        flags |= HR_DBG_FLAG_NO_VALID_CANDIDATE;

    if (ctx->sm.result.state == HR_STATE_HOLDOVER)
        flags |= HR_DBG_FLAG_HOLDOVER_ACTIVE;

    if (ctx->sm.result.state == HR_STATE_REACQUIRE)
        flags |= HR_DBG_FLAG_REACQUIRE_ACTIVE;

    /* Detect main_ch switch vs previous cycle.
     * Skipped on first cycle (process_count == 0) since there is
     * no meaningful "previous" channel. */
    if (data_ready &&
        ctx->main.process_count > 0 &&
        ctx->sqi.result.main_ch != ctx->main.prev_main_ch)
        flags |= HR_DBG_FLAG_MAIN_CH_SWITCHED;

    if (ctx->sm.result.signal_recovered)
        flags |= HR_DBG_FLAG_SIGNAL_RECOVERED;

    return flags;
}

/** Construct an invalid candidate stub for debug when pipeline skipped. */
static hr_candidate_t invalid_cand(hr_source_t src)
{
    hr_candidate_t c;
    c.valid  = false;
    c.bpm    = 0.0f;
    c.score  = 0.0f;
    c.source = src;
    return c;
}

/**
 * Fill hr_debug_frame_t (step 12).
 *
 * Every cycle produces a complete debug frame, including degraded
 * scenarios (INIT, data-not-ready, HOLDOVER).  No cycle may be
 * skipped.
 */
static void fill_debug_frame(hr_algo_ctx_t *ctx, bool data_ready)
{
    hr_debug_frame_t  *d   = &ctx->debug;
    const hr_output_t *out = &ctx->output;

    /*
     * V1 timestamp = processing cadence time, not sensor physical time.
     * process_count is incremented AFTER this function returns;
     * first cycle → timestamp = 1000 ms.
     */
    d->timestamp_ms = (ctx->main.process_count + 1) * 1000;

    d->hr_state     = out->hr_state;
    d->motion_state = out->motion_state;
    d->main_ch      = out->main_ch;
    d->backup_ch    = out->backup_ch;

    if (data_ready) {
        for (int i = 0; i < 4; i++)
            d->sqi[i] = ctx->sqi.result.ch[i].total;
    } else {
        for (int i = 0; i < 4; i++)
            d->sqi[i] = 0.0f;
    }

    if (data_ready) {
        d->cand_peak_raw = ctx->candidate.result.cand_peak_raw;
        d->cand_peak_mac = ctx->candidate.result.cand_peak_mac;
        d->cand_fft_raw  = ctx->candidate.result.cand_fft_raw;
        d->cand_fft_mac  = ctx->candidate.result.cand_fft_mac;
        d->cand_acf_raw  = ctx->candidate.result.cand_acf_raw;
        d->cand_acf_mac  = ctx->candidate.result.cand_acf_mac;
        d->cand_pred     = ctx->candidate.result.cand_pred;
    } else {
        d->cand_peak_raw = invalid_cand(HR_SOURCE_PEAK_RAW);
        d->cand_peak_mac = invalid_cand(HR_SOURCE_PEAK_MAC);
        d->cand_fft_raw  = invalid_cand(HR_SOURCE_FFT_RAW);
        d->cand_fft_mac  = invalid_cand(HR_SOURCE_FFT_MAC);
        d->cand_acf_raw  = invalid_cand(HR_SOURCE_ACF_RAW);
        d->cand_acf_mac  = invalid_cand(HR_SOURCE_ACF_MAC);
        d->cand_pred     = invalid_cand(HR_SOURCE_PRED);
    }

    d->hr_out      = out->hr_bpm;
    d->confidence  = out->confidence;
    d->flags       = build_flags(ctx, data_ready);
}

/* ================================================================== */
/*  Public API: hr_algo_default_params                                 */
/* ================================================================== */

hr_params_t hr_algo_default_params(void)
{
    hr_params_t p;
    memset(&p, 0, sizeof(p));

    /* M2 Preprocessing — defaults from hr_algo_params.h comments */
    p.preproc.ppg_bp_low_hz    = 0.5f;
    p.preproc.ppg_bp_high_hz   = 4.0f;
    p.preproc.acc_lp_cutoff_hz = 10.0f;

    /* M3 SQI — defaults from hr_algo_params.h comments */
    p.sqi.sqi_good_threshold  = 0.6f;
    p.sqi.sqi_poor_threshold  = 0.3f;
    p.sqi.switch_margin       = 0.05f;
    p.sqi.switch_hold_count   = 3;

    /*
     * M4 Motion — energy thresholds for dynamic_energy classification.
     * dynamic_energy = mean(||acc_dyn||^2) in sensor units^2.
     * V1 conservative values aligned with M4 test suite; adjust for
     * specific sensor gain / placement if needed.
     */
    p.motion.rest_energy_threshold = 0.5f;
    p.motion.walk_energy_threshold = 2.0f;
    p.motion.run_energy_threshold  = 10.0f;

    /* M5 MAC — defaults from hr_algo_params.h comments */
    p.mac.nlms_step_size = 0.01f;
    p.mac.filter_order   = 8;

    /* M6 Candidate */
    p.candidate.hr_min_bpm = 40.0f;
    p.candidate.hr_max_bpm = 220.0f;
    /*
     * peak_min_prominence: minimum relative prominence for peak
     * detection.  0.1 is conservative; filters trivial noise peaks
     * while admitting most physiological PPG peaks.  No explicit
     * default in hr_algo_params.h — value chosen here by M9.
     */
    p.candidate.peak_min_prominence = 0.1f;

    /* M7 Fusion — smooth_alpha default from hr_algo_params.h comment */
    p.fusion.smooth_alpha = 0.3f;
    /*
     * bpm_jump_threshold: max BPM change per cycle.  Heart rate
     * rarely exceeds 20 BPM/s even in vigorous exercise transitions.
     * No explicit default in hr_algo_params.h — value chosen here
     * by M9 based on physiological limits.
     */
    p.fusion.bpm_jump_threshold = 20.0f;

    /* M8 State machine — defaults from hr_algo_params.h comments */
    p.statemachine.acquire_stable_count   = 5;
    p.statemachine.holdover_max_count     = 10;
    p.statemachine.reacquire_stable_count = 3;

    return p;
}

/* ================================================================== */
/*  Public API: hr_algo_init                                           */
/* ================================================================== */

void hr_algo_init(hr_algo_ctx_t *ctx, const hr_params_t *params)
{
    if (!ctx) return;

    hr_params_t effective;
    if (params) {
        effective = *params;
    } else {
        effective = hr_algo_default_params();
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->params = effective;

    hr_sampling_init(&ctx->sampling);
    hr_preproc_init(&ctx->preproc, &ctx->params);
    hr_sqi_init(&ctx->sqi);
    hr_motion_init(&ctx->motion);
    hr_mac_init(&ctx->mac, &ctx->params);
    hr_candidate_init(&ctx->candidate);
    hr_fusion_init(&ctx->fusion);
    hr_sm_init(&ctx->sm);

    /* main staging: already zeroed by memset.
     * output / debug: zeroed = cold-start (hr_bpm=0, confidence=0). */
}

/* ================================================================== */
/*  Public API: hr_algo_feed_acc / hr_algo_feed_ppg                    */
/*                                                                     */
/*  Pure delegation to M1.  No business logic at feed time.            */
/* ================================================================== */

void hr_algo_feed_acc(hr_algo_ctx_t *ctx,
                      int16_t ax, int16_t ay, int16_t az)
{
    if (!ctx) return;
    hr_sampling_push_acc(&ctx->sampling, ax, ay, az);
}

void hr_algo_feed_ppg(hr_algo_ctx_t *ctx,
                      int32_t ch0, int32_t ch1,
                      int32_t ch2, int32_t ch3)
{
    if (!ctx) return;
    hr_sampling_push_ppg(&ctx->sampling, ch0, ch1, ch2, ch3);
}

/* ================================================================== */
/*  Public API: hr_algo_process_1s                                     */
/*                                                                     */
/*  12-step pipeline, strict order per architecture doc section 5.     */
/*                                                                     */
/*  Step   Operation              Notes                                */
/*  ----   ---------------------  ------------------------------------ */
/*    1    Data readiness check                                        */
/*    2    Export 200-pt window    skip if !data_ready                  */
/*    3    M2 Preprocess           skip if !data_ready                  */
/*    4    M3 SQI                  skip if !data_ready                  */
/*    5    M4 Motion               skip if !data_ready                  */
/*    6    M5 MAC                  skip if !data_ready                  */
/*    7    M6 Candidates           skip if !data_ready                  */
/*    8    M8 State machine        always runs (enables INIT->ACQUIRE) */
/*    9    M7 Fusion               skip if !data_ready                  */
/*   10    Compute confidence      always runs (M9 sole writer)        */
/*   11    Fill hr_output_t        always runs                         */
/*   12    Fill hr_debug_frame_t   always runs                         */
/* ================================================================== */

void hr_algo_process_1s(hr_algo_ctx_t *ctx)
{
    if (!ctx) return;

    /* -------------------------------------------------------------- */
    /*  Step 1: Data readiness check                                   */
    /* -------------------------------------------------------------- */
    bool data_ready = hr_sampling_ready_for_window(&ctx->sampling);

    if (data_ready) {

        /* ---------------------------------------------------------- */
        /*  Step 2: Export most recent 200 points to non-stack staging */
        /* ---------------------------------------------------------- */
        hr_sampling_export_recent_acc(&ctx->sampling,
                                     ctx->main.acc_raw,
                                     HR_WINDOW_SIZE);
        hr_sampling_export_recent_ppg(&ctx->sampling,
                                     ctx->main.ppg_raw,
                                     HR_WINDOW_SIZE);

        /* ---------------------------------------------------------- */
        /*  Step 3: M2 Preprocess ACC/PPG                              */
        /* ---------------------------------------------------------- */
        hr_preproc_run(&ctx->preproc,
                       ctx->main.acc_raw,
                       ctx->main.ppg_raw,
                       HR_WINDOW_SIZE);

        /* ---------------------------------------------------------- */
        /*  Step 4: M3 SQI & channel selection                         */
        /* ---------------------------------------------------------- */
        hr_sqi_run(&ctx->sqi,
                   &ctx->preproc.ppg_out,
                   HR_WINDOW_SIZE,
                   &ctx->params);

        /* ---------------------------------------------------------- */
        /*  Step 5: M4 Motion state detection                          */
        /* ---------------------------------------------------------- */
        hr_motion_run(&ctx->motion,
                      &ctx->preproc.acc_out,
                      HR_WINDOW_SIZE,
                      &ctx->params);

        /* ---------------------------------------------------------- */
        /*  Step 6: M5 MAC (motion artifact cancellation)              */
        /* ---------------------------------------------------------- */
        hr_mac_run(&ctx->mac,
                   &ctx->preproc.ppg_out,
                   &ctx->preproc.acc_out,
                   &ctx->sqi.result,
                   &ctx->motion.result,
                   HR_WINDOW_SIZE,
                   &ctx->params);

        /* ---------------------------------------------------------- */
        /*  Step 7: M6 Candidate estimation                            */
        /* ---------------------------------------------------------- */
        hr_candidate_run(&ctx->candidate,
                         &ctx->preproc.ppg_out,
                         &ctx->mac.result,
                         &ctx->mac.out,
                         &ctx->sqi.result,
                         &ctx->motion.result,
                         HR_WINDOW_SIZE,
                         &ctx->params);

        /* ---------------------------------------------------------- */
        /*  Step 8: M8 State machine                                   */
        /* ---------------------------------------------------------- */
        hr_sm_update(&ctx->sm,
                     true,
                     &ctx->candidate.result,
                     &ctx->sqi.result,
                     &ctx->motion.result,
                     &ctx->fusion,
                     &ctx->params);

        /* ---------------------------------------------------------- */
        /*  Step 9: M7 Fusion & smoothing                              */
        /* ---------------------------------------------------------- */
        hr_fusion_run(&ctx->fusion,
                      &ctx->candidate.result,
                      ctx->sm.result.state,
                      ctx->motion.result.state,
                      &ctx->params,
                      &ctx->candidate);

    } else {
        /*
         * Degraded path: ring buffer < 200 samples.
         * Steps 2-7 skipped — no raw data to process.
         * Steps 8-12 still execute for correct state progression.
         *
         * M8 must still run so that INIT -> ACQUIRE occurs on the
         * very first hr_algo_process_1s() call (architecture
         * requirement: ring-not-full maps to ACQUIRE, not INIT).
         *
         * M7 is NOT called: no candidates exist, calling fusion with
         * stale or empty candidates would produce meaningless results
         * and risk corrupting prev_hr_bpm.
         */

        /* Step 8: M8 State machine (degraded inputs) */
        hr_sm_update(&ctx->sm,
                     false,        /* data_ready = false             */
                     NULL,         /* no candidates available        */
                     NULL,         /* no SQI available               */
                     NULL,         /* no motion result available     */
                     &ctx->fusion, /* fusion history for M8 context  */
                     &ctx->params);

        /* Step 9: M7 skipped */
    }

    /* -------------------------------------------------------------- */
    /*  Step 10: Compute confidence (M9 sole writer)                   */
    /*                                                                 */
    /*  Reads hr_state from M8 (step 8 complete).                      */
    /*  hr_state transition does NOT depend on current confidence       */
    /*  (causal constraint: architecture doc section 5.1).             */
    /* -------------------------------------------------------------- */
    uint8_t confidence = compute_confidence(ctx, data_ready);

    /* -------------------------------------------------------------- */
    /*  Step 11: Fill hr_output_t (M9 sole writer)                     */
    /* -------------------------------------------------------------- */
    fill_output(ctx, confidence, data_ready);

    /* -------------------------------------------------------------- */
    /*  Step 12: Fill hr_debug_frame_t (M9 sole writer)                */
    /* -------------------------------------------------------------- */
    fill_debug_frame(ctx, data_ready);

    /* -------------------------------------------------------------- */
    /*  Post-pipeline bookkeeping                                      */
    /* -------------------------------------------------------------- */
    ctx->main.prev_confidence = confidence;
    if (data_ready) {
        ctx->main.prev_main_ch = ctx->sqi.result.main_ch;
    }
    ctx->main.process_count++;
}

/* ================================================================== */
/*  Public API: hr_algo_get_output / hr_algo_get_debug                 */
/* ================================================================== */

const hr_output_t *hr_algo_get_output(const hr_algo_ctx_t *ctx)
{
    if (!ctx) return NULL;
    return &ctx->output;
}

const hr_debug_frame_t *hr_algo_get_debug(const hr_algo_ctx_t *ctx)
{
    if (!ctx) return NULL;
    return &ctx->debug;
}

/* ================================================================== */
/*  Public API: hr_algo_ctx_sizeof                                     */
/* ================================================================== */

size_t hr_algo_ctx_sizeof(void)
{
    return sizeof(struct hr_algo_ctx);
}
