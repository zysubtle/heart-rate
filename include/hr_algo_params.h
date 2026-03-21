/**
 * @file hr_algo_params.h
 * @brief Unified parameter entry for heart rate algorithm.
 *
 * [Strong Freeze] The principle of a single unified parameter structure
 *                  is frozen. All tunable constants must live here.
 * [Medium Freeze] Individual sub-structure fields may be added or removed
 *                  during module implementation, but the grouped layout
 *                  (one sub-struct per module) must be preserved.
 */

#ifndef HR_ALGO_PARAMS_H
#define HR_ALGO_PARAMS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {

    /* --- Preprocessing (M2) --- */
    struct {
        float ppg_bp_low_hz;        /* PPG bandpass lower cutoff  (default 0.5)  */
        float ppg_bp_high_hz;       /* PPG bandpass upper cutoff  (default 4.0)  */
        float acc_lp_cutoff_hz;     /* ACC lowpass cutoff         (default 10.0) */
    } preproc;

    /* --- SQI & channel selection (M3) --- */
    struct {
        float sqi_good_threshold;   /* SQI "good" threshold       (default 0.6)  */
        float sqi_poor_threshold;   /* SQI "poor" threshold       (default 0.3)  */
        float switch_margin;        /* Min SQI advantage to trigger main_ch
                                       switch (O11 hysteresis)    (default 0.05) */
        uint8_t switch_hold_count;  /* Consecutive cycles candidate must exceed
                                       current main_ch before switch (default 3) */
    } sqi;

    /* --- Motion detection (M4) --- */
    struct {
        float rest_energy_threshold;
        float walk_energy_threshold;
        float run_energy_threshold;
    } motion;

    /* --- MAC / adaptive filtering (M5) --- */
    struct {
        float    nlms_step_size;    /* NLMS step size             (default 0.01) */
        uint16_t filter_order;      /* Adaptive filter order      (default 8)    */
    } mac;

    /* --- Candidate estimation (M6) --- */
    struct {
        float hr_min_bpm;           /* Minimum valid HR           (default 40.0)  */
        float hr_max_bpm;           /* Maximum valid HR           (default 220.0) */
        float peak_min_prominence;  /* Peak detection prominence threshold        */
    } candidate;

    /* --- Fusion & smoothing (M7) --- */
    struct {
        float smooth_alpha;         /* Smoothing coefficient      (default 0.3) */
        float bpm_jump_threshold;   /* Max BPM jump per cycle                   */
    } fusion;

    /* --- State machine (M8) --- */
    struct {
        uint8_t acquire_stable_count;   /* Cycles needed ACQUIRE->TRACK    (default 5)  */
        uint8_t holdover_max_count;     /* Max HOLDOVER cycles             (default 10) */
        uint8_t reacquire_stable_count; /* Cycles needed REACQUIRE->TRACK  (default 3)  */
    } statemachine;

} hr_params_t;

#ifdef __cplusplus
}
#endif

#endif /* HR_ALGO_PARAMS_H */
