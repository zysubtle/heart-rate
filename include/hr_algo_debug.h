/**
 * @file hr_algo_debug.h
 * @brief Debug / diagnostic frame for heart rate algorithm.
 *
 * [Strong Freeze] The main framework (7 candidates + state + output + flags)
 *                  must not be reduced. New fields may only be appended.
 * [Medium Freeze] The flags bit-field layout is determined during module
 *                  implementation.
 */

#ifndef HR_ALGO_DEBUG_H
#define HR_ALGO_DEBUG_H

#include "hr_algo_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/*  hr_debug_frame_t  [Strong Freeze — main framework]                */
/*                                                                    */
/*  One frame must be produced per processing cycle, including        */
/*  degraded / INIT / HOLDOVER scenarios.                             */
/* ------------------------------------------------------------------ */
typedef struct {

    /* --- Timestamp & state --- */
    uint32_t         timestamp_ms;
    hr_state_t       hr_state;
    motion_state_t   motion_state;

    /* --- Channel info --- */
    uint8_t          main_ch;
    uint8_t          backup_ch;
    float            sqi[4];

    /* --- Candidates (each carries valid/bpm/score/source) --- */
    hr_candidate_t   cand_peak_raw;
    hr_candidate_t   cand_peak_mac;
    hr_candidate_t   cand_fft_raw;
    hr_candidate_t   cand_fft_mac;
    hr_candidate_t   cand_acf_raw;
    hr_candidate_t   cand_acf_mac;
    hr_candidate_t   cand_pred;

    /* --- Final output --- */
    float            hr_out;
    uint8_t          confidence;

    /* --- Anomaly / event flags [Medium Freeze: bit layout TBD] --- */
    uint32_t         flags;

} hr_debug_frame_t;

#ifdef __cplusplus
}
#endif

#endif /* HR_ALGO_DEBUG_H */
