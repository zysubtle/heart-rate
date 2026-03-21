/**
 * @file hr_algo_types.h
 * @brief Core type definitions for heart rate algorithm.
 *
 * [Strong Freeze] All enums and structures in this file are frozen.
 * Existing values/fields must not be removed or reordered.
 * New values/fields may only be appended.
 */

#ifndef HR_ALGO_TYPES_H
#define HR_ALGO_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/*  hr_state_t  [Strong Freeze]                                       */
/* ------------------------------------------------------------------ */
typedef enum {
    HR_STATE_INIT       = 0,
    HR_STATE_ACQUIRE    = 1,
    HR_STATE_TRACK      = 2,
    HR_STATE_HOLDOVER   = 3,
    HR_STATE_REACQUIRE  = 4
} hr_state_t;

/* ------------------------------------------------------------------ */
/*  motion_state_t  [Strong Freeze]                                   */
/* ------------------------------------------------------------------ */
typedef enum {
    MOTION_STATE_REST      = 0,
    MOTION_STATE_WALK      = 1,
    MOTION_STATE_RUN       = 2,
    MOTION_STATE_IRREGULAR = 3
} motion_state_t;

/* ------------------------------------------------------------------ */
/*  hr_source_t  [Strong Freeze]                                      */
/* ------------------------------------------------------------------ */
typedef enum {
    HR_SOURCE_NONE      = 0,
    HR_SOURCE_PEAK_RAW  = 1,
    HR_SOURCE_PEAK_MAC  = 2,
    HR_SOURCE_FFT_RAW   = 3,
    HR_SOURCE_FFT_MAC   = 4,
    HR_SOURCE_ACF_RAW   = 5,
    HR_SOURCE_ACF_MAC   = 6,
    HR_SOURCE_PRED      = 7
} hr_source_t;

/* ------------------------------------------------------------------ */
/*  hr_candidate_t  [Strong Freeze]                                   */
/*                                                                    */
/*  score: internal ranking quality [0.0, 1.0].                       */
/*         NOT the same as the final output confidence.               */
/*  When valid==false: bpm must be 0.0f, score must be 0.0f.          */
/* ------------------------------------------------------------------ */
typedef struct {
    bool          valid;
    float         bpm;
    float         score;
    hr_source_t   source;
} hr_candidate_t;

/* ------------------------------------------------------------------ */
/*  hr_output_t  [Strong Freeze]                                      */
/*                                                                    */
/*  hr_bpm:     0.0f means no valid output.                           */
/*  confidence: [0..100], final output confidence (NOT candidate      */
/*              score). 0 = unreliable, 100 = highly reliable.        */
/*  main_ch:    [0..3], primary PPG channel.                          */
/*  backup_ch:  [0..3], equals main_ch when no independent backup.    */
/*  sqi_main:   [0.0, 1.0], SQI of main channel.                     */
/* ------------------------------------------------------------------ */
typedef struct {
    float            hr_bpm;
    uint8_t          confidence;
    hr_state_t       hr_state;
    motion_state_t   motion_state;
    uint8_t          main_ch;
    uint8_t          backup_ch;
    float            sqi_main;
} hr_output_t;

#ifdef __cplusplus
}
#endif

#endif /* HR_ALGO_TYPES_H */
