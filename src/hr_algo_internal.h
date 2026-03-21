/**
 * @file hr_algo_internal.h
 * @brief Internal (full) definition of hr_algo_ctx_t.
 *
 * This header is internal to src/ and MUST NOT be placed in include/.
 * It provides the complete definition of the opaque hr_algo_ctx_t
 * whose forward declaration lives in include/hr_algo_api.h.
 *
 * Each module (M2–M8) will add its sub-context field here as it is
 * implemented.  Keep additions at the end of the struct.
 */

#ifndef HR_ALGO_INTERNAL_H
#define HR_ALGO_INTERNAL_H

#include "hr_algo_types.h"
#include "hr_algo_params.h"
#include "hr_algo_debug.h"
#include "hr_sampling.h"
#include "hr_preproc.h"
#include "hr_sqi.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/*  hr_algo_ctx — full definition  [Medium Freeze: internal layout]    */
/* ------------------------------------------------------------------ */
struct hr_algo_ctx {
    hr_params_t         params;
    hr_sampling_ctx_t   sampling;   /* M1 */

    /* --- Current-cycle outputs ------------------------------------ */
    hr_output_t         output;
    hr_debug_frame_t    debug;

    /* --- Module sub-contexts (appended in implementation order) --- */
    hr_preproc_ctx_t    preproc;    /* M2 */
    hr_sqi_ctx_t        sqi;        /* M3 */
    /* M4–M8 sub-contexts will be appended here by their modules.    */
};

#ifdef __cplusplus
}
#endif

#endif /* HR_ALGO_INTERNAL_H */
