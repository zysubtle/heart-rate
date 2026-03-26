# heart-rate

Wrist-worn smart watch exercise heart rate algorithm for Cortex-M4F / Apollo 3.5 class MCU.

## Architecture

9-module pipeline: Sampling, Preprocessing, SQI, Motion, MAC, Candidate, Fusion, StateMachine, MainPipeline.
See `docs/hr_algo_architecture.md` for the full architecture freeze document.

## Public API

External callers include only `include/hr_algo_api.h`. The context type `hr_algo_ctx_t`
is opaque; use `hr_algo_ctx_sizeof()` to obtain the allocation size.

```c
#include "hr_algo_api.h"
#include <stdint.h>
#include <string.h>

static uint8_t ctx_buf[/* hr_algo_ctx_sizeof() at link time */];

void example(void) {
    hr_algo_ctx_t *ctx = (hr_algo_ctx_t *)ctx_buf;
    hr_params_t p = hr_algo_default_params();
    hr_algo_init(ctx, &p);

    /* 25 Hz sensor loop */
    hr_algo_feed_acc(ctx, ax, ay, az);
    hr_algo_feed_ppg(ctx, ch0, ch1, ch2, ch3);

    /* 1 Hz processing */
    hr_algo_process_1s(ctx);
    const hr_output_t *out = hr_algo_get_output(ctx);
    const hr_debug_frame_t *dbg = hr_algo_get_debug(ctx);
}
```

## Build & Test

```bash
make -C tests clean run
```

Requires a C11 compiler with `-lm`. No external dependencies.

## Project Status

- M1-M9: implemented and tested (166 tests, all passing)
- Public API: functional (`hr_algo_ctx_sizeof()` enables external allocation)
- O10 (log transport) and O14 (CMake): not yet implemented