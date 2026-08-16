#ifndef ALGO_VLAMP_H_
#define ALGO_VLAMP_H_

#include "algo_vision_target.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef algo_vision_target_result_t algo_vlamp_result_t;

#define ALGO_VLAMP_SOURCE_NONE            ALGO_VISION_TARGET_SOURCE_NONE
#define ALGO_VLAMP_SOURCE_STRICT_V        ALGO_VISION_TARGET_SOURCE_MEASUREMENT
#define ALGO_VLAMP_SOURCE_BRIGHT_FALLBACK ALGO_VISION_TARGET_SOURCE_BRIGHT_FALLBACK
#define ALGO_VLAMP_SOURCE_RECOVERY        ALGO_VISION_TARGET_SOURCE_RECOVERY
#define ALGO_VLAMP_SOURCE_PURE_HOLD       ALGO_VISION_TARGET_SOURCE_PURE_HOLD

void algo_vlamp_reset(void);
void algo_vlamp_process(const uint8 *src_image);
const algo_vlamp_result_t *algo_vlamp_get_result(void);

#ifdef __cplusplus
}
#endif

#endif
