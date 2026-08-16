#ifndef ALGO_VISION_TARGET_DETECTOR_H_
#define ALGO_VISION_TARGET_DETECTOR_H_

#include "algo_vision_target.h"

#ifdef __cplusplus
extern "C" {
#endif

void algo_vision_target_detector_reset(void);
void algo_vision_target_detector_process(const uint8 *src_image);
const algo_vision_target_result_t *algo_vision_target_detector_get_result(void);
uint8 algo_vision_target_detector_is_fresh_measurement(const algo_vision_target_result_t *result);

#ifdef __cplusplus
}
#endif

#endif
