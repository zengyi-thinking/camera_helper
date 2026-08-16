#include "algo_vision_target_detector.h"

#include "algo_vlamp.h"
#include "algo_onelamp.h"
#include "../../hardware/hw_config.h"

#if (HW_VISION_TARGET_SHAPE != HW_VISION_TARGET_SHAPE_V) \
 && (HW_VISION_TARGET_SHAPE != HW_VISION_TARGET_SHAPE_ONE)
#error "Unsupported HW_VISION_TARGET_SHAPE"
#endif

void algo_vision_target_detector_reset(void)
{
#if HW_VISION_TARGET_SHAPE == HW_VISION_TARGET_SHAPE_V
    algo_vlamp_reset();
#else
    algo_onelamp_reset();
#endif
}

void algo_vision_target_detector_process(const uint8 *src_image)
{
#if HW_VISION_TARGET_SHAPE == HW_VISION_TARGET_SHAPE_V
    algo_vlamp_process(src_image);
#else
    algo_onelamp_process(src_image);
#endif
}

const algo_vision_target_result_t *algo_vision_target_detector_get_result(void)
{
#if HW_VISION_TARGET_SHAPE == HW_VISION_TARGET_SHAPE_V
    return algo_vlamp_get_result();
#else
    return algo_onelamp_get_result();
#endif
}

uint8 algo_vision_target_detector_is_fresh_measurement(const algo_vision_target_result_t *result)
{
    return ((result != 0U) && (result->found != 0U)
            && (result->hold_active == 0U)
            && (result->source == ALGO_VISION_TARGET_SOURCE_MEASUREMENT)) ? 1U : 0U;
}
