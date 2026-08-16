#ifndef ALGO_VISION_TARGET_PIPELINE_H
#define ALGO_VISION_TARGET_PIPELINE_H

#include "algo_vision_target.h"
#include "algo_beacon.h"

typedef struct
{
    algo_vision_target_result_t target_display;
    algo_vision_target_result_t target_control;
    algo_beacon_result_t beacon_display;
    algo_beacon_result_t beacon_publish;
    algo_beacon_result_list_t beacon_candidates;
} algo_vision_target_pipeline_result_t;

void algo_vision_target_pipeline_init(void);
void algo_vision_target_pipeline_detect_target(const uint8 *gray_image,
                                               uint16 image_width,
                                               uint16 image_height,
                                               uint16 bbox_long_limit_px,
                                               algo_vision_target_result_t *result_out);
void algo_vision_target_pipeline_update(const uint8 *gray_image,
                                        uint32 frame_id,
                                        algo_vision_target_pipeline_result_t *result_out);

#endif
