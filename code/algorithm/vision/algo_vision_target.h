#ifndef ALGO_VISION_TARGET_H_
#define ALGO_VISION_TARGET_H_

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint8_t found;
    uint8_t hold_active;
    uint8_t source;
    /* Pixel coordinates after the application orientation step: x grows to the
     * image right and y grows to the image bottom. */
    float center_x;
    float center_y;
    int16_t heading_cdeg;
    float heading_x;
    float heading_y;
    uint16_t bbox_x;
    uint16_t bbox_y;
    uint16_t bbox_w;
    uint16_t bbox_h;
    uint16_t area;
    uint16_t confidence_permille;
} algo_vision_target_result_t;

typedef enum
{
    ALGO_VISION_TARGET_SOURCE_NONE = 0,
    ALGO_VISION_TARGET_SOURCE_MEASUREMENT,
    ALGO_VISION_TARGET_SOURCE_BRIGHT_FALLBACK,
    ALGO_VISION_TARGET_SOURCE_RECOVERY,
    ALGO_VISION_TARGET_SOURCE_PURE_HOLD
} algo_vision_target_source_t;

#ifdef __cplusplus
}
#endif

#endif
