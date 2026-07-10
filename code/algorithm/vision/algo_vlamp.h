#ifndef ALGO_VLAMP_H_
#define ALGO_VLAMP_H_

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint8_t found;
    uint8_t hold_active;   /* 本帧结果来自短时hold而非当前帧主检测; app层据此不更新控制目标位置 */
    /* Raw image pixel coordinates after the vision app orientation step:
     * center_x increases to the screen right; center_y increases to the screen bottom.
     * Do not treat these as physical/body axes. app_vision.c converts them to
     * target_x_err/target_y_err before publishing to flight control.
     */
    float center_x;
    float center_y;
    int16_t heading_cdeg;  /* signed heading angle derived from the same frame */
    float heading_x;        /* unit vector of lamp heading in canonical image frame */
    float heading_y;        /* unit vector of lamp heading in canonical image frame */
    uint16_t bbox_x;
    uint16_t bbox_y;
    uint16_t bbox_w;
    uint16_t bbox_h;
    uint16_t area;
    uint16_t confidence_permille;
} algo_vlamp_result_t;

void algo_vlamp_reset(void);
void algo_vlamp_process(const uint8 *src_image);
const algo_vlamp_result_t *algo_vlamp_get_result(void);

#ifdef __cplusplus
}
#endif

#endif
