#ifndef APP_CAMERA_HELPER_H_
#define APP_CAMERA_HELPER_H_

#include "zf_common_headfile.h"
#include "algorithm/vision/algo_vision_target_pipeline.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAMERA_HELPER_REQ_HEAD          (0xB1U)
#define CAMERA_HELPER_RSP_HEAD          (0xB2U)
#define CAMERA_HELPER_FRAME_TAIL        (0x55U)

#define CAMERA_HELPER_FLAG_HAS_V        (1U << 0)
#define CAMERA_HELPER_FLAG_HAS_B        (1U << 1)
#define CAMERA_HELPER_FLAG_MOVE_VALID   (1U << 2)
#define CAMERA_HELPER_FLAG_FRESH        (1U << 3)
#define CAMERA_HELPER_FLAG_MAPPED_VALID (1U << 4)

typedef struct
{
    uint8 has_vlamp;
    uint8 has_beacon;
    uint8 move_valid;
    uint8 flags;
    uint16 frame_id;
    uint32 timestamp_ms;
    int16 v_x;
    int16 v_y;
    int16 beacon_x;
    int16 beacon_y;
    int16 raw_dx;
    int16 raw_dy;
    int16 mapped_dx;
    int16 mapped_dy;
    uint16 v_confidence_permille;
    uint16 beacon_confidence_permille;
    uint8 beacon_candidate_count;
    algo_vision_target_result_t vlamp_result;
    algo_beacon_result_t beacon_result;
} app_camera_helper_snapshot_t;

void app_camera_helper_init(void);
void app_camera_helper_tick(void);
void app_camera_helper_on_byte(uint8 byte);
const app_camera_helper_snapshot_t *app_camera_helper_get_snapshot(void);

#ifdef __cplusplus
}
#endif

#endif
