#ifndef ALGO_BEACON_H_
#define ALGO_BEACON_H_

#include "zf_common_headfile.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint8_t found;
    uint8_t hold_active;   /* 1 when the low-level detector reuses its previous result after a miss */
    uint8_t source;
    uint8_t zone_flags;
    float center_x;
    float center_y;
    uint16_t bbox_x;
    uint16_t bbox_y;
    uint16_t bbox_w;
    uint16_t bbox_h;
    uint16_t area;
    uint16_t confidence_permille;
} algo_beacon_result_t;

#define ALGO_BEACON_MAX_CANDIDATES (3U)

typedef struct
{
    uint8_t full_detect;
    uint8_t count;
    algo_beacon_result_t items[ALGO_BEACON_MAX_CANDIDATES];
} algo_beacon_result_list_t;

#define ALGO_BEACON_RESULT_SOURCE_BLOB      (0U)
#define ALGO_BEACON_RESULT_SOURCE_FAR_POINT (1U)

#define ALGO_BEACON_ZONE_FLAG_FAR_POINT       (1U << 0)
#define ALGO_BEACON_ZONE_FLAG_EDGE            (1U << 1)
#define ALGO_BEACON_ZONE_FLAG_PERIOD_CONFIRMED (1U << 2)
#define ALGO_BEACON_ZONE_FLAG_DIFFUSE_CANDIDATE (1U << 3)

void algo_beacon_reset(void);
void algo_beacon_set_ignore_rect(uint16_t x,
                                 uint16_t y,
                                 uint16_t w,
                                 uint16_t h,
                                 uint16_t margin);
void algo_beacon_clear_ignore_rect(void);
void algo_beacon_process(const uint8 *src_image);
const algo_beacon_result_t *algo_beacon_get_result(void);
const algo_beacon_result_list_t *algo_beacon_get_candidates(void);

#ifdef __cplusplus
}
#endif

#endif
