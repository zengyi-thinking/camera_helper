#ifndef ALGO_VISION_TARGET_POLICY_H_
#define ALGO_VISION_TARGET_POLICY_H_

#include "algo_beacon.h"
#include "algo_vision_target.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ALGO_VISION_TARGET_POLICY_REJECT_NONE          (0U)
#define ALGO_VISION_TARGET_POLICY_REJECT_ACCESSORY     (1U << 0)
#define ALGO_VISION_TARGET_POLICY_REJECT_RECENT_ALIAS  (1U << 1)
#define ALGO_VISION_TARGET_POLICY_REJECT_ONE_FRAGMENT  (1U << 2)

typedef struct
{
    uint8 shape;
    algo_vision_target_result_t current_target;
    algo_vision_target_result_t recent_target;
} algo_vision_target_policy_context_t;

void algo_vision_target_policy_make_context(
    const algo_vision_target_result_t *current_target,
    const algo_vision_target_result_t *recent_target,
    algo_vision_target_policy_context_t *context_out);

uint8 algo_vision_target_policy_filter_beacon(
    const algo_vision_target_policy_context_t *context,
    const algo_beacon_result_t *raw_beacon,
    uint8 accessory_like,
    uint8 recent_alias_like,
    algo_beacon_result_t *filtered_beacon,
    uint8 *reject_reason);

char algo_vision_target_policy_mode_char(void);

#ifdef __cplusplus
}
#endif

#endif
