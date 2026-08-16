#include "algo_vision_target_policy.h"

#include "../../hardware/hw_config.h"

#include <math.h>
#include <string.h>

#define ALGO_VISION_ONE_FRAGMENT_CORRIDOR_MARGIN_PX (8.0f)
#define ALGO_VISION_ONE_FRAGMENT_LONG_MAX           (20U)
#define ALGO_VISION_ONE_FRAGMENT_AREA_MAX           (100U)
#define ALGO_VISION_ONE_FRAGMENT_RATIO_MIN          (1.55f)

#if HW_VISION_TARGET_SHAPE == HW_VISION_TARGET_SHAPE_ONE
static uint16 algo_vision_target_policy_long_side(uint16 w, uint16 h)
{
    return (w > h) ? w : h;
}

static uint16 algo_vision_target_policy_short_side(uint16 w, uint16 h)
{
    return (w > h) ? h : w;
}

static float algo_vision_target_policy_point_gap_to_rect(float px,
                                                         float py,
                                                         uint16 rx,
                                                         uint16 ry,
                                                         uint16 rw,
                                                         uint16 rh)
{
    float x0 = (float)rx;
    float y0 = (float)ry;
    float x1 = (rw > 0U) ? (float)(rx + rw - 1U) : x0;
    float y1 = (rh > 0U) ? (float)(ry + rh - 1U) : y0;
    float dx;
    float dy;

    if(px < x0) { dx = x0 - px; }
    else if(px > x1) { dx = px - x1; }
    else { dx = 0.0f; }
    if(py < y0) { dy = y0 - py; }
    else if(py > y1) { dy = py - y1; }
    else { dy = 0.0f; }
    return sqrtf((dx * dx) + (dy * dy));
}

static uint8 algo_vision_target_policy_period_confirmed(const algo_beacon_result_t *beacon)
{
    return ((beacon != 0U) && (beacon->found != 0U)
            && ((beacon->zone_flags & ALGO_BEACON_ZONE_FLAG_PERIOD_CONFIRMED) != 0U)) ? 1U : 0U;
}
#endif

static uint8 algo_vision_target_policy_one_fragment(
    const algo_vision_target_policy_context_t *context,
    const algo_beacon_result_t *beacon)
{
#if HW_VISION_TARGET_SHAPE == HW_VISION_TARGET_SHAPE_ONE
    const algo_vision_target_result_t *guard_target;
    uint16 long_side;
    uint16 short_side;
    float ratio;
    float gap;

    if((context == 0U) || (beacon == 0U) || (beacon->found == 0U)
       || (algo_vision_target_policy_period_confirmed(beacon) != 0U))
    {
        return 0U;
    }
    guard_target = (context->current_target.found != 0U)
                 ? &context->current_target : &context->recent_target;
    if((guard_target->found == 0U) || (guard_target->bbox_w == 0U)
       || (guard_target->bbox_h == 0U))
    {
        return 0U;
    }

    long_side = algo_vision_target_policy_long_side(beacon->bbox_w, beacon->bbox_h);
    short_side = algo_vision_target_policy_short_side(beacon->bbox_w, beacon->bbox_h);
    ratio = (short_side > 0U) ? ((float)long_side / (float)short_side) : 999.0f;
    if((long_side > ALGO_VISION_ONE_FRAGMENT_LONG_MAX)
       || (beacon->area > ALGO_VISION_ONE_FRAGMENT_AREA_MAX)
       || (ratio < ALGO_VISION_ONE_FRAGMENT_RATIO_MIN))
    {
        return 0U;
    }
    gap = algo_vision_target_policy_point_gap_to_rect(beacon->center_x,
                                                       beacon->center_y,
                                                       guard_target->bbox_x,
                                                       guard_target->bbox_y,
                                                       guard_target->bbox_w,
                                                       guard_target->bbox_h);
    return (gap <= ALGO_VISION_ONE_FRAGMENT_CORRIDOR_MARGIN_PX) ? 1U : 0U;
#else
    (void)context;
    (void)beacon;
    return 0U;
#endif
}

void algo_vision_target_policy_make_context(
    const algo_vision_target_result_t *current_target,
    const algo_vision_target_result_t *recent_target,
    algo_vision_target_policy_context_t *context_out)
{
    if(context_out == 0U)
    {
        return;
    }
    memset(context_out, 0, sizeof(*context_out));
    context_out->shape = HW_VISION_TARGET_SHAPE;
    if(current_target != 0U) { context_out->current_target = *current_target; }
    if(recent_target != 0U) { context_out->recent_target = *recent_target; }
}

uint8 algo_vision_target_policy_filter_beacon(
    const algo_vision_target_policy_context_t *context,
    const algo_beacon_result_t *raw_beacon,
    uint8 accessory_like,
    uint8 recent_alias_like,
    algo_beacon_result_t *filtered_beacon,
    uint8 *reject_reason)
{
    uint8 reason = ALGO_VISION_TARGET_POLICY_REJECT_NONE;

    if(filtered_beacon == 0U)
    {
        return 0U;
    }
    memset(filtered_beacon, 0, sizeof(*filtered_beacon));
    if(raw_beacon == 0U)
    {
        if(reject_reason != 0U) { *reject_reason = reason; }
        return 0U;
    }
    *filtered_beacon = *raw_beacon;

    if(accessory_like != 0U)
    {
        reason |= ALGO_VISION_TARGET_POLICY_REJECT_ACCESSORY;
    }
    if(recent_alias_like != 0U)
    {
        reason |= ALGO_VISION_TARGET_POLICY_REJECT_RECENT_ALIAS;
    }
    if(algo_vision_target_policy_one_fragment(context, raw_beacon) != 0U)
    {
        reason |= ALGO_VISION_TARGET_POLICY_REJECT_ONE_FRAGMENT;
    }
    if(reason != ALGO_VISION_TARGET_POLICY_REJECT_NONE)
    {
        memset(filtered_beacon, 0, sizeof(*filtered_beacon));
    }
    if(reject_reason != 0U) { *reject_reason = reason; }
    return (filtered_beacon->found != 0U) ? 1U : 0U;
}

char algo_vision_target_policy_mode_char(void)
{
#if HW_VISION_TARGET_SHAPE == HW_VISION_TARGET_SHAPE_ONE
    return '1';
#else
    return 'V';
#endif
}
