#include "algo_vision_target_pipeline.h"
#include "algo_vision_target_detector.h"
#include "algo_vision_target_policy.h"
#include "algo_beacon_activity.h"
#include "../../hardware/hw_config.h"

#include <math.h>
#include <string.h>

#define VISION_BEACON_IGNORE_V_MARGIN    (4U)
/* V近边缘判定与冲突裁决: V bbox 中心距图像边 <= 此值视为近边缘 (与 beacon 边缘带 12px
 * 对齐 +4px 余量)。近边缘 V 塌缩成小亮核时极易满足 beacon 胜出条件 (conf>=360,
 * V conf<420, V area 小, V long 小), 故把近边缘时的 beacon 胜出门槛提高到 420,
 * 让边缘 V 更难被 beacon 抢走。 */
#define VISION_TARGET_NEAR_EDGE_MARGIN    (16U)
#define VISION_BEACON_ACCESSORY_CONF_KEEP_MIN (650U)
#define VISION_BEACON_ACCESSORY_LONG_MAX (10U)
#define VISION_BEACON_ACCESSORY_EDGE_LONG_MAX (8U)
#define VISION_BEACON_ACCESSORY_AREA_MAX (36U)
/* V 附属光 (V 灯臂上的小反射点/光晕碎片) 通常紧贴 V bbox (gap<12px)。
 * 旧值 24px 把中心距 V 边缘 14~24px 的真信标也当附属光清掉了。
 * 收到 14px: 只清真正紧贴 V 的附属光, 让 gap>14px 的近距真信标保留。
 * 边缘区保持 12px (边缘附属光更紧凑)。 */
#define VISION_BEACON_ACCESSORY_GAP_MAX_PX (14.0f)
#define VISION_BEACON_ACCESSORY_EDGE_GAP_MAX_PX (12.0f)
#define VISION_BEACON_NEAR_DIFFUSE_CONF_MIN (320U)
/* 近距扩散真信标的最小面积: 靠近 V 时 V 光晕可能让 blob 面积略偏小,
 * 从 18 降到 12 让近距小信标也能通过 near_diffuse 放行 (仍需 long>=7
 * + ratio<=1.85 + conf>=320 三重约束防误识别)。 */
#define VISION_BEACON_NEAR_DIFFUSE_AREA_MIN (12U)
#define VISION_BEACON_NEAR_DIFFUSE_LONG_MIN (7U)
#define VISION_BEACON_NEAR_DIFFUSE_RATIO_MAX (1.85f)
#define VISION_BEACON_ACQUIRE_HITS (3U)
#define VISION_BEACON_ACQUIRE_PERIOD_HITS (2U)
#define VISION_BEACON_SWITCH_HITS (2U)
#define VISION_BEACON_PUBLISH_DIST_MAX_PX (20.0f)
#define VISION_BEACON_PUBLISH_CENTER_ALPHA (0.35f)
#define VISION_BEACON_TRACK_CENTER_ALPHA (0.70f)
#define VISION_BEACON_TRACK_HOLD_MISSES (2U)
#define VISION_BEACON_DETECT_FLAG_FULL (1U << 0)
#define VISION_BEACON_DETECT_FLAG_FRESH (1U << 1)
#define VISION_BEACON_PUBLISH_CONF_MIN (360U)
#define VISION_BEACON_PUBLISH_PERIOD_CONF_MIN (600U)
#define VISION_BEACON_PUBLISH_BLOB_AREA_MIN (10U)
#define VISION_BEACON_PUBLISH_BLOB_LONG_MIN (4U)
#define VISION_BEACON_PUBLISH_BLOB_RATIO_MAX (2.10f)
#define VISION_BEACON_PUBLISH_EDGE_BLOB_RATIO_MAX (2.55f)
#define VISION_BEACON_PUBLISH_BLOB_FILL_MIN (0.26f)
#define VISION_BEACON_PUBLISH_EDGE_BLOB_FILL_MIN (0.18f)
#define VISION_BEACON_PUBLISH_FAR_AREA_MAX (18U)
#define VISION_BEACON_PUBLISH_FAR_LONG_MAX (7U)
#define VISION_BEACON_PUBLISH_VLIKE_LONG_MIN (10U)
#define VISION_BEACON_PUBLISH_VLIKE_AREA_MIN (22U)
#define VISION_BEACON_PUBLISH_VLIKE_RATIO_MIN (1.45f)
#define VISION_BEACON_PUBLISH_VLIKE_FILL_MAX (0.42f)
#define VISION_BEACON_PUBLISH_UNCONF_LONG_MAX_CENTER (16U)
#define VISION_BEACON_PUBLISH_UNCONF_LONG_MAX_EDGE (18U)
#define VISION_BEACON_PUBLISH_UNCONF_AREA_MAX_CENTER (78U)
#define VISION_BEACON_PUBLISH_UNCONF_AREA_MAX_EDGE (84U)
#define VISION_BEACON_PUBLISH_UNCONF_RATIO_MAX_CENTER (1.85f)
#define VISION_BEACON_PUBLISH_UNCONF_RATIO_MAX_EDGE (2.25f)
#define VISION_BEACON_PUBLISH_UNCONF_FILL_MIN_CENTER (0.30f)
#define VISION_BEACON_PUBLISH_UNCONF_FILL_MIN_EDGE (0.22f)
#define VISION_BEACON_CLOSE_KEEP_CONF_MIN (650U)
#define VISION_BEACON_CLOSE_LARGE_CONF_MIN VISION_BEACON_NEAR_DIFFUSE_CONF_MIN
#define VISION_BEACON_CLOSE_CENTER_DIST_MIN_PX (14.0f)
#define VISION_BEACON_CLOSE_PERIOD_DIST_MIN_PX (7.0f)
#define VISION_BEACON_CLOSE_DIFFUSE_DIST_MIN_PX (9.0f)
#define VISION_BEACON_CLOSE_STRONG_DIST_MIN_PX (11.0f)
#define VISION_BEACON_CLOSE_GAP_MIN_PX (5.0f)
#define VISION_BEACON_DIFFUSE_KEEP_GAP_MIN_PX (6.0f)
#define VISION_BEACON_ALIAS_SAME_CENTER_PX (5.0f)
#define VISION_BEACON_CLOSE_RATIO_MAX (2.80f)
#define VISION_BEACON_CLOSE_LONG_MAX (28U)
/* 第一级 margin 检出的 beacon 离 V bbox 的 gap <= 此值时, 认为近距信标已被
 * 检出, 不需要 margin=0 回退补检。gap > 此值说明检出的 beacon 在远处, 近处可能
 * 有被 margin 屏蔽掉的真信标, 需要 margin=0 回退补检。
 * 取 margin+6: 比 ignore margin 略大, 给信标 bbox 自身尺寸留余量。 */
#define VISION_BEACON_NEAR_V_PROXIMITY_PX  ((float)(VISION_BEACON_IGNORE_V_MARGIN) + 6.0f)
#define VISION_BEACON_VLAMP_PART_LONG_MIN (12U)
#define VISION_BEACON_VLAMP_PART_AREA_MIN (38U)
#define VISION_BEACON_VLAMP_PART_GAP_MAX_PX (20.0f)
#define VISION_BEACON_VLAMP_PART_CENTER_MAX_PX (38.0f)
#define VISION_BEACON_RECENT_V_ALIAS_GAP_MAX_PX (24.0f)
#define VISION_BEACON_RECENT_V_ALIAS_CENTER_MAX_PX (50.0f)
#define VISION_TARGET_RECENT_HOLD_FRAMES (22U)
/* V灯发布 bbox 长边绝对上限兜底, 防止 halo 与 V 连通成单 blob 后 bbox 扩展
 * 覆盖邻近信标、再经 beacon ignore rect 屏蔽信标检测。center/heading/area 不动,
 * 仅等比收缩 bbox 外框, 故 V 追踪控制不受影响。依据: 真实 V 实测 bbox 约 41x25,
 * 52 留约 25% 余量不误裁; 吞信标后 long 通常 >52 可被拦截。极近距 V 据实调大。 */
#define VISION_TARGET_BBOX_LONG_MAX_PX    (52U)
#define VISION_TARGET_CONTROL_MAIN_STEP_MAX_PX (12.0f)
#define VISION_TARGET_CONTROL_LOW_CONF_PERMILLE (150U)
#define VISION_TARGET_CONTROL_JUMP_CONFIRM_PX (10.0f)
#define VISION_TARGET_CONTROL_PENDING_MATCH_PX (8.0f)
#define VISION_TARGET_CONTROL_CONFIRM_HITS (2U)


static const uint8 *s_gray_image = 0;
static uint32 s_frame_id = 0U;
static uint8 s_target_recent_hold = 0U;
static float s_target_recent_center_x = 0.0f;
static float s_target_recent_center_y = 0.0f;
static float s_target_recent_heading_x = 1.0f;
static float s_target_recent_heading_y = 0.0f;
static uint16 s_target_recent_bbox_x = 0U;
static uint16 s_target_recent_bbox_y = 0U;
static uint16 s_target_recent_bbox_w = 0U;
static uint16 s_target_recent_bbox_h = 0U;
static uint8 s_target_control_valid = 0U;
static algo_vision_target_result_t s_target_control_last;
static uint8 s_target_control_pending_valid = 0U;
static uint8 s_target_control_pending_hits = 0U;
static algo_vision_target_result_t s_target_control_pending;
static uint8 s_beacon_candidate_valid = 0U;
static uint8 s_beacon_candidate_hits = 0U;
static float s_beacon_candidate_center_x = 0.0f;
static float s_beacon_candidate_center_y = 0.0f;
static uint8 s_beacon_track_valid = 0U;
static uint8 s_beacon_track_misses = 0U;
static algo_beacon_result_t s_beacon_track_result;
static uint8 s_beacon_detect_cache_valid = 0U;
static algo_beacon_result_t s_beacon_detect_cache;

typedef enum
{
    VISION_TARGET_BEACON_KEEP_BOTH = 0,
    VISION_TARGET_BEACON_KEEP_TARGET,
    VISION_TARGET_BEACON_KEEP_BEACON
} vision_target_beacon_decision_t;

static uint8 vision_beacon_center_near_edge(const algo_beacon_result_t *beacon_result);
static uint8 vision_beacon_period_confirmed(const algo_beacon_result_t *beacon_result);
static float vision_point_gap_to_rect(float px, float py, uint16 rx, uint16 ry, uint16 rw, uint16 rh);
static float vision_clamp_float(float value, float min_value, float max_value)
{
    if(!isfinite(value)) { return min_value; }
    if(value < min_value) { return min_value; }
    if(value > max_value) { return max_value; }
    return value;
}
static float algo_vision_target_pipeline_clamp(float value, float min_value, float max_value)
{
    if(!isfinite(value)) { return min_value; }
    if(value < min_value) { return min_value; }
    if(value > max_value) { return max_value; }
    return value;
}

static void algo_vision_target_pipeline_set_bbox(algo_vision_target_result_t *result,
                                                 uint16 image_width,
                                                 uint16 image_height,
                                                 uint16 bbox_w,
                                                 uint16 bbox_h)
{
    int32 x0;
    int32 y0;
    uint16 max_x0;
    uint16 max_y0;

    if((result == 0) || (image_width == 0U) || (image_height == 0U)) { return; }
    if(bbox_w == 0U) { bbox_w = 1U; }
    if(bbox_h == 0U) { bbox_h = 1U; }
    if(bbox_w > image_width) { bbox_w = image_width; }
    if(bbox_h > image_height) { bbox_h = image_height; }
    result->center_x = algo_vision_target_pipeline_clamp(result->center_x, 0.0f, (float)(image_width - 1U));
    result->center_y = algo_vision_target_pipeline_clamp(result->center_y, 0.0f, (float)(image_height - 1U));
    max_x0 = (uint16)(image_width - bbox_w);
    max_y0 = (uint16)(image_height - bbox_h);
    x0 = (int32)(result->center_x - (((float)bbox_w - 1.0f) * 0.5f) + 0.5f);
    y0 = (int32)(result->center_y - (((float)bbox_h - 1.0f) * 0.5f) + 0.5f);
    if(x0 < 0) { x0 = 0; }
    if(y0 < 0) { y0 = 0; }
    if(x0 > (int32)max_x0) { x0 = (int32)max_x0; }
    if(y0 > (int32)max_y0) { y0 = (int32)max_y0; }
    result->bbox_x = (uint16)x0;
    result->bbox_y = (uint16)y0;
    result->bbox_w = bbox_w;
    result->bbox_h = bbox_h;
}

static void algo_vision_target_pipeline_limit_bbox(algo_vision_target_result_t *result,
                                                    uint16 image_width,
                                                    uint16 image_height,
                                                    uint16 bbox_long_limit_px)
{
    uint16 long_side;
    float scale;
    uint16 new_w;
    uint16 new_h;

    if((result == 0) || (result->found == 0U)
       || (result->bbox_w == 0U) || (result->bbox_h == 0U)
       || (bbox_long_limit_px == 0U)) { return; }
    long_side = (result->bbox_w > result->bbox_h) ? result->bbox_w : result->bbox_h;
    if(long_side <= bbox_long_limit_px) { return; }
    scale = (float)bbox_long_limit_px / (float)long_side;
    new_w = (uint16)(((float)result->bbox_w * scale) + 0.5f);
    new_h = (uint16)(((float)result->bbox_h * scale) + 0.5f);
    if(new_w < 1U) { new_w = 1U; }
    if(new_h < 1U) { new_h = 1U; }
    if(new_w > bbox_long_limit_px) { new_w = bbox_long_limit_px; }
    if(new_h > bbox_long_limit_px) { new_h = bbox_long_limit_px; }
    algo_vision_target_pipeline_set_bbox(result, image_width, image_height, new_w, new_h);
}

void algo_vision_target_pipeline_detect_target(const uint8 *gray_image,
                                               uint16 image_width,
                                               uint16 image_height,
                                               uint16 bbox_long_limit_px,
                                               algo_vision_target_result_t *result_out)
{
    const algo_vision_target_result_t *detected;

    if(result_out == 0) { return; }
    memset(result_out, 0, sizeof(*result_out));
    if((gray_image == 0) || (image_width == 0U) || (image_height == 0U)) { return; }
    algo_vision_target_detector_process(gray_image);
    detected = algo_vision_target_detector_get_result();
    if(detected != 0) { *result_out = *detected; }
    algo_vision_target_pipeline_limit_bbox(result_out,
                                           image_width,
                                           image_height,
                                           bbox_long_limit_px);
}

static uint32 vision_rect_area_u32(uint16 w, uint16 h)
{
    return (uint32)w * (uint32)h;
}

static uint16 vision_rect_long_side(uint16 w, uint16 h)
{
    return (w > h) ? w : h;
}

static uint16 vision_rect_short_side(uint16 w, uint16 h)
{
    return (w > h) ? h : w;
}

static uint32 vision_rect_overlap_area_u32(uint16 ax,
                                           uint16 ay,
                                           uint16 aw,
                                           uint16 ah,
                                           uint16 bx,
                                           uint16 by,
                                           uint16 bw,
                                           uint16 bh)
{
    uint16 ax1;
    uint16 ay1;
    uint16 bx1;
    uint16 by1;
    uint16 ox0;
    uint16 oy0;
    uint16 ox1;
    uint16 oy1;

    if((aw == 0U) || (ah == 0U) || (bw == 0U) || (bh == 0U))
    {
        return 0U;
    }

    ax1 = (uint16)(ax + aw - 1U);
    ay1 = (uint16)(ay + ah - 1U);
    bx1 = (uint16)(bx + bw - 1U);
    by1 = (uint16)(by + bh - 1U);

    if((ax > bx1) || (bx > ax1) || (ay > by1) || (by > ay1))
    {
        return 0U;
    }

    ox0 = (ax > bx) ? ax : bx;
    oy0 = (ay > by) ? ay : by;
    ox1 = (ax1 < bx1) ? ax1 : bx1;
    oy1 = (ay1 < by1) ? ay1 : by1;

    return vision_rect_area_u32((uint16)(ox1 - ox0 + 1U), (uint16)(oy1 - oy0 + 1U));
}

static uint8 vision_recent_target_bbox_valid(void)
{
    return ((s_target_recent_hold > 0U)
            && (s_target_recent_bbox_x < MT9V03X_W)
            && (s_target_recent_bbox_y < MT9V03X_H)
            && (s_target_recent_bbox_w > 0U)
            && (s_target_recent_bbox_h > 0U)) ? 1U : 0U;
}

static void vision_target_set_bbox_around_center(algo_vision_target_result_t *result,
                                                float center_x,
                                                float center_y,
                                                uint16 bbox_w,
                                                uint16 bbox_h)
{
    int32 x0_i;
    int32 y0_i;
    uint16 max_x0;
    uint16 max_y0;

    if(result == 0U)
    {
        return;
    }

    if(bbox_w == 0U) { bbox_w = 1U; }
    if(bbox_h == 0U) { bbox_h = 1U; }
    if(bbox_w > MT9V03X_W) { bbox_w = MT9V03X_W; }
    if(bbox_h > MT9V03X_H) { bbox_h = MT9V03X_H; }

    center_x = vision_clamp_float(center_x, 0.0f, (float)(MT9V03X_W - 1U));
    center_y = vision_clamp_float(center_y, 0.0f, (float)(MT9V03X_H - 1U));

    max_x0 = (uint16)(MT9V03X_W - bbox_w);
    max_y0 = (uint16)(MT9V03X_H - bbox_h);
    x0_i = (int32)(center_x - (((float)bbox_w - 1.0f) * 0.5f) + 0.5f);
    y0_i = (int32)(center_y - (((float)bbox_h - 1.0f) * 0.5f) + 0.5f);

    if(x0_i < 0) { x0_i = 0; }
    if(y0_i < 0) { y0_i = 0; }
    if(x0_i > (int32)max_x0) { x0_i = (int32)max_x0; }
    if(y0_i > (int32)max_y0) { y0_i = (int32)max_y0; }

    result->center_x = center_x;
    result->center_y = center_y;
    result->bbox_x = (uint16)x0_i;
    result->bbox_y = (uint16)y0_i;
    result->bbox_w = bbox_w;
    result->bbox_h = bbox_h;
}

/* V灯发布 bbox 绝对长边兜底: 等比收缩 bbox 外框到上限, 保持 center 不变。
 * 用途: 防止 halo 与 V 连通成单 blob 时, V 发布 bbox 扩展覆盖邻近信标,
 *       经 beacon ignore rect 屏蔽信标检测。center/heading/area 不动,
 *       只缩 bbox_x/y/w/h, 所以 V 追踪控制不受影响, 屏幕绿框不再吞信标。
 * 不调用: found=0、bbox 无效、或长边已在上限内的结果直接返回。 */
static void vision_clamp_target_bbox_to_limit(algo_vision_target_result_t *result)
{
    uint16 long_side;
    float scale;
    uint16 new_w;
    uint16 new_h;

    if((result == 0U) || (result->found == 0U)
       || (result->bbox_w == 0U) || (result->bbox_h == 0U))
    {
        return;
    }

    long_side = (result->bbox_w > result->bbox_h) ? result->bbox_w : result->bbox_h;
    if(long_side <= VISION_TARGET_BBOX_LONG_MAX_PX)
    {
        return;
    }

    /* 等比收缩: 以长边为基准 scale, 短边按相同比例缩小, 保持长宽比。
     * 浮点取整误差可能导致新长边略超上限, 故再夹一次。 */
    scale = (float)VISION_TARGET_BBOX_LONG_MAX_PX / (float)long_side;
    new_w = (uint16)(((float)result->bbox_w * scale) + 0.5f);
    new_h = (uint16)(((float)result->bbox_h * scale) + 0.5f);
    if(new_w < 1U) { new_w = 1U; }
    if(new_h < 1U) { new_h = 1U; }
    if(new_w > VISION_TARGET_BBOX_LONG_MAX_PX) { new_w = VISION_TARGET_BBOX_LONG_MAX_PX; }
    if(new_h > VISION_TARGET_BBOX_LONG_MAX_PX) { new_h = VISION_TARGET_BBOX_LONG_MAX_PX; }

    vision_target_set_bbox_around_center(result,
                                        result->center_x,
                                        result->center_y,
                                        new_w,
                                        new_h);
}

static uint8 vision_targets_conflict(const algo_vision_target_result_t *target_result,
                                     const algo_beacon_result_t *beacon_result)
{
    uint32 target_area;
    uint32 b_area;
    uint32 overlap;
    float dx;
    float dy;
    float dist2;
    uint16 b_long;

    if((target_result == 0U) || (beacon_result == 0U)
       || (target_result->found == 0U) || (beacon_result->found == 0U))
    {
        return 0U;
    }

    target_area = vision_rect_area_u32(target_result->bbox_w, target_result->bbox_h);
    b_area = vision_rect_area_u32(beacon_result->bbox_w, beacon_result->bbox_h);
    overlap = vision_rect_overlap_area_u32(target_result->bbox_x,
                                           target_result->bbox_y,
                                           target_result->bbox_w,
                                           target_result->bbox_h,
                                           beacon_result->bbox_x,
                                           beacon_result->bbox_y,
                                           beacon_result->bbox_w,
                                           beacon_result->bbox_h);

    if((overlap > 0U)
       && ((overlap * 100U) >= ((target_area < b_area) ? target_area : b_area) * 45U))
    {
        return 1U;
    }

    dx = target_result->center_x - beacon_result->center_x;
    dy = target_result->center_y - beacon_result->center_y;
    dist2 = (dx * dx) + (dy * dy);
    b_long = vision_rect_long_side(beacon_result->bbox_w, beacon_result->bbox_h);
    if(dist2 < ((float)(b_long + 6U) * (float)(b_long + 6U)))
    {
        return 1U;
    }

    return 0U;
}

static float vision_target_beacon_center_distance(const algo_vision_target_result_t *target_result,
                                                 const algo_beacon_result_t *beacon_result)
{
    float dx;
    float dy;

    if((target_result == 0U) || (beacon_result == 0U))
    {
        return 0.0f;
    }

    dx = beacon_result->center_x - target_result->center_x;
    dy = beacon_result->center_y - target_result->center_y;
    return sqrtf((dx * dx) + (dy * dy));
}

static uint8 vision_beacon_is_close_independent_target(const algo_vision_target_result_t *target_result,
                                                       const algo_beacon_result_t *beacon_result)
{
    uint16 b_long;
    uint16 b_short;
    float ratio;
    float dist;
    float gap;
    float dist_min;
    float near_dist_min;
    uint8 b_edge;
    uint16 accessory_long_max;
    uint8 period_confirmed;
    uint8 diffuse_like;
    uint8 large_or_strong;

    if((target_result == 0U) || (beacon_result == 0U)
       || (target_result->found == 0U) || (beacon_result->found == 0U))
    {
        return 0U;
    }

    b_long = vision_rect_long_side(beacon_result->bbox_w, beacon_result->bbox_h);
    b_short = vision_rect_short_side(beacon_result->bbox_w, beacon_result->bbox_h);
    ratio = (b_short > 0U) ? ((float)b_long / (float)b_short) : 999.0f;
    if((b_long > VISION_BEACON_CLOSE_LONG_MAX) || (ratio > VISION_BEACON_CLOSE_RATIO_MAX))
    {
        return 0U;
    }

    dist = vision_target_beacon_center_distance(target_result, beacon_result);
    gap = vision_point_gap_to_rect(beacon_result->center_x,
                                   beacon_result->center_y,
                                   target_result->bbox_x,
                                   target_result->bbox_y,
                                   target_result->bbox_w,
                                   target_result->bbox_h);
    dist_min = (float)b_long + 3.0f;
    if(dist_min < VISION_BEACON_CLOSE_CENTER_DIST_MIN_PX)
    {
        dist_min = VISION_BEACON_CLOSE_CENTER_DIST_MIN_PX;
    }
    period_confirmed = vision_beacon_period_confirmed(beacon_result);

    b_edge = vision_beacon_center_near_edge(beacon_result);
    accessory_long_max = (b_edge != 0U)
                         ? VISION_BEACON_ACCESSORY_EDGE_LONG_MAX
                         : VISION_BEACON_ACCESSORY_LONG_MAX;
    large_or_strong = ((beacon_result->confidence_permille >= VISION_BEACON_CLOSE_LARGE_CONF_MIN)
                       && ((b_long > accessory_long_max)
                           || (beacon_result->area > VISION_BEACON_ACCESSORY_AREA_MAX))) ? 1U : 0U;
    diffuse_like = ((beacon_result->confidence_permille >= VISION_BEACON_NEAR_DIFFUSE_CONF_MIN)
                    && (beacon_result->area >= VISION_BEACON_NEAR_DIFFUSE_AREA_MIN)
                    && (b_long >= VISION_BEACON_NEAR_DIFFUSE_LONG_MIN)
                    && (ratio <= VISION_BEACON_NEAR_DIFFUSE_RATIO_MAX)) ? 1U : 0U;

    if(period_confirmed != 0U)
    {
        near_dist_min = (float)b_long + 1.0f;
        if(near_dist_min < VISION_BEACON_CLOSE_PERIOD_DIST_MIN_PX)
        {
            near_dist_min = VISION_BEACON_CLOSE_PERIOD_DIST_MIN_PX;
        }
        return (dist >= near_dist_min) ? 1U : 0U;
    }

    if((diffuse_like != 0U) || (large_or_strong != 0U))
    {
        if(gap < VISION_BEACON_CLOSE_GAP_MIN_PX)
        {
            return 0U;
        }

        near_dist_min = (diffuse_like != 0U)
                      ? VISION_BEACON_CLOSE_DIFFUSE_DIST_MIN_PX
                      : VISION_BEACON_CLOSE_STRONG_DIST_MIN_PX;
        if(dist >= near_dist_min)
        {
            return 1U;
        }
    }

    if((beacon_result->confidence_permille >= VISION_BEACON_CLOSE_KEEP_CONF_MIN)
       && (dist >= dist_min)
       && (gap >= VISION_BEACON_CLOSE_GAP_MIN_PX))
    {
        return 1U;
    }

    return 0U;
}

static uint8 vision_beacon_is_near_target_large_unconfirmed(const algo_vision_target_result_t *target_result,
                                                       const algo_beacon_result_t *beacon_result)
{
    uint16 b_long;
    float gap;
    float dist;

    if((target_result == 0U) || (beacon_result == 0U)
       || (target_result->found == 0U) || (beacon_result->found == 0U))
    {
        return 0U;
    }

    if(vision_beacon_period_confirmed(beacon_result) != 0U)
    {
        return 0U;
    }

    b_long = vision_rect_long_side(beacon_result->bbox_w, beacon_result->bbox_h);
    if((b_long < VISION_BEACON_VLAMP_PART_LONG_MIN)
       || (beacon_result->area < VISION_BEACON_VLAMP_PART_AREA_MIN))
    {
        return 0U;
    }

    gap = vision_point_gap_to_rect(beacon_result->center_x,
                                   beacon_result->center_y,
                                   target_result->bbox_x,
                                   target_result->bbox_y,
                                   target_result->bbox_w,
                                   target_result->bbox_h);
    dist = vision_target_beacon_center_distance(target_result, beacon_result);
    return ((gap <= VISION_BEACON_VLAMP_PART_GAP_MAX_PX)
            || (dist <= VISION_BEACON_VLAMP_PART_CENTER_MAX_PX)) ? 1U : 0U;
}

static void vision_make_recent_target_result(algo_vision_target_result_t *result)
{
    if(result == 0U)
    {
        return;
    }

    memset(result, 0, sizeof(*result));
    if(vision_recent_target_bbox_valid() == 0U)
    {
        return;
    }

    result->found = 1U;
    result->center_x = s_target_recent_center_x;
    result->center_y = s_target_recent_center_y;
    result->heading_x = s_target_recent_heading_x;
    result->heading_y = s_target_recent_heading_y;
    result->bbox_x = s_target_recent_bbox_x;
    result->bbox_y = s_target_recent_bbox_y;
    result->bbox_w = s_target_recent_bbox_w;
    result->bbox_h = s_target_recent_bbox_h;
    result->confidence_permille = 360U;
}

static uint8 vision_beacon_is_recent_target_alias(const algo_vision_target_result_t *target_result,
                                                 const algo_beacon_result_t *beacon_result)
{
    algo_vision_target_result_t recent_target;
    float gap;
    float dx;
    float dy;
    float dist;

    if((beacon_result == 0U) || (beacon_result->found == 0U)
       || (vision_recent_target_bbox_valid() == 0U))
    {
        return 0U;
    }

#if HW_VISION_TARGET_SHAPE == HW_VISION_TARGET_SHAPE_V
    /* A weak current V result is continuity evidence, so compare a conflicting
     * beacon against the last strict V geometry as well. Period confirmation
     * alone is not enough to turn the same recent lamp pixels into a beacon;
     * a spatially independent beacon is still admitted below. */
    if((target_result != 0U) && (target_result->found != 0U)
       && (target_result->source == ALGO_VISION_TARGET_SOURCE_MEASUREMENT))
    {
        return 0U;
    }
#else
    if((target_result != 0U) && (target_result->found != 0U))
    {
        return 0U;
    }

    if(vision_beacon_period_confirmed(beacon_result) != 0U)
    {
        return 0U;
    }
#endif

    vision_make_recent_target_result(&recent_target);
    if(recent_target.found == 0U)
    {
        return 0U;
    }

    dx = beacon_result->center_x - recent_target.center_x;
    dy = beacon_result->center_y - recent_target.center_y;
    dist = sqrtf((dx * dx) + (dy * dy));
    if(vision_beacon_is_close_independent_target(&recent_target, beacon_result) != 0U)
    {
        return 0U;
    }

    if(dist <= VISION_BEACON_ALIAS_SAME_CENTER_PX)
    {
        return 1U;
    }

    if((vision_beacon_is_near_target_large_unconfirmed(&recent_target, beacon_result) != 0U)
       && (dist <= VISION_BEACON_RECENT_V_ALIAS_CENTER_MAX_PX))
    {
        return 1U;
    }

    if(vision_targets_conflict(&recent_target, beacon_result) != 0U)
    {
        return 1U;
    }

    gap = vision_point_gap_to_rect(beacon_result->center_x,
                                   beacon_result->center_y,
                                   recent_target.bbox_x,
                                   recent_target.bbox_y,
                                   recent_target.bbox_w,
                                   recent_target.bbox_h);
    return (gap <= VISION_BEACON_RECENT_V_ALIAS_GAP_MAX_PX) ? 1U : 0U;
}

static vision_target_beacon_decision_t vision_decide_target_beacon(const algo_vision_target_result_t *target_result,
                                                     const algo_beacon_result_t *beacon_result)
{
    if((target_result == 0U) || (beacon_result == 0U)
       || (target_result->found == 0U) || (beacon_result->found == 0U))
    {
        return VISION_TARGET_BEACON_KEEP_BOTH;
    }

    if(vision_beacon_is_close_independent_target(target_result, beacon_result) != 0U)
    {
        return VISION_TARGET_BEACON_KEEP_BOTH;
    }

#if HW_VISION_TARGET_SHAPE == HW_VISION_TARGET_SHAPE_ONE
    /* A periodic compact beacon is stronger class evidence than a geometric
     * bar when both occupy the same pixels. This prevents a saturated or
     * cable-cut beacon from being consumed as the vehicle lamp. */
    if((vision_beacon_period_confirmed(beacon_result) != 0U)
       && (vision_targets_conflict(target_result, beacon_result) != 0U))
    {
        return VISION_TARGET_BEACON_KEEP_BEACON;
    }
#else

    /* Only a strict current-frame V may win an overlap with a confirmed beacon.
     * Recovery, fallback and pure hold are continuity evidence, not class proof. */
    if((target_result->source != ALGO_VISION_TARGET_SOURCE_MEASUREMENT)
       && (vision_beacon_period_confirmed(beacon_result) != 0U)
       && (vision_targets_conflict(target_result, beacon_result) != 0U))
    {
        return VISION_TARGET_BEACON_KEEP_BEACON;
    }
#endif

    /* 简化裁决: 真正独立的近距信标保留两者; 无冲突保留两者;
     * 同一物体冲突则默认 V 优先, 避免红框抢走飞控目标。 */
    if(vision_targets_conflict(target_result, beacon_result) == 0U)
    {
        return VISION_TARGET_BEACON_KEEP_BOTH;
    }

    /* 同一物体冲突时默认保留 V: 小车控制目标不能被红框抢走。
     * 真信标靠近 V 时必须先满足 close independent 规则, 才允许 KEEP_BOTH。 */
    return VISION_TARGET_BEACON_KEEP_TARGET;
}

static vision_target_beacon_decision_t vision_apply_target_beacon_decision(algo_vision_target_result_t *target_result,
                                                     algo_beacon_result_t *beacon_result)
{
    vision_target_beacon_decision_t decision = vision_decide_target_beacon(target_result, beacon_result);

    if(decision == VISION_TARGET_BEACON_KEEP_TARGET)
    {
        memset(beacon_result, 0, sizeof(*beacon_result));
        algo_beacon_reset();
    }
    else if(decision == VISION_TARGET_BEACON_KEEP_BEACON)
    {
        memset(target_result, 0, sizeof(*target_result));
    }

    return decision;
}

static uint8 vision_beacon_center_near_edge(const algo_beacon_result_t *beacon_result)
{
    float cx;
    float cy;

    if((beacon_result == 0U) || (beacon_result->found == 0U))
    {
        return 0U;
    }

    cx = beacon_result->center_x;
    cy = beacon_result->center_y;

    return ((cx <= (float)VISION_TARGET_NEAR_EDGE_MARGIN)
            || (cy <= (float)VISION_TARGET_NEAR_EDGE_MARGIN)
            || (cx >= (float)(MT9V03X_W - 1U - VISION_TARGET_NEAR_EDGE_MARGIN))
            || (cy >= (float)(MT9V03X_H - 1U - VISION_TARGET_NEAR_EDGE_MARGIN))) ? 1U : 0U;
}

static float vision_point_gap_to_rect(float px,
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

static uint8 vision_beacon_is_near_target_accessory_light(const algo_vision_target_result_t *target_result,
                                                     const algo_beacon_result_t *beacon_result)
{
    uint16 b_long;
    uint16 b_short;
    uint8 b_edge;
    uint16 long_max;
    float gap_max;
    float gap;
    float ratio;

    if((target_result == 0U) || (beacon_result == 0U)
       || (target_result->found == 0U) || (beacon_result->found == 0U))
    {
        return 0U;
    }

    if(beacon_result->confidence_permille >= VISION_BEACON_ACCESSORY_CONF_KEEP_MIN)
    {
        return 0U;
    }

    b_edge = vision_beacon_center_near_edge(beacon_result);
    long_max = (b_edge != 0U)
               ? VISION_BEACON_ACCESSORY_EDGE_LONG_MAX
               : VISION_BEACON_ACCESSORY_LONG_MAX;
    b_long = vision_rect_long_side(beacon_result->bbox_w, beacon_result->bbox_h);
    b_short = vision_rect_short_side(beacon_result->bbox_w, beacon_result->bbox_h);
    ratio = (b_short > 0U) ? ((float)b_long / (float)b_short) : 999.0f;
    gap_max = (b_edge != 0U)
              ? VISION_BEACON_ACCESSORY_EDGE_GAP_MAX_PX
              : VISION_BEACON_ACCESSORY_GAP_MAX_PX;
    gap = vision_point_gap_to_rect(beacon_result->center_x,
                                   beacon_result->center_y,
                                   target_result->bbox_x,
                                   target_result->bbox_y,
                                   target_result->bbox_w,
                                   target_result->bbox_h);
    if((beacon_result->confidence_permille >= VISION_BEACON_NEAR_DIFFUSE_CONF_MIN)
       && (beacon_result->area >= VISION_BEACON_NEAR_DIFFUSE_AREA_MIN)
       && (b_long >= VISION_BEACON_NEAR_DIFFUSE_LONG_MIN)
       && (ratio <= VISION_BEACON_NEAR_DIFFUSE_RATIO_MAX))
    {
        return (gap < VISION_BEACON_DIFFUSE_KEEP_GAP_MIN_PX) ? 1U : 0U;
    }

    if((b_long > long_max) || (beacon_result->area > VISION_BEACON_ACCESSORY_AREA_MAX))
    {
        return 0U;
    }

    return (gap <= gap_max) ? 1U : 0U;
}

static void vision_filter_beacon_by_target_policy(
    const algo_vision_target_result_t *target_result,
    algo_beacon_result_t *beacon_result)
{
    algo_vision_target_policy_context_t context;
    algo_vision_target_result_t recent_target;
    algo_beacon_result_t filtered;
    uint8 accessory_like;
    uint8 recent_alias_like;
    uint8 reject_reason;

    if(beacon_result == 0U)
    {
        return;
    }
    vision_make_recent_target_result(&recent_target);
    algo_vision_target_policy_make_context(target_result, &recent_target, &context);
    accessory_like = ((vision_beacon_is_near_target_accessory_light(target_result, beacon_result) != 0U)
                      || (vision_beacon_is_near_target_large_unconfirmed(target_result, beacon_result) != 0U)) ? 1U : 0U;
    recent_alias_like = vision_beacon_is_recent_target_alias(target_result, beacon_result);
    (void)algo_vision_target_policy_filter_beacon(&context,
                                                  beacon_result,
                                                  accessory_like,
                                                  recent_alias_like,
                                                  &filtered,
                                                  &reject_reason);
    (void)reject_reason;
    *beacon_result = filtered;
}

static void vision_update_target_recent_track(const algo_vision_target_result_t *target_result,
                                             uint8 allow_geometry_refresh)
{
    if((target_result != 0U) && (target_result->found != 0U))
    {
        if(target_result->source != ALGO_VISION_TARGET_SOURCE_MEASUREMENT)
        {
            if(s_target_recent_hold > 0U)
            {
                s_target_recent_hold--;
            }
            return;
        }
        if(allow_geometry_refresh != 0U)
        {
            s_target_recent_center_x = target_result->center_x;
            s_target_recent_center_y = target_result->center_y;
            s_target_recent_heading_x = target_result->heading_x;
            s_target_recent_heading_y = target_result->heading_y;
            if((target_result->bbox_w > 0U) && (target_result->bbox_h > 0U))
            {
                s_target_recent_bbox_x = target_result->bbox_x;
                s_target_recent_bbox_y = target_result->bbox_y;
                s_target_recent_bbox_w = target_result->bbox_w;
                s_target_recent_bbox_h = target_result->bbox_h;
            }
            s_target_recent_hold = VISION_TARGET_RECENT_HOLD_FRAMES;
        }
        else if(s_target_recent_hold > 0U)
        {
            s_target_recent_hold--;
        }
    }
    else if(s_target_recent_hold > 0U)
    {
        s_target_recent_hold--;
    }
}

static uint8 vision_target_control_measurement_ok(const algo_vision_target_result_t *target_result)
{
    if((target_result == 0U) || (target_result->found == 0U)
       || (target_result->source == ALGO_VISION_TARGET_SOURCE_PURE_HOLD)
       || (target_result->bbox_w == 0U) || (target_result->bbox_h == 0U)
       || (target_result->area == 0U) || (target_result->confidence_permille == 0U))
    {
        return 0U;
    }

    return 1U;
}

static void vision_target_control_pending_reset(void)
{
    s_target_control_pending_valid = 0U;
    s_target_control_pending_hits = 0U;
    memset(&s_target_control_pending, 0, sizeof(s_target_control_pending));
}

static void vision_target_control_reset(void)
{
    s_target_control_valid = 0U;
    memset(&s_target_control_last, 0, sizeof(s_target_control_last));
    vision_target_control_pending_reset();
}

static void vision_target_control_copy_held(algo_vision_target_result_t *control_result)
{
    if((control_result == 0U) || (s_target_control_valid == 0U))
    {
        return;
    }

    *control_result = s_target_control_last;
    control_result->hold_active = 1U;
    control_result->source = ALGO_VISION_TARGET_SOURCE_PURE_HOLD;
}

static uint8 vision_target_control_candidate_needs_confirm(const algo_vision_target_result_t *result)
{
    float dx;
    float dy;
    float dist;

    if(result == 0U)
    {
        return 1U;
    }
    if(s_target_control_valid == 0U)
    {
        return 1U;
    }
    dx = result->center_x - s_target_control_last.center_x;
    dy = result->center_y - s_target_control_last.center_y;
    dist = sqrtf((dx * dx) + (dy * dy));

    if((result->source == ALGO_VISION_TARGET_SOURCE_BRIGHT_FALLBACK)
       || (result->source == ALGO_VISION_TARGET_SOURCE_RECOVERY))
    {
        if((s_target_control_last.source == result->source)
           && (dist <= VISION_TARGET_CONTROL_PENDING_MATCH_PX))
        {
            return 0U;
        }
        return 1U;
    }
    if(result->confidence_permille < VISION_TARGET_CONTROL_LOW_CONF_PERMILLE)
    {
        return 1U;
    }
    return (dist > VISION_TARGET_CONTROL_JUMP_CONFIRM_PX) ? 1U : 0U;
}

static uint8 vision_target_control_candidate_confirmed(const algo_vision_target_result_t *result)
{
    float dx;
    float dy;
    float dist;

    if(result == 0U)
    {
        vision_target_control_pending_reset();
        return 0U;
    }

    if(s_target_control_pending_valid == 0U)
    {
        s_target_control_pending = *result;
        s_target_control_pending_valid = 1U;
        s_target_control_pending_hits = 1U;
        return 0U;
    }

    /* 确认以位置连续性为准, 检测来源(严格V/局部恢复/亮区回退)交替
     * 不再重置计数: 远距弱检测下来源逐帧交替曾把确认计数反复清零,
     * 控制位置被冻结在旧值, 表现为对远处目标毫无追赶。 */
    dx = result->center_x - s_target_control_pending.center_x;
    dy = result->center_y - s_target_control_pending.center_y;
    dist = sqrtf((dx * dx) + (dy * dy));
    if(dist > VISION_TARGET_CONTROL_PENDING_MATCH_PX)
    {
        s_target_control_pending = *result;
        s_target_control_pending_hits = 1U;
        return 0U;
    }

    s_target_control_pending = *result;
    if(s_target_control_pending_hits < 255U)
    {
        s_target_control_pending_hits++;
    }
    return (s_target_control_pending_hits >= VISION_TARGET_CONTROL_CONFIRM_HITS) ? 1U : 0U;
}

static void vision_target_control_update_bbox(algo_vision_target_result_t *result)
{
    uint16 bbox_w;
    uint16 bbox_h;

    if(result == 0U)
    {
        return;
    }

    bbox_w = (result->bbox_w > 0U) ? result->bbox_w : 1U;
    bbox_h = (result->bbox_h > 0U) ? result->bbox_h : 1U;
    vision_target_set_bbox_around_center(result,
                                        result->center_x,
                                        result->center_y,
                                        bbox_w,
                                        bbox_h);
}

static void vision_prepare_target_control(const algo_vision_target_result_t *display_result,
                                                const algo_beacon_result_t *beacon_result,
                                                algo_vision_target_result_t *control_result)
{
    uint8 use_measurement;

    if(control_result == 0U)
    {
        return;
    }
    memset(control_result, 0, sizeof(*control_result));

    use_measurement = vision_target_control_measurement_ok(display_result);
    /* V 短时 hold 帧 (algo层单帧漏检但 result 保持): 当成"无新测量"处理。
     * 内部保留上一帧位置用于连续恢复, 同时显式标为 PURE_HOLD, 让发布层
     * 不把旧像素当成新控制观测。仅已有有效 control track 时才保留。 */
    if((display_result != 0U) && (display_result->hold_active != 0U))
    {
        /* 单帧漏检的 hold 帧保留确认候选计数(位置真的变了由距离检查
         * 重置), 否则"检出-漏检-检出"的弱信号永远凑不满确认帧数,
         * 控制位置被冻结。 */
        use_measurement = 0U;
        if(s_target_control_valid != 0U)
        {
            vision_target_control_copy_held(control_result);
            return;
        }
    }

    if(use_measurement != 0U)
    {
        if(vision_target_control_candidate_needs_confirm(display_result) != 0U)
        {
            if(vision_target_control_candidate_confirmed(display_result) == 0U)
            {
                if(s_target_control_valid != 0U)
                {
                    vision_target_control_copy_held(control_result);
                }
                return;
            }
        }
        vision_target_control_pending_reset();
    }

    if((use_measurement == 0U)
       && (s_target_control_valid != 0U)
       && (display_result != 0U)
       && (display_result->found != 0U)
       && (beacon_result != 0U)
       && (beacon_result->found != 0U)
       && (vision_beacon_period_confirmed(beacon_result) == 0U)
       && (vision_targets_conflict(display_result, beacon_result) != 0U))
    {
        vision_target_control_copy_held(control_result);
        return;
    }

    if(use_measurement != 0U)
    {
        float prev_x;
        float prev_y;
        float dx;
        float dy;

        *control_result = *display_result;
        if(s_target_control_valid != 0U)
        {
            prev_x = s_target_control_last.center_x;
            prev_y = s_target_control_last.center_y;
            dx = vision_clamp_float(control_result->center_x - prev_x,
                                    -VISION_TARGET_CONTROL_MAIN_STEP_MAX_PX,
                                    VISION_TARGET_CONTROL_MAIN_STEP_MAX_PX);
            dy = vision_clamp_float(control_result->center_y - prev_y,
                                    -VISION_TARGET_CONTROL_MAIN_STEP_MAX_PX,
                                    VISION_TARGET_CONTROL_MAIN_STEP_MAX_PX);
            control_result->center_x = vision_clamp_float(prev_x + dx, 0.0f, (float)(MT9V03X_W - 1U));
            control_result->center_y = vision_clamp_float(prev_y + dy, 0.0f, (float)(MT9V03X_H - 1U));
            vision_target_control_update_bbox(control_result);
        }

        s_target_control_last = *control_result;
        s_target_control_valid = 1U;
        return;
    }

    vision_target_control_reset();
    (void)beacon_result;
}


static void vision_detect_main_target(algo_vision_target_result_t *target_out)
{
    algo_vision_target_pipeline_detect_target(s_gray_image,
                                               MT9V03X_W,
                                               MT9V03X_H,
                                               VISION_TARGET_BBOX_LONG_MAX_PX,
                                               target_out);
}

static void vision_apply_beacon_ignore_from_target(const algo_vision_target_result_t *target_result)
{
#if HW_VISION_TARGET_SHAPE == HW_VISION_TARGET_SHAPE_ONE
    /* Do not mask the complete bar bounding box. A cable-cut bar can span a
     * real beacon, and the ONE policy already rejects only elongated lamp
     * fragments while preserving compact or periodic beacons. */
    (void)target_result;
    algo_beacon_clear_ignore_rect();
#else
    if((target_result != 0U) && (target_result->found != 0U)
       && (target_result->source == ALGO_VISION_TARGET_SOURCE_MEASUREMENT))
    {
        algo_beacon_set_ignore_rect(target_result->bbox_x,
                                    target_result->bbox_y,
                                    target_result->bbox_w,
                                    target_result->bbox_h,
                                    VISION_BEACON_IGNORE_V_MARGIN);
    }
    else
    {
        algo_beacon_clear_ignore_rect();
    }
#endif
}

static void vision_copy_current_beacon_result(algo_beacon_result_t *beacon_out)
{
    const algo_beacon_result_t *beacon_result;

    if(beacon_out == 0U)
    {
        return;
    }

    beacon_result = algo_beacon_get_result();
    if(beacon_result != 0U)
    {
        *beacon_out = *beacon_result;
    }
    else
    {
        memset(beacon_out, 0, sizeof(*beacon_out));
    }
}

static void vision_beacon_candidate_reset(void)
{
    s_beacon_candidate_valid = 0U;
    s_beacon_candidate_hits = 0U;
    s_beacon_candidate_center_x = 0.0f;
    s_beacon_candidate_center_y = 0.0f;
}

static void vision_beacon_track_reset(void)
{
    s_beacon_track_valid = 0U;
    s_beacon_track_misses = 0U;
    memset(&s_beacon_track_result, 0, sizeof(s_beacon_track_result));
}

static void vision_beacon_publish_reset(void)
{
    vision_beacon_candidate_reset();
    vision_beacon_track_reset();
}

static void vision_beacon_detect_cache_reset(void)
{
    s_beacon_detect_cache_valid = 0U;
    memset(&s_beacon_detect_cache, 0, sizeof(s_beacon_detect_cache));
}

static void vision_beacon_detect_cache_store(const algo_beacon_result_t *result)
{
    if(result == 0U)
    {
        vision_beacon_detect_cache_reset();
        return;
    }

    s_beacon_detect_cache = *result;
    s_beacon_detect_cache_valid = 1U;
}

static uint8 vision_beacon_should_run_full_detect(void)
{
    if(s_beacon_detect_cache_valid == 0U)
    {
        return 1U;
    }

    if((s_beacon_track_valid == 0U) || (s_beacon_candidate_valid != 0U))
    {
        return 1U;
    }

    return ((s_frame_id & 1U) == 0U) ? 1U : 0U;
}

static uint8 vision_beacon_period_confirmed(const algo_beacon_result_t *beacon_result)
{
    return ((beacon_result != 0U)
            && (beacon_result->found != 0U)
            && ((beacon_result->zone_flags & ALGO_BEACON_ZONE_FLAG_PERIOD_CONFIRMED) != 0U)) ? 1U : 0U;
}

static uint8 vision_beacon_publish_vlike_reject(const algo_beacon_result_t *raw_result,
                                                uint16 b_long,
                                                float ratio,
                                                float fill,
                                                uint8 edge_near)
{
    uint8 far_point;

    if((raw_result == 0U) || (raw_result->found == 0U)
       || (vision_beacon_period_confirmed(raw_result) != 0U))
    {
        return 0U;
    }

    far_point = ((raw_result->source == ALGO_BEACON_RESULT_SOURCE_FAR_POINT)
                 || ((raw_result->zone_flags & ALGO_BEACON_ZONE_FLAG_FAR_POINT) != 0U)) ? 1U : 0U;
    if(far_point != 0U)
    {
        return 0U;
    }

    if((b_long >= 18U) && (raw_result->area >= 55U))
    {
        return 1U;
    }

    if((b_long < VISION_BEACON_PUBLISH_VLIKE_LONG_MIN)
       || (raw_result->area < VISION_BEACON_PUBLISH_VLIKE_AREA_MIN))
    {
        return 0U;
    }

    if((edge_near != 0U) && (raw_result->area < 30U) && (b_long < 13U))
    {
        return 0U;
    }

    return ((ratio >= VISION_BEACON_PUBLISH_VLIKE_RATIO_MIN)
            || ((fill <= VISION_BEACON_PUBLISH_VLIKE_FILL_MAX)
                && (ratio >= 1.12f))) ? 1U : 0U;
}

static uint8 vision_beacon_publish_unconfirmed_shape_ok(const algo_beacon_result_t *raw_result,
                                                        uint16 b_long,
                                                        float ratio,
                                                        float fill,
                                                        uint8 edge_near,
                                                        uint8 far_point)
{
    uint16 long_max;
    uint16 area_max;
    float ratio_max;
    float fill_min;

    if((raw_result == 0U) || (raw_result->found == 0U))
    {
        return 0U;
    }

    if((far_point != 0U) || (vision_beacon_period_confirmed(raw_result) != 0U))
    {
        return 1U;
    }

    long_max = (edge_near != 0U)
               ? VISION_BEACON_PUBLISH_UNCONF_LONG_MAX_EDGE
               : VISION_BEACON_PUBLISH_UNCONF_LONG_MAX_CENTER;
    area_max = (edge_near != 0U)
               ? VISION_BEACON_PUBLISH_UNCONF_AREA_MAX_EDGE
               : VISION_BEACON_PUBLISH_UNCONF_AREA_MAX_CENTER;
    ratio_max = (edge_near != 0U)
                ? VISION_BEACON_PUBLISH_UNCONF_RATIO_MAX_EDGE
                : VISION_BEACON_PUBLISH_UNCONF_RATIO_MAX_CENTER;
    fill_min = (edge_near != 0U)
               ? VISION_BEACON_PUBLISH_UNCONF_FILL_MIN_EDGE
               : VISION_BEACON_PUBLISH_UNCONF_FILL_MIN_CENTER;

    return ((b_long <= long_max)
            && (raw_result->area <= area_max)
            && (ratio <= ratio_max)
            && (fill >= fill_min)) ? 1U : 0U;
}

static uint8 vision_beacon_publish_features_ok(const algo_beacon_result_t *raw_result)
{
    uint16 b_long;
    uint16 b_short;
    uint32 bbox_area;
    float ratio;
    float fill;
    uint8 edge_near;
    uint8 far_point;

    if((raw_result == 0U) || (raw_result->found == 0U)
       || (raw_result->bbox_w == 0U) || (raw_result->bbox_h == 0U)
       || (raw_result->area == 0U))
    {
        return 0U;
    }

    b_long = vision_rect_long_side(raw_result->bbox_w, raw_result->bbox_h);
    b_short = vision_rect_short_side(raw_result->bbox_w, raw_result->bbox_h);
    bbox_area = vision_rect_area_u32(raw_result->bbox_w, raw_result->bbox_h);
    ratio = (b_short > 0U) ? ((float)b_long / (float)b_short) : 999.0f;
    fill = (bbox_area > 0U) ? ((float)raw_result->area / (float)bbox_area) : 0.0f;
    edge_near = ((raw_result->zone_flags & ALGO_BEACON_ZONE_FLAG_EDGE) != 0U) ? 1U : 0U;
    far_point = ((raw_result->source == ALGO_BEACON_RESULT_SOURCE_FAR_POINT)
                 || ((raw_result->zone_flags & ALGO_BEACON_ZONE_FLAG_FAR_POINT) != 0U)) ? 1U : 0U;

    if(vision_beacon_period_confirmed(raw_result) != 0U)
    {
        return (raw_result->confidence_permille >= VISION_BEACON_PUBLISH_PERIOD_CONF_MIN) ? 1U : 0U;
    }

    if(raw_result->confidence_permille < VISION_BEACON_PUBLISH_CONF_MIN)
    {
        return 0U;
    }

    if(far_point != 0U)
    {
        return ((raw_result->area <= VISION_BEACON_PUBLISH_FAR_AREA_MAX)
                && (b_long <= VISION_BEACON_PUBLISH_FAR_LONG_MAX)) ? 1U : 0U;
    }

    if(vision_beacon_publish_vlike_reject(raw_result, b_long, ratio, fill, edge_near) != 0U)
    {
        return 0U;
    }

    if(vision_beacon_publish_unconfirmed_shape_ok(raw_result,
                                                  b_long,
                                                  ratio,
                                                  fill,
                                                  edge_near,
                                                  far_point) == 0U)
    {
        return 0U;
    }

    if((raw_result->area < VISION_BEACON_PUBLISH_BLOB_AREA_MIN)
       || (b_long < VISION_BEACON_PUBLISH_BLOB_LONG_MIN))
    {
        return 0U;
    }

    if(ratio > ((edge_near != 0U) ? VISION_BEACON_PUBLISH_EDGE_BLOB_RATIO_MAX
                                  : VISION_BEACON_PUBLISH_BLOB_RATIO_MAX))
    {
        return 0U;
    }

    if(fill < ((edge_near != 0U) ? VISION_BEACON_PUBLISH_EDGE_BLOB_FILL_MIN
                                 : VISION_BEACON_PUBLISH_BLOB_FILL_MIN))
    {
        return 0U;
    }

    return 1U;
}

static uint8 vision_beacon_publish_hits_required(const algo_beacon_result_t *raw_result)
{
    if(vision_beacon_period_confirmed(raw_result) != 0U)
    {
        return VISION_BEACON_ACQUIRE_PERIOD_HITS;
    }

    return VISION_BEACON_ACQUIRE_HITS;
}

static uint8 vision_beacon_candidate_update(const algo_beacon_result_t *raw_result)
{
    float dx;
    float dy;
    float dist;

    if((raw_result == 0U) || (raw_result->found == 0U))
    {
        vision_beacon_candidate_reset();
        return 0U;
    }

    if(s_beacon_candidate_valid == 0U)
    {
        s_beacon_candidate_valid = 1U;
        s_beacon_candidate_hits = 1U;
        s_beacon_candidate_center_x = raw_result->center_x;
        s_beacon_candidate_center_y = raw_result->center_y;
        return s_beacon_candidate_hits;
    }

    dx = raw_result->center_x - s_beacon_candidate_center_x;
    dy = raw_result->center_y - s_beacon_candidate_center_y;
    dist = sqrtf((dx * dx) + (dy * dy));
    if(dist > VISION_BEACON_PUBLISH_DIST_MAX_PX)
    {
        s_beacon_candidate_hits = 1U;
        s_beacon_candidate_center_x = raw_result->center_x;
        s_beacon_candidate_center_y = raw_result->center_y;
        return s_beacon_candidate_hits;
    }

    if(s_beacon_candidate_hits < 255U)
    {
        s_beacon_candidate_hits++;
    }
    s_beacon_candidate_center_x += VISION_BEACON_PUBLISH_CENTER_ALPHA
                                      * (raw_result->center_x - s_beacon_candidate_center_x);
    s_beacon_candidate_center_y += VISION_BEACON_PUBLISH_CENTER_ALPHA
                                      * (raw_result->center_y - s_beacon_candidate_center_y);
    return s_beacon_candidate_hits;
}

static void vision_beacon_track_accept(const algo_beacon_result_t *raw_result,
                                       uint8 smooth_center)
{
    float center_x;
    float center_y;

    if(raw_result == 0U)
    {
        return;
    }

    center_x = raw_result->center_x;
    center_y = raw_result->center_y;
    if((smooth_center != 0U) && (s_beacon_track_valid != 0U))
    {
        center_x = s_beacon_track_result.center_x
                 + VISION_BEACON_TRACK_CENTER_ALPHA
                 * (raw_result->center_x - s_beacon_track_result.center_x);
        center_y = s_beacon_track_result.center_y
                 + VISION_BEACON_TRACK_CENTER_ALPHA
                 * (raw_result->center_y - s_beacon_track_result.center_y);
    }

    s_beacon_track_result = *raw_result;
    s_beacon_track_result.center_x = center_x;
    s_beacon_track_result.center_y = center_y;
    s_beacon_track_result.hold_active = 0U;
    s_beacon_track_valid = 1U;
    s_beacon_track_misses = 0U;
    vision_beacon_candidate_reset();
}

static void vision_beacon_track_note_miss(void)
{
    if((s_beacon_track_valid != 0U) && (s_beacon_track_misses < 255U))
    {
        s_beacon_track_misses++;
    }
    if(s_beacon_track_misses > VISION_BEACON_TRACK_HOLD_MISSES)
    {
        vision_beacon_track_reset();
    }
}

static void vision_filter_beacon_for_publish(const algo_beacon_result_t *raw_result,
                                              uint8 detect_flags,
                                              algo_beacon_result_t *publish_result)
{
    float dx;
    float dy;
    float dist;
    uint8 full_detect;
    uint8 fresh_measurement;
    uint8 candidate_hits;
    uint8 hits_required;

    if(publish_result == 0U)
    {
        return;
    }
    memset(publish_result, 0, sizeof(*publish_result));

    full_detect = ((detect_flags & VISION_BEACON_DETECT_FLAG_FULL) != 0U) ? 1U : 0U;
    fresh_measurement = ((detect_flags & VISION_BEACON_DETECT_FLAG_FRESH) != 0U) ? 1U : 0U;

    if(full_detect == 0U)
    {
        if(s_beacon_track_valid != 0U)
        {
            *publish_result = s_beacon_track_result;
        }
        return;
    }

    if((fresh_measurement == 0U)
       || (vision_beacon_publish_features_ok(raw_result) == 0U))
    {
        vision_beacon_candidate_reset();
        vision_beacon_track_note_miss();
        if(s_beacon_track_valid != 0U)
        {
            *publish_result = s_beacon_track_result;
        }
        return;
    }

    if(s_beacon_track_valid != 0U)
    {
        dx = raw_result->center_x - s_beacon_track_result.center_x;
        dy = raw_result->center_y - s_beacon_track_result.center_y;
        dist = sqrtf((dx * dx) + (dy * dy));
        if(dist <= VISION_BEACON_PUBLISH_DIST_MAX_PX)
        {
            vision_beacon_track_accept(raw_result, 1U);
            *publish_result = s_beacon_track_result;
            return;
        }

        candidate_hits = vision_beacon_candidate_update(raw_result);
        vision_beacon_track_note_miss();
        if(candidate_hits >= VISION_BEACON_SWITCH_HITS)
        {
            vision_beacon_track_accept(raw_result, 0U);
            *publish_result = s_beacon_track_result;
            return;
        }

        if(s_beacon_track_valid != 0U)
        {
            *publish_result = s_beacon_track_result;
        }
        return;
    }

    candidate_hits = vision_beacon_candidate_update(raw_result);
    hits_required = vision_beacon_publish_hits_required(raw_result);
    if(candidate_hits >= hits_required)
    {
        vision_beacon_track_accept(raw_result, 0U);
        *publish_result = s_beacon_track_result;
    }
}

static uint8 vision_beacon_full_detect_flags(const algo_beacon_result_t *beacon_result)
{
    uint8 flags = VISION_BEACON_DETECT_FLAG_FULL;

    if((beacon_result != 0U) && (beacon_result->found != 0U)
       && (beacon_result->hold_active == 0U))
    {
        flags |= VISION_BEACON_DETECT_FLAG_FRESH;
    }
    return flags;
}

static uint8 vision_detect_beacon_target(const algo_vision_target_result_t *target_result,
                                         algo_beacon_result_t *beacon_out)
{
#if HW_VISION_TARGET_SHAPE == HW_VISION_TARGET_SHAPE_V
    algo_beacon_result_t retry_result;
#endif

    if(beacon_out == 0U)
    {
        return 0U;
    }

    if(vision_beacon_should_run_full_detect() == 0U)
    {
        *beacon_out = s_beacon_detect_cache;
        vision_filter_beacon_by_target_policy(target_result, beacon_out);
        return 0U;
    }

    vision_apply_beacon_ignore_from_target(target_result);
    algo_beacon_process(s_gray_image);
    vision_copy_current_beacon_result(beacon_out);
    vision_filter_beacon_by_target_policy(target_result, beacon_out);

#if HW_VISION_TARGET_SHAPE == HW_VISION_TARGET_SHAPE_ONE
    /* ONE mode always detects beacons on the unmasked frame. Its compactness
     * and periodicity gates separate beacons from bar fragments without
     * erasing the space between two cable-split lamp segments. */
    vision_beacon_detect_cache_store(beacon_out);
    return vision_beacon_full_detect_flags(beacon_out);
#else

    if((target_result == 0U) || (target_result->found == 0U))
    {
        vision_beacon_detect_cache_store(beacon_out);
        return vision_beacon_full_detect_flags(beacon_out);
    }

    if((beacon_out->found != 0U) && (beacon_out->hold_active == 0U))
    {
        const float primary_gap = vision_point_gap_to_rect(beacon_out->center_x,
                                                            beacon_out->center_y,
                                                            target_result->bbox_x,
                                                            target_result->bbox_y,
                                                            target_result->bbox_w,
                                                            target_result->bbox_h);
        if(primary_gap <= VISION_BEACON_NEAR_V_PROXIMITY_PX)
        {
            vision_beacon_detect_cache_store(beacon_out);
            return vision_beacon_full_detect_flags(beacon_out);
        }
    }

    /* Recover a real beacon that is adjacent to, or partially overlaps, the
     * V bbox in the same camera frame. The final unmasked pass remains gated
     * by vision_beacon_is_close_independent_target() below. */
    algo_beacon_set_ignore_rect(target_result->bbox_x,
                                target_result->bbox_y,
                                target_result->bbox_w,
                                target_result->bbox_h,
                                0U);
    algo_beacon_process(s_gray_image);
    vision_copy_current_beacon_result(&retry_result);
    vision_filter_beacon_by_target_policy(target_result, &retry_result);
    if((retry_result.found != 0U) && (retry_result.hold_active == 0U))
    {
        *beacon_out = retry_result;
        vision_apply_beacon_ignore_from_target(target_result);
        vision_beacon_detect_cache_store(beacon_out);
        return vision_beacon_full_detect_flags(beacon_out);
    }

    algo_beacon_clear_ignore_rect();
    algo_beacon_process(s_gray_image);
    vision_copy_current_beacon_result(&retry_result);
    vision_filter_beacon_by_target_policy(target_result, &retry_result);
    if((retry_result.found != 0U) && (retry_result.hold_active == 0U)
       && (vision_beacon_is_close_independent_target(target_result, &retry_result) != 0U))
    {
        *beacon_out = retry_result;
    }
    else if(retry_result.found != 0U)
    {
        algo_beacon_reset();
    }
    vision_apply_beacon_ignore_from_target(target_result);
    vision_beacon_detect_cache_store(beacon_out);
    return vision_beacon_full_detect_flags(beacon_out);
#endif
}

static uint8 vision_recover_and_resolve_display_targets(algo_vision_target_result_t *target_result,
                                                        algo_beacon_result_t *beacon_result)
{
    (void)vision_apply_target_beacon_decision(target_result, beacon_result);
    vision_clamp_target_bbox_to_limit(target_result);
    vision_update_target_recent_track(target_result, 1U);
    return 0U;
}

static void vision_prepare_and_resolve_control_target(const algo_vision_target_result_t *display_result,
                                                      algo_beacon_result_t *beacon_result,
                                                      algo_vision_target_result_t *control_result)
{
    vision_prepare_target_control(display_result,
                                        beacon_result,
                                        control_result);
    (void)vision_apply_target_beacon_decision(control_result, beacon_result);
}

/* 漫射候选临时放行检查: 晕开的大面积信标绕过 V 灯附属光排除策略,
 * 但必须离 V 灯足够远(中心距大于6px 且不在 V 框内), 尺寸有硬上限,
 * 防止 V 灯本体的晕开亮斑借该通道混入候选列表。 */
static uint8 vision_beacon_provisional_candidate_ok(
    const algo_vision_target_result_t *target_result,
    const algo_beacon_result_t *candidate)
{
    float dx;
    float dy;
    uint16 b_long;

    if((candidate == 0U) || (candidate->found == 0U)
       || ((candidate->zone_flags & ALGO_BEACON_ZONE_FLAG_DIFFUSE_CANDIDATE) == 0U))
    {
        return 0U;
    }

    b_long = vision_rect_long_side(candidate->bbox_w, candidate->bbox_h);
    if((candidate->area == 0U) || (candidate->area > 700U) || (b_long > 44U))
    {
        return 0U;
    }

    if((target_result == 0U) || (target_result->found == 0U)) { return 1U; }
    dx = candidate->center_x - target_result->center_x;
    dy = candidate->center_y - target_result->center_y;
    if(((dx * dx) + (dy * dy)) <= 36.0f) { return 0U; }
    if((candidate->center_x >= (float)target_result->bbox_x)
       && (candidate->center_x <= (float)(target_result->bbox_x + target_result->bbox_w - 1U))
       && (candidate->center_y >= (float)target_result->bbox_y)
       && (candidate->center_y <= (float)(target_result->bbox_y + target_result->bbox_h - 1U)))
    {
        return 0U;
    }

    return 1U;
}

static void vision_prepare_beacon_candidate_list(const algo_vision_target_result_t *target_result,
                                                 const algo_beacon_result_t *primary_result,
                                                 uint8 detect_flags,
                                                 algo_beacon_result_list_t *list_out)
{
    const algo_beacon_result_list_t *raw_list;
    algo_beacon_result_list_t raw_snapshot;
    algo_beacon_result_t approved_primary;
    uint8 i;

    if(list_out == 0U)
    {
        return;
    }
    memset(list_out, 0, sizeof(*list_out));
    if((detect_flags & VISION_BEACON_DETECT_FLAG_FULL) == 0U)
    {
        return;
    }

    list_out->full_detect = 1U;
    memset(&approved_primary, 0, sizeof(approved_primary));
    if(primary_result != 0U)
    {
        approved_primary = *primary_result;
        vision_filter_beacon_by_target_policy(target_result, &approved_primary);
        if((approved_primary.found != 0U)
           && (vision_decide_target_beacon(target_result, &approved_primary) == VISION_TARGET_BEACON_KEEP_TARGET))
        {
            memset(&approved_primary, 0, sizeof(approved_primary));
        }
    }
    if(((detect_flags & VISION_BEACON_DETECT_FLAG_FRESH) != 0U)
       && (approved_primary.found != 0U)
       && (approved_primary.hold_active == 0U)
       && (vision_beacon_publish_features_ok(&approved_primary) != 0U))
    {
        list_out->items[0] = approved_primary;
        list_out->count = 1U;
    }

    raw_list = algo_beacon_get_candidates();
    if(raw_list == 0U)
    {
        return;
    }
    raw_snapshot = *raw_list;

    for(i = 0U; (i < raw_snapshot.count) && (list_out->count < ALGO_BEACON_MAX_CANDIDATES); i++)
    {
        algo_beacon_result_t candidate = raw_snapshot.items[i];
        uint8 duplicate = 0U;
        uint8 j;
        uint8 provisional_diffuse = ((candidate.zone_flags
            & ALGO_BEACON_ZONE_FLAG_DIFFUSE_CANDIDATE) != 0U) ? 1U : 0U;

        if(provisional_diffuse != 0U)
        {
            /* 漫射候选走临时放行通道: 不做 V 灯附属光排除(晕开信标
             * 常被误判成附属光), 也不套常规发布几何门, 只靠临时检查把关。 */
            if(vision_beacon_provisional_candidate_ok(target_result, &candidate) == 0U)
            {
                memset(&candidate, 0, sizeof(candidate));
            }
        }
        else
        {
            vision_filter_beacon_by_target_policy(target_result, &candidate);
            if((candidate.found != 0U)
               && (vision_decide_target_beacon(target_result, &candidate) == VISION_TARGET_BEACON_KEEP_TARGET))
            {
                memset(&candidate, 0, sizeof(candidate));
            }
        }
        if((candidate.found != 0U)
           && (candidate.hold_active == 0U)
           && ((provisional_diffuse != 0U)
               || (vision_beacon_publish_features_ok(&candidate) != 0U)))
        {
            for(j = 0U; j < list_out->count; j++)
            {
                float dx = candidate.center_x - list_out->items[j].center_x;
                float dy = candidate.center_y - list_out->items[j].center_y;
                if(sqrtf((dx * dx) + (dy * dy)) <= 8.0f)
                {
                    duplicate = 1U;
                    break;
                }
            }
            if(duplicate == 0U)
            {
                list_out->items[list_out->count++] = candidate;
            }
        }
    }
}

void algo_vision_target_pipeline_init(void)
{
    s_gray_image = 0;
    s_frame_id = 0U;
    s_target_recent_hold = 0U;
    s_target_recent_center_x = 0.0f;
    s_target_recent_center_y = 0.0f;
    s_target_recent_heading_x = 1.0f;
    s_target_recent_heading_y = 0.0f;
    s_target_recent_bbox_x = 0U;
    s_target_recent_bbox_y = 0U;
    s_target_recent_bbox_w = 0U;
    s_target_recent_bbox_h = 0U;
    vision_target_control_reset();
    vision_beacon_publish_reset();
    vision_beacon_detect_cache_reset();
    algo_beacon_activity_reset();
    algo_beacon_reset();
    algo_vision_target_detector_reset();
}

void algo_vision_target_pipeline_update(const uint8 *gray_image,
                                        uint32 frame_id,
                                        algo_vision_target_pipeline_result_t *result_out)
{
    uint8 beacon_detect_flags;

    if(result_out == 0) { return; }
    memset(result_out, 0, sizeof(*result_out));
    if(gray_image == 0) { return; }

    s_gray_image = gray_image;
    s_frame_id = frame_id;
    vision_detect_main_target(&result_out->target_display);
    beacon_detect_flags = vision_detect_beacon_target(&result_out->target_display,
                                                       &result_out->beacon_display);
    (void)vision_recover_and_resolve_display_targets(&result_out->target_display,
                                                     &result_out->beacon_display);
    vision_prepare_and_resolve_control_target(&result_out->target_display,
                                               &result_out->beacon_display,
                                               &result_out->target_control);
    vision_prepare_beacon_candidate_list(&result_out->target_display,
                                         &result_out->beacon_display,
                                         beacon_detect_flags,
                                         &result_out->beacon_candidates);
    algo_beacon_activity_update(gray_image,
                                &result_out->beacon_candidates,
                                ((beacon_detect_flags & VISION_BEACON_DETECT_FLAG_FULL) != 0U) ? 1U : 0U);
    algo_beacon_activity_filter_list(&result_out->beacon_candidates);
    if((result_out->beacon_display.found != 0U)
       && (algo_beacon_activity_is_confirmed(&result_out->beacon_display) == 0U))
    {
        memset(&result_out->beacon_display, 0, sizeof(result_out->beacon_display));
    }
    vision_filter_beacon_for_publish(&result_out->beacon_display,
                                     beacon_detect_flags,
                                     &result_out->beacon_publish);
}
