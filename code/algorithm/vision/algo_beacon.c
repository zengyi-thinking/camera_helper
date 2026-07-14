#include "algo_beacon.h"

#include <math.h>
#include <string.h>

#define ALGO_BEACON_MAX_BLOBS             (36U)
#define ALGO_BEACON_QUEUE_CAP             (1024U)
#define ALGO_BEACON_AREA_MIN              (3U)
#define ALGO_BEACON_AREA_MAX              (360U)
#define ALGO_BEACON_LONG_MAX_CENTER       (24U)
#define ALGO_BEACON_LONG_MAX_EDGE         (30U)
#define ALGO_BEACON_VLAMP_SIZE_LONG_CENTER (20U)
#define ALGO_BEACON_VLAMP_SIZE_AREA_CENTER (90U)
#define ALGO_BEACON_VLAMP_SIZE_LONG_EDGE  (13U)
#define ALGO_BEACON_VLAMP_SIZE_AREA_EDGE  (36U)
/* V 灯碎片拒绝门槛 (OR 路径): V 灯在画面边缘部分出界被截断后, 面积仍远大于
 * 信标。信标典型面积 10~50, V 碎片即使截断也 >60。
 * 边缘区: 面积>=60 直接拒 (边缘信标面积很少超 50, V 碎片截断后 >60)。
 * 中心区: 长边>=16 且 面积>=70 (长边门槛保护大块扩散信标, 其长边可能到 15)。 */
#define ALGO_BEACON_VLAMP_FRAGMENT_AREA_EDGE   (50U)
#define ALGO_BEACON_VLAMP_FRAGMENT_LONG_CENTER (16U)
#define ALGO_BEACON_VLAMP_FRAGMENT_AREA_CENTER (60U)
#define ALGO_BEACON_VLAMP_SMALL_LONG           (10U)
#define ALGO_BEACON_VLAMP_SMALL_AREA           (18U)
#define ALGO_BEACON_VLAMP_SMALL_RATIO          (1.45f)
#define ALGO_BEACON_VLAMP_SMALL_AXIS           (1.60f)
#define ALGO_BEACON_VLAMP_SMALL_SPARSE_FILL    (0.42f)
#define ALGO_BEACON_EDGE_MARGIN           (20U)
#define ALGO_BEACON_EDGE_RETRY_FACTOR     (0.42f)
#define ALGO_BEACON_EDGE_RETRY_ADD        (26.0f)
#define ALGO_BEACON_EDGE_RETRY_MIN        (80.0f)
#define ALGO_BEACON_EDGE_RETRY_MAX        (190.0f)
#define ALGO_BEACON_FAR_RADIUS            (5)
#define ALGO_BEACON_FAR_CORE_RADIUS       (1)
#define ALGO_BEACON_FAR_AREA_MAX          (18U)
#define ALGO_BEACON_FAR_LONG_MAX          (7U)
#define ALGO_BEACON_TRACK_DIST_MAX_PX     (18.0f)
/* 信标丢失后短时 hold 保持: 上一帧结果延续若干帧, 抗瞬时遮挡/闪烁断流。
 * 重构后从旧版 8 帧砍到 2 帧过于激进(40ms 就清零), 提到 4 帧(80ms@50Hz)
 * 兼顾抗断流与不过度保持假目标。hold 期间置信度渐进衰减(见 process)。 */
#define ALGO_BEACON_HOLD_FRAMES           (4U)
#define ALGO_BEACON_CONF_CAP_UNCONFIRMED  (400U)
#define ALGO_BEACON_CONF_FLOOR_CONFIRMED  (700U)
#define ALGO_BEACON_FLICKER_HIST_LEN      (24U)
#define ALGO_BEACON_FLICKER_HIST_MIN      (14U)

typedef struct
{
    uint8 valid;
    uint8 source;
    uint8 edge_near;
    uint16 min_x;
    uint16 min_y;
    uint16 max_x;
    uint16 max_y;
    uint16 area;
    uint16 peak;
    uint32 sum_x;
    uint32 sum_y;
    uint32 sum_gray;
    float sum_xx;
    float sum_yy;
    float sum_xy;
    float center_x;
    float center_y;
    float fill;
    float ratio;
    float axis_ratio;
    float score;
} algo_beacon_candidate_t;

static uint8 s_binary[MT9V03X_H][MT9V03X_W];
static uint16 s_queue_x[ALGO_BEACON_QUEUE_CAP];
static uint16 s_queue_y[ALGO_BEACON_QUEUE_CAP];
static algo_beacon_result_t s_result;
static algo_beacon_result_t s_last_result;
static uint8 s_hold_left = 0U;
static uint8 s_ignore_enabled = 0U;
static uint16 s_ignore_x0 = 0U;
static uint16 s_ignore_y0 = 0U;
static uint16 s_ignore_x1 = 0U;
static uint16 s_ignore_y1 = 0U;
static uint8 s_threshold_center = 0U;
static uint8 s_threshold_edge = 0U;
static uint8 s_flicker_valid = 0U;
static float s_flicker_x = 0.0f;
static float s_flicker_y = 0.0f;
static float s_flicker_base = 0.0f;
static float s_flicker_hist[ALGO_BEACON_FLICKER_HIST_LEN];
static uint8 s_flicker_count = 0U;
static uint8 s_flicker_index = 0U;
static uint8 s_flicker_confirmed = 0U;

static float algo_beacon_clamp_float(float value, float min_value, float max_value)
{
    if(value < min_value) { return min_value; }
    if(value > max_value) { return max_value; }
    return value;
}

static uint8 algo_beacon_clamp_u8(float value)
{
    value = algo_beacon_clamp_float(value, 0.0f, 255.0f);
    return (uint8)(value + 0.5f);
}

static uint16 algo_beacon_long_side(uint16 w, uint16 h)
{
    return (w > h) ? w : h;
}

static uint16 algo_beacon_short_side(uint16 w, uint16 h)
{
    return (w > h) ? h : w;
}

static void algo_beacon_clear_result(void)
{
    memset(&s_result, 0, sizeof(s_result));
}

static uint8 algo_beacon_near_edge(uint16 x, uint16 y)
{
    return ((x <= ALGO_BEACON_EDGE_MARGIN)
            || (y <= ALGO_BEACON_EDGE_MARGIN)
            || ((x + ALGO_BEACON_EDGE_MARGIN) >= (MT9V03X_W - 1U))
            || ((y + ALGO_BEACON_EDGE_MARGIN) >= (MT9V03X_H - 1U))) ? 1U : 0U;
}

static uint8 algo_beacon_in_ignore_rect(uint16 x, uint16 y)
{
    if(s_ignore_enabled == 0U)
    {
        return 0U;
    }

    return ((x >= s_ignore_x0) && (x <= s_ignore_x1)
            && (y >= s_ignore_y0) && (y <= s_ignore_y1)) ? 1U : 0U;
}

static uint8 algo_beacon_zone_flags(uint8 source, uint8 edge_near, uint8 period_confirmed)
{
    uint8 flags = 0U;
    if(source == ALGO_BEACON_RESULT_SOURCE_FAR_POINT)
    {
        flags |= ALGO_BEACON_ZONE_FLAG_FAR_POINT;
    }
    if(edge_near != 0U)
    {
        flags |= ALGO_BEACON_ZONE_FLAG_EDGE;
    }
    if(period_confirmed != 0U)
    {
        flags |= ALGO_BEACON_ZONE_FLAG_PERIOD_CONFIRMED;
    }
    return flags;
}

static void algo_beacon_reset_flicker(void)
{
    s_flicker_valid = 0U;
    s_flicker_x = 0.0f;
    s_flicker_y = 0.0f;
    s_flicker_base = 0.0f;
    memset(s_flicker_hist, 0, sizeof(s_flicker_hist));
    s_flicker_count = 0U;
    s_flicker_index = 0U;
    s_flicker_confirmed = 0U;
}

static void algo_beacon_build_binary(const uint8 *src_image, uint8 edge_retry)
{
    uint32 sum = 0U;
    uint8 gray_max = 0U;
    float mean;
    float threshold;
    uint16 x;
    uint16 y;

    memset(s_binary, 0, sizeof(s_binary));
    if(src_image == 0)
    {
        return;
    }

    for(y = 0U; y < MT9V03X_H; y++)
    {
        for(x = 0U; x < MT9V03X_W; x++)
        {
            uint8 gray = src_image[(uint32)y * MT9V03X_W + x];
            sum += gray;
            if(gray > gray_max) { gray_max = gray; }
        }
    }

    mean = (float)sum / ((float)MT9V03X_W * (float)MT9V03X_H);
    threshold = (float)gray_max * 0.62f;
    if((mean + 44.0f) > threshold) { threshold = mean + 44.0f; }
    if(threshold < 150.0f) { threshold = 150.0f; }
    if(threshold > 220.0f) { threshold = 220.0f; }
    s_threshold_center = algo_beacon_clamp_u8(threshold);

    if(edge_retry != 0U)
    {
        threshold = (float)gray_max * ALGO_BEACON_EDGE_RETRY_FACTOR;
        if((mean + ALGO_BEACON_EDGE_RETRY_ADD) > threshold)
        {
            threshold = mean + ALGO_BEACON_EDGE_RETRY_ADD;
        }
        if(threshold < ALGO_BEACON_EDGE_RETRY_MIN) { threshold = ALGO_BEACON_EDGE_RETRY_MIN; }
        if(threshold > ALGO_BEACON_EDGE_RETRY_MAX) { threshold = ALGO_BEACON_EDGE_RETRY_MAX; }
    }
    else
    {
        threshold = (float)gray_max * 0.30f;
        if((mean + 18.0f) > threshold) { threshold = mean + 18.0f; }
        if(threshold < 58.0f) { threshold = 58.0f; }
        if(threshold > 155.0f) { threshold = 155.0f; }
    }
    s_threshold_edge = algo_beacon_clamp_u8(threshold);

    for(y = 0U; y < MT9V03X_H; y++)
    {
        for(x = 0U; x < MT9V03X_W; x++)
        {
            uint8 threshold_px = (algo_beacon_near_edge(x, y) != 0U) ? s_threshold_edge : s_threshold_center;
            if((algo_beacon_in_ignore_rect(x, y) == 0U)
               && (src_image[(uint32)y * MT9V03X_W + x] >= threshold_px))
            {
                s_binary[y][x] = 1U;
            }
        }
    }
}

static void algo_beacon_accum_candidate(algo_beacon_candidate_t *candidate,
                                        uint16 x,
                                        uint16 y,
                                        uint8 gray)
{
    if(candidate->area == 0U)
    {
        candidate->min_x = x;
        candidate->max_x = x;
        candidate->min_y = y;
        candidate->max_y = y;
    }
    else
    {
        if(x < candidate->min_x) { candidate->min_x = x; }
        if(x > candidate->max_x) { candidate->max_x = x; }
        if(y < candidate->min_y) { candidate->min_y = y; }
        if(y > candidate->max_y) { candidate->max_y = y; }
    }

    candidate->area++;
    candidate->sum_x += x;
    candidate->sum_y += y;
    candidate->sum_gray += gray;
    candidate->sum_xx += (float)x * (float)x;
    candidate->sum_yy += (float)y * (float)y;
    candidate->sum_xy += (float)x * (float)y;
    if(gray > candidate->peak) { candidate->peak = gray; }
}

static void algo_beacon_finish_candidate(algo_beacon_candidate_t *candidate)
{
    uint16 w;
    uint16 h;
    uint16 short_side;
    float inv_area;
    float mx;
    float my;
    float var_x;
    float var_y;
    float cov_xy;
    float trace;
    float disc;
    float lambda1;
    float lambda2;

    if((candidate == 0) || (candidate->area == 0U))
    {
        return;
    }

    w = (uint16)(candidate->max_x - candidate->min_x + 1U);
    h = (uint16)(candidate->max_y - candidate->min_y + 1U);
    short_side = algo_beacon_short_side(w, h);
    inv_area = 1.0f / (float)candidate->area;
    mx = (float)candidate->sum_x * inv_area;
    my = (float)candidate->sum_y * inv_area;
    var_x = (candidate->sum_xx * inv_area) - (mx * mx);
    var_y = (candidate->sum_yy * inv_area) - (my * my);
    cov_xy = (candidate->sum_xy * inv_area) - (mx * my);
    trace = var_x + var_y;
    disc = sqrtf(((var_x - var_y) * (var_x - var_y)) + (4.0f * cov_xy * cov_xy));
    lambda1 = 0.5f * (trace + disc);
    lambda2 = 0.5f * (trace - disc);

    candidate->center_x = mx;
    candidate->center_y = my;
    candidate->fill = (float)candidate->area / ((float)w * (float)h);
    candidate->ratio = (short_side > 0U) ? ((float)algo_beacon_long_side(w, h) / (float)short_side) : 999.0f;
    candidate->axis_ratio = (lambda2 > 0.05f) ? sqrtf(lambda1 / lambda2) : 8.0f;
    candidate->edge_near = (algo_beacon_near_edge(candidate->min_x, candidate->min_y)
                            || algo_beacon_near_edge(candidate->max_x, candidate->max_y)) ? 1U : 0U;
    candidate->valid = 1U;
}

static uint8 algo_beacon_collect_component(const uint8 *src_image,
                                           uint16 start_x,
                                           uint16 start_y,
                                           algo_beacon_candidate_t *candidate_out)
{
    uint16 head = 0U;
    uint16 tail = 0U;
    uint8 overflow = 0U;

    if((src_image == 0) || (candidate_out == 0))
    {
        return 0U;
    }

    memset(candidate_out, 0, sizeof(*candidate_out));
    s_binary[start_y][start_x] = 2U;
    s_queue_x[tail] = start_x;
    s_queue_y[tail] = start_y;
    tail++;

    while(head < tail)
    {
        uint16 x = s_queue_x[head];
        uint16 y = s_queue_y[head];
        int8 dy;
        int8 dx;
        uint8 gray = src_image[(uint32)y * MT9V03X_W + x];

        head++;
        algo_beacon_accum_candidate(candidate_out, x, y, gray);

        for(dy = -1; dy <= 1; dy++)
        {
            int16 ny = (int16)y + dy;
            if((ny < 0) || (ny >= (int16)MT9V03X_H)) { continue; }
            for(dx = -1; dx <= 1; dx++)
            {
                int16 nx = (int16)x + dx;
                if((dx == 0) && (dy == 0)) { continue; }
                if((nx < 0) || (nx >= (int16)MT9V03X_W)) { continue; }
                if(s_binary[ny][nx] != 1U) { continue; }
                s_binary[ny][nx] = 2U;
                if(tail < ALGO_BEACON_QUEUE_CAP)
                {
                    s_queue_x[tail] = (uint16)nx;
                    s_queue_y[tail] = (uint16)ny;
                    tail++;
                }
                else
                {
                    overflow = 1U;
                }
            }
        }
    }

    if(overflow != 0U)
    {
        return 0U;
    }

    algo_beacon_finish_candidate(candidate_out);
    candidate_out->source = ALGO_BEACON_RESULT_SOURCE_BLOB;
    return (candidate_out->valid != 0U) ? 1U : 0U;
}

static uint8 algo_beacon_blob_valid(algo_beacon_candidate_t *candidate)
{
    uint16 w;
    uint16 h;
    uint16 long_side;
    uint16 max_long;
    float ratio_max;
    float fill_min;
    float area_score;
    float round_score;
    float fill_score;
    float peak_score;
    float shape_score;

    if((candidate == 0) || (candidate->valid == 0U))
    {
        return 0U;
    }

    w = (uint16)(candidate->max_x - candidate->min_x + 1U);
    h = (uint16)(candidate->max_y - candidate->min_y + 1U);
    long_side = algo_beacon_long_side(w, h);
    max_long = (candidate->edge_near != 0U) ? ALGO_BEACON_LONG_MAX_EDGE : ALGO_BEACON_LONG_MAX_CENTER;
    ratio_max = (candidate->edge_near != 0U) ? 2.65f : 2.10f;
    fill_min = (candidate->edge_near != 0U) ? 0.16f : 0.24f;

    /* V 灯拒绝: V 灯是面光源 (实测 bbox 约 41x25, 面积 600+), 即使在画面边缘
     * 部分出界被截断, 残余碎片面积也远大于信标 (信标典型面积 10~50)。
     * 双路拒绝:
     * A) 主门槛 (大块 V 整体): 长边>=LONG 且 面积>=AREA;
     * B) 碎片门槛 (截断 V): 面积>=FRAG_AREA 且 长边>=FRAG_LONG。
     *    中心区 V 碎片即使截断面积也>70 (V 灯 600+ 截大半仍 >70),
     *    长边>=16 过滤掉真信标 (信标典型长边 <14)。
     *    边缘区放宽: 面积>=60 且 长边>=14 (边缘信标长边可达 14 但面积通常<50)。 */
    if(candidate->edge_near != 0U)
    {
        if(((long_side >= ALGO_BEACON_VLAMP_SIZE_LONG_EDGE)
            && (candidate->area >= ALGO_BEACON_VLAMP_SIZE_AREA_EDGE))
           || (candidate->area >= ALGO_BEACON_VLAMP_FRAGMENT_AREA_EDGE))
        {
            return 0U;
        }
    }
    else
    {
        if(((long_side >= ALGO_BEACON_VLAMP_SIZE_LONG_CENTER)
            && (candidate->area >= ALGO_BEACON_VLAMP_SIZE_AREA_CENTER))
           || ((long_side >= ALGO_BEACON_VLAMP_FRAGMENT_LONG_CENTER)
               && (candidate->area >= ALGO_BEACON_VLAMP_FRAGMENT_AREA_CENTER)))
        {
            return 0U;
        }
    }

    /* 1.3m 后 V 灯可能从大结构塌缩为中等小光板, 尺寸不再触发上面的大 V
     * 拒绝, 但形态仍比信标更拉伸/更疏。未经过 10Hz 确认前, 这类小 V-like
     * blob 不应进入信标候选；圆润柔光信标会保留。 */
    if((long_side >= ALGO_BEACON_VLAMP_SMALL_LONG)
       && (candidate->area >= ALGO_BEACON_VLAMP_SMALL_AREA)
       && ((candidate->edge_near == 0U)
           || ((candidate->area >= 30U) && (long_side >= 13U)))
       && ((candidate->ratio >= ALGO_BEACON_VLAMP_SMALL_RATIO)
           || (candidate->axis_ratio >= ALGO_BEACON_VLAMP_SMALL_AXIS)
           || ((candidate->fill <= ALGO_BEACON_VLAMP_SMALL_SPARSE_FILL)
               && (candidate->axis_ratio >= 1.30f))))
    {
        return 0U;
    }

    if((candidate->area < ALGO_BEACON_AREA_MIN)
       || (candidate->area > ALGO_BEACON_AREA_MAX)
       || (long_side > max_long)
       || (candidate->ratio > ratio_max)
       || (candidate->fill < fill_min)
       || (candidate->fill > 0.99f)
       || (candidate->axis_ratio > ((candidate->edge_near != 0U) ? 3.3f : 2.4f)))
    {
        return 0U;
    }

    if((long_side >= 22U) && (candidate->area >= 95U) && (candidate->ratio > 1.75f))
    {
        return 0U;
    }

    area_score = 1.0f - (fabsf((float)candidate->area - (candidate->edge_near ? 30.0f : 24.0f))
                          / (candidate->edge_near ? 68.0f : 58.0f));
    area_score = algo_beacon_clamp_float(area_score, 0.0f, 1.0f);
    round_score = algo_beacon_clamp_float((ratio_max - candidate->ratio) / (ratio_max - 1.0f), 0.0f, 1.0f);
    fill_score = algo_beacon_clamp_float((candidate->fill - fill_min) / 0.46f, 0.0f, 1.0f);
    peak_score = algo_beacon_clamp_float(((float)candidate->peak - (float)(candidate->edge_near ? s_threshold_edge : s_threshold_center)) / 95.0f,
                                         0.0f,
                                         1.0f);
    shape_score = (area_score * 0.26f)
                + (round_score * 0.28f)
                + (fill_score * 0.24f)
                + (peak_score * 0.22f);
    if(candidate->edge_near != 0U)
    {
        shape_score += 0.04f;
    }

    candidate->score = shape_score;
    return (shape_score >= 0.32f) ? 1U : 0U;
}

static uint8 algo_beacon_local_max(const uint8 *src_image, uint16 cx, uint16 cy)
{
    int8 dx;
    int8 dy;
    uint8 center;

    center = src_image[(uint32)cy * MT9V03X_W + cx];
    for(dy = -1; dy <= 1; dy++)
    {
        int16 y = (int16)cy + dy;
        if((y < 0) || (y >= (int16)MT9V03X_H)) { continue; }
        for(dx = -1; dx <= 1; dx++)
        {
            int16 x = (int16)cx + dx;
            if((dx == 0) && (dy == 0)) { continue; }
            if((x < 0) || (x >= (int16)MT9V03X_W)) { continue; }
            if(src_image[(uint32)y * MT9V03X_W + (uint16)x] > center)
            {
                return 0U;
            }
        }
    }
    return 1U;
}

static uint8 algo_beacon_collect_far_point(const uint8 *src_image,
                                           uint16 cx,
                                           uint16 cy,
                                           algo_beacon_candidate_t *candidate_out)
{
    uint8 peak;
    uint8 peak_min;
    float contrast_min;
    float score_min;
    float threshold_factor;
    float threshold_add;
    float bg_sum = 0.0f;
    float bg_count = 0.0f;
    float bg_mean;
    float contrast;
    float threshold_f;
    uint8 threshold;
    int16 x0;
    int16 y0;
    int16 x1;
    int16 y1;
    int16 x;
    int16 y;
    uint16 area = 0U;
    uint32 sum_x = 0U;
    uint32 sum_y = 0U;
    uint32 sum_gray = 0U;
    uint16 min_x = MT9V03X_W;
    uint16 min_y = MT9V03X_H;
    uint16 max_x = 0U;
    uint16 max_y = 0U;
    uint16 long_side;
    uint8 edge_near;
    float contrast_score;
    float peak_score;
    float area_score;
    float score;

    if((src_image == 0) || (candidate_out == 0)
       || (algo_beacon_in_ignore_rect(cx, cy) != 0U))
    {
        return 0U;
    }

    peak = src_image[(uint32)cy * MT9V03X_W + cx];
    edge_near = algo_beacon_near_edge(cx, cy);
    if(edge_near != 0U)
    {
        peak_min = 50U;
        contrast_min = 12.0f;
        score_min = 0.34f;
        threshold_factor = 0.34f;
        threshold_add = 5.0f;
    }
    else
    {
        peak_min = 64U;
        contrast_min = 18.0f;
        score_min = 0.42f;
        threshold_factor = 0.38f;
        threshold_add = 7.0f;
    }

    if((peak < peak_min) || (algo_beacon_local_max(src_image, cx, cy) == 0U))
    {
        return 0U;
    }

    x0 = (int16)cx - ALGO_BEACON_FAR_RADIUS;
    y0 = (int16)cy - ALGO_BEACON_FAR_RADIUS;
    x1 = (int16)cx + ALGO_BEACON_FAR_RADIUS;
    y1 = (int16)cy + ALGO_BEACON_FAR_RADIUS;
    if(x0 < 0) { x0 = 0; }
    if(y0 < 0) { y0 = 0; }
    if(x1 >= (int16)MT9V03X_W) { x1 = (int16)MT9V03X_W - 1; }
    if(y1 >= (int16)MT9V03X_H) { y1 = (int16)MT9V03X_H - 1; }

    for(y = y0; y <= y1; y++)
    {
        for(x = x0; x <= x1; x++)
        {
            int16 dx = x - (int16)cx;
            int16 dy = y - (int16)cy;
            if((dx >= -ALGO_BEACON_FAR_CORE_RADIUS) && (dx <= ALGO_BEACON_FAR_CORE_RADIUS)
               && (dy >= -ALGO_BEACON_FAR_CORE_RADIUS) && (dy <= ALGO_BEACON_FAR_CORE_RADIUS))
            {
                continue;
            }
            bg_sum += (float)src_image[(uint32)y * MT9V03X_W + (uint16)x];
            bg_count += 1.0f;
        }
    }

    if(bg_count < 1.0f)
    {
        return 0U;
    }

    bg_mean = bg_sum / bg_count;
    contrast = (float)peak - bg_mean;
    if(contrast < contrast_min)
    {
        return 0U;
    }

    threshold_f = ((float)peak * threshold_factor);
    if((bg_mean + threshold_add) > threshold_f)
    {
        threshold_f = bg_mean + threshold_add;
    }
    threshold = algo_beacon_clamp_u8(threshold_f);

    for(y = y0; y <= y1; y++)
    {
        for(x = x0; x <= x1; x++)
        {
            uint8 gray = src_image[(uint32)y * MT9V03X_W + (uint16)x];
            if((algo_beacon_in_ignore_rect((uint16)x, (uint16)y) == 0U) && (gray >= threshold))
            {
                area++;
                sum_x += (uint16)x;
                sum_y += (uint16)y;
                sum_gray += gray;
                if((uint16)x < min_x) { min_x = (uint16)x; }
                if((uint16)x > max_x) { max_x = (uint16)x; }
                if((uint16)y < min_y) { min_y = (uint16)y; }
                if((uint16)y > max_y) { max_y = (uint16)y; }
            }
        }
    }

    if((area == 0U) || (area > ALGO_BEACON_FAR_AREA_MAX) || (sum_gray == 0U))
    {
        return 0U;
    }

    long_side = algo_beacon_long_side((uint16)(max_x - min_x + 1U), (uint16)(max_y - min_y + 1U));
    if(long_side > ALGO_BEACON_FAR_LONG_MAX)
    {
        return 0U;
    }

    contrast_score = algo_beacon_clamp_float((contrast - contrast_min) / 45.0f, 0.0f, 1.0f);
    peak_score = algo_beacon_clamp_float(((float)peak - (float)peak_min) / 115.0f, 0.0f, 1.0f);
    area_score = algo_beacon_clamp_float((float)(ALGO_BEACON_FAR_AREA_MAX + 1U - area) / (float)ALGO_BEACON_FAR_AREA_MAX,
                                         0.0f,
                                         1.0f);
    score = (contrast_score * 0.38f) + (peak_score * 0.42f) + (area_score * 0.20f);
    if(score < score_min)
    {
        return 0U;
    }

    memset(candidate_out, 0, sizeof(*candidate_out));
    candidate_out->valid = 1U;
    candidate_out->source = ALGO_BEACON_RESULT_SOURCE_FAR_POINT;
    candidate_out->edge_near = edge_near;
    candidate_out->min_x = min_x;
    candidate_out->min_y = min_y;
    candidate_out->max_x = max_x;
    candidate_out->max_y = max_y;
    candidate_out->area = area;
    candidate_out->peak = peak;
    candidate_out->sum_x = sum_x;
    candidate_out->sum_y = sum_y;
    candidate_out->sum_gray = sum_gray;
    candidate_out->center_x = (float)sum_x / (float)area;
    candidate_out->center_y = (float)sum_y / (float)area;
    candidate_out->fill = 1.0f;
    candidate_out->ratio = 1.0f;
    candidate_out->axis_ratio = 1.0f;
    candidate_out->score = score + ((edge_near != 0U) ? 0.04f : 0.0f);
    return 1U;
}

static uint8 algo_beacon_detect_blob(const uint8 *src_image,
                                     uint8 edge_retry,
                                     algo_beacon_candidate_t *best_out)
{
    uint16 x;
    uint16 y;
    uint8 found = 0U;

    memset(best_out, 0, sizeof(*best_out));
    algo_beacon_build_binary(src_image, edge_retry);

    for(y = 0U; y < MT9V03X_H; y++)
    {
        for(x = 0U; x < MT9V03X_W; x++)
        {
            if(s_binary[y][x] == 1U)
            {
                algo_beacon_candidate_t candidate;
                if((algo_beacon_collect_component(src_image, x, y, &candidate) != 0U)
                   && (algo_beacon_blob_valid(&candidate) != 0U))
                {
                    if((found == 0U) || (candidate.score > best_out->score))
                    {
                        *best_out = candidate;
                        found = 1U;
                    }
                }
            }
        }
    }

    return found;
}

static uint8 algo_beacon_detect_far(const uint8 *src_image, algo_beacon_candidate_t *best_out)
{
    uint16 x;
    uint16 y;
    uint8 found = 0U;

    memset(best_out, 0, sizeof(*best_out));
    if(src_image == 0)
    {
        return 0U;
    }

    for(y = 0U; y < MT9V03X_H; y++)
    {
        for(x = 0U; x < MT9V03X_W; x++)
        {
            uint8 peak_min = (algo_beacon_near_edge(x, y) != 0U) ? 50U : 64U;
            if(src_image[(uint32)y * MT9V03X_W + x] < peak_min)
            {
                continue;
            }
            {
                algo_beacon_candidate_t candidate;
                if(algo_beacon_collect_far_point(src_image, x, y, &candidate) != 0U)
                {
                    if((found == 0U) || (candidate.score > best_out->score))
                    {
                        *best_out = candidate;
                        found = 1U;
                    }
                }
            }
        }
    }

    return found;
}

static float algo_beacon_sample_roi(const uint8 *src_image, float cx, float cy)
{
    int16 ix = (int16)(cx + 0.5f);
    int16 iy = (int16)(cy + 0.5f);
    int16 x0 = ix - 3;
    int16 y0 = iy - 3;
    int16 x1 = ix + 3;
    int16 y1 = iy + 3;
    int16 x;
    int16 y;
    uint32 sum = 0U;
    uint16 count = 0U;

    if(src_image == 0)
    {
        return 0.0f;
    }

    if(x0 < 0) { x0 = 0; }
    if(y0 < 0) { y0 = 0; }
    if(x1 >= (int16)MT9V03X_W) { x1 = (int16)MT9V03X_W - 1; }
    if(y1 >= (int16)MT9V03X_H) { y1 = (int16)MT9V03X_H - 1; }

    for(y = y0; y <= y1; y++)
    {
        for(x = x0; x <= x1; x++)
        {
            sum += src_image[(uint32)y * MT9V03X_W + (uint16)x];
            count++;
        }
    }

    return (count > 0U) ? ((float)sum / (float)count) : 0.0f;
}

static uint8 algo_beacon_update_flicker(const uint8 *src_image, const algo_beacon_candidate_t *candidate)
{
    float sample;
    float delta;
    float dx;
    float dy;
    float min_delta = 999.0f;
    float max_delta = -999.0f;
    uint8 i;
    uint8 crossings = 0U;
    int8 last_sign = 0;

    if((src_image == 0) || (candidate == 0) || (candidate->valid == 0U))
    {
        return 0U;
    }

    if(s_flicker_valid != 0U)
    {
        dx = candidate->center_x - s_flicker_x;
        dy = candidate->center_y - s_flicker_y;
        if(sqrtf((dx * dx) + (dy * dy)) > ALGO_BEACON_TRACK_DIST_MAX_PX)
        {
            algo_beacon_reset_flicker();
        }
    }

    sample = algo_beacon_sample_roi(src_image, candidate->center_x, candidate->center_y);
    if(s_flicker_valid == 0U)
    {
        s_flicker_valid = 1U;
        s_flicker_x = candidate->center_x;
        s_flicker_y = candidate->center_y;
        s_flicker_base = sample;
        s_flicker_count = 0U;
        s_flicker_index = 0U;
        s_flicker_confirmed = 0U;
    }

    delta = sample - s_flicker_base;
    s_flicker_base += (sample - s_flicker_base) * 0.12f;
    s_flicker_x += (candidate->center_x - s_flicker_x) * 0.35f;
    s_flicker_y += (candidate->center_y - s_flicker_y) * 0.35f;
    s_flicker_hist[s_flicker_index] = delta;
    s_flicker_index = (uint8)((s_flicker_index + 1U) % ALGO_BEACON_FLICKER_HIST_LEN);
    if(s_flicker_count < ALGO_BEACON_FLICKER_HIST_LEN)
    {
        s_flicker_count++;
    }

    if(s_flicker_count < ALGO_BEACON_FLICKER_HIST_MIN)
    {
        return s_flicker_confirmed;
    }

    for(i = 0U; i < s_flicker_count; i++)
    {
        float v = s_flicker_hist[i];
        int8 sign = 0;
        if(v > 3.0f) { sign = 1; }
        else if(v < -3.0f) { sign = -1; }
        if(v < min_delta) { min_delta = v; }
        if(v > max_delta) { max_delta = v; }
        if(sign != 0)
        {
            if((last_sign != 0) && (sign != last_sign))
            {
                crossings++;
            }
            last_sign = sign;
        }
    }

    s_flicker_confirmed = (((max_delta - min_delta) >= 8.0f)
                           && (crossings >= 3U)
                           && (crossings <= 14U)) ? 1U : 0U;
    return s_flicker_confirmed;
}

static void algo_beacon_commit_candidate(const uint8 *src_image, const algo_beacon_candidate_t *candidate)
{
    uint8 period_confirmed;
    uint32 confidence;

    memset(&s_result, 0, sizeof(s_result));
    if((candidate == 0) || (candidate->valid == 0U))
    {
        return;
    }

    period_confirmed = algo_beacon_update_flicker(src_image, candidate);
    confidence = (uint32)(candidate->score * 760.0f) + 90U;
    if(period_confirmed != 0U)
    {
        confidence += 200U;
        if(confidence < ALGO_BEACON_CONF_FLOOR_CONFIRMED)
        {
            confidence = ALGO_BEACON_CONF_FLOOR_CONFIRMED;
        }
    }
    else if(confidence > ALGO_BEACON_CONF_CAP_UNCONFIRMED)
    {
        confidence = ALGO_BEACON_CONF_CAP_UNCONFIRMED;
    }
    if(confidence > 1000U)
    {
        confidence = 1000U;
    }

    s_result.found = 1U;
    s_result.source = candidate->source;
    s_result.zone_flags = algo_beacon_zone_flags(candidate->source,
                                                 candidate->edge_near,
                                                 period_confirmed);
    s_result.center_x = candidate->center_x;
    s_result.center_y = candidate->center_y;
    s_result.bbox_x = candidate->min_x;
    s_result.bbox_y = candidate->min_y;
    s_result.bbox_w = (uint16_t)(candidate->max_x - candidate->min_x + 1U);
    s_result.bbox_h = (uint16_t)(candidate->max_y - candidate->min_y + 1U);
    s_result.area = candidate->area;
    s_result.confidence_permille = (uint16_t)confidence;
    s_last_result = s_result;
    s_hold_left = ALGO_BEACON_HOLD_FRAMES;
}

void algo_beacon_reset(void)
{
    algo_beacon_clear_result();
    memset(&s_last_result, 0, sizeof(s_last_result));
    s_hold_left = 0U;
    algo_beacon_reset_flicker();
    memset(s_binary, 0, sizeof(s_binary));
}

void algo_beacon_set_ignore_rect(uint16_t x,
                                 uint16_t y,
                                 uint16_t w,
                                 uint16_t h,
                                 uint16_t margin)
{
    uint32 x0;
    uint32 y0;
    uint32 x1;
    uint32 y1;

    if((w == 0U) || (h == 0U))
    {
        algo_beacon_clear_ignore_rect();
        return;
    }

    x0 = (x > margin) ? (uint32)(x - margin) : 0U;
    y0 = (y > margin) ? (uint32)(y - margin) : 0U;
    x1 = (uint32)x + (uint32)w - 1U + (uint32)margin;
    y1 = (uint32)y + (uint32)h - 1U + (uint32)margin;
    if(x1 >= MT9V03X_W) { x1 = MT9V03X_W - 1U; }
    if(y1 >= MT9V03X_H) { y1 = MT9V03X_H - 1U; }

    s_ignore_enabled = 1U;
    s_ignore_x0 = (uint16)x0;
    s_ignore_y0 = (uint16)y0;
    s_ignore_x1 = (uint16)x1;
    s_ignore_y1 = (uint16)y1;
}

void algo_beacon_clear_ignore_rect(void)
{
    s_ignore_enabled = 0U;
    s_ignore_x0 = 0U;
    s_ignore_y0 = 0U;
    s_ignore_x1 = 0U;
    s_ignore_y1 = 0U;
}

void algo_beacon_process(const uint8 *src_image)
{
    algo_beacon_candidate_t blob_candidate;
    algo_beacon_candidate_t far_candidate;
    algo_beacon_candidate_t best_candidate;
    uint8 blob_found;
    uint8 far_found;

    memset(&blob_candidate, 0, sizeof(blob_candidate));
    memset(&far_candidate, 0, sizeof(far_candidate));
    memset(&best_candidate, 0, sizeof(best_candidate));

    if(src_image == 0)
    {
        algo_beacon_clear_result();
        return;
    }

    blob_found = algo_beacon_detect_blob(src_image, 0U, &blob_candidate);
    far_found = algo_beacon_detect_far(src_image, &far_candidate);
    if((blob_found == 0U) && (far_found == 0U))
    {
        /* 低边缘阈值可能把灯核与背景连成大块。仅在首轮完全失败时，
         * 用较高边缘阈值再分割一次；中心区阈值和形状规则保持不变。 */
        blob_found = algo_beacon_detect_blob(src_image, 1U, &blob_candidate);
    }

    if((blob_found != 0U) && (far_found != 0U))
    {
        best_candidate = (far_candidate.score > (blob_candidate.score + 0.08f))
                       ? far_candidate
                       : blob_candidate;
    }
    else if(blob_found != 0U)
    {
        best_candidate = blob_candidate;
    }
    else if(far_found != 0U)
    {
        best_candidate = far_candidate;
    }

    if(best_candidate.valid != 0U)
    {
        algo_beacon_commit_candidate(src_image, &best_candidate);
        return;
    }

    if((s_hold_left > 0U) && (s_last_result.found != 0U))
    {
        s_result = s_last_result;
        s_result.hold_active = 1U;
        if((s_result.zone_flags & ALGO_BEACON_ZONE_FLAG_PERIOD_CONFIRMED) == 0U)
        {
            /* 未确认信标 hold 期间渐进衰减: 第1帧压到340, 之后每帧再降40,
             * 让下游 publish confirm 自然淘汰过期 hold (低于发布门360时不再发布)。
             * 已确认(period_confirmed)信标不衰减, 保持满置信度跨闪烁间隙。 */
            uint32 hold_cap;
            hold_cap = 340U - ((ALGO_BEACON_HOLD_FRAMES - s_hold_left) * 40U);
            s_result.confidence_permille = (s_result.confidence_permille > hold_cap) ? (uint16)hold_cap : s_result.confidence_permille;
        }
        s_hold_left--;
        return;
    }

    algo_beacon_clear_result();
}

const algo_beacon_result_t *algo_beacon_get_result(void)
{
    return &s_result;
}
