#include "algo_onelamp.h"

#include <math.h>
#include <string.h>

#define ALGO_ONELAMP_QUEUE_CAP             (4096U)
#define ALGO_ONELAMP_MAX_BLOBS             (12U)
#define ALGO_ONELAMP_BRIGHT_REF_RANK       (12U)
#define ALGO_ONELAMP_COMPONENT_AREA_MIN    (6U)
#define ALGO_ONELAMP_AREA_MIN              (10U)
#define ALGO_ONELAMP_AREA_MAX              (3000U)
#define ALGO_ONELAMP_LONG_MIN              (10U)
#define ALGO_ONELAMP_LONG_MAX              (96U)
#define ALGO_ONELAMP_AXIS_RATIO_MIN        (2.40f)
#define ALGO_ONELAMP_FILL_MIN              (0.08f)
#define ALGO_ONELAMP_FILL_MAX              (0.96f)
#define ALGO_ONELAMP_STRONG_MIN            (3U)
#define ALGO_ONELAMP_TRACK_DIST_MAX_PX     (24.0f)
#define ALGO_ONELAMP_PENDING_DIST_PX       (8.0f)
#define ALGO_ONELAMP_ACQUIRE_HITS          (2U)
#define ALGO_ONELAMP_SEGMENT_AXIS_MIN      (1.60f)
#define ALGO_ONELAMP_MERGE_ALIGN_MIN       (0.90f)
#define ALGO_ONELAMP_MERGE_PERP_MAX_PX     (6.0f)
#define ALGO_ONELAMP_MERGE_GAP_MAX_PX      (20.0f)
#define ALGO_ONELAMP_HOLD_FRAMES           (3U)
#define ALGO_ONELAMP_SCORE_MIN             (0.43f)

static uint8 s_binary[MT9V03X_H][MT9V03X_W];
static uint16 s_queue_x[ALGO_ONELAMP_QUEUE_CAP];
static uint16 s_queue_y[ALGO_ONELAMP_QUEUE_CAP];
static algo_vision_target_result_t s_result;
static algo_vision_target_result_t s_last;
static algo_vision_target_result_t s_pending;
static uint8 s_track_valid;
static uint8 s_hold_left;
static uint8 s_pending_valid;
static uint8 s_pending_hits;
static uint8 s_threshold_strong;
static uint8 s_component_overflow;

typedef struct
{
    uint16 min_x;
    uint16 min_y;
    uint16 max_x;
    uint16 max_y;
    uint16 area;
    uint16 strong_area;
    uint16 peak;
    uint32 sum_x;
    uint32 sum_y;
    float sum_xx;
    float sum_yy;
    float sum_xy;
} algo_onelamp_blob_t;

typedef struct
{
    float center_x;
    float center_y;
    float heading_x;
    float heading_y;
    float axis_ratio;
    float half_length;
} algo_onelamp_geometry_t;

static float algo_onelamp_clamp(float value, float min_value, float max_value)
{
    if(value < min_value) { return min_value; }
    if(value > max_value) { return max_value; }
    return value;
}

static uint8 algo_onelamp_u8(float value)
{
    return (uint8)(algo_onelamp_clamp(value, 0.0f, 255.0f) + 0.5f);
}

static uint16 algo_onelamp_long(uint16 w, uint16 h)
{
    return (w > h) ? w : h;
}

static void algo_onelamp_build_binary(const uint8 *src_image)
{
    uint16 histogram[256];
    uint32 sum = 0U;
    uint16 ranked = 0U;
    uint16 x;
    uint16 y;
    int16 gray;
    uint8 gray_max = 0U;
    uint8 bright_ref = 0U;
    float mean;
    float threshold;

    memset(s_binary, 0, sizeof(s_binary));
    memset(histogram, 0, sizeof(histogram));
    for(y = 0U; y < MT9V03X_H; y++)
    {
        for(x = 0U; x < MT9V03X_W; x++)
        {
            uint8 value = src_image[(uint32)y * MT9V03X_W + x];
            sum += value;
            if(histogram[value] < 65535U) { histogram[value]++; }
            if(value > gray_max) { gray_max = value; }
        }
    }
    mean = (float)sum / ((float)MT9V03X_W * (float)MT9V03X_H);
    for(gray = 255; gray >= 0; gray--)
    {
        ranked = (uint16)(ranked + histogram[gray]);
        if(ranked >= ALGO_ONELAMP_BRIGHT_REF_RANK)
        {
            bright_ref = (uint8)gray;
            break;
        }
    }
    if(bright_ref == 0U) { bright_ref = gray_max; }

    threshold = (float)bright_ref * 0.68f;
    if((mean + 18.0f) > threshold) { threshold = mean + 18.0f; }
    threshold = algo_onelamp_clamp(threshold, 65.0f, 210.0f);
    s_threshold_strong = algo_onelamp_u8(algo_onelamp_clamp((float)bright_ref * 0.84f,
                                                            threshold + 12.0f,
                                                            235.0f));
    for(y = 0U; y < MT9V03X_H; y++)
    {
        for(x = 0U; x < MT9V03X_W; x++)
        {
            if(src_image[(uint32)y * MT9V03X_W + x] >= (uint8)threshold)
            {
                s_binary[y][x] = 1U;
            }
        }
    }
}

static uint8 algo_onelamp_collect(const uint8 *src_image,
                                  uint16 start_x,
                                  uint16 start_y,
                                  algo_onelamp_blob_t *blob)
{
    uint16 head = 0U;
    uint16 tail = 0U;

    memset(blob, 0, sizeof(*blob));
    s_binary[start_y][start_x] = 2U;
    s_queue_x[tail] = start_x;
    s_queue_y[tail] = start_y;
    tail++;
    while(head < tail)
    {
        uint16 x = s_queue_x[head];
        uint16 y = s_queue_y[head];
        uint8 gray = src_image[(uint32)y * MT9V03X_W + x];
        int8 dy;
        int8 dx;
        head++;

        if(blob->area == 0U)
        {
            blob->min_x = blob->max_x = x;
            blob->min_y = blob->max_y = y;
        }
        else
        {
            if(x < blob->min_x) { blob->min_x = x; }
            if(x > blob->max_x) { blob->max_x = x; }
            if(y < blob->min_y) { blob->min_y = y; }
            if(y > blob->max_y) { blob->max_y = y; }
        }
        blob->area++;
        blob->sum_x += x;
        blob->sum_y += y;
        blob->sum_xx += (float)x * (float)x;
        blob->sum_yy += (float)y * (float)y;
        blob->sum_xy += (float)x * (float)y;
        if(gray > blob->peak) { blob->peak = gray; }
        if(gray >= s_threshold_strong) { blob->strong_area++; }

        for(dy = -1; dy <= 1; dy++)
        {
            for(dx = -1; dx <= 1; dx++)
            {
                int32 nx;
                int32 ny;
                if((dx == 0) && (dy == 0)) { continue; }
                nx = (int32)x + dx;
                ny = (int32)y + dy;
                if((nx < 0) || (ny < 0) || (nx >= MT9V03X_W) || (ny >= MT9V03X_H)) { continue; }
                if(s_binary[ny][nx] != 1U) { continue; }
                s_binary[ny][nx] = 2U;
                if(tail >= ALGO_ONELAMP_QUEUE_CAP)
                {
                    s_component_overflow = 1U;
                    return 0U;
                }
                s_queue_x[tail] = (uint16)nx;
                s_queue_y[tail] = (uint16)ny;
                tail++;
            }
        }
    }
    return ((blob->area >= ALGO_ONELAMP_COMPONENT_AREA_MIN)
            && (blob->area <= ALGO_ONELAMP_AREA_MAX)) ? 1U : 0U;
}

static uint8 algo_onelamp_geometry(const algo_onelamp_blob_t *blob,
                                   algo_onelamp_geometry_t *geometry)
{
    float mx;
    float my;
    float var_x;
    float var_y;
    float cov_xy;
    float trace;
    float disc;
    float lambda1;
    float lambda2;
    float hx;
    float hy;
    float norm;

    if((blob == 0) || (geometry == 0) || (blob->area == 0U))
    {
        return 0U;
    }
    mx = (float)blob->sum_x / (float)blob->area;
    my = (float)blob->sum_y / (float)blob->area;
    var_x = (blob->sum_xx / (float)blob->area) - (mx * mx);
    var_y = (blob->sum_yy / (float)blob->area) - (my * my);
    cov_xy = (blob->sum_xy / (float)blob->area) - (mx * my);
    trace = var_x + var_y;
    disc = sqrtf(((var_x - var_y) * (var_x - var_y)) + (4.0f * cov_xy * cov_xy));
    lambda1 = 0.5f * (trace + disc);
    lambda2 = 0.5f * (trace - disc);
    if(lambda1 < 0.0f) { lambda1 = 0.0f; }
    if(lambda2 < 0.0f) { lambda2 = 0.0f; }
    if(fabsf(cov_xy) > 0.001f) { hx = cov_xy; hy = lambda1 - var_x; }
    else if(var_x >= var_y) { hx = 1.0f; hy = 0.0f; }
    else { hx = 0.0f; hy = 1.0f; }
    norm = sqrtf((hx * hx) + (hy * hy));
    if(norm < 0.001f) { hx = 1.0f; hy = 0.0f; norm = 1.0f; }

    geometry->center_x = mx;
    geometry->center_y = my;
    geometry->heading_x = hx / norm;
    geometry->heading_y = hy / norm;
    geometry->axis_ratio = sqrtf((lambda1 + 0.25f) / (lambda2 + 0.25f));
    geometry->half_length = sqrtf(3.0f * lambda1) + 1.0f;
    return 1U;
}

static void algo_onelamp_merge_blobs(const algo_onelamp_blob_t *a,
                                     const algo_onelamp_blob_t *b,
                                     algo_onelamp_blob_t *merged)
{
    memset(merged, 0, sizeof(*merged));
    merged->min_x = (a->min_x < b->min_x) ? a->min_x : b->min_x;
    merged->min_y = (a->min_y < b->min_y) ? a->min_y : b->min_y;
    merged->max_x = (a->max_x > b->max_x) ? a->max_x : b->max_x;
    merged->max_y = (a->max_y > b->max_y) ? a->max_y : b->max_y;
    merged->area = (uint16)(a->area + b->area);
    merged->strong_area = (uint16)(a->strong_area + b->strong_area);
    merged->peak = (a->peak > b->peak) ? a->peak : b->peak;
    merged->sum_x = a->sum_x + b->sum_x;
    merged->sum_y = a->sum_y + b->sum_y;
    merged->sum_xx = a->sum_xx + b->sum_xx;
    merged->sum_yy = a->sum_yy + b->sum_yy;
    merged->sum_xy = a->sum_xy + b->sum_xy;
}

static uint8 algo_onelamp_segments_mergeable(const algo_onelamp_blob_t *a,
                                              const algo_onelamp_blob_t *b)
{
    algo_onelamp_geometry_t ga;
    algo_onelamp_geometry_t gb;
    float dot;
    float dx;
    float dy;
    float along;
    float perp;

    if((algo_onelamp_geometry(a, &ga) == 0U)
       || (algo_onelamp_geometry(b, &gb) == 0U)
       || (ga.axis_ratio < ALGO_ONELAMP_SEGMENT_AXIS_MIN)
       || (gb.axis_ratio < ALGO_ONELAMP_SEGMENT_AXIS_MIN))
    {
        return 0U;
    }
    dot = fabsf((ga.heading_x * gb.heading_x) + (ga.heading_y * gb.heading_y));
    if(dot < ALGO_ONELAMP_MERGE_ALIGN_MIN)
    {
        return 0U;
    }
    dx = gb.center_x - ga.center_x;
    dy = gb.center_y - ga.center_y;
    along = fabsf((dx * ga.heading_x) + (dy * ga.heading_y));
    perp = fabsf((-dx * ga.heading_y) + (dy * ga.heading_x));
    if(perp > ALGO_ONELAMP_MERGE_PERP_MAX_PX)
    {
        return 0U;
    }
    return (along <= (ga.half_length + gb.half_length + ALGO_ONELAMP_MERGE_GAP_MAX_PX)) ? 1U : 0U;
}

static float algo_onelamp_score(const algo_onelamp_blob_t *blob,
                                algo_vision_target_result_t *candidate)
{
    uint16 w = (uint16)(blob->max_x - blob->min_x + 1U);
    uint16 h = (uint16)(blob->max_y - blob->min_y + 1U);
    uint16 long_side = algo_onelamp_long(w, h);
    algo_onelamp_geometry_t geometry;
    float fill;
    float elongation_score;
    float fill_score;
    float area_score;
    float strong_score;
    float score;

    if((blob->area < ALGO_ONELAMP_AREA_MIN) || (blob->area > ALGO_ONELAMP_AREA_MAX)
       || (long_side < ALGO_ONELAMP_LONG_MIN) || (long_side > ALGO_ONELAMP_LONG_MAX)
       || (blob->strong_area < ALGO_ONELAMP_STRONG_MIN)
       || (algo_onelamp_geometry(blob, &geometry) == 0U))
    {
        return 0.0f;
    }
    fill = (float)blob->area / ((float)w * (float)h);
    if((geometry.axis_ratio < ALGO_ONELAMP_AXIS_RATIO_MIN)
       || (fill < ALGO_ONELAMP_FILL_MIN) || (fill > ALGO_ONELAMP_FILL_MAX))
    {
        return 0.0f;
    }
    elongation_score = algo_onelamp_clamp((geometry.axis_ratio - ALGO_ONELAMP_AXIS_RATIO_MIN) / 3.0f,
                                           0.0f,
                                           1.0f);
    fill_score = 1.0f - algo_onelamp_clamp(fabsf(fill - 0.54f) / 0.38f, 0.0f, 1.0f);
    area_score = algo_onelamp_clamp(((float)blob->area - 8.0f) / 180.0f, 0.0f, 1.0f);
    strong_score = algo_onelamp_clamp((float)blob->strong_area / 20.0f, 0.0f, 1.0f);
    score = (elongation_score * 0.38f) + (fill_score * 0.22f)
          + (area_score * 0.18f) + (strong_score * 0.14f) + 0.08f;
    if(s_track_valid != 0U)
    {
        float dx = geometry.center_x - s_last.center_x;
        float dy = geometry.center_y - s_last.center_y;
        float dist = sqrtf((dx * dx) + (dy * dy));
        if(dist <= ALGO_ONELAMP_TRACK_DIST_MAX_PX)
        {
            score += (1.0f - (dist / ALGO_ONELAMP_TRACK_DIST_MAX_PX)) * 0.12f;
        }
    }
    if(score < ALGO_ONELAMP_SCORE_MIN) { return 0.0f; }

    memset(candidate, 0, sizeof(*candidate));
    candidate->found = 1U;
    candidate->source = ALGO_VISION_TARGET_SOURCE_MEASUREMENT;
    candidate->center_x = geometry.center_x;
    candidate->center_y = geometry.center_y;
    candidate->heading_x = geometry.heading_x;
    candidate->heading_y = geometry.heading_y;
    candidate->heading_cdeg = (int16)(atan2f(geometry.heading_y, geometry.heading_x) * 5729.57795f);
    candidate->bbox_x = blob->min_x;
    candidate->bbox_y = blob->min_y;
    candidate->bbox_w = w;
    candidate->bbox_h = h;
    candidate->area = blob->area;
    candidate->confidence_permille = (uint16)algo_onelamp_clamp(score * 1000.0f, 0.0f, 1000.0f);
    return score;
}

static float algo_onelamp_result_distance(const algo_vision_target_result_t *a,
                                          const algo_vision_target_result_t *b)
{
    float dx = a->center_x - b->center_x;
    float dy = a->center_y - b->center_y;
    return sqrtf((dx * dx) + (dy * dy));
}

static void algo_onelamp_clear_pending(void)
{
    memset(&s_pending, 0, sizeof(s_pending));
    s_pending_valid = 0U;
    s_pending_hits = 0U;
}

static void algo_onelamp_commit(const algo_vision_target_result_t *candidate)
{
    s_result = *candidate;
    s_last = *candidate;
    s_track_valid = 1U;
    s_hold_left = ALGO_ONELAMP_HOLD_FRAMES;
    algo_onelamp_clear_pending();
}

static uint8 algo_onelamp_confirm_new_candidate(const algo_vision_target_result_t *candidate)
{
    if((s_pending_valid != 0U)
       && (algo_onelamp_result_distance(candidate, &s_pending) <= ALGO_ONELAMP_PENDING_DIST_PX))
    {
        if(s_pending_hits < 255U) { s_pending_hits++; }
        s_pending = *candidate;
    }
    else
    {
        s_pending = *candidate;
        s_pending_valid = 1U;
        s_pending_hits = 1U;
    }
    if(s_pending_hits >= ALGO_ONELAMP_ACQUIRE_HITS)
    {
        algo_onelamp_commit(&s_pending);
        return 1U;
    }
    return 0U;
}

void algo_onelamp_reset(void)
{
    memset(&s_result, 0, sizeof(s_result));
    memset(&s_last, 0, sizeof(s_last));
    s_track_valid = 0U;
    s_hold_left = 0U;
    algo_onelamp_clear_pending();
}

void algo_onelamp_process(const uint8 *src_image)
{
    algo_onelamp_blob_t blobs[ALGO_ONELAMP_MAX_BLOBS];
    uint8 blob_count = 0U;
    algo_vision_target_result_t best;
    algo_vision_target_result_t tracked;
    float best_score = 0.0f;
    float tracked_score = 0.0f;
    float tracked_distance = 1000000.0f;
    uint8 i;
    uint8 j;
    uint16 x;
    uint16 y;

    memset(&s_result, 0, sizeof(s_result));
    memset(&best, 0, sizeof(best));
    memset(&tracked, 0, sizeof(tracked));
    memset(blobs, 0, sizeof(blobs));
    if(src_image == 0) { return; }
    algo_onelamp_build_binary(src_image);
    s_component_overflow = 0U;
    for(y = 0U; y < MT9V03X_H; y++)
    {
        for(x = 0U; x < MT9V03X_W; x++)
        {
            if(s_binary[y][x] == 1U)
            {
                algo_onelamp_blob_t blob;
                if(algo_onelamp_collect(src_image, x, y, &blob) != 0U)
                {
                    if(blob_count < ALGO_ONELAMP_MAX_BLOBS)
                    {
                        blobs[blob_count++] = blob;
                    }
                }
                if(s_component_overflow != 0U)
                {
                    /* A saturated region is not a valid bar. Stop this frame
                     * rather than rescanning its unqueued tail as fake blobs. */
                    memset(&best, 0, sizeof(best));
                    best_score = 0.0f;
                    y = MT9V03X_H;
                    break;
                }
            }
        }
    }

    if(s_component_overflow != 0U)
    {
        blob_count = 0U;
    }

    for(i = 0U; i < blob_count; i++)
    {
        algo_vision_target_result_t candidate;
        float score;
        memset(&candidate, 0, sizeof(candidate));
        score = algo_onelamp_score(&blobs[i], &candidate);
        if(score > best_score)
        {
            best_score = score;
            best = candidate;
        }
        if((s_track_valid != 0U) && (candidate.found != 0U))
        {
            float distance = algo_onelamp_result_distance(&candidate, &s_last);
            if((distance <= ALGO_ONELAMP_TRACK_DIST_MAX_PX)
               && ((distance < tracked_distance)
                   || ((fabsf(distance - tracked_distance) < 0.5f) && (score > tracked_score))))
            {
                tracked = candidate;
                tracked_score = score;
                tracked_distance = distance;
            }
        }
        for(j = (uint8)(i + 1U); j < blob_count; j++)
        {
            algo_onelamp_blob_t merged;
            if(algo_onelamp_segments_mergeable(&blobs[i], &blobs[j]) != 0U)
            {
                algo_vision_target_result_t merged_candidate;
                float merged_score;
                memset(&merged_candidate, 0, sizeof(merged_candidate));
                algo_onelamp_merge_blobs(&blobs[i], &blobs[j], &merged);
                merged_score = algo_onelamp_score(&merged, &merged_candidate);
                if(merged_score > 0.0f)
                {
                    merged_score += 0.05f;
                    if(merged_score > best_score)
                    {
                        best_score = merged_score;
                        best = merged_candidate;
                    }
                    if(s_track_valid != 0U)
                    {
                        float distance = algo_onelamp_result_distance(&merged_candidate, &s_last);
                        if((distance <= ALGO_ONELAMP_TRACK_DIST_MAX_PX)
                           && ((distance < tracked_distance)
                               || ((fabsf(distance - tracked_distance) < 0.5f)
                                   && (merged_score > tracked_score))))
                        {
                            tracked = merged_candidate;
                            tracked_score = merged_score;
                            tracked_distance = distance;
                        }
                    }
                }
            }
        }
    }

    if(tracked.found != 0U)
    {
        algo_onelamp_commit(&tracked);
        return;
    }
    if(best.found != 0U)
    {
        if(algo_onelamp_confirm_new_candidate(&best) != 0U)
        {
            return;
        }
    }
    else
    {
        algo_onelamp_clear_pending();
    }
    if((s_track_valid != 0U) && (s_hold_left > 0U))
    {
        s_result = s_last;
        s_result.hold_active = 1U;
        s_result.source = ALGO_VISION_TARGET_SOURCE_PURE_HOLD;
        s_hold_left--;
        return;
    }
    memset(&s_last, 0, sizeof(s_last));
    s_track_valid = 0U;
    algo_onelamp_clear_pending();
}

const algo_vision_target_result_t *algo_onelamp_get_result(void)
{
    return &s_result;
}
