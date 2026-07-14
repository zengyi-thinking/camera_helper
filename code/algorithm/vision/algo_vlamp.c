#include "algo_vlamp.h"

#include <math.h>
#include <string.h>

#define ALGO_VLAMP_MAX_BLOBS             (40U)
#define ALGO_VLAMP_QUEUE_CAP             (4096U)
#define ALGO_VLAMP_AREA_MIN              (5U)
#define ALGO_VLAMP_AREA_MAX              (3000U)
#define ALGO_VLAMP_MERGE_GAP_PX          (24U)
#define ALGO_VLAMP_MIN_LONG_SIDE         (8U)
#define ALGO_VLAMP_COMPACT_LONG_MAX      (24U)
#define ALGO_VLAMP_COMPACT_FILL_MIN      (0.45f)
#define ALGO_VLAMP_COMPACT_RATIO_MAX     (1.65f)
#define ALGO_VLAMP_DEFORMED_LONG_MIN     (12U)
#define ALGO_VLAMP_DEFORMED_AREA_MIN     (12U)
#define ALGO_VLAMP_SPLIT_PART_LONG_MIN   (12U)
#define ALGO_VLAMP_SPLIT_PART_AREA_MIN   (14U)
#define ALGO_VLAMP_SPLIT_PART_STRONG_MIN (3U)
#define ALGO_VLAMP_PLATE_LONG_MIN        (11U)
#define ALGO_VLAMP_PLATE_AREA_MIN        (20U)
#define ALGO_VLAMP_PLATE_STRONG_MIN      (4U)
#define ALGO_VLAMP_PLATE_RATIO_MIN       (1.25f)
#define ALGO_VLAMP_PLATE_AXIS_MIN        (1.32f)
#define ALGO_VLAMP_PLATE_SPARSE_FILL_MAX (0.46f)
#define ALGO_VLAMP_TRACK_DIST_MAX_PX     (50.0f)
#define ALGO_VLAMP_HOLD_FRAMES           (5U)
#define ALGO_VLAMP_BRIGHT_REF_RANK       (12U)
#define ALGO_VLAMP_OUTLINE_LONG_MIN      (10U)
#define ALGO_VLAMP_OUTLINE_AREA_MIN      (12U)
#define ALGO_VLAMP_OUTLINE_STRONG_MIN    (4U)
#define ALGO_VLAMP_OUTLINE_SKEW_MIN      (0.16f)
#define ALGO_VLAMP_RECOVER_MARGIN_PX     (16U)
#define ALGO_VLAMP_RECOVER_CENTER_MAX_PX (28.0f)
#define ALGO_VLAMP_RECOVER_CONF_MIN      (320U)

typedef struct
{
    uint8 valid;
    uint16 min_x;
    uint16 min_y;
    uint16 max_x;
    uint16 max_y;
    uint16 area;
    uint16 strong_area;
    uint16 peak;
    uint32 sum_x;
    uint32 sum_y;
    uint32 sum_gray;
    float sum_xx;
    float sum_yy;
    float sum_xy;
    float sum_xxx;
    float sum_yyy;
    float sum_xxy;
    float sum_xyy;
    float center_x;
    float center_y;
    float fill;
    float ratio;
    float axis_ratio;
    float heading_x;
    float heading_y;
} algo_vlamp_blob_t;

static uint8 s_binary[MT9V03X_H][MT9V03X_W];
static uint16 s_queue_x[ALGO_VLAMP_QUEUE_CAP];
static uint16 s_queue_y[ALGO_VLAMP_QUEUE_CAP];
static algo_vlamp_result_t s_result;
static algo_vlamp_result_t s_last_main;
static uint8 s_track_valid = 0U;
static uint8 s_hold_left = 0U;
static uint8 s_threshold_low = 0U;
static uint8 s_threshold_strong = 0U;

static float algo_vlamp_clamp_float(float value, float min_value, float max_value)
{
    if(value < min_value) { return min_value; }
    if(value > max_value) { return max_value; }
    return value;
}

static uint8 algo_vlamp_clamp_u8(float value)
{
    value = algo_vlamp_clamp_float(value, 0.0f, 255.0f);
    return (uint8)(value + 0.5f);
}

static void algo_vlamp_clear_result(void)
{
    memset(&s_result, 0, sizeof(s_result));
}

static void algo_vlamp_reset_track(void)
{
    memset(&s_last_main, 0, sizeof(s_last_main));
    s_track_valid = 0U;
    s_hold_left = 0U;
}

static uint16 algo_vlamp_long_side(uint16 w, uint16 h)
{
    return (w > h) ? w : h;
}

static uint16 algo_vlamp_short_side(uint16 w, uint16 h)
{
    return (w > h) ? h : w;
}

static uint8 algo_vlamp_blob_compact(const algo_vlamp_blob_t *blob)
{
    uint16 w;
    uint16 h;
    uint16 long_side;

    if((blob == 0) || (blob->valid == 0U))
    {
        return 1U;
    }

    w = (uint16)(blob->max_x - blob->min_x + 1U);
    h = (uint16)(blob->max_y - blob->min_y + 1U);
    long_side = algo_vlamp_long_side(w, h);

    return ((long_side <= ALGO_VLAMP_COMPACT_LONG_MAX)
            && (blob->fill >= ALGO_VLAMP_COMPACT_FILL_MIN)
            && (blob->ratio <= ALGO_VLAMP_COMPACT_RATIO_MAX)) ? 1U : 0U;
}

static uint8 algo_vlamp_blob_near_track(const algo_vlamp_blob_t *blob, float max_dist_px)
{
    float dx;
    float dy;
    float dist;

    if((blob == 0) || (blob->valid == 0U) || (s_track_valid == 0U))
    {
        return 0U;
    }

    dx = blob->center_x - s_last_main.center_x;
    dy = blob->center_y - s_last_main.center_y;
    dist = sqrtf((dx * dx) + (dy * dy));
    return (dist <= max_dist_px) ? 1U : 0U;
}

static uint8 algo_vlamp_blob_deformed_vlike(const algo_vlamp_blob_t *blob)
{
    uint16 w;
    uint16 h;
    uint16 long_side;

    if((blob == 0) || (blob->valid == 0U))
    {
        return 0U;
    }

    w = (uint16)(blob->max_x - blob->min_x + 1U);
    h = (uint16)(blob->max_y - blob->min_y + 1U);
    long_side = algo_vlamp_long_side(w, h);

    return ((long_side >= ALGO_VLAMP_DEFORMED_LONG_MIN)
            && (blob->area >= ALGO_VLAMP_DEFORMED_AREA_MIN)
            && (algo_vlamp_blob_near_track(blob, ALGO_VLAMP_TRACK_DIST_MAX_PX) != 0U)) ? 1U : 0U;
}

static uint8 algo_vlamp_blob_split_part_like(const algo_vlamp_blob_t *blob)
{
    uint16 w;
    uint16 h;
    uint16 long_side;

    if((blob == 0) || (blob->valid == 0U))
    {
        return 0U;
    }

    w = (uint16)(blob->max_x - blob->min_x + 1U);
    h = (uint16)(blob->max_y - blob->min_y + 1U);
    long_side = algo_vlamp_long_side(w, h);

    return ((long_side >= ALGO_VLAMP_SPLIT_PART_LONG_MIN)
            && (blob->area >= ALGO_VLAMP_SPLIT_PART_AREA_MIN)
            && (blob->strong_area >= ALGO_VLAMP_SPLIT_PART_STRONG_MIN)) ? 1U : 0U;
}

static uint8 algo_vlamp_blob_plate_like(const algo_vlamp_blob_t *blob)
{
    uint16 w;
    uint16 h;
    uint16 long_side;

    if((blob == 0) || (blob->valid == 0U))
    {
        return 0U;
    }

    w = (uint16)(blob->max_x - blob->min_x + 1U);
    h = (uint16)(blob->max_y - blob->min_y + 1U);
    long_side = algo_vlamp_long_side(w, h);
    if((long_side < ALGO_VLAMP_PLATE_LONG_MIN)
       || (blob->area < ALGO_VLAMP_PLATE_AREA_MIN)
       || (blob->strong_area < ALGO_VLAMP_PLATE_STRONG_MIN))
    {
        return 0U;
    }

    if((blob->ratio >= ALGO_VLAMP_PLATE_RATIO_MIN)
       || (blob->axis_ratio >= ALGO_VLAMP_PLATE_AXIS_MIN))
    {
        return 1U;
    }

    if((blob->fill <= ALGO_VLAMP_PLATE_SPARSE_FILL_MAX)
       && ((blob->ratio >= 1.12f) || (blob->axis_ratio >= 1.18f)))
    {
        return 1U;
    }

    if((algo_vlamp_blob_near_track(blob, ALGO_VLAMP_TRACK_DIST_MAX_PX) != 0U)
       && (blob->fill <= 0.54f)
       && ((blob->ratio >= 1.08f) || (blob->axis_ratio >= 1.12f)))
    {
        return 1U;
    }

    return 0U;
}

/* A small edge-clipped V can look compact, but it still carries a bright,
 * slightly stretched or sparse outline. This only admits a candidate; the
 * group-level taper check below remains the final V-versus-beacon gate. */
static uint8 algo_vlamp_blob_outline_seed_like(const algo_vlamp_blob_t *blob)
{
    uint16 w;
    uint16 h;
    uint16 long_side;

    if((blob == 0) || (blob->valid == 0U))
    {
        return 0U;
    }

    w = (uint16)(blob->max_x - blob->min_x + 1U);
    h = (uint16)(blob->max_y - blob->min_y + 1U);
    long_side = algo_vlamp_long_side(w, h);
    if((long_side < ALGO_VLAMP_OUTLINE_LONG_MIN)
       || (blob->area < ALGO_VLAMP_OUTLINE_AREA_MIN)
       || (blob->strong_area < ALGO_VLAMP_OUTLINE_STRONG_MIN))
    {
        return 0U;
    }

    return (((blob->fill <= 0.58f)
             && ((blob->ratio >= 1.10f) || (blob->axis_ratio >= 1.16f)))
            || (blob->fill <= 0.38f)) ? 1U : 0U;
}

/* A V or hook has a narrow end and a wider end along its principal axis.
 * A round beacon halo is close to symmetric, so its projected skew is small. */
static float algo_vlamp_shape_taper(float center_x,
                                    float center_y,
                                    float heading_x,
                                    float heading_y,
                                    float lambda1,
                                    float area,
                                    float sum_xx,
                                    float sum_yy,
                                    float sum_xy,
                                    float sum_xxx,
                                    float sum_yyy,
                                    float sum_xxy,
                                    float sum_xyy)
{
    float inv_area;
    float exx;
    float eyy;
    float exy;
    float exxx;
    float eyyy;
    float exxy;
    float exyy;
    float muxxx;
    float muyyy;
    float muxxy;
    float muxyy;
    float projected;
    float denom;

    if((area < 1.0f) || (lambda1 < 0.10f))
    {
        return 0.0f;
    }

    inv_area = 1.0f / area;
    exx = sum_xx * inv_area;
    eyy = sum_yy * inv_area;
    exy = sum_xy * inv_area;
    exxx = sum_xxx * inv_area;
    eyyy = sum_yyy * inv_area;
    exxy = sum_xxy * inv_area;
    exyy = sum_xyy * inv_area;
    muxxx = exxx - (3.0f * center_x * exx) + (2.0f * center_x * center_x * center_x);
    muyyy = eyyy - (3.0f * center_y * eyy) + (2.0f * center_y * center_y * center_y);
    muxxy = exxy - (center_y * exx) - (2.0f * center_x * exy)
           + (2.0f * center_x * center_x * center_y);
    muxyy = exyy - (center_x * eyy) - (2.0f * center_y * exy)
           + (2.0f * center_x * center_y * center_y);
    projected = (heading_x * heading_x * heading_x * muxxx)
              + (3.0f * heading_x * heading_x * heading_y * muxxy)
              + (3.0f * heading_x * heading_y * heading_y * muxyy)
              + (heading_y * heading_y * heading_y * muyyy);
    denom = lambda1 * sqrtf(lambda1);
    if(denom < 0.001f)
    {
        return 0.0f;
    }

    return fabsf(projected) / denom;
}

static void algo_vlamp_finish_blob(algo_vlamp_blob_t *blob)
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
    float hx;
    float hy;
    float norm;

    if((blob == 0) || (blob->area == 0U))
    {
        return;
    }

    w = (uint16)(blob->max_x - blob->min_x + 1U);
    h = (uint16)(blob->max_y - blob->min_y + 1U);
    short_side = algo_vlamp_short_side(w, h);
    inv_area = 1.0f / (float)blob->area;
    mx = (float)blob->sum_x * inv_area;
    my = (float)blob->sum_y * inv_area;
    var_x = (blob->sum_xx * inv_area) - (mx * mx);
    var_y = (blob->sum_yy * inv_area) - (my * my);
    cov_xy = (blob->sum_xy * inv_area) - (mx * my);
    trace = var_x + var_y;
    disc = sqrtf(((var_x - var_y) * (var_x - var_y)) + (4.0f * cov_xy * cov_xy));
    lambda1 = 0.5f * (trace + disc);
    lambda2 = 0.5f * (trace - disc);

    blob->center_x = mx;
    blob->center_y = my;
    blob->fill = (float)blob->area / ((float)w * (float)h);
    blob->ratio = (short_side > 0U) ? ((float)algo_vlamp_long_side(w, h) / (float)short_side) : 999.0f;
    blob->axis_ratio = (lambda2 > 0.05f) ? sqrtf(lambda1 / lambda2) : 8.0f;

    if(fabsf(cov_xy) > 0.001f)
    {
        hx = cov_xy;
        hy = lambda1 - var_x;
    }
    else if(var_x >= var_y)
    {
        hx = 1.0f;
        hy = 0.0f;
    }
    else
    {
        hx = 0.0f;
        hy = 1.0f;
    }

    norm = sqrtf((hx * hx) + (hy * hy));
    if(norm < 0.001f)
    {
        hx = 1.0f;
        hy = 0.0f;
        norm = 1.0f;
    }
    blob->heading_x = hx / norm;
    blob->heading_y = hy / norm;
    blob->valid = 1U;
}

static void algo_vlamp_accum_blob(algo_vlamp_blob_t *blob, uint16 x, uint16 y, uint8 gray)
{
    if(blob->area == 0U)
    {
        blob->min_x = x;
        blob->max_x = x;
        blob->min_y = y;
        blob->max_y = y;
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
    blob->sum_xxx += (float)x * (float)x * (float)x;
    blob->sum_yyy += (float)y * (float)y * (float)y;
    blob->sum_xxy += (float)x * (float)x * (float)y;
    blob->sum_xyy += (float)x * (float)y * (float)y;
    blob->sum_gray += gray;
    if(gray > blob->peak) { blob->peak = gray; }
    if(gray >= s_threshold_strong) { blob->strong_area++; }
}

static void algo_vlamp_build_binary(const uint8 *src_image)
{
    uint32 sum = 0U;
    uint16 histogram[256];
    uint8 gray_max = 0U;
    uint8 bright_ref = 0U;
    float mean;
    float threshold;
    uint16 x;
    uint16 y;
    int16 gray;
    uint16 ranked_count = 0U;

    memset(s_binary, 0, sizeof(s_binary));
    memset(histogram, 0, sizeof(histogram));
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
            if(histogram[gray] < 65535U) { histogram[gray]++; }
            if(gray > gray_max) { gray_max = gray; }
        }
    }

    mean = (float)sum / ((float)MT9V03X_W * (float)MT9V03X_H);
    for(gray = 255; gray >= 0; gray--)
    {
        ranked_count = (uint16)(ranked_count + histogram[gray]);
        if(ranked_count >= ALGO_VLAMP_BRIGHT_REF_RANK)
        {
            bright_ref = (uint8)gray;
            break;
        }
    }
    if(bright_ref == 0U) { bright_ref = gray_max; }

    threshold = (float)bright_ref * 0.68f;
    if((mean + 18.0f) > threshold) { threshold = mean + 18.0f; }
    if(threshold < 65.0f) { threshold = 65.0f; }
    if(threshold > 210.0f) { threshold = 210.0f; }
    s_threshold_low = algo_vlamp_clamp_u8(threshold);

    threshold = (float)bright_ref * 0.84f;
    if((mean + 40.0f) > threshold) { threshold = mean + 40.0f; }
    if(threshold < 95.0f) { threshold = 95.0f; }
    if(threshold > 235.0f) { threshold = 235.0f; }
    s_threshold_strong = algo_vlamp_clamp_u8(threshold);

    for(y = 0U; y < MT9V03X_H; y++)
    {
        for(x = 0U; x < MT9V03X_W; x++)
        {
            if(src_image[(uint32)y * MT9V03X_W + x] >= s_threshold_low)
            {
                s_binary[y][x] = 1U;
            }
        }
    }
}

static uint8 algo_vlamp_collect_component(const uint8 *src_image,
                                          uint16 start_x,
                                          uint16 start_y,
                                          algo_vlamp_blob_t *blob_out)
{
    uint16 head = 0U;
    uint16 tail = 0U;
    uint8 overflow = 0U;

    if((src_image == 0) || (blob_out == 0))
    {
        return 0U;
    }

    memset(blob_out, 0, sizeof(*blob_out));
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
        algo_vlamp_accum_blob(blob_out, x, y, gray);

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
                if(tail < ALGO_VLAMP_QUEUE_CAP)
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

    algo_vlamp_finish_blob(blob_out);
    return ((blob_out->valid != 0U)
            && (blob_out->area >= ALGO_VLAMP_AREA_MIN)
            && (blob_out->area <= ALGO_VLAMP_AREA_MAX)) ? 1U : 0U;
}

static float algo_vlamp_blob_gap(const algo_vlamp_blob_t *a, const algo_vlamp_blob_t *b)
{
    int16 dx = 0;
    int16 dy = 0;

    if((a == 0) || (b == 0))
    {
        return 999.0f;
    }

    if(a->max_x < b->min_x) { dx = (int16)(b->min_x - a->max_x); }
    else if(b->max_x < a->min_x) { dx = (int16)(a->min_x - b->max_x); }
    if(a->max_y < b->min_y) { dy = (int16)(b->min_y - a->max_y); }
    else if(b->max_y < a->min_y) { dy = (int16)(a->min_y - b->max_y); }

    return sqrtf(((float)dx * (float)dx) + ((float)dy * (float)dy));
}

static float algo_vlamp_seed_score(const algo_vlamp_blob_t *blob)
{
    uint16 w;
    uint16 h;
    uint16 long_side;
    float area_score;
    float long_score;
    float axis_score;
    float strong_score;
    float sparse_score;
    float score;
    uint8 plate_like;

    if((blob == 0) || (blob->valid == 0U))
    {
        return 0.0f;
    }

    w = (uint16)(blob->max_x - blob->min_x + 1U);
    h = (uint16)(blob->max_y - blob->min_y + 1U);
    long_side = algo_vlamp_long_side(w, h);
    plate_like = algo_vlamp_blob_plate_like(blob);
    if((long_side < ALGO_VLAMP_MIN_LONG_SIDE)
       || ((algo_vlamp_blob_compact(blob) != 0U)
           && (algo_vlamp_blob_deformed_vlike(blob) == 0U)
           && (algo_vlamp_blob_split_part_like(blob) == 0U)
           && (plate_like == 0U)
           && (algo_vlamp_blob_outline_seed_like(blob) == 0U)))
    {
        return 0.0f;
    }

    area_score = algo_vlamp_clamp_float(((float)blob->area - 4.0f) / 220.0f, 0.0f, 1.0f);
    long_score = algo_vlamp_clamp_float(((float)long_side - 8.0f) / 34.0f, 0.0f, 1.0f);
    axis_score = algo_vlamp_clamp_float((blob->axis_ratio - 1.10f) / 2.4f, 0.0f, 1.0f);
    strong_score = algo_vlamp_clamp_float((float)blob->strong_area / 18.0f, 0.0f, 1.0f);
    sparse_score = algo_vlamp_clamp_float((0.78f - blob->fill) / 0.62f, 0.0f, 1.0f);
    score = (area_score * 0.24f)
          + (long_score * 0.28f)
          + (axis_score * 0.22f)
          + (strong_score * 0.14f)
          + (sparse_score * 0.12f);
    if(plate_like != 0U)
    {
        score += 0.10f;
    }

    if(s_track_valid != 0U)
    {
        float dx = blob->center_x - s_last_main.center_x;
        float dy = blob->center_y - s_last_main.center_y;
        float dist = sqrtf((dx * dx) + (dy * dy));
        if(dist < ALGO_VLAMP_TRACK_DIST_MAX_PX)
        {
            float near_score = 1.0f - (dist / ALGO_VLAMP_TRACK_DIST_MAX_PX);
            score = (score * 0.86f) + (near_score * 0.14f);
        }
    }

    return score;
}

/* The connected-component scan is raster ordered. Keep the most V-like
 * candidates instead of letting early environmental dots fill the fixed pool. */
static float algo_vlamp_blob_priority(const algo_vlamp_blob_t *blob)
{
    uint16 w;
    uint16 h;
    uint16 long_side;
    float priority;

    if((blob == 0) || (blob->valid == 0U))
    {
        return 0.0f;
    }

    w = (uint16)(blob->max_x - blob->min_x + 1U);
    h = (uint16)(blob->max_y - blob->min_y + 1U);
    long_side = algo_vlamp_long_side(w, h);
    priority = (algo_vlamp_clamp_float((float)long_side / 42.0f, 0.0f, 1.0f) * 0.34f)
             + (algo_vlamp_clamp_float((float)blob->area / 260.0f, 0.0f, 1.0f) * 0.20f)
             + (algo_vlamp_clamp_float((float)blob->strong_area / 24.0f, 0.0f, 1.0f) * 0.26f)
             + (algo_vlamp_clamp_float((blob->axis_ratio - 1.0f) / 2.5f, 0.0f, 1.0f) * 0.14f)
             + (algo_vlamp_clamp_float((0.74f - blob->fill) / 0.60f, 0.0f, 1.0f) * 0.06f);
    if(algo_vlamp_blob_outline_seed_like(blob) != 0U)
    {
        priority += 0.10f;
    }
    return priority;
}

static void algo_vlamp_group_result(const algo_vlamp_blob_t blobs[],
                                    uint8 blob_count,
                                    uint8 seed,
                                    algo_vlamp_result_t *result_out,
                                    float *score_out)
{
    uint8 selected[ALGO_VLAMP_MAX_BLOBS];
    uint8 i;
    uint8 pass;
    uint16 min_x = MT9V03X_W;
    uint16 min_y = MT9V03X_H;
    uint16 max_x = 0U;
    uint16 max_y = 0U;
    uint32 area = 0U;
    uint32 strong_area = 0U;
    uint32 sum_x = 0U;
    uint32 sum_y = 0U;
    float sum_xx = 0.0f;
    float sum_yy = 0.0f;
    float sum_xy = 0.0f;
    float sum_xxx = 0.0f;
    float sum_yyy = 0.0f;
    float sum_xxy = 0.0f;
    float sum_xyy = 0.0f;
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
    uint16 bbox_w;
    uint16 bbox_h;
    uint16 long_side;
    uint16 short_side;
    float fill;
    float ratio;
    float axis_ratio;
    float outline_taper;
    float score;
    uint8 plate_like;
    uint8 outline_like;
    uint8 selected_count = 0U;

    memset(result_out, 0, sizeof(*result_out));
    memset(selected, 0, sizeof(selected));
    selected[seed] = 1U;

    for(pass = 0U; pass < 2U; pass++)
    {
        for(i = 0U; i < blob_count; i++)
        {
            uint8 j;
            if((selected[i] != 0U) || (blobs[i].valid == 0U)
               || ((algo_vlamp_blob_compact(&blobs[i]) != 0U)
                   && (algo_vlamp_blob_deformed_vlike(&blobs[i]) == 0U)
                   && (algo_vlamp_blob_split_part_like(&blobs[i]) == 0U)
                   && (algo_vlamp_blob_plate_like(&blobs[i]) == 0U)
                   && (algo_vlamp_blob_outline_seed_like(&blobs[i]) == 0U)))
            {
                continue;
            }
            for(j = 0U; j < blob_count; j++)
            {
                if((selected[j] != 0U)
                   && (algo_vlamp_blob_gap(&blobs[i], &blobs[j]) <= ALGO_VLAMP_MERGE_GAP_PX))
                {
                    selected[i] = 1U;
                    break;
                }
            }
        }
    }

    for(i = 0U; i < blob_count; i++)
    {
        if(selected[i] == 0U)
        {
            continue;
        }
        selected_count++;
        if(blobs[i].min_x < min_x) { min_x = blobs[i].min_x; }
        if(blobs[i].min_y < min_y) { min_y = blobs[i].min_y; }
        if(blobs[i].max_x > max_x) { max_x = blobs[i].max_x; }
        if(blobs[i].max_y > max_y) { max_y = blobs[i].max_y; }
        area += blobs[i].area;
        strong_area += blobs[i].strong_area;
        sum_x += blobs[i].sum_x;
        sum_y += blobs[i].sum_y;
        sum_xx += blobs[i].sum_xx;
        sum_yy += blobs[i].sum_yy;
        sum_xy += blobs[i].sum_xy;
        sum_xxx += blobs[i].sum_xxx;
        sum_yyy += blobs[i].sum_yyy;
        sum_xxy += blobs[i].sum_xxy;
        sum_xyy += blobs[i].sum_xyy;
    }

    if((area == 0U) || (area > ALGO_VLAMP_AREA_MAX))
    {
        *score_out = 0.0f;
        return;
    }

    bbox_w = (uint16)(max_x - min_x + 1U);
    bbox_h = (uint16)(max_y - min_y + 1U);
    long_side = algo_vlamp_long_side(bbox_w, bbox_h);
    short_side = algo_vlamp_short_side(bbox_w, bbox_h);
    fill = (float)area / ((float)bbox_w * (float)bbox_h);
    ratio = (short_side > 0U) ? ((float)long_side / (float)short_side) : 999.0f;
    mx = (float)sum_x / (float)area;
    my = (float)sum_y / (float)area;
    var_x = (sum_xx / (float)area) - (mx * mx);
    var_y = (sum_yy / (float)area) - (my * my);
    cov_xy = (sum_xy / (float)area) - (mx * my);
    trace = var_x + var_y;
    disc = sqrtf(((var_x - var_y) * (var_x - var_y)) + (4.0f * cov_xy * cov_xy));
    lambda1 = 0.5f * (trace + disc);
    lambda2 = 0.5f * (trace - disc);
    axis_ratio = (lambda2 > 0.05f) ? sqrtf(lambda1 / lambda2) : 8.0f;
    plate_like = ((long_side >= ALGO_VLAMP_PLATE_LONG_MIN)
                  && (area >= ALGO_VLAMP_PLATE_AREA_MIN)
                  && (strong_area >= ALGO_VLAMP_PLATE_STRONG_MIN)
                  && ((ratio >= ALGO_VLAMP_PLATE_RATIO_MIN)
                      || (axis_ratio >= ALGO_VLAMP_PLATE_AXIS_MIN)
                      || ((fill <= ALGO_VLAMP_PLATE_SPARSE_FILL_MAX)
                          && ((ratio >= 1.12f) || (axis_ratio >= 1.18f))))) ? 1U : 0U;

    if(fabsf(cov_xy) > 0.001f)
    {
        hx = cov_xy;
        hy = lambda1 - var_x;
    }
    else if(var_x >= var_y)
    {
        hx = 1.0f;
        hy = 0.0f;
    }
    else
    {
        hx = 0.0f;
        hy = 1.0f;
    }
    norm = sqrtf((hx * hx) + (hy * hy));
    if(norm < 0.001f)
    {
        hx = 1.0f;
        hy = 0.0f;
        norm = 1.0f;
    }
    hx /= norm;
    hy /= norm;

    outline_taper = algo_vlamp_shape_taper(mx,
                                            my,
                                            hx,
                                            hy,
                                            lambda1,
                                            (float)area,
                                            sum_xx,
                                            sum_yy,
                                            sum_xy,
                                            sum_xxx,
                                            sum_yyy,
                                            sum_xxy,
                                            sum_xyy);
    outline_like = ((long_side >= ALGO_VLAMP_OUTLINE_LONG_MIN)
                    && (area >= ALGO_VLAMP_OUTLINE_AREA_MIN)
                    && (strong_area >= ALGO_VLAMP_OUTLINE_STRONG_MIN)
                    && ((outline_taper >= ALGO_VLAMP_OUTLINE_SKEW_MIN)
                        || ((selected_count > 1U)
                            && ((ratio >= 1.10f) || (axis_ratio >= 1.16f))))) ? 1U : 0U;

    if((long_side < ALGO_VLAMP_MIN_LONG_SIDE)
       || ((selected_count <= 1U) && (long_side < 14U) && (outline_like == 0U))
       || ((long_side <= ALGO_VLAMP_COMPACT_LONG_MAX)
           && (fill >= ALGO_VLAMP_COMPACT_FILL_MIN)
           && (ratio <= ALGO_VLAMP_COMPACT_RATIO_MAX)
           && (selected_count <= 1U)
           && (plate_like == 0U)
           && (outline_like == 0U)))
    {
        *score_out = 0.0f;
        return;
    }

    score = (algo_vlamp_clamp_float(((float)area - 8.0f) / 320.0f, 0.0f, 1.0f) * 0.22f)
          + (algo_vlamp_clamp_float(((float)long_side - 8.0f) / 42.0f, 0.0f, 1.0f) * 0.26f)
          + (algo_vlamp_clamp_float((axis_ratio - 1.10f) / 2.8f, 0.0f, 1.0f) * 0.22f)
          + (algo_vlamp_clamp_float((float)strong_area / 24.0f, 0.0f, 1.0f) * 0.14f)
          + (algo_vlamp_clamp_float((0.82f - fill) / 0.70f, 0.0f, 1.0f) * 0.10f)
          + ((selected_count > 1U) ? 0.06f : 0.0f)
          + (algo_vlamp_clamp_float((outline_taper - 0.08f) / 0.52f, 0.0f, 1.0f) * 0.12f);
    if(plate_like != 0U)
    {
        score += 0.08f;
    }
    if(outline_like != 0U)
    {
        score += 0.06f;
    }

    if(s_track_valid != 0U)
    {
        float dx = mx - s_last_main.center_x;
        float dy = my - s_last_main.center_y;
        float dist = sqrtf((dx * dx) + (dy * dy));
        if(dist < ALGO_VLAMP_TRACK_DIST_MAX_PX)
        {
            score += (1.0f - (dist / ALGO_VLAMP_TRACK_DIST_MAX_PX)) * 0.12f;
        }
    }

    if(score < ((plate_like != 0U) ? 0.29f : ((outline_like != 0U) ? 0.30f : 0.34f)))
    {
        *score_out = 0.0f;
        return;
    }

    result_out->found = 1U;
    result_out->hold_active = 0U;
    result_out->center_x = mx;
    result_out->center_y = my;
    result_out->heading_x = hx;
    result_out->heading_y = hy;
    result_out->heading_cdeg = (int16_t)(atan2f(hy, hx) * 5729.57795f);
    result_out->bbox_x = min_x;
    result_out->bbox_y = min_y;
    result_out->bbox_w = bbox_w;
    result_out->bbox_h = bbox_h;
    result_out->area = (area > 65535U) ? 65535U : (uint16)area;
    result_out->confidence_permille = (uint16_t)algo_vlamp_clamp_float(score * 1000.0f, 0.0f, 1000.0f);
    *score_out = score;
}

static uint8 algo_vlamp_detect_main(const uint8 *src_image, algo_vlamp_result_t *result_out)
{
    algo_vlamp_blob_t blobs[ALGO_VLAMP_MAX_BLOBS];
    float blob_priority[ALGO_VLAMP_MAX_BLOBS];
    uint8 blob_count = 0U;
    uint8 i;
    uint16 x;
    uint16 y;
    float best_score = 0.0f;
    algo_vlamp_result_t best_result;

    memset(blobs, 0, sizeof(blobs));
    memset(blob_priority, 0, sizeof(blob_priority));
    memset(&best_result, 0, sizeof(best_result));
    algo_vlamp_build_binary(src_image);

    for(y = 0U; y < MT9V03X_H; y++)
    {
        for(x = 0U; x < MT9V03X_W; x++)
        {
            if(s_binary[y][x] == 1U)
            {
                algo_vlamp_blob_t blob;
                if(algo_vlamp_collect_component(src_image, x, y, &blob) != 0U)
                {
                    float priority = algo_vlamp_blob_priority(&blob);
                    if(blob_count < ALGO_VLAMP_MAX_BLOBS)
                    {
                        blobs[blob_count] = blob;
                        blob_priority[blob_count] = priority;
                        blob_count++;
                    }
                    else
                    {
                        uint8 worst = 0U;
                        uint8 candidate_index;
                        for(candidate_index = 1U; candidate_index < ALGO_VLAMP_MAX_BLOBS; candidate_index++)
                        {
                            if(blob_priority[candidate_index] < blob_priority[worst])
                            {
                                worst = candidate_index;
                            }
                        }
                        if(priority > blob_priority[worst])
                        {
                            blobs[worst] = blob;
                            blob_priority[worst] = priority;
                        }
                    }
                }
            }
        }
    }

    for(i = 0U; i < blob_count; i++)
    {
        float seed_score = algo_vlamp_seed_score(&blobs[i]);
        algo_vlamp_result_t candidate;
        float candidate_score = 0.0f;
        if(seed_score <= 0.0f)
        {
            continue;
        }
        algo_vlamp_group_result(blobs, blob_count, i, &candidate, &candidate_score);
        candidate_score += seed_score * 0.15f;
        if((candidate.found != 0U) && (candidate_score > best_score))
        {
            best_score = candidate_score;
            best_result = candidate;
        }
    }

    if(best_result.found == 0U)
    {
        memset(result_out, 0, sizeof(*result_out));
        return 0U;
    }

    *result_out = best_result;
    return 1U;
}

static void algo_vlamp_recenter_stable_bbox(algo_vlamp_result_t *result,
                                            float center_x,
                                            float center_y)
{
    int32 x0;
    int32 y0;
    uint16 bbox_w;
    uint16 bbox_h;

    if(result == 0)
    {
        return;
    }

    bbox_w = (result->bbox_w > 0U) ? result->bbox_w : 1U;
    bbox_h = (result->bbox_h > 0U) ? result->bbox_h : 1U;
    center_x = algo_vlamp_clamp_float(center_x, 0.0f, (float)(MT9V03X_W - 1U));
    center_y = algo_vlamp_clamp_float(center_y, 0.0f, (float)(MT9V03X_H - 1U));
    x0 = (int32)(center_x - (((float)bbox_w - 1.0f) * 0.5f) + 0.5f);
    y0 = (int32)(center_y - (((float)bbox_h - 1.0f) * 0.5f) + 0.5f);
    if(x0 < 0) { x0 = 0; }
    if(y0 < 0) { y0 = 0; }
    if(x0 > (int32)(MT9V03X_W - bbox_w)) { x0 = (int32)(MT9V03X_W - bbox_w); }
    if(y0 > (int32)(MT9V03X_H - bbox_h)) { y0 = (int32)(MT9V03X_H - bbox_h); }
    result->center_x = center_x;
    result->center_y = center_y;
    result->bbox_x = (uint16)x0;
    result->bbox_y = (uint16)y0;
    result->bbox_w = bbox_w;
    result->bbox_h = bbox_h;
}

static uint8 algo_vlamp_detect_local_recovery(const uint8 *src_image,
                                               algo_vlamp_result_t *result_out)
{
    algo_vlamp_blob_t blobs[16];
    algo_vlamp_result_t best_result;
    uint8 blob_count = 0U;
    uint8 i;
    uint16 x0;
    uint16 y0;
    uint16 x1;
    uint16 y1;
    uint16 x;
    uint16 y;
    uint32 sum = 0U;
    uint32 pixel_count = 0U;
    uint8 peak = 0U;
    float mean;
    float threshold;
    float best_score = 0.0f;

    if((src_image == 0) || (result_out == 0)
       || (s_track_valid == 0U) || (s_hold_left == 0U))
    {
        return 0U;
    }

    x0 = (s_last_main.bbox_x > ALGO_VLAMP_RECOVER_MARGIN_PX)
       ? (uint16)(s_last_main.bbox_x - ALGO_VLAMP_RECOVER_MARGIN_PX) : 0U;
    y0 = (s_last_main.bbox_y > ALGO_VLAMP_RECOVER_MARGIN_PX)
       ? (uint16)(s_last_main.bbox_y - ALGO_VLAMP_RECOVER_MARGIN_PX) : 0U;
    x1 = (uint16)(s_last_main.bbox_x + s_last_main.bbox_w + ALGO_VLAMP_RECOVER_MARGIN_PX);
    y1 = (uint16)(s_last_main.bbox_y + s_last_main.bbox_h + ALGO_VLAMP_RECOVER_MARGIN_PX);
    if(x1 >= MT9V03X_W) { x1 = (uint16)(MT9V03X_W - 1U); }
    if(y1 >= MT9V03X_H) { y1 = (uint16)(MT9V03X_H - 1U); }

    for(y = y0; y <= y1; y++)
    {
        for(x = x0; x <= x1; x++)
        {
            uint8 gray = src_image[(uint32)y * MT9V03X_W + x];
            sum += gray;
            pixel_count++;
            if(gray > peak) { peak = gray; }
        }
    }
    if((pixel_count == 0U) || (peak < 50U))
    {
        return 0U;
    }

    mean = (float)sum / (float)pixel_count;
    threshold = (float)peak * 0.62f;
    if((mean + 10.0f) > threshold) { threshold = mean + 10.0f; }
    s_threshold_low = algo_vlamp_clamp_u8(algo_vlamp_clamp_float(threshold, 58.0f, 190.0f));
    threshold = (float)peak * 0.82f;
    if((mean + 28.0f) > threshold) { threshold = mean + 28.0f; }
    s_threshold_strong = algo_vlamp_clamp_u8(algo_vlamp_clamp_float(threshold, 88.0f, 235.0f));

    memset(s_binary, 0, sizeof(s_binary));
    memset(blobs, 0, sizeof(blobs));
    memset(&best_result, 0, sizeof(best_result));
    for(y = y0; y <= y1; y++)
    {
        for(x = x0; x <= x1; x++)
        {
            if(src_image[(uint32)y * MT9V03X_W + x] >= s_threshold_low)
            {
                s_binary[y][x] = 1U;
            }
        }
    }

    for(y = y0; y <= y1; y++)
    {
        for(x = x0; x <= x1; x++)
        {
            if(s_binary[y][x] == 1U)
            {
                algo_vlamp_blob_t blob;
                if((algo_vlamp_collect_component(src_image, x, y, &blob) != 0U)
                   && (blob_count < 16U))
                {
                    blobs[blob_count] = blob;
                    blob_count++;
                }
            }
        }
    }

    for(i = 0U; i < blob_count; i++)
    {
        float seed_score = algo_vlamp_seed_score(&blobs[i]);
        algo_vlamp_result_t candidate;
        float candidate_score = 0.0f;
        float dx;
        float dy;
        float distance;
        if(seed_score <= 0.0f) { continue; }
        algo_vlamp_group_result(blobs, blob_count, i, &candidate, &candidate_score);
        if(candidate.found == 0U) { continue; }
        dx = candidate.center_x - s_last_main.center_x;
        dy = candidate.center_y - s_last_main.center_y;
        distance = sqrtf((dx * dx) + (dy * dy));
        candidate_score += seed_score * 0.15f;
        if((distance <= ALGO_VLAMP_RECOVER_CENTER_MAX_PX)
           && (candidate.confidence_permille >= ALGO_VLAMP_RECOVER_CONF_MIN)
           && (candidate_score > best_score))
        {
            best_score = candidate_score;
            best_result = candidate;
        }
    }

    if(best_result.found == 0U)
    {
        return 0U;
    }

    *result_out = s_last_main;
    result_out->found = 1U;
    result_out->hold_active = 1U;
    result_out->area = best_result.area;
    result_out->confidence_permille = best_result.confidence_permille;
    algo_vlamp_recenter_stable_bbox(result_out, best_result.center_x, best_result.center_y);
    return 1U;
}

void algo_vlamp_reset(void)
{
    algo_vlamp_clear_result();
    algo_vlamp_reset_track();
    memset(s_binary, 0, sizeof(s_binary));
}

void algo_vlamp_process(const uint8 *src_image)
{
    algo_vlamp_result_t result;

    memset(&result, 0, sizeof(result));
    if(src_image == 0)
    {
        algo_vlamp_clear_result();
        return;
    }

    if(algo_vlamp_detect_main(src_image, &result) != 0U)
    {
        s_result = result;
        s_last_main = result;
        s_track_valid = 1U;
        s_hold_left = ALGO_VLAMP_HOLD_FRAMES;
        return;
    }

    if(algo_vlamp_detect_local_recovery(src_image, &result) != 0U)
    {
        s_result = result;
        if(s_hold_left > 0U) { s_hold_left--; }
        return;
    }

    algo_vlamp_clear_result();
    if(s_hold_left > 0U)
    {
        s_hold_left--;
    }
    else
    {
        algo_vlamp_reset_track();
    }
}

const algo_vlamp_result_t *algo_vlamp_get_result(void)
{
    return &s_result;
}
