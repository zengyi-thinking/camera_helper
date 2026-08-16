#include "algo_beacon_activity.h"

#include <string.h>

#define BEACON_ACTIVITY_TRACKS             (ALGO_BEACON_MAX_CANDIDATES)
#define BEACON_ACTIVITY_HISTORY_FRAMES     (10U)
#define BEACON_ACTIVITY_HITS_MIN           (4U)
#define BEACON_ACTIVITY_RANGE_MIN          (4.0f)
#define BEACON_ACTIVITY_CROSSINGS_MIN      (2U)
#define BEACON_ACTIVITY_CROSSINGS_MAX      (7U)
#define BEACON_ACTIVITY_SIGN_MIN           (0.75f)
#define BEACON_ACTIVITY_MATCH_PX           (12.0f)
#define BEACON_ACTIVITY_QUERY_PX           (20.0f)
#define BEACON_ACTIVITY_CENTER_ALPHA       (0.35f)
#define BEACON_ACTIVITY_STALE_FULL_FRAMES  (8U)
#define BEACON_ACTIVITY_RELEASE_FRAMES     (5U)

typedef struct
{
    uint8_t valid;
    uint8_t matched;
    uint8_t present;
    uint8_t confirmed;
    uint8_t release_frames;
    uint8_t stale_full_frames;
    uint8_t history_count;
    uint8_t history_index;
    float center_x;
    float center_y;
    float contrast_history[BEACON_ACTIVITY_HISTORY_FRAMES];
    uint8_t hit_history[BEACON_ACTIVITY_HISTORY_FRAMES];
} beacon_activity_track_t;

static beacon_activity_track_t s_tracks[BEACON_ACTIVITY_TRACKS];

static float beacon_activity_sample_contrast(const uint8_t *gray_image,
                                             float center_x,
                                             float center_y)
{
    int16_t cx = (int16_t)(center_x + 0.5f);
    int16_t cy = (int16_t)(center_y + 0.5f);
    int16_t x;
    int16_t y;
    uint32_t inner_sum = 0U;
    uint32_t outer_sum = 0U;
    uint16_t inner_count = 0U;
    uint16_t outer_count = 0U;

    if(gray_image == 0) { return 0.0f; }
    for(y = (int16_t)(cy - 6); y <= (int16_t)(cy + 6); y++)
    {
        if((y < 0) || (y >= (int16_t)MT9V03X_H)) { continue; }
        for(x = (int16_t)(cx - 6); x <= (int16_t)(cx + 6); x++)
        {
            int16_t dx;
            int16_t dy;
            int16_t radius_sq;
            uint8_t gray;
            if((x < 0) || (x >= (int16_t)MT9V03X_W)) { continue; }
            dx = (int16_t)(x - cx);
            dy = (int16_t)(y - cy);
            radius_sq = (int16_t)(dx * dx + dy * dy);
            gray = gray_image[(uint32_t)y * MT9V03X_W + (uint16_t)x];
            if(radius_sq <= 9)
            {
                inner_sum += gray;
                inner_count++;
            }
            else if((radius_sq >= 25) && (radius_sq <= 36))
            {
                outer_sum += gray;
                outer_count++;
            }
        }
    }
    if((inner_count == 0U) || (outer_count == 0U)) { return 0.0f; }
    return (float)inner_sum / (float)inner_count
         - (float)outer_sum / (float)outer_count;
}

static uint8_t beacon_activity_recent(const beacon_activity_track_t *track)
{
    float values[BEACON_ACTIVITY_HISTORY_FRAMES];
    float mean = 0.0f;
    float min_value = 999.0f;
    float max_value = -999.0f;
    uint8_t crossings = 0U;
    uint8_t hits = 0U;
    uint8_t i;
    int8_t previous_sign = 0;

    if((track == 0) || (track->history_count < BEACON_ACTIVITY_HISTORY_FRAMES))
    {
        return 0U;
    }
    for(i = 0U; i < BEACON_ACTIVITY_HISTORY_FRAMES; i++)
    {
        uint8_t index = (uint8_t)((track->history_index + BEACON_ACTIVITY_HISTORY_FRAMES
            - BEACON_ACTIVITY_HISTORY_FRAMES + i) % BEACON_ACTIVITY_HISTORY_FRAMES);
        values[i] = track->contrast_history[index];
        mean += values[i];
        if(track->hit_history[index] != 0U) { hits++; }
    }
    if(hits < BEACON_ACTIVITY_HITS_MIN) { return 0U; }
    mean /= (float)BEACON_ACTIVITY_HISTORY_FRAMES;
    for(i = 0U; i < BEACON_ACTIVITY_HISTORY_FRAMES; i++)
    {
        float centered = values[i] - mean;
        int8_t sign = (centered > BEACON_ACTIVITY_SIGN_MIN) ? 1
                    : ((centered < -BEACON_ACTIVITY_SIGN_MIN) ? -1 : 0);
        if(centered < min_value) { min_value = centered; }
        if(centered > max_value) { max_value = centered; }
        if(sign != 0)
        {
            if((previous_sign != 0) && (sign != previous_sign)) { crossings++; }
            previous_sign = sign;
        }
    }
    return (((max_value - min_value) >= BEACON_ACTIVITY_RANGE_MIN)
            && (crossings >= BEACON_ACTIVITY_CROSSINGS_MIN)
            && (crossings <= BEACON_ACTIVITY_CROSSINGS_MAX)) ? 1U : 0U;
}

static uint8_t beacon_activity_find_track(const algo_beacon_result_t *measurement)
{
    float best_distance_sq = BEACON_ACTIVITY_MATCH_PX * BEACON_ACTIVITY_MATCH_PX;
    uint8_t best = 0xFFU;
    uint8_t i;

    for(i = 0U; i < BEACON_ACTIVITY_TRACKS; i++)
    {
        float dx;
        float dy;
        float distance_sq;
        if((s_tracks[i].valid == 0U) || (s_tracks[i].matched != 0U)) { continue; }
        dx = measurement->center_x - s_tracks[i].center_x;
        dy = measurement->center_y - s_tracks[i].center_y;
        distance_sq = dx * dx + dy * dy;
        if(distance_sq <= best_distance_sq)
        {
            best_distance_sq = distance_sq;
            best = i;
        }
    }
    return best;
}

static uint8_t beacon_activity_allocate_track(void)
{
    uint8_t i;
    uint8_t stalest = 0xFFU;

    for(i = 0U; i < BEACON_ACTIVITY_TRACKS; i++)
    {
        if(s_tracks[i].valid == 0U) { return i; }
        if(s_tracks[i].matched != 0U) { continue; }
        if((stalest == 0xFFU)
           || (s_tracks[i].stale_full_frames > s_tracks[stalest].stale_full_frames))
        {
            stalest = i;
        }
    }
    return stalest;
}

void algo_beacon_activity_reset(void)
{
    memset(s_tracks, 0, sizeof(s_tracks));
}

void algo_beacon_activity_update(const uint8_t *gray_image,
                                 const algo_beacon_result_list_t *measurements,
                                 uint8_t full_detect)
{
    uint8_t i;

    if(gray_image == 0) { return; }
    if(full_detect != 0U)
    {
        uint8_t count = (measurements != 0) ? measurements->count : 0U;
        if(count > ALGO_BEACON_MAX_CANDIDATES) { count = ALGO_BEACON_MAX_CANDIDATES; }
        for(i = 0U; i < BEACON_ACTIVITY_TRACKS; i++) { s_tracks[i].matched = 0U; }
        for(i = 0U; i < count; i++)
        {
            const algo_beacon_result_t *measurement = &measurements->items[i];
            uint8_t track_index;
            beacon_activity_track_t *track;
            if((measurement->found == 0U) || (measurement->hold_active != 0U)) { continue; }
            track_index = beacon_activity_find_track(measurement);
            if(track_index == 0xFFU)
            {
                track_index = beacon_activity_allocate_track();
                if(track_index == 0xFFU) { continue; }
                memset(&s_tracks[track_index], 0, sizeof(s_tracks[track_index]));
                s_tracks[track_index].valid = 1U;
                s_tracks[track_index].center_x = measurement->center_x;
                s_tracks[track_index].center_y = measurement->center_y;
            }
            track = &s_tracks[track_index];
            track->center_x += BEACON_ACTIVITY_CENTER_ALPHA * (measurement->center_x - track->center_x);
            track->center_y += BEACON_ACTIVITY_CENTER_ALPHA * (measurement->center_y - track->center_y);
            track->matched = 1U;
            track->present = 1U;
            track->stale_full_frames = 0U;
        }
        for(i = 0U; i < BEACON_ACTIVITY_TRACKS; i++)
        {
            beacon_activity_track_t *track = &s_tracks[i];
            if((track->valid == 0U) || (track->matched != 0U)) { continue; }
            track->present = 0U;
            if(track->stale_full_frames < 255U) { track->stale_full_frames++; }
            if(track->stale_full_frames > BEACON_ACTIVITY_STALE_FULL_FRAMES)
            {
                memset(track, 0, sizeof(*track));
            }
        }
    }

    for(i = 0U; i < BEACON_ACTIVITY_TRACKS; i++)
    {
        beacon_activity_track_t *track = &s_tracks[i];
        uint8_t index;
        if(track->valid == 0U) { continue; }
        index = track->history_index;
        track->contrast_history[index] = beacon_activity_sample_contrast(gray_image,
                                                                          track->center_x,
                                                                          track->center_y);
        track->hit_history[index] = track->present;
        track->history_index = (uint8_t)((index + 1U) % BEACON_ACTIVITY_HISTORY_FRAMES);
        if(track->history_count < BEACON_ACTIVITY_HISTORY_FRAMES) { track->history_count++; }
        if(beacon_activity_recent(track) != 0U)
        {
            track->confirmed = 1U;
            track->release_frames = 0U;
        }
        else if(track->confirmed != 0U)
        {
            if(track->release_frames < BEACON_ACTIVITY_RELEASE_FRAMES) { track->release_frames++; }
            else { track->confirmed = 0U; }
        }
    }
}

uint8_t algo_beacon_activity_is_confirmed(const algo_beacon_result_t *result)
{
    float max_distance_sq = BEACON_ACTIVITY_QUERY_PX * BEACON_ACTIVITY_QUERY_PX;
    uint8_t i;

    if((result == 0) || (result->found == 0U)) { return 0U; }
    for(i = 0U; i < BEACON_ACTIVITY_TRACKS; i++)
    {
        float dx;
        float dy;
        if((s_tracks[i].valid == 0U) || (s_tracks[i].confirmed == 0U)) { continue; }
        dx = result->center_x - s_tracks[i].center_x;
        dy = result->center_y - s_tracks[i].center_y;
        if((dx * dx + dy * dy) <= max_distance_sq) { return 1U; }
    }
    return 0U;
}

void algo_beacon_activity_filter_list(algo_beacon_result_list_t *list)
{
    algo_beacon_result_list_t input;
    uint8_t i;

    if(list == 0) { return; }
    input = *list;
    memset(list, 0, sizeof(*list));
    list->full_detect = input.full_detect;
    for(i = 0U; (i < input.count) && (list->count < ALGO_BEACON_MAX_CANDIDATES); i++)
    {
        if(algo_beacon_activity_is_confirmed(&input.items[i]) != 0U)
        {
            list->items[list->count++] = input.items[i];
        }
    }
}
