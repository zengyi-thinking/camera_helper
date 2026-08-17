#include "app_camera_helper.h"

#include "algorithm/vision/algo_camera_geometry.h"
#include "hardware/hw_config.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define CAMERA_HELPER_IPS200_TYPE       (IPS200_TYPE_SPI)
#define CAMERA_HELPER_UART_INDEX        (UART_6)
#define CAMERA_HELPER_UART_BAUD         (230400U)
#define CAMERA_HELPER_UART_TX_PIN       (UART6_TX_P03_1)
#define CAMERA_HELPER_UART_RX_PIN       (UART6_RX_P03_0)
#define CAMERA_HELPER_TIMER             (TC_TIME2_CH0)

#define CAMERA_HELPER_EXPOSURE_TIME     (64U)
#define CAMERA_HELPER_DISPLAY_DIV       (6U)
#define CAMERA_HELPER_STATUS_H          (54U)
#define CAMERA_HELPER_STATUS_LINE0_Y    (2U)
#define CAMERA_HELPER_STATUS_LINE1_Y    (12U)
#define CAMERA_HELPER_STATUS_LINE2_Y    (22U)
#define CAMERA_HELPER_STATUS_LINE3_Y    (32U)
#define CAMERA_HELPER_STATUS_LINE4_Y    (42U)
#define CAMERA_HELPER_STATUS_CHARS      (39U)
#define CAMERA_HELPER_IMAGE_X           (4U)
#define CAMERA_HELPER_IMAGE_Y           (54U)
#define CAMERA_HELPER_IMAGE_W           (232U)
#define CAMERA_HELPER_IMAGE_H           (266U)
#define CAMERA_HELPER_UART_STALE_MS     (350U)
#define CAMERA_HELPER_HEARTBEAT_MS      (200U)

#define CAMERA_HELPER_REQ_LEN           (5U)
#define CAMERA_HELPER_RSP_LEN           (27U)
#define CAMERA_HELPER_RSP_CHECK_INDEX   (25U)

static app_camera_helper_snapshot_t s_snapshot;
static algo_camera_geometry_t s_lens_geometry;
static uint8 s_boot_ready = 0U;
static uint8 s_display_divider = 0U;
static uint8 s_request_buf[CAMERA_HELPER_REQ_LEN];
static uint8 s_request_pos = 0U;
static uint8 s_last_request_seq = 0U;
static uint32 s_request_count = 0U;
static uint32 s_last_request_ms = 0U;
static uint32 s_last_heartbeat_ms = 0U;
static char s_status_line_cache[5][CAMERA_HELPER_STATUS_CHARS + 1U];
static uint16 s_status_line_color_cache[5];

static uint8 helper_xor(const uint8 *data, uint8 len)
{
    uint8 value = 0U;
    uint8 i;

    if(data == 0)
    {
        return 0U;
    }

    for(i = 0U; i < len; i++)
    {
        value ^= data[i];
    }
    return value;
}

static int16 helper_float_to_i16(float value)
{
    if(!isfinite(value))
    {
        return 0;
    }
    if(value > 32767.0f)
    {
        return 32767;
    }
    if(value < -32768.0f)
    {
        return -32768;
    }
    if(value >= 0.0f)
    {
        return (int16)(value + 0.5f);
    }
    return (int16)(value - 0.5f);
}

static uint16 helper_lcd_width(void)
{
    return ips200_width_max;
}

static uint16 helper_lcd_height(void)
{
    return ips200_height_max;
}

static uint16 helper_clamp_u16(int32 value, uint16 limit)
{
    if(value < 0)
    {
        return 0U;
    }
    if(value >= (int32)limit)
    {
        return (uint16)(limit - 1U);
    }
    return (uint16)value;
}

static uint16 helper_map_camera_x(float x)
{
    float scale = (float)(CAMERA_HELPER_IMAGE_W - 1U) / (float)(MT9V03X_W - 1U);
    int32 display_x = (int32)(CAMERA_HELPER_IMAGE_X + (x * scale) + 0.5f);

    return helper_clamp_u16(display_x, helper_lcd_width());
}

static uint16 helper_map_camera_y(float y)
{
    float scale = (float)(CAMERA_HELPER_IMAGE_H - 1U) / (float)(MT9V03X_H - 1U);
    int32 display_y = (int32)(CAMERA_HELPER_IMAGE_Y + (y * scale) + 0.5f);

    return helper_clamp_u16(display_y, helper_lcd_height());
}

static void helper_draw_cross(uint16 x, uint16 y, uint8 radius, uint16 color)
{
    uint16 x0 = (x > radius) ? (uint16)(x - radius) : 0U;
    uint16 y0 = (y > radius) ? (uint16)(y - radius) : 0U;
    uint16 x1 = (uint16)(x + radius);
    uint16 y1 = (uint16)(y + radius);

    if(x1 >= helper_lcd_width())
    {
        x1 = (uint16)(helper_lcd_width() - 1U);
    }
    if(y1 >= helper_lcd_height())
    {
        y1 = (uint16)(helper_lcd_height() - 1U);
    }

    ips200_draw_line(x0, y, x1, y, color);
    ips200_draw_line(x, y0, x, y1, color);
}

static void helper_draw_rect(uint16 x0, uint16 y0, uint16 x1, uint16 y1, uint16 color)
{
    if((x1 < x0) || (y1 < y0))
    {
        return;
    }

    ips200_draw_line(x0, y0, x1, y0, color);
    ips200_draw_line(x0, y1, x1, y1, color);
    ips200_draw_line(x0, y0, x0, y1, color);
    ips200_draw_line(x1, y0, x1, y1, color);
}

static void helper_status_cache_reset(void)
{
    uint8 i;

    for(i = 0U; i < 5U; i++)
    {
        s_status_line_cache[i][0] = '\0';
        s_status_line_color_cache[i] = 0U;
    }
}

static uint8 helper_status_line_slot(uint16 y)
{
    if(y == CAMERA_HELPER_STATUS_LINE0_Y) { return 0U; }
    if(y == CAMERA_HELPER_STATUS_LINE1_Y) { return 1U; }
    if(y == CAMERA_HELPER_STATUS_LINE2_Y) { return 2U; }
    if(y == CAMERA_HELPER_STATUS_LINE3_Y) { return 3U; }
    if(y == CAMERA_HELPER_STATUS_LINE4_Y) { return 4U; }
    return 5U;
}

static void helper_show_status_line(uint16 y, const char *text, uint16 color)
{
    char clipped[CAMERA_HELPER_STATUS_CHARS + 1U];
    uint32 len = 0U;
    uint8 slot = helper_status_line_slot(y);

    while((text != 0) && (text[len] != '\0') && (len < CAMERA_HELPER_STATUS_CHARS))
    {
        clipped[len] = text[len];
        len++;
    }
    while(len < CAMERA_HELPER_STATUS_CHARS)
    {
        clipped[len] = ' ';
        len++;
    }
    clipped[len] = '\0';

    if(slot < 5U)
    {
        if((strcmp(s_status_line_cache[slot], clipped) == 0)
           && (s_status_line_color_cache[slot] == color))
        {
            return;
        }
        (void)strcpy(s_status_line_cache[slot], clipped);
        s_status_line_color_cache[slot] = color;
    }

    ips200_set_color(color, RGB565_BLACK);
    ips200_show_string(2U, y, clipped);
}

static void helper_draw_status(void)
{
    char line[72];
    uint32 now_ms = timer_get(CAMERA_HELPER_TIMER);

    (void)snprintf(line,
                   sizeof(line),
                   "V:%u X:%03d Y:%03d C:%03u",
                   (unsigned int)s_snapshot.has_vlamp,
                   (int)s_snapshot.v_x,
                   (int)s_snapshot.v_y,
                   (unsigned int)s_snapshot.v_confidence_permille);
    helper_show_status_line(CAMERA_HELPER_STATUS_LINE1_Y,
                            line,
                            s_snapshot.has_vlamp ? RGB565_GREEN : RGB565_GRAY);

    (void)snprintf(line,
                   sizeof(line),
                   "B:%u N:%u X:%03d Y:%03d C:%03u",
                   (unsigned int)s_snapshot.has_beacon,
                   (unsigned int)s_snapshot.beacon_candidate_count,
                   (int)s_snapshot.beacon_x,
                   (int)s_snapshot.beacon_y,
                   (unsigned int)s_snapshot.beacon_confidence_permille);
    helper_show_status_line(CAMERA_HELPER_STATUS_LINE0_Y,
                            line,
                            s_snapshot.has_beacon ? RGB565_RED : RGB565_GRAY);

    (void)snprintf(line,
                   sizeof(line),
                   "RAW:%+4d/%+4d VIEW",
                   (int)s_snapshot.raw_dx,
                   (int)s_snapshot.raw_dy);
    helper_show_status_line(CAMERA_HELPER_STATUS_LINE2_Y,
                            line,
                            s_snapshot.move_valid ? RGB565_YELLOW : RGB565_GRAY);

    (void)snprintf(line,
                   sizeof(line),
                   "MAP:%+4d/%+4d RQ:%03lu A:%03lu",
                   (int)s_snapshot.mapped_dx,
                   (int)s_snapshot.mapped_dy,
                   (unsigned long)(s_request_count % 1000U),
                   (unsigned long)((now_ms - s_last_request_ms) % 1000U));
    helper_show_status_line(CAMERA_HELPER_STATUS_LINE3_Y, line, RGB565_CYAN);

    (void)snprintf(line,
                   sizeof(line),
                   "IMG:%ux%u->%ux%u DIV:%u UART6",
                   (unsigned int)MT9V03X_W,
                   (unsigned int)MT9V03X_H,
                   (unsigned int)CAMERA_HELPER_IMAGE_W,
                   (unsigned int)CAMERA_HELPER_IMAGE_H,
                   (unsigned int)CAMERA_HELPER_DISPLAY_DIV);
    helper_show_status_line(CAMERA_HELPER_STATUS_LINE4_Y, line, RGB565_WHITE);
}

static void helper_draw_vlamp_overlay(const algo_vision_target_result_t *result)
{
    uint16 x0;
    uint16 y0;
    uint16 x1;
    uint16 y1;
    uint16 cx;
    uint16 cy;

    if((result == 0) || (result->found == 0U))
    {
        return;
    }

    x0 = helper_map_camera_x((float)result->bbox_x);
    y0 = helper_map_camera_y((float)result->bbox_y);
    x1 = helper_map_camera_x((float)(result->bbox_x + result->bbox_w - 1U));
    y1 = helper_map_camera_y((float)(result->bbox_y + result->bbox_h - 1U));
    cx = helper_map_camera_x(result->center_x);
    cy = helper_map_camera_y(result->center_y);

    helper_draw_rect(x0, y0, x1, y1, RGB565_GREEN);
    helper_draw_cross(cx, cy, 5U, RGB565_GREEN);
    ips200_set_color(RGB565_GREEN, RGB565_BLACK);
    ips200_show_string(x0, (y0 > 8U) ? (uint16)(y0 - 8U) : y0, "V");

}

static void helper_draw_beacon_overlay(const algo_beacon_result_t *result)
{
    uint16 x0;
    uint16 y0;
    uint16 x1;
    uint16 y1;
    uint16 cx;
    uint16 cy;

    if((result == 0) || (result->found == 0U))
    {
        return;
    }

    x0 = helper_map_camera_x((float)result->bbox_x);
    y0 = helper_map_camera_y((float)result->bbox_y);
    x1 = helper_map_camera_x((float)(result->bbox_x + result->bbox_w - 1U));
    y1 = helper_map_camera_y((float)(result->bbox_y + result->bbox_h - 1U));
    cx = helper_map_camera_x(result->center_x);
    cy = helper_map_camera_y(result->center_y);

    helper_draw_rect(x0, y0, x1, y1, RGB565_RED);
    helper_draw_cross(cx, cy, 5U, RGB565_RED);
    ips200_set_color(RGB565_RED, RGB565_BLACK);
    ips200_show_string(x0, (y0 > 8U) ? (uint16)(y0 - 8U) : y0, "B");
}

static void helper_draw_vb_link(const algo_vision_target_result_t *v_result,
                                const algo_beacon_result_t *b_result)
{
    if((v_result == 0) || (b_result == 0)
       || (v_result->found == 0U) || (b_result->found == 0U))
    {
        return;
    }

    ips200_draw_line(helper_map_camera_x(v_result->center_x),
                     helper_map_camera_y(v_result->center_y),
                     helper_map_camera_x(b_result->center_x),
                     helper_map_camera_y(b_result->center_y),
                     RGB565_YELLOW);
}

static void helper_draw_frame(void)
{
    ips200_show_gray_image(CAMERA_HELPER_IMAGE_X,
                           CAMERA_HELPER_IMAGE_Y,
                           (const uint8 *)mt9v03x_image,
                           MT9V03X_W,
                           MT9V03X_H,
                           CAMERA_HELPER_IMAGE_W,
                           CAMERA_HELPER_IMAGE_H,
                           0U);
    helper_draw_vb_link(&s_snapshot.vlamp_result, &s_snapshot.beacon_result);
    helper_draw_vlamp_overlay(&s_snapshot.vlamp_result);
    helper_draw_beacon_overlay(&s_snapshot.beacon_result);
    helper_draw_status();
}

static void helper_put_i16_be(uint8 *buffer, uint8 index, int16 value)
{
    uint16 uvalue = (uint16)value;
    buffer[index] = (uint8)((uvalue >> 8) & 0xFFU);
    buffer[index + 1U] = (uint8)(uvalue & 0xFFU);
}

static void helper_put_u16_be(uint8 *buffer, uint8 index, uint16 value)
{
    buffer[index] = (uint8)((value >> 8) & 0xFFU);
    buffer[index + 1U] = (uint8)(value & 0xFFU);
}

static uint8 helper_snapshot_is_fresh(void)
{
    uint32 now_ms = timer_get(CAMERA_HELPER_TIMER);

    return ((now_ms - s_snapshot.timestamp_ms) <= CAMERA_HELPER_UART_STALE_MS) ? 1U : 0U;
}

static void helper_send_response(uint8 seq)
{
    uint8 response[CAMERA_HELPER_RSP_LEN];
    uint8 flags = 0U;

    memset(response, 0, sizeof(response));
    if(helper_snapshot_is_fresh() != 0U)
    {
        flags |= CAMERA_HELPER_FLAG_FRESH;
        if(s_snapshot.has_vlamp != 0U)
        {
            flags |= CAMERA_HELPER_FLAG_HAS_V;
        }
        if(s_snapshot.has_beacon != 0U)
        {
            flags |= CAMERA_HELPER_FLAG_HAS_B;
        }
        if(s_snapshot.move_valid != 0U)
        {
            flags |= CAMERA_HELPER_FLAG_MOVE_VALID | CAMERA_HELPER_FLAG_MAPPED_VALID;
        }
    }

    response[0] = CAMERA_HELPER_RSP_HEAD;
    response[1] = seq;
    response[2] = flags;
    helper_put_i16_be(response, 3U,  (flags & CAMERA_HELPER_FLAG_HAS_V) ? s_snapshot.v_x : 0);
    helper_put_i16_be(response, 5U,  (flags & CAMERA_HELPER_FLAG_HAS_V) ? s_snapshot.v_y : 0);
    helper_put_i16_be(response, 7U,  (flags & CAMERA_HELPER_FLAG_HAS_B) ? s_snapshot.beacon_x : 0);
    helper_put_i16_be(response, 9U,  (flags & CAMERA_HELPER_FLAG_HAS_B) ? s_snapshot.beacon_y : 0);
    helper_put_i16_be(response, 11U, (flags & CAMERA_HELPER_FLAG_MOVE_VALID) ? s_snapshot.raw_dx : 0);
    helper_put_i16_be(response, 13U, (flags & CAMERA_HELPER_FLAG_MOVE_VALID) ? s_snapshot.raw_dy : 0);
    helper_put_i16_be(response, 15U, (flags & CAMERA_HELPER_FLAG_MAPPED_VALID) ? s_snapshot.mapped_dx : 0);
    helper_put_i16_be(response, 17U, (flags & CAMERA_HELPER_FLAG_MAPPED_VALID) ? s_snapshot.mapped_dy : 0);
    helper_put_u16_be(response, 19U, (flags & CAMERA_HELPER_FLAG_HAS_V) ? s_snapshot.v_confidence_permille : 0U);
    helper_put_u16_be(response, 21U, (flags & CAMERA_HELPER_FLAG_HAS_B) ? s_snapshot.beacon_confidence_permille : 0U);
    helper_put_u16_be(response, 23U, s_snapshot.frame_id);
    response[CAMERA_HELPER_RSP_CHECK_INDEX] = helper_xor(response, CAMERA_HELPER_RSP_CHECK_INDEX);
    response[26] = CAMERA_HELPER_FRAME_TAIL;

    uart_write_buffer(CAMERA_HELPER_UART_INDEX, response, CAMERA_HELPER_RSP_LEN);
}

static void helper_process_request_packet(void)
{
    uint8 checksum;

    if((s_request_buf[0] != CAMERA_HELPER_REQ_HEAD)
       || (s_request_buf[4] != CAMERA_HELPER_FRAME_TAIL))
    {
        return;
    }

    checksum = helper_xor(s_request_buf, 3U);
    if(checksum != s_request_buf[3])
    {
        return;
    }

    s_request_count++;
    s_last_request_seq = s_request_buf[1];
    s_last_request_ms = timer_get(CAMERA_HELPER_TIMER);
    helper_send_response(s_last_request_seq);
}

static void helper_send_heartbeat_if_needed(void)
{
    uint32 now_ms = timer_get(CAMERA_HELPER_TIMER);

    if((uint32)(now_ms - s_last_heartbeat_ms) < CAMERA_HELPER_HEARTBEAT_MS)
    {
        return;
    }

    s_last_heartbeat_ms = now_ms;
    helper_send_response(s_last_request_seq);
}

static void helper_process_uart_byte(uint8 byte)
{
    if(s_request_pos == 0U)
    {
        if(byte == CAMERA_HELPER_REQ_HEAD)
        {
            s_request_buf[0] = byte;
            s_request_pos = 1U;
        }
        return;
    }

    s_request_buf[s_request_pos] = byte;
    s_request_pos++;

    if(s_request_pos >= CAMERA_HELPER_REQ_LEN)
    {
        helper_process_request_packet();
        s_request_pos = 0U;

        if(byte == CAMERA_HELPER_REQ_HEAD)
        {
            s_request_buf[0] = byte;
            s_request_pos = 1U;
        }
    }
}

void app_camera_helper_on_byte(uint8 byte)
{
    helper_process_uart_byte(byte);
}

static void helper_update_snapshot(const algo_vision_target_pipeline_result_t *targets,
                                   uint16 frame_id)
{
    const algo_vision_target_result_t *v_result;
    const algo_beacon_result_t *b_result;

    if(targets == 0)
    {
        return;
    }
    v_result = &targets->target_control;
    b_result = &targets->beacon_publish;

    memset(&s_snapshot, 0, sizeof(s_snapshot));
    s_snapshot.timestamp_ms = timer_get(CAMERA_HELPER_TIMER);
    s_snapshot.frame_id = frame_id;
    s_snapshot.beacon_candidate_count = targets->beacon_candidates.count;

    if((v_result != 0) && (v_result->found != 0U)
       && (v_result->hold_active == 0U)
       && (v_result->source != ALGO_VISION_TARGET_SOURCE_PURE_HOLD))
    {
        s_snapshot.has_vlamp = 1U;
        s_snapshot.v_x = helper_float_to_i16(v_result->center_x);
        s_snapshot.v_y = helper_float_to_i16(v_result->center_y);
        s_snapshot.v_confidence_permille = v_result->confidence_permille;
        s_snapshot.vlamp_result = *v_result;
    }

    if((b_result != 0) && (b_result->found != 0U))
    {
        s_snapshot.has_beacon = 1U;
        s_snapshot.beacon_x = helper_float_to_i16(b_result->center_x);
        s_snapshot.beacon_y = helper_float_to_i16(b_result->center_y);
        s_snapshot.beacon_confidence_permille = b_result->confidence_permille;
        s_snapshot.beacon_result = *b_result;
    }

    if((s_snapshot.has_vlamp != 0U) && (s_snapshot.has_beacon != 0U))
    {
        float corrected_v_x;
        float corrected_v_y;
        float corrected_b_x;
        float corrected_b_y;
        float corrected_dx;
        float corrected_dy;

        s_snapshot.move_valid = 1U;
        s_snapshot.raw_dx = (int16)(s_snapshot.beacon_x - s_snapshot.v_x);
        s_snapshot.raw_dy = (int16)(s_snapshot.beacon_y - s_snapshot.v_y);
        algo_camera_geometry_correct_point(&s_lens_geometry,
                                           (float)s_snapshot.v_x,
                                           (float)s_snapshot.v_y,
                                           &corrected_v_x,
                                           &corrected_v_y);
        algo_camera_geometry_correct_point(&s_lens_geometry,
                                           (float)s_snapshot.beacon_x,
                                           (float)s_snapshot.beacon_y,
                                           &corrected_b_x,
                                           &corrected_b_y);
        corrected_dx = corrected_b_x - corrected_v_x;
        corrected_dy = corrected_b_y - corrected_v_y;
        /* Helper is mounted 90 degrees from the main camera.  Rotate the
         * corrected relative vector once here; the main board must not apply
         * lens correction to this mapped vector again. */
        s_snapshot.mapped_dx = helper_float_to_i16(-corrected_dy);
        s_snapshot.mapped_dy = helper_float_to_i16(corrected_dx);
    }

    if(s_snapshot.has_vlamp != 0U)
    {
        s_snapshot.flags |= CAMERA_HELPER_FLAG_HAS_V;
    }
    if(s_snapshot.has_beacon != 0U)
    {
        s_snapshot.flags |= CAMERA_HELPER_FLAG_HAS_B;
    }
    if(s_snapshot.move_valid != 0U)
    {
        s_snapshot.flags |= CAMERA_HELPER_FLAG_MOVE_VALID | CAMERA_HELPER_FLAG_MAPPED_VALID;
    }
    s_snapshot.flags |= CAMERA_HELPER_FLAG_FRESH;
}

void app_camera_helper_init(void)
{
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    algo_camera_geometry_configure(&s_lens_geometry,
                                   MT9V03X_W,
                                   MT9V03X_H,
                                   HW_VISION_LENS_DIAGONAL_FOV_DEG,
                                   HW_VISION_LENS_MAX_SCALE,
                                   HW_VISION_LENS_CORRECTION_ENABLE);
    s_request_pos = 0U;
    s_last_request_seq = 0U;
    s_request_count = 0U;
    s_last_request_ms = 0U;
    s_last_heartbeat_ms = 0U;
    helper_status_cache_reset();

    ips200_init(CAMERA_HELPER_IPS200_TYPE);
    ips200_set_font(IPS200_6X8_FONT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_full(RGB565_BLACK);
    ips200_show_string(0U, 0U, "CAMERA HELPER BOOT");

    while(mt9v03x_init() != 0U)
    {
        ips200_show_string(0U, 16U, "MT9V03X REINIT");
        system_delay_ms(500);
    }
    (void)mt9v03x_set_exposure_time(CAMERA_HELPER_EXPOSURE_TIME);

    uart_init(CAMERA_HELPER_UART_INDEX,
              CAMERA_HELPER_UART_BAUD,
              CAMERA_HELPER_UART_TX_PIN,
              CAMERA_HELPER_UART_RX_PIN);
    timer_init(CAMERA_HELPER_TIMER, TIMER_MS);
    timer_start(CAMERA_HELPER_TIMER);
    uart_rx_interrupt(CAMERA_HELPER_UART_INDEX, 1U);
    algo_vision_target_pipeline_init();

    ips200_full(RGB565_BLACK);
    helper_draw_status();
    s_boot_ready = 1U;
}

void app_camera_helper_tick(void)
{
    algo_vision_target_pipeline_result_t targets;
    uint16 frame_id;

    if(s_boot_ready == 0U)
    {
        return;
    }

    helper_send_heartbeat_if_needed();

    if(mt9v03x_finish_flag == 0U)
    {
        return;
    }

    mt9v03x_finish_flag = 0U;
    frame_id = (uint16)(s_snapshot.frame_id + 1U);
    algo_vision_target_pipeline_update((const uint8 *)mt9v03x_image,
                                       frame_id,
                                       &targets);
    helper_update_snapshot(&targets, frame_id);

    s_display_divider++;
    if(s_display_divider >= CAMERA_HELPER_DISPLAY_DIV)
    {
        s_display_divider = 0U;
        helper_draw_frame();
    }
}

const app_camera_helper_snapshot_t *app_camera_helper_get_snapshot(void)
{
    return &s_snapshot;
}
