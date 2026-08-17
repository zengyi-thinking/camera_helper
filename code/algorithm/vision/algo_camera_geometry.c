#include "algo_camera_geometry.h"

#include <math.h>

#define ALGO_CAMERA_GEOMETRY_DEG_TO_RAD (0.01745329251994329577f)
#define ALGO_CAMERA_GEOMETRY_THETA_LIMIT_RAD (1.5533430343f)

void algo_camera_geometry_configure(algo_camera_geometry_t *geometry,
                                    uint16 width,
                                    uint16 height,
                                    float diagonal_fov_deg,
                                    float max_scale,
                                    uint8 enabled)
{
    float half_width;
    float half_height;
    float half_diagonal;
    float half_fov_rad;

    if(geometry == 0)
    {
        return;
    }

    half_width = (width > 1U) ? ((float)(width - 1U) * 0.5f) : 0.0f;
    half_height = (height > 1U) ? ((float)(height - 1U) * 0.5f) : 0.0f;
    half_diagonal = sqrtf((half_width * half_width) + (half_height * half_height));
    half_fov_rad = diagonal_fov_deg * 0.5f * ALGO_CAMERA_GEOMETRY_DEG_TO_RAD;

    geometry->center_x = half_width;
    geometry->center_y = half_height;
    geometry->max_scale = (max_scale >= 1.0f) ? max_scale : 1.0f;
    geometry->enabled = 0U;
    geometry->focal_px = 0.0f;

    if((enabled != 0U) && (half_diagonal > 1.0f)
       && (half_fov_rad > 0.01f)
       && (half_fov_rad < ALGO_CAMERA_GEOMETRY_THETA_LIMIT_RAD))
    {
        geometry->focal_px = half_diagonal / half_fov_rad;
        geometry->enabled = 1U;
    }
}

void algo_camera_geometry_correct_point(const algo_camera_geometry_t *geometry,
                                        float raw_x,
                                        float raw_y,
                                        float *corrected_x,
                                        float *corrected_y)
{
    float dx;
    float dy;
    float radius;
    float theta;
    float scale = 1.0f;

    if(corrected_x != 0)
    {
        *corrected_x = raw_x;
    }
    if(corrected_y != 0)
    {
        *corrected_y = raw_y;
    }
    if((geometry == 0) || (geometry->enabled == 0U)
       || (geometry->focal_px <= 0.0f)
       || (!isfinite(raw_x)) || (!isfinite(raw_y)))
    {
        return;
    }

    dx = raw_x - geometry->center_x;
    dy = raw_y - geometry->center_y;
    radius = sqrtf((dx * dx) + (dy * dy));
    if(radius > 0.001f)
    {
        theta = radius / geometry->focal_px;
        if(theta > ALGO_CAMERA_GEOMETRY_THETA_LIMIT_RAD)
        {
            theta = ALGO_CAMERA_GEOMETRY_THETA_LIMIT_RAD;
        }
        scale = tanf(theta) / theta;
        if((!isfinite(scale)) || (scale > geometry->max_scale))
        {
            scale = geometry->max_scale;
        }
        if(scale < 1.0f)
        {
            scale = 1.0f;
        }
    }

    if(corrected_x != 0)
    {
        *corrected_x = geometry->center_x + (dx * scale);
    }
    if(corrected_y != 0)
    {
        *corrected_y = geometry->center_y + (dy * scale);
    }
}
