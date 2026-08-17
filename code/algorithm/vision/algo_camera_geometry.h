#ifndef ALGO_CAMERA_GEOMETRY_H
#define ALGO_CAMERA_GEOMETRY_H

#include "zf_common_typedef.h"

typedef struct
{
    float center_x;
    float center_y;
    float focal_px;
    float max_scale;
    uint8 enabled;
} algo_camera_geometry_t;

void algo_camera_geometry_configure(algo_camera_geometry_t *geometry,
                                    uint16 width,
                                    uint16 height,
                                    float diagonal_fov_deg,
                                    float max_scale,
                                    uint8 enabled);
void algo_camera_geometry_correct_point(const algo_camera_geometry_t *geometry,
                                        float raw_x,
                                        float raw_y,
                                        float *corrected_x,
                                        float *corrected_y);

#endif
