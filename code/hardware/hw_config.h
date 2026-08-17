#ifndef HW_CONFIG_H
#define HW_CONFIG_H

/* Keep the helper camera on the same target-shape contract as flight_control. */
#define HW_VISION_TARGET_SHAPE_V          (0U)
#define HW_VISION_TARGET_SHAPE_ONE        (1U)
#define HW_VISION_TARGET_SHAPE            (HW_VISION_TARGET_SHAPE_V)

/* Must match the main camera control-coordinate calibration. */
#define HW_VISION_LENS_CORRECTION_ENABLE   (1U)
#define HW_VISION_LENS_DIAGONAL_FOV_DEG    (170.0f)
#define HW_VISION_LENS_MAX_SCALE           (2.50f)

#endif
