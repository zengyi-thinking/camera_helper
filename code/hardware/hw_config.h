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

/* 信标径向亮度补偿: 与主摄同参数, 广角鱼眼边缘光斑门槛按到画面
 * 中心的归一化半径分段放宽, 中心区保持原值, 带噪声下限。 */
#define HW_BEACON_RADIAL_COMP_ENABLE       (1U)
#define HW_BEACON_RADIAL_MID_RADIUS        (0.45f)
#define HW_BEACON_RADIAL_OUTER_RADIUS      (0.75f)
#define HW_BEACON_RADIAL_MID_FACTOR        (0.70f)
#define HW_BEACON_RADIAL_OUTER_FACTOR      (0.50f)
#define HW_BEACON_RADIAL_PEAK_FLOOR        (24U)
#define HW_BEACON_RADIAL_CONTRAST_FLOOR    (4.0f)

#endif
