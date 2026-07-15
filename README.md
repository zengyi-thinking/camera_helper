# camera_helper

Auxiliary camera board for side-view visual assistance. The board captures MT9V03X grayscale frames, detects both V-lamp and beacon targets, shows debug overlays on IPS200, and answers main-board UART6 requests.

## Hardware

- Camera: MT9V03X, same wiring as the `E8_camera` template.
- Display: IPS200 SPI.
- Main-board link: `UART_6`.
- Pins: `TX=P03_1`, `RX=P03_0`.
- Baud rate: `115200`.

## Source Layout

- `code/application/camera_helper/app_camera_helper.*`: capture, display, UART request/response, V/B fusion.
- `code/algorithm/vision/algo_vlamp.*`: V-lamp detection.
- `code/algorithm/vision/algo_beacon.*`: beacon detection.
- `iar/build_only.bat`: build entry only.

## Visual Flow

Each camera frame runs in this order:

1. Detect V-lamp with `algo_vlamp_process()`.
2. Detect beacon with the V-lamp bbox used as an ignore rect.
3. Reject beacon candidates that overlap the V-lamp or look like V-lamp fragments.
4. If both targets are valid, compute V-to-beacon deltas.

The display shows:

- Grayscale image using the same display window as `flight_control`: `x=4, y=54, w=232, h=266`.
- Five-line top status bar with cached line redraw to reduce flicker.
- Green V-lamp bbox/cross.
- Red beacon bbox/cross.
- Yellow line from V-lamp center to beacon center.

The old auxiliary debug grid and mapped-axis overlay are intentionally disabled because they were redrawn over every grayscale refresh and made the image look like it was flashing.

## Coordinate Mapping

The auxiliary camera is treated as a counter-clockwise 90 degree rotated view relative to the main camera:

```c
raw_vb_x = beacon_x - v_x;
raw_vb_y = beacon_y - v_y;
mapped_vb_x = -raw_vb_y;
mapped_vb_y =  raw_vb_x;
```

## UART Protocol

Main-board request, 5 bytes:

```text
B1 seq mode checksum 55
```

Auxiliary response, 27 bytes:

```text
B2 seq flags vx vy bx by raw_dx raw_dy map_dx map_dy vconf bconf frame_id checksum 55
```

Field encoding:

- `vx/vy/bx/by/raw_dx/raw_dy/map_dx/map_dy`: signed `int16`, big-endian.
- `vconf/bconf/frame_id`: unsigned `uint16`, big-endian.
- `checksum`: XOR from byte 0 through the last data byte.

Flags:

- `bit0 HAS_V`: V-lamp detected.
- `bit1 HAS_B`: beacon detected.
- `bit2 MOVE_VALID`: V-lamp and beacon are both valid.
- `bit3 FRESH`: response data is recent.
- `bit4 MAPPED_VALID`: mapped VB delta is valid.

The main board should use this result only when the main camera cannot publish a beacon and the auxiliary response has `MOVE_VALID=1`.

## Timed Auto-Land Mission Compatibility

The `codex/timed-auto-land` branch intentionally keeps the helper firmware single-target and protocol-compatible. B1/B2 framing and local V/beacon detection are unchanged; the main camera alone owns the two-candidate queue and target lock, while CM7_1 owns the 70-second landing timer. During a live lock, helper data may only support the same mapped target, and it is not allowed to replace a target after the main mission enters its near/missing phase.

Do not add B3, global target IDs, a lamp counter, or a second helper-side target to this mission version. All three devices still need the matching mission branch flashed because the flight controller shared-memory version and the car A4 flag behavior changed.

## Build

```bat
cd /d D:\CYT4BB7_Library\Example\Motherboard_Demo\camera_helper
iar\build_only.bat
```

Verified on `codex/timed-auto-land`: CM7_0 and CM7_1 both build with `0 errors / 0 warnings`.
