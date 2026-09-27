| Supported Targets | ESP32-S3 |
| ----------------- | -------- |

# CWA50 Pump Controller (ESP-IDF + LVGL)

An ESP-IDF application for the Waveshare ESP32-S3-Touch-LCD-1.69 board that drives a
CWA50-style pump over PWM, with a touchscreen slider for live speed control. This
project started from Espressif's `02_ESP_IDF_ST7789_LVGL` display/touch example and
was ported from an Arduino sketch (`esp32_cwa50.ino`) that used a potentiometer to
set pump duty cycle.

<img width="4032" height="3024" alt="IMG_2686" src="https://github.com/user-attachments/assets/9465e2f2-2727-404a-a745-140b4028017a" />
<img width="4032" height="3024" alt="IMG_2687" src="https://github.com/user-attachments/assets/57fbbcdd-17cf-4a7a-8769-08a88e4e65f2" />

## Features

- **ST7789 LCD + CST816S touch**, driven via `esp_lvgl_port` (LVGL v8).
- **Display rotated 90° clockwise** in software (`sw_rotate`), with touch coordinates
  mirrored/rotated to match.
- **0–100% touch slider** maps onto the pump's datasheet-safe **13–85% PWM duty**
  range; position `0` fully disables the pump (0% duty).
- **Pump wake sequence**: a brief high-duty pulse followed by an off interval, run
  before LVGL starts so the UI never shows a stale state.
- **Persistent slider position** stored in NVS, restored on boot and saved when the
  slider is released (not on every drag event, to limit flash writes).

## Hardware

- Board: Waveshare ESP32-S3-Touch-LCD-1.69 (240x280 ST7789 + CST816S touch)
- Pump PWM output: `GPIO17` (1 kHz, 8-bit LEDC duty)
- Display/touch pins are fixed by the board and configured in [`main/main.c`](main/main.c)

Adjust `PUMP_PWM_GPIO` and the PWM range macros in [`main/main.c`](main/main.c) if you
target different hardware or a pump with different duty-cycle limits.

## Building and flashing

This is a standard ESP-IDF project (built/tested with ESP-IDF v5.5.x):

```
idf.py set-target esp32s3
idf.py build
idf.py -p <PORT> flash monitor
```

Or use the ESP-IDF extension's Build / Flash (UART) / Monitor commands in VS Code.

## Project layout

```
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
│   ├── main.c              Application: display, touch, PWM, slider UI, NVS persistence
│   ├── idf_component.yml   Managed component dependencies (LVGL, touch driver, lvgl_port)
│   └── lv_conf.h           Present for reference; LVGL is configured via Kconfig (sdkconfig)
├── esp32_cwa50.ino         Original Arduino sketch this app was ported from
└── README.md
```

## Notes

- LVGL is configured through `sdkconfig` (`CONFIG_LV_CONF_SKIP=y`); the app does not
  include `lv_conf.h` directly, since that combination causes macro-redefinition
  build errors.
- Touch and display rotation flags must be changed together — see the comments in
  `app_lcd_init()` and `app_lvgl_init()` in [`main/main.c`](main/main.c) before altering
  orientation.

