| Supported Targets | ESP32-S3 |
| ----------------- | -------- |

# CWA50 Pump Controller (ESP-IDF + LVGL)

An ESP-IDF application for the Waveshare ESP32-S3-Touch-LCD-1.69 board that drives a
CWA50-style pump over PWM, with touchscreen presets for manual control. This
project started from Espressif's `02_ESP_IDF_ST7789_LVGL` display/touch example and
was ported from an Arduino sketch (`esp32_cwa50.ino`) that used a potentiometer to
set pump duty cycle.

## Features

- **ST7789 LCD + CST816S touch**, driven via `esp_lvgl_port` (LVGL v8).
- **Display rotated 90° clockwise** in software (`sw_rotate`), with touch coordinates
  mirrored/rotated to match.
- **OFF / LOW / MED / HIGH touch presets** use control positions `0 / 10 / 55 / 100`;
  nonzero positions map onto the configured **13–85% PWM duty** range.
- **Live status** shows whether the controller is running or off, and highlights
  the selected preset. Preset names do not imply measured flow rates.
- **Animated pump illustration** sits above a single row of preset buttons. At
  OFF it bobs slowly; at LOW, MED, and HIGH it bobs, jitters, tilts, and pulses a
  soft seafoam glow, all intensifying with each preset.
- **Boot splash** displays the Scumpunk logo while the pump startup wake sequence
  runs.
- **Pump wake sequence**: a brief high-duty pulse followed by an off interval, run
  at startup immediately before the restored preset is applied.
- **Persistent preset ID** stored in NVS and restored on boot. New installations
  default to MEDIUM.

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
│   ├── main.c              Application: display, touch, PWM, preset UI, NVS persistence
│   ├── pump_image.c        Embedded RGB565-alpha data for the pump illustration
│   ├── splash_image.c      Embedded RGB565-alpha data for the boot splash
│   ├── assets
│   │   ├── 8bit_CWA200.png   Source pixel-art pump illustration
│   │   ├── pump.png          Display-sized version with transparent background
│   │   └── splash_scumpunk.png Boot splash logo
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
