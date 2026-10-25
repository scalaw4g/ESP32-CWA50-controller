| Supported Targets | ESP32-S3 |
| ----------------- | -------- |

# CWA50 Pump Controller (ESP-IDF + LVGL)

An ESP-IDF application for the Waveshare ESP32-S3-Touch-LCD-1.69 board that drives a
CWA50-style pump over PWM, with touchscreen presets, an automatic (PI) mode driven by
intake temperature, and a live graph of intake temperature, coolant temperature and boost.
It started from Espressif's `02_ESP_IDF_ST7789_LVGL` display/touch example and was ported
from an Arduino sketch that used a potentiometer to set pump duty.

<img width="4032" height="3024" alt="IMG_2686" src="https://github.com/user-attachments/assets/9465e2f2-2727-404a-a745-140b4028017a" />
<img width="4032" height="3024" alt="IMG_2687" src="https://github.com/user-attachments/assets/57fbbcdd-17cf-4a7a-8769-08a88e4e65f2" />

## Features

- **Display and touch:** ST7789 LCD and CST816S touch via `esp_lvgl_port` (LVGL v8), rotated
  90° clockwise in software with touch coordinates matched.
- **Presets:** OFF / LOW / HIGH / AUTO. Tap to select. Long-press LOW or HIGH to open a
  **- / +** editor (1% steps, hold to repeat) for that preset's duty within the duty limits;
  **DONE** saves. The selected preset is restored on boot.
- **Pump screen:** an animated pump illustration (bob, shake, tilt, glow, and flow particles
  from the outlet) that intensifies with the duty, over a gradient background tinted with the
  accent color.
- **Boot splash:** Scumpunk logo while a pump wake sequence runs (a brief high-duty pulse,
  then an off interval) before the saved preset is applied.
- **Graph screen:** swipe left from the pump screen, swipe right to return. Plots Intake,
  Coolant and Boost over 3 minutes. Whole-number readings sit in bordered tiles with icons
  (wind, water drop, three-blade fan) in each sensor's color. Boost uses a fixed 0-30 psi
  scale; temperatures auto-scale. With no sensor pin assigned, the Graph demo toggle shows
  sample data; Off shows "--".
- **Auto mode:** see [Auto mode](#auto-mode).

## Settings

Gear icon, top right of the pump screen. **- / +** change values live (hold to repeat);
**DONE** saves everything to NVS. Swipe between pages (page dots at the bottom).

| Page | Setting | Options |
|---|---|---|
| Display | Backlight | 10-100%, 5% steps |
| Display | Screen off | Never, 15 s, 30 s, 1 min, 2 min, 5 min (pump keeps running; the first touch wakes the screen and is not passed to the UI) |
| Display | Accent | Seafoam, Ocean, Amber, Violet, Rose, Lime |
| Display | Theme | Dark or Light (the boot splash stays dark) |
| Effects | Flow | Off, Drops, Lines, Pulses (count and speed scale with duty) |
| Effects | Motion | Full, Gentle, Off (bob, shake, tilt; glow and flow unaffected) |
| Pump | PWM freq | 100-1000 Hz, 10 Hz steps (default 150) |
| Pump | Min / Max duty | Limits for all presets (defaults 13% / 85%, at least 5% apart); presets are clamped |
| Output | Sound | Off, Tick, Chirp, Low, Knock (GPIO42 buzzer) |
| Output | PWM pin | GPIO 2, 3, 17 or 18 for the pump output; moves live |
| Auto | Target | 15-60 C intake target |
| Auto | Learned gain | Shows the learned gain; either button resets it |

**Graph settings** (gear on the graph screen):

| Page | Setting | Options |
|---|---|---|
| Sensors | Pin per sensor | Off, or GPIO 2, 3, 17, 18 (ADC input; pins used by the pump or another sensor are skipped) |
| Colors | Color per sensor | Blue, Red, Green, Orange, Yellow, Purple, Cyan, White |
| Graph | Graph demo | On / Off |
| Graph | Boost sensor | 2.5, 3 or 4 bar |

## Auto mode

AUTO runs a PI loop once a second on the Intake temperature and drives the duty between Min and Max duty.

- The gain self-calibrates every 60 s: cut 15% if the error keeps flipping sign (oscillation), raised 15% if the error lingers while the pump is not at a duty limit. It is saved when it changes. The integral gain is 2.5% of the proportional gain.
- Duty changes at most 5% per second, with integral anti-windup.
- Boost above 10 psi adds 1% duty per psi.
- If Intake minus Coolant is under 3 C, the pump drops to Min duty and learning pauses.
- If the Intake sensor is missing, the pump runs at the HIGH preset duty (fail-safe).

## Hardware

- Board: Waveshare ESP32-S3-Touch-LCD-1.69 (240x280 ST7789 + CST816S touch)
- Pump PWM output: `GPIO2` by default (8-bit LEDC duty)
- LCD backlight: `GPIO15` (5 kHz LEDC PWM); buzzer: `GPIO42`
- Display and touch pins are fixed by the board and configured in [`main/main.c`](main/main.c)

Adjust `PUMP_PWM_GPIO` and the PWM range macros in [`main/main.c`](main/main.c) for different
hardware or a pump with different duty limits.

## Building and flashing

Standard ESP-IDF project (built and tested with ESP-IDF v5.5.x):

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
│   ├── main.c              Application: display, touch, PWM, UI, sensors, Auto mode, NVS
│   ├── pump_image.c        Embedded pump illustration
│   ├── splash_image.c      Embedded boot splash
│   ├── glow_image.c        Embedded pump glow
│   ├── img_test.c
│   ├── assets              Source PNGs (pump, splash logo)
│   ├── idf_component.yml   Managed components (LVGL, touch driver, lvgl_port)
│   └── lv_conf.h           For reference; LVGL is configured via Kconfig (sdkconfig)
└── README.md
```

## Notes

- LVGL is configured through `sdkconfig` (`CONFIG_LV_CONF_SKIP=y`); the app does not
  include `lv_conf.h` directly, since that causes macro-redefinition build errors.
- Touch and display rotation flags must be changed together; see the comments in
  `app_lcd_init()` and `app_lvgl_init()` in [`main/main.c`](main/main.c).
## Sensors and wiring

Sensor GPIOs are chosen in graph settings > Sensors (pool: GPIO 2, 3, 17, 18, shared with the pump output; GPIO3 is a strapping pin). The ADC reads up to about 3.1 V and the ESP32 inputs are **not 5 V tolerant**.

| Channel | Sensor | Type | Interface |
|---|---|---|---|
| Intake | GM 25036751 | NTC thermistor | 3.3 V pull-up, 2.2 k |
| Coolant | Bosch 0280130040 | NTC thermistor (about 2.5 k at 20 C) | 3.3 V pull-up, 2.2 k |
| Boost | Bosch 03K906051 (4 bar) | 0.5-4.5 V pressure sensor, car-powered | 10 k / 15 k divider |

### Schematic

Intake and Coolant (one circuit per sensor, each with its own resistor and capacitor):

```
 board 3V3 ---[ 2.2k 1% ]---+--------------- GPIO (sensor pin)
                            |
                            +---[ 100nF ]--- GND
                            |
                            +---[ NTC sensor ]--- GND
```

Boost (tap the car's signal wire with a back-probe or T-tap; do not cut the harness):

```
 car sensor signal ---[ 10k ]---+--------------- GPIO (boost pin)
                                |
                                +---[ 100nF ]--- GND
                                |
                                +---[ 15k ]--- car sensor ground (ECU sensor ground)
```

Power:

```
 car ignition 12 V --[fuse]--[TVS]--[5 V regulator]--+--- board 5 V
                                                     +--- 470-1000 uF to GND (cranking dips)
 board GND ---------- common with sensor grounds and the ECU sensor ground
```

### Wiring notes

- Pull-up supply: the thermistors use the board's 3V3 pin (about 1.5 mA each). Confirm the 3V3 pin on the board header.
- The boost sensor is powered by the car (5 V from the ECU). Do not power it from the ESP32; only its signal wire is read, through the divider. Board GND must connect to the ECU sensor ground.
- Divider scales the signal by 0.6: 4.5 V becomes 2.7 V and 5 V becomes 3.0 V. Load is 25 k, light for the ECU.
- Keep the 100 nF capacitors close to the GPIO pins; keep sensor wires short or twisted.
- The ADC uses its own reference, so 5 V rail ripple does not affect readings.

### Firmware conversions

- Temperature: resistance from the divider (`R = 2200 x mV / (3300 - mV)`), then log-interpolated against a per-sensor table. Open or shorted sensors read as no data ("--").
- Boost: pin mV / 0.6 gives sensor volts; kPa = (V - 0.5) / 4 x full-scale (250, 300 or 400 kPa absolute); psi = (kPa - 101.3) x 0.145, clamped at 0. Select 2.5 / 3 / 4 bar under graph settings > Graph > Boost sensor.

### To do when the sensors are installed

- Check each thermistor with a multimeter (about 2.5 k at 20 C for the Bosch, about 3.5 k for the GM) against the tables, which are typical published curves.
- Check the boost reading at rest (engine off should read about 0 psi); add an offset if it does not.
- Gain auto-calibration for Auto mode needs real data to settle.