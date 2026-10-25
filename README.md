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
- **OFF / LOW / MED / HIGH touch presets.** Tap to select. **Long-press LOW, MED, or
  HIGH** to open an editor with **− / +** (1% steps, hold to repeat) that adjusts that
  preset's PWM duty live within the configured duty limits; **DONE** saves it. Defaults
  are 20% / 52% / 85%.
- **Settings screen** (gear icon, top right). **− / +** adjust each value live (hold to
  repeat); **DONE** saves them all to NVS:
  - **PWM freq:** 100–1000 Hz in 10 Hz steps (default 150 Hz).
  - **Min / Max duty:** limits for all presets (defaults 13% / 85%, kept at least 5% apart).
    Presets outside new limits are clamped.
  - **Backlight:** 10–100% brightness in 5% steps.
  - **Screen off:** Never, 15 s, 30 s, 1 min, 2 min, or 5 min of inactivity. The pump keeps
    running; the first touch wakes the screen and is not passed to the UI.
  - **Accent:** Seafoam, Ocean, Amber, Violet, Rose, or Lime. Applies immediately and is
    saved with DONE.
  - **Theme:** Dark or Light. Light darkens the accent for contrast. Applies immediately and is
    saved with DONE (the boot splash stays dark).
  - **Motion:** Pump illustration movement (bob, shake, tilt): Full, Gentle, or Off. The glow
    and flow animation are unaffected.
  - **Flow:** Off, Drops, Lines, or Pulses animation from the pump outlet. Particle count
    and speed scale with the preset (LOW/MED/HIGH). Saved with DONE (default Drops).
  - Settings are split into four swipeable pages (Display, Effects, Pump, Output) with page dots.
  - **Sound** (Output page): button feedback through the GPIO42 buzzer: Off, Tick, Chirp, Low or Knock. The new sound plays as you select it.
  - **Graph settings** (gear on the graph screen; pages Sensors, Colors, Graph): **ADC pin** (Off or
    GPIO 2, 3, 17 or 18) and **Color** (Blue, Red, Green, Orange, Yellow, Purple, Cyan, White) for each
    sensor (Intake, Coolant and Boost), plus the **Graph demo** toggle. Boost is plotted in psi on a fixed 0-30 psi scale (temperatures auto-scale; the chart has no grid lines or axis labels) (`app_boost_mv_to_tenths_psi()` is a placeholder until the pressure sensor is known). Sensor pins are read as ADC inputs (raw millivolts for now,
    shown as tenths of a degree until the sensor type is set in `app_sensor_mv_to_tenths()`); pins already used by the pump or the other sensor are skipped. Saved settings are read by
    `app_sensor_read_tenths()` via `settings.sensor_gpio[]`.
  - **PWM pin** (Output page): choose GPIO 2, 3, 17 or 18 for the pump output; it moves live and is saved.
- **Background:** subtle vertical gradient tinted with the current accent color
  (dithered to reduce banding).
- **Live status** shows whether the controller is running or off, and highlights
  the selected preset. Preset names do not imply measured flow rates.
- **Animated pump illustration** sits above a single row of preset buttons. At
  OFF it bobs slowly; at LOW, MED, and HIGH it bobs, jitters, tilts, and pulses a
  soft seafoam glow, all intensifying with each preset.
- **Boot splash** displays the Scumpunk logo while the pump startup wake sequence
  runs.
- **Pump wake sequence**: a brief high-duty pulse followed by an off interval, run
  at startup immediately before the restored preset is applied.
- **Persistent preset ID and per-preset duties** stored in NVS and restored on boot. New
  installations default to MEDIUM.
- **Temperature graph screen**: swipe left on the main screen and swipe right to return; two page dots
  at the bottom show which screen is active. Plots Intake, Coolant and Boost over a 3-minute
  window with live values shown in bordered tiles under the chart, each with an icon (wind for Intake, water drop for Coolant, three-blade fan for Boost) and whole-number readings in the sensor colour. Until sensors are wired, the
  graph settings **Graph demo** toggle; Off shows "No sensors
  connected". To add real sensors, implement `app_sensor_read_tenths()` in
  `main/main.c` (returns tenths of °C per channel).

## Hardware

- Board: Waveshare ESP32-S3-Touch-LCD-1.69 (240x280 ST7789 + CST816S touch)
- Pump PWM output: `GPIO2` by default, selectable in Settings (8-bit LEDC duty; frequency adjustable, default 150 Hz)
- LCD backlight: `GPIO15`, dimmed with 5 kHz LEDC PWM
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

## Auto (PID) mode

The fourth preset button, **AUTO**, runs a PI loop (once a second) on the Intake temperature and drives the pump duty between Min and Max duty. The preset buttons are OFF, LOW, HIGH, AUTO.

- Settings page **Auto**: Target (C) and the learned gain. Pressing either gain button resets it to the default (2.0).
- The gain self-calibrates: every 60 s it is cut 15% if the error keeps flipping sign (oscillation) or raised 15% if the error lingers without the pump saturating. It is saved to NVS when it changes. The integral gain is 2.5% of the proportional gain.
- Duty is slew-limited to 5%/s, with integral anti-windup.
- Boost above 10 psi adds 1% duty per psi.
- If Intake minus Coolant is under 3 C, the pump drops to Min duty and learning pauses.
- If the Intake sensor is missing, the pump runs at the HIGH preset duty (fail-safe).
- A previously saved MED preset starts as LOW.
- Sensor conversions use real curves (see Sensors).

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
- Flash the latest firmware (it was built but not yet flashed because the board was not connected).
- Gain auto-calibration for Auto mode needs real data to settle.