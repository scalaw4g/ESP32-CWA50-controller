/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * CWA50 pump controller for the Waveshare ESP32-S3-Touch-LCD-1.69.
 * The app uses LVGL preset buttons and LEDC for pump PWM.
 */
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lvgl_port.h"
#include "esp_lcd_touch_cst816s.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <math.h>
/* Native ST7789 panel dimensions; LVGL rotates the finished UI to landscape. */
#define EXAMPLE_LCD_H_RES (240)
#define EXAMPLE_LCD_V_RES (280)

/* LCD settings */
#define EXAMPLE_LCD_SPI_NUM (SPI2_HOST)
#define EXAMPLE_LCD_PIXEL_CLK_HZ (40 * 1000 * 1000)
#define EXAMPLE_LCD_CMD_BITS (8)
#define EXAMPLE_LCD_PARAM_BITS (8)
#define EXAMPLE_LCD_COLOR_SPACE (ESP_LCD_COLOR_SPACE_RGB)
#define EXAMPLE_LCD_BITS_PER_PIXEL (16)
#define EXAMPLE_LCD_DRAW_BUFF_DOUBLE (1)
#define EXAMPLE_LCD_DRAW_BUFF_HEIGHT (50)

/* LCD pins */
#define EXAMPLE_LCD_GPIO_SCLK (GPIO_NUM_6)
#define EXAMPLE_LCD_GPIO_MOSI (GPIO_NUM_7)
#define EXAMPLE_LCD_GPIO_RST (GPIO_NUM_8)
#define EXAMPLE_LCD_GPIO_DC (GPIO_NUM_4)
#define EXAMPLE_LCD_GPIO_CS (GPIO_NUM_5)
#define EXAMPLE_LCD_GPIO_BL (GPIO_NUM_15)

#define EXAMPLE_USE_TOUCH 1

#define TOUCH_HOST I2C_NUM_0
#define PUMP_PWM_GPIO GPIO_NUM_2
#define PUMP_PWM_FREQUENCY_HZ 150
#define PUMP_PWM_FREQUENCY_MIN_HZ 100
#define PUMP_PWM_FREQUENCY_MAX_HZ 1000
#define PUMP_PWM_FREQUENCY_STEP_HZ 10
#define BACKLIGHT_PWM_FREQUENCY_HZ 5000
/* All LEDC slow-speed timers share one clock; pin it so the pump and backlight timers agree. */
#define PWM_LEDC_CLK LEDC_USE_RC_FAST_CLK
#define BACKLIGHT_MIN_PERCENT 10
#define BACKLIGHT_STEP_PERCENT 5
/* Minimum spacing between the duty limits so the pump always has a usable range. */
#define DUTY_LIMIT_MIN_GAP_PERCENT 5
#define APP_SPLASH_DURATION_MS 2000
#define APP_SPLASH_FADE_DURATION_MS 500

/* Default duty limits (the pump's datasheet range); adjustable in Settings. */
#define PUMP_DEFAULT_MIN_PERCENT 13
#define PUMP_DEFAULT_MAX_PERCENT 85
#define PUMP_DEFAULT_LOW_PERCENT 20
#define PUMP_DEFAULT_MEDIUM_PERCENT 52
#define PUMP_DEFAULT_HIGH_PERCENT 85

typedef enum
{
    APP_PRESET_OFF = 0,
    APP_PRESET_LOW,
    APP_PRESET_MEDIUM,
    APP_PRESET_HIGH,
    APP_PRESET_AUTO,
} app_preset_id_t;

typedef struct
{
    int pwm_freq_hz;
    int duty_min_pct;
    int duty_max_pct;
    int backlight_pct;
    int timeout_idx;
    int accent_idx;
    int flow_idx;
    int light_mode;
    int motion_idx;
    int pwm_gpio;
    int sound_idx;
    int demo_graph;
    int sensor_gpio[3];
    int sensor_color[3];
    int auto_target_c;
    int auto_kp10;
    int boost_range_idx;
} app_settings_t;

typedef enum
{
    SOUND_OFF = 0,
    SOUND_TICK,
    SOUND_CHIRP,
    SOUND_LOW,
    SOUND_KNOCK,
    SOUND_COUNT,
} app_sound_t;

typedef enum
{
    MOTION_FULL = 0,
    MOTION_GENTLE,
    MOTION_OFF,
    MOTION_COUNT,
} app_motion_t;

typedef enum
{
    FLOW_OFF = 0,
    FLOW_DROPS,
    FLOW_LINES,
    FLOW_PULSES,
    FLOW_STYLE_COUNT,
} app_flow_style_t;

#if EXAMPLE_USE_TOUCH
#define EXAMPLE_PIN_NUM_TOUCH_SCL (GPIO_NUM_10)
#define EXAMPLE_PIN_NUM_TOUCH_SDA (GPIO_NUM_11)
#define EXAMPLE_PIN_NUM_TOUCH_RST (GPIO_NUM_13)
#define EXAMPLE_PIN_NUM_TOUCH_INT (GPIO_NUM_14)

esp_lcd_touch_handle_t tp = NULL;
#endif

static const char *TAG = "EXAMPLE";
static lv_obj_t *pump_image;
static lv_obj_t *pump_glow;
static lv_obj_t *splash_logo;
static lv_obj_t *preset_buttons[5];
static lv_obj_t *preset_labels[5];
static lv_coord_t pump_image_base_y;
static app_preset_id_t startup_preset = APP_PRESET_MEDIUM;
static app_preset_id_t active_preset = APP_PRESET_MEDIUM;
static app_settings_t settings = {
    .pwm_freq_hz = PUMP_PWM_FREQUENCY_HZ,
    .duty_min_pct = PUMP_DEFAULT_MIN_PERCENT,
    .duty_max_pct = PUMP_DEFAULT_MAX_PERCENT,
    .backlight_pct = 100,
    .timeout_idx = 0,
    .accent_idx = 0,
    .flow_idx = FLOW_DROPS,
    .light_mode = 0,
    .motion_idx = MOTION_FULL,
    .pwm_gpio = PUMP_PWM_GPIO,
    .sound_idx = SOUND_TICK,
    .demo_graph = 1,
    .sensor_gpio = {0, 0, 0},
    .sensor_color = {0, 1, 2},
    .auto_target_c = 30,
    .auto_kp10 = 20,
    .boost_range_idx = 2,
};
static const char *sound_names[] = {"Off", "Tick", "Chirp", "Low", "Knock"};
static const int pwm_gpio_options[] = {2, 3, 17, 18};
/* Sensor pin choices; 0 means not connected. */
static const int sensor_gpio_options[] = {0, 2, 3, 17, 18};
#define SENSOR_GPIO_OPTION_COUNT ((int)(sizeof(sensor_gpio_options) / sizeof(sensor_gpio_options[0])))
/* Graph line colors as {dark theme, light theme}. */
static const char *const sensor_color_names[] = {"Blue", "Red", "Green", "Orange", "Yellow", "Purple", "Cyan", "White"};
static const uint32_t sensor_color_hex[][2] = {
    {0x4DA3FF, 0x1E6FD9}, {0xFF5252, 0xD32F2F}, {0x5CD65C, 0x2E9E2E}, {0xFFA040, 0xD9700A},
    {0xFFD84D, 0xB89400}, {0xB388FF, 0x7B4DD6}, {0x4DE0E0, 0x0A9A9A}, {0xFFFFFF, 0x404040}};
#define SENSOR_COLOR_COUNT ((int)(sizeof(sensor_color_hex) / sizeof(sensor_color_hex[0])))
#define PWM_GPIO_OPTION_COUNT ((int)(sizeof(pwm_gpio_options) / sizeof(pwm_gpio_options[0])))
static const char *motion_names[] = {"Full", "Gentle", "Off"};
static const char *flow_names[] = {"Off", "Drops", "Lines", "Pulses"};
static const int timeout_seconds[] = {0, 15, 30, 60, 120, 300};
static const char *timeout_names[] = {"Never", "15 s", "30 s", "1 min", "2 min", "5 min"};
#define TIMEOUT_COUNT (sizeof(timeout_seconds) / sizeof(timeout_seconds[0]))
#define SETTING_ROWS 21
#define SETTINGS_PAGE_COUNT 5
#define GSET_PAGE_COUNT 3
static lv_obj_t *settings_panel;
static lv_obj_t *settings_tabview;
static lv_obj_t *settings_title;
static lv_obj_t *settings_dots[SETTINGS_PAGE_COUNT];
static const char *settings_page_titles[] = {"Display", "Effects", "Pump", "Output", "Auto"};
static const char *gset_page_titles[] = {"Sensors", "Colors", "Graph"};
static const int gset_page_rows[GSET_PAGE_COUNT][3] = {{12, 14, 16}, {13, 15, 17}, {11, 20, -1}};
static lv_obj_t *gset_panel;
static lv_obj_t *gset_tabview;
static lv_obj_t *gset_title;
static lv_obj_t *gset_dots[GSET_PAGE_COUNT];
static lv_obj_t *settings_labels[SETTING_ROWS];
static lv_obj_t *wake_overlay;
static bool screen_asleep;
static int64_t splash_started_at_us;
static lv_obj_t *edit_panel;
static lv_obj_t *edit_label;
static app_preset_id_t editing_preset = APP_PRESET_MEDIUM;
/* Duty percent per preset; OFF is always 0. */
static int preset_percent[] = {
    0,
    PUMP_DEFAULT_LOW_PERCENT,
    PUMP_DEFAULT_MEDIUM_PERCENT,
    PUMP_DEFAULT_HIGH_PERCENT,
    0,
};
static const char *preset_percent_keys[] = {NULL, "duty_low", "duty_med", "duty_high", NULL};
static app_preset_id_t preset_ids[] = {
    APP_PRESET_OFF,
    APP_PRESET_LOW,
    APP_PRESET_MEDIUM,
    APP_PRESET_HIGH,
    APP_PRESET_AUTO,
};
static const char *preset_names[] = {"OFF", "LOW", "MED", "HIGH", "AUTO"};
static float auto_duty_pct = PUMP_DEFAULT_MEDIUM_PERCENT;
static float auto_integral;
static float auto_kp = 2.0f;
extern const lv_img_dsc_t pump_image_asset;
extern const lv_img_dsc_t splash_image_asset;
extern const lv_img_dsc_t glow_image_asset;

#define PRESET_COUNT (sizeof(preset_ids) / sizeof(preset_ids[0]))
#define NVS_KEY_PRESET_ID "preset_id"
#define UI_COLOR_BACKGROUND 0x080A09
#define UI_COLOR_TILE 0x151918
#define UI_BACKGROUND_TINT 130

typedef struct
{
    const char *name;
    uint32_t accent;
    uint32_t muted;
    uint32_t selected;
} app_theme_t;

static const app_theme_t themes[] = {
    {"Seafoam", 0x8FE3C1, 0x4E7968, 0x18352D},
    {"Ocean", 0x7CC4FF, 0x4A7391, 0x182E40},
    {"Amber", 0xFFC266, 0x8A6A38, 0x3A2D14},
    {"Violet", 0xC4A5FF, 0x705C94, 0x2C2342},
    {"Rose", 0xFF9EB8, 0x94566A, 0x40222E},
    {"Lime", 0xC2EE7A, 0x6A8540, 0x2A3814},
};
#define THEME_COUNT (sizeof(themes) / sizeof(themes[0]))
static uint32_t ui_accent = 0x8FE3C1;
static uint32_t ui_accent_muted = 0x4E7968;
static uint32_t ui_tile_selected = 0x18352D;
static uint32_t ui_bg = UI_COLOR_BACKGROUND;
static uint32_t ui_tile = UI_COLOR_TILE;
static uint32_t ui_accent_bright = 0x8FE3C1;

#define UI_LIGHT_BACKGROUND 0xEEF2F1
#define UI_LIGHT_TILE 0xFFFFFF

/* LCD IO and panel */
static esp_lcd_panel_io_handle_t lcd_io = NULL;
static esp_lcd_panel_handle_t lcd_panel = NULL;

/* LVGL display and touch */
static lv_display_t *lvgl_disp = NULL;

/* Configure the 1 kHz, 8-bit LEDC output used for normal pump operation. */
static esp_err_t app_control_init(void)
{
    const ledc_timer_config_t pwm_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = settings.pwm_freq_hz,
        .clk_cfg = PWM_LEDC_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&pwm_timer), TAG, "PWM timer initialization failed");

    const ledc_channel_config_t pwm_channel = {
        .gpio_num = settings.pwm_gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&pwm_channel), TAG, "PWM channel initialization failed");

    return ESP_OK;
}

static void pump_set_duty(uint32_t duty)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

/* The backlight must be initialized after the pump timer so both share one LEDC slow clock. */
static esp_err_t app_backlight_init(void)
{
    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = BACKLIGHT_PWM_FREQUENCY_HZ,
        .clk_cfg = PWM_LEDC_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), TAG, "Backlight timer initialization failed");

    const ledc_channel_config_t channel = {
        .gpio_num = EXAMPLE_LCD_GPIO_BL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_1,
        .duty = 0,
        .hpoint = 0,
    };
    return ledc_channel_config(&channel);
}

static void app_backlight_set(int percent)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, (uint32_t)percent * 1023 / 100));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
}

#define BUZZER_GPIO GPIO_NUM_42
#define BUZZER_FREQUENCY_HZ 2700
#define BUZZER_MAX_STEPS 4
typedef struct
{
    int freq_hz; /* 0 = silent gap */
    int ms;      /* 0 ends the sequence */
} buzzer_step_t;

/* Indexed by app_sound_t; index 0 (Off) is silent. */
static const buzzer_step_t buzzer_sounds[SOUND_COUNT][BUZZER_MAX_STEPS] = {
    {{0, 0}},
    {{4000, 3}},
    {{1800, 35}, {2800, 45}},
    {{1200, 30}},
    {{500, 45}},
};
static esp_timer_handle_t buzzer_timer;
static const buzzer_step_t *buzzer_next_step;

static void app_buzzer_play_step(void)
{
    const buzzer_step_t *step = buzzer_next_step;
    if (step == NULL || step->ms <= 0)
    {
        ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
        buzzer_next_step = NULL;
        return;
    }
    buzzer_next_step = (step - buzzer_sounds[settings.sound_idx] + 1 < BUZZER_MAX_STEPS) ? step + 1 : NULL;
    if (step->freq_hz > 0)
    {
        ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_2, (uint32_t)step->freq_hz);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 512);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    }
    else
    {
        ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
    }
    esp_timer_start_once(buzzer_timer, (uint64_t)step->ms * 1000);
}

static void app_buzzer_off_cb(void *arg)
{
    (void)arg;
    app_buzzer_play_step();
}

/* The buzzer shares the LEDC slow clock, so initialize it after the pump timer. */
static esp_err_t app_buzzer_init(void)
{
    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_2,
        .freq_hz = BUZZER_FREQUENCY_HZ,
        .clk_cfg = PWM_LEDC_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), TAG, "Buzzer timer initialization failed");

    const ledc_channel_config_t channel = {
        .gpio_num = BUZZER_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_2,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_2,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&channel), TAG, "Buzzer channel initialization failed");

    const esp_timer_create_args_t timer_args = {.callback = app_buzzer_off_cb, .name = "buzzer"};
    return esp_timer_create(&timer_args, &buzzer_timer);
}

static void app_buzzer_click(void)
{
    if (settings.sound_idx <= SOUND_OFF || settings.sound_idx >= SOUND_COUNT || buzzer_timer == NULL)
    {
        return;
    }
    esp_timer_stop(buzzer_timer);
    buzzer_next_step = buzzer_sounds[settings.sound_idx];
    app_buzzer_play_step();
}

static void app_click_sound_cb(lv_event_t *event)
{
    (void)event;
    app_buzzer_click();
}

static int app_clamp_int(int value, int low, int high)
{
    return value < low ? low : (value > high ? high : value);
}

/* Load the selected preset ID; initialize new installations to MEDIUM. */
static app_preset_id_t app_load_preset(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    else
    {
        ESP_ERROR_CHECK(err);
    }

    nvs_handle_t nvs_handle;
    ESP_ERROR_CHECK(nvs_open("pump", NVS_READWRITE, &nvs_handle));

    int32_t saved_preset_id = APP_PRESET_MEDIUM;
    err = nvs_get_i32(nvs_handle, NVS_KEY_PRESET_ID, &saved_preset_id);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        err = nvs_set_i32(nvs_handle, NVS_KEY_PRESET_ID, APP_PRESET_MEDIUM);
        if (err == ESP_OK)
        {
            err = nvs_commit(nvs_handle);
        }
        nvs_close(nvs_handle);
        ESP_ERROR_CHECK(err);
        return APP_PRESET_MEDIUM;
    }
    if (err != ESP_OK)
    {
        nvs_close(nvs_handle);
        ESP_ERROR_CHECK(err);
    }
    if (saved_preset_id < APP_PRESET_OFF || saved_preset_id > APP_PRESET_AUTO)
    {
        ESP_LOGW(TAG, "Saved preset ID is invalid; using MEDIUM");
        saved_preset_id = APP_PRESET_MEDIUM;
        err = nvs_set_i32(nvs_handle, NVS_KEY_PRESET_ID, saved_preset_id);
        if (err == ESP_OK)
        {
            err = nvs_commit(nvs_handle);
        }
        nvs_close(nvs_handle);
        ESP_ERROR_CHECK(err);
        return APP_PRESET_MEDIUM;
    }

    nvs_close(nvs_handle);
    return (app_preset_id_t)saved_preset_id;
}

/* Restore saved preset duties, ignoring missing or out-of-range values. NVS must be initialized. */
static void app_load_preset_percents(void)
{
    nvs_handle_t nvs_handle;
    if (nvs_open("pump", NVS_READONLY, &nvs_handle) != ESP_OK)
    {
        return;
    }
    for (size_t i = APP_PRESET_LOW; i <= APP_PRESET_HIGH; i++)
    {
        int32_t value;
        if (nvs_get_i32(nvs_handle, preset_percent_keys[i], &value) == ESP_OK &&
            value >= 1 && value <= 100)
        {
            preset_percent[i] = (int)value;
        }
    }
    nvs_close(nvs_handle);
}

static void app_save_preset_percent(app_preset_id_t preset)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("pump", NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK)
    {
        err = nvs_set_i32(nvs_handle, preset_percent_keys[preset], preset_percent[preset]);
        if (err == ESP_OK)
        {
            err = nvs_commit(nvs_handle);
        }
        nvs_close(nvs_handle);
    }
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not save %s duty: %s", preset_names[preset], esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "Saved %s duty %d%%", preset_names[preset], preset_percent[preset]);
}

/* Keep each preset inside the configured duty limits. */
static void app_clamp_presets(void)
{
    for (size_t i = APP_PRESET_LOW; i <= APP_PRESET_HIGH; i++)
    {
        preset_percent[i] = app_clamp_int(preset_percent[i], settings.duty_min_pct, settings.duty_max_pct);
    }
}

/* Restore saved settings, ignoring missing or invalid values. NVS must be initialized. */
static void app_load_settings(void)
{
    nvs_handle_t nvs_handle;
    if (nvs_open("pump", NVS_READONLY, &nvs_handle) != ESP_OK)
    {
        return;
    }

    int32_t value;
    if (nvs_get_i32(nvs_handle, "pwm_freq", &value) == ESP_OK &&
        value >= PUMP_PWM_FREQUENCY_MIN_HZ && value <= PUMP_PWM_FREQUENCY_MAX_HZ)
    {
        settings.pwm_freq_hz = (int)value;
    }
    if (nvs_get_i32(nvs_handle, "backlight", &value) == ESP_OK &&
        value >= BACKLIGHT_MIN_PERCENT && value <= 100)
    {
        settings.backlight_pct = (int)value;
    }
    if (nvs_get_i32(nvs_handle, "timeout", &value) == ESP_OK && value >= 0 &&
        value < (int32_t)TIMEOUT_COUNT)
    {
        settings.timeout_idx = (int)value;
    }
    if (nvs_get_i32(nvs_handle, "flow", &value) == ESP_OK && value >= 0 &&
        value < FLOW_STYLE_COUNT)
    {
        settings.flow_idx = (int)value;
    }
    if (nvs_get_i32(nvs_handle, "motion", &value) == ESP_OK && value >= 0 && value < MOTION_COUNT)
    {
        settings.motion_idx = (int)value;
    }
    if (nvs_get_i32(nvs_handle, "pwm_gpio", &value) == ESP_OK)
    {
        for (int i = 0; i < PWM_GPIO_OPTION_COUNT; i++)
        {
            if (pwm_gpio_options[i] == value)
            {
                settings.pwm_gpio = (int)value;
            }
        }
    }
    static const char *const sensor_keys[3] = {"sens1", "sens2", "sens3"};
    for (int ch = 0; ch < 3; ch++)
    {
        if (nvs_get_i32(nvs_handle, sensor_keys[ch], &value) == ESP_OK)
        {
            for (int i = 0; i < SENSOR_GPIO_OPTION_COUNT; i++)
            {
                if (sensor_gpio_options[i] == value && (value == 0 || value != settings.pwm_gpio))
                {
                    settings.sensor_gpio[ch] = (int)value;
                }
            }
        }
    }
    static const char *const color_keys[3] = {"scol1", "scol2", "scol3"};
    for (int ch = 0; ch < 3; ch++)
    {
        if (nvs_get_i32(nvs_handle, color_keys[ch], &value) == ESP_OK && value >= 0 && value < SENSOR_COLOR_COUNT)
        {
            settings.sensor_color[ch] = (int)value;
        }
    }
    if (nvs_get_i32(nvs_handle, "a_tgt", &value) == ESP_OK && value >= 15 && value <= 60)
    {
        settings.auto_target_c = (int)value;
    }
    if (nvs_get_i32(nvs_handle, "a_kp", &value) == ESP_OK && value >= 1 && value <= 100)
    {
        settings.auto_kp10 = (int)value;
        auto_kp = value / 10.0f;
    }
    if (nvs_get_i32(nvs_handle, "bsens", &value) == ESP_OK && value >= 0 && value <= 2)
    {
        settings.boost_range_idx = (int)value;
    }
    if (nvs_get_i32(nvs_handle, "demo", &value) == ESP_OK && (value == 0 || value == 1))
    {
        settings.demo_graph = (int)value;
    }
    if (nvs_get_i32(nvs_handle, "sound", &value) == ESP_OK && value >= 0 && value < SOUND_COUNT)
    {
        settings.sound_idx = (int)value;
    }
    if (nvs_get_i32(nvs_handle, "light", &value) == ESP_OK && (value == 0 || value == 1))
    {
        settings.light_mode = (int)value;
    }
    if (nvs_get_i32(nvs_handle, "accent", &value) == ESP_OK && value >= 0 &&
        value < (int32_t)THEME_COUNT)
    {
        settings.accent_idx = (int)value;
    }

    int32_t duty_min = settings.duty_min_pct;
    int32_t duty_max = settings.duty_max_pct;
    if (nvs_get_i32(nvs_handle, "duty_min", &value) == ESP_OK)
    {
        duty_min = value;
    }
    if (nvs_get_i32(nvs_handle, "duty_max", &value) == ESP_OK)
    {
        duty_max = value;
    }
    if (duty_min >= 1 && duty_max <= 100 && duty_max - duty_min >= DUTY_LIMIT_MIN_GAP_PERCENT)
    {
        settings.duty_min_pct = (int)duty_min;
        settings.duty_max_pct = (int)duty_max;
    }
    nvs_close(nvs_handle);
}

static void app_save_settings(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("pump", NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK)
    {
        err = nvs_set_i32(nvs_handle, "pwm_freq", settings.pwm_freq_hz);
        if (err == ESP_OK)
            err = nvs_set_i32(nvs_handle, "duty_min", settings.duty_min_pct);
        if (err == ESP_OK)
            err = nvs_set_i32(nvs_handle, "duty_max", settings.duty_max_pct);
        if (err == ESP_OK)
            err = nvs_set_i32(nvs_handle, "backlight", settings.backlight_pct);
        if (err == ESP_OK)
            err = nvs_set_i32(nvs_handle, "timeout", settings.timeout_idx);
        if (err == ESP_OK)
            err = nvs_set_i32(nvs_handle, "accent", settings.accent_idx);
        if (err == ESP_OK)
            err = nvs_set_i32(nvs_handle, "flow", settings.flow_idx);
        if (err == ESP_OK)
            err = nvs_set_i32(nvs_handle, "light", settings.light_mode);
        if (err == ESP_OK)
            err = nvs_set_i32(nvs_handle, "motion", settings.motion_idx);
        if (err == ESP_OK)
            err = nvs_set_i32(nvs_handle, "pwm_gpio", settings.pwm_gpio);
        if (err == ESP_OK)
            err = nvs_set_i32(nvs_handle, "sound", settings.sound_idx);
        if (err == ESP_OK)
            err = nvs_set_i32(nvs_handle, "demo", settings.demo_graph);
        if (err == ESP_OK)
            err = nvs_set_i32(nvs_handle, "sens1", settings.sensor_gpio[0]);
        if (err == ESP_OK)
            err = nvs_set_i32(nvs_handle, "sens2", settings.sensor_gpio[1]);
        if (err == ESP_OK)
            err = nvs_set_i32(nvs_handle, "scol1", settings.sensor_color[0]);
        if (err == ESP_OK)
            err = nvs_set_i32(nvs_handle, "scol2", settings.sensor_color[1]);
        if (err == ESP_OK)
            err = nvs_set_i32(nvs_handle, "sens3", settings.sensor_gpio[2]);
        if (err == ESP_OK)
            err = nvs_set_i32(nvs_handle, "scol3", settings.sensor_color[2]);
        if (err == ESP_OK)
            err = nvs_set_i32(nvs_handle, "bsens", settings.boost_range_idx);
        if (err == ESP_OK)
            err = nvs_set_i32(nvs_handle, "a_tgt", settings.auto_target_c);
        if (err == ESP_OK)
            err = nvs_set_i32(nvs_handle, "a_kp", settings.auto_kp10);
        for (size_t i = APP_PRESET_LOW; i <= APP_PRESET_HIGH && err == ESP_OK; i++)
        {
            err = nvs_set_i32(nvs_handle, preset_percent_keys[i], preset_percent[i]);
        }
        if (err == ESP_OK)
            err = nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
    }
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not save settings: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "Saved settings");
}

static void app_save_preset_position(lv_event_t *event)
{
    app_preset_id_t *preset = lv_event_get_user_data(event);
    if (preset == NULL || *preset < APP_PRESET_OFF || *preset > APP_PRESET_AUTO)
    {
        ESP_LOGE(TAG, "Could not save preset: event has invalid preset ID");
        return;
    }

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("pump", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not open NVS to save preset: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_set_i32(nvs_handle, NVS_KEY_PRESET_ID, *preset);
    if (err == ESP_OK)
    {
        err = nvs_commit(nvs_handle);
    }
    nvs_close(nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not save preset: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "Saved preset %s", preset_names[*preset]);
}

static void app_update_preset_button_styles(app_preset_id_t selected_preset)
{
    for (size_t i = 0; i < PRESET_COUNT; i++)
    {
        if (preset_buttons[i] == NULL)
        {
            continue;
        }
        const bool selected = i == (size_t)selected_preset;
        lv_obj_set_style_bg_color(preset_buttons[i],
                                  lv_color_hex(selected ? ui_tile_selected : ui_tile),
                                  LV_PART_MAIN);
        lv_obj_set_style_border_color(preset_buttons[i],
                                      lv_color_hex(selected ? ui_accent : ui_accent_muted),
                                      LV_PART_MAIN);
        lv_obj_set_style_border_width(preset_buttons[i], selected ? 2 : 1, LV_PART_MAIN);
        lv_obj_set_style_text_color(preset_labels[i], lv_color_hex(ui_accent), LV_PART_MAIN);
    }
}

/* Convert a preset's duty percent into a safe pump PWM duty. */
static void app_apply_preset(app_preset_id_t preset)
{
    if (preset == APP_PRESET_OFF)
    {
        pump_set_duty(0);
        return;
    }

    const float percent = preset == APP_PRESET_AUTO ? auto_duty_pct : (float)preset_percent[preset];
    int duty = (int)(percent * 255.0f / 100.0f + 0.5f);
    /* An 8-bit LEDC period has 256 steps. */
    pump_set_duty((uint32_t)duty);
}

/* Auto mode borrows the LOW/MED/HIGH look based on its current duty. */
static app_preset_id_t app_effective_preset(app_preset_id_t preset)
{
    if (preset != APP_PRESET_AUTO)
    {
        return preset;
    }
    return auto_duty_pct <= 33.0f ? APP_PRESET_LOW : (auto_duty_pct <= 66.0f ? APP_PRESET_MEDIUM : APP_PRESET_HIGH);
}

static void app_pump_animation_exec_cb(void *object, int32_t y)
{
    lv_obj_set_y((lv_obj_t *)object, y);
}

static void app_pump_shake_exec_cb(void *object, int32_t x)
{
    lv_obj_set_style_translate_x((lv_obj_t *)object, x, LV_PART_MAIN);
}

static void app_pump_rotate_exec_cb(void *object, int32_t angle)
{
    lv_img_set_angle((lv_obj_t *)object, angle);
}

static void app_pump_glow_exec_cb(void *object, int32_t opa)
{
    lv_obj_set_style_img_opa((lv_obj_t *)object, opa, LV_PART_MAIN);
}

static void app_start_pump_anim(void *object, lv_anim_exec_xcb_t exec_cb, int32_t from, int32_t to,
                                uint32_t time_ms)
{
    lv_anim_t animation;
    lv_anim_init(&animation);
    lv_anim_set_var(&animation, object);
    lv_anim_set_exec_cb(&animation, exec_cb);
    lv_anim_set_values(&animation, from, to);
    lv_anim_set_time(&animation, time_ms);
    lv_anim_set_playback_time(&animation, time_ms);
    lv_anim_set_repeat_count(&animation, LV_ANIM_REPEAT_INFINITE);
    lv_anim_set_path_cb(&animation, lv_anim_path_ease_in_out);
    lv_anim_start(&animation);
}

#define FLOW_MAX_PARTICLES 14
#define FLOW_ORIGIN_X 142
#define FLOW_ORIGIN_Y 66

typedef struct
{
    lv_obj_t *obj;
    int index;
    int32_t last_t;
    int reach;
    int fall;
    int x0;
    int y0;
} flow_particle_t;

static flow_particle_t flow_particles[FLOW_MAX_PARTICLES];
static lv_obj_t *flow_track[3];

static void app_flow_randomize(flow_particle_t *particle)
{
    particle->reach = 40 + (int)lv_rand(0, 40);
    particle->fall = 14 + (int)lv_rand(0, 30);
    particle->x0 = (int)lv_rand(0, 8);
    particle->y0 = (int)lv_rand(0, 10) - 5;
}

/* Progress t runs 0..1000 as a particle travels from the pump outlet. */
static void app_flow_exec_cb(void *var, int32_t t)
{
    flow_particle_t *particle = var;
    if (t < particle->last_t)
    {
        app_flow_randomize(particle);
    }
    particle->last_t = t;
    const int lane = particle->index % 3;
    lv_coord_t x = FLOW_ORIGIN_X;
    lv_coord_t y = FLOW_ORIGIN_Y;
    switch (settings.flow_idx)
    {
    case FLOW_DROPS:
        x = FLOW_ORIGIN_X - 5 - particle->x0 - particle->reach * t / 1000;
        y = FLOW_ORIGIN_Y - 2 + particle->y0 + particle->fall * t / 1000 * t / 1000;
        break;
    case FLOW_LINES:
        x = FLOW_ORIGIN_X - 22 - 52 * t / 1000;
        y = FLOW_ORIGIN_Y - 9 + lane * 9;
        break;
    case FLOW_PULSES:
        x = FLOW_ORIGIN_X - 14 - 58 * t / 1000;
        y = FLOW_ORIGIN_Y - 8 + lane * 8;
        break;
    default:
        break;
    }
    const int32_t opa = t < 150 ? t * 255 / 150 : (t > 650 ? (1000 - t) * 255 / 350 : 255);
    lv_obj_set_pos(particle->obj, x, y);
    lv_obj_set_style_bg_opa(particle->obj, (lv_opa_t)opa, LV_PART_MAIN);
}

static void app_update_flow(app_preset_id_t preset)
{
    preset = app_effective_preset(preset);
    for (size_t i = 0; i < FLOW_MAX_PARTICLES; i++)
    {
        lv_anim_del(&flow_particles[i], app_flow_exec_cb);
        lv_obj_add_flag(flow_particles[i].obj, LV_OBJ_FLAG_HIDDEN);
    }
    for (int i = 0; i < 3; i++)
    {
        lv_obj_add_flag(flow_track[i], LV_OBJ_FLAG_HIDDEN);
    }
    if (preset == APP_PRESET_OFF || settings.flow_idx == FLOW_OFF)
    {
        return;
    }

    int count = 3;
    uint32_t cycle_ms = 1500;
    if (preset == APP_PRESET_MEDIUM)
    {
        count = 5;
        cycle_ms = 1000;
    }
    else if (preset == APP_PRESET_HIGH)
    {
        count = 7;
        cycle_ms = 700;
    }

    if (settings.flow_idx == FLOW_DROPS)
    {
        count *= 2;
    }

    lv_coord_t width = 5;
    lv_coord_t height = 5;
    lv_coord_t radius = LV_RADIUS_CIRCLE;
    if (settings.flow_idx == FLOW_LINES)
    {
        width = 16;
        height = 2;
        radius = 1;
    }
    else if (settings.flow_idx == FLOW_PULSES)
    {
        width = 8;
        height = 4;
        radius = 2;
        for (int i = 0; i < 3; i++)
        {
            lv_obj_set_style_bg_color(flow_track[i], lv_color_hex(ui_accent_muted), LV_PART_MAIN);
            lv_obj_clear_flag(flow_track[i], LV_OBJ_FLAG_HIDDEN);
        }
    }

    for (int i = 0; i < count; i++)
    {
        flow_particle_t *particle = &flow_particles[i];
        particle->index = i;
        particle->last_t = 0;
        app_flow_randomize(particle);
        lv_obj_set_size(particle->obj, width, height);
        lv_obj_set_style_radius(particle->obj, radius, LV_PART_MAIN);
        lv_obj_set_style_bg_color(particle->obj, lv_color_hex(ui_accent), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(particle->obj, LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_clear_flag(particle->obj, LV_OBJ_FLAG_HIDDEN);

        lv_anim_t animation;
        lv_anim_init(&animation);
        lv_anim_set_var(&animation, particle);
        lv_anim_set_exec_cb(&animation, app_flow_exec_cb);
        lv_anim_set_values(&animation, 0, 1000);
        lv_anim_set_time(&animation, cycle_ms);
        lv_anim_set_delay(&animation, (uint32_t)i * cycle_ms / (uint32_t)count);
        lv_anim_set_repeat_count(&animation, LV_ANIM_REPEAT_INFINITE);
        lv_anim_set_path_cb(&animation, lv_anim_path_linear);
        lv_anim_start(&animation);
    }
}

static void app_update_pump_animation(app_preset_id_t preset)
{
    preset = app_effective_preset(preset);
    app_update_flow(preset);
    lv_anim_del(pump_image, app_pump_animation_exec_cb);
    lv_anim_del(pump_image, app_pump_shake_exec_cb);
    lv_anim_del(pump_image, app_pump_rotate_exec_cb);
    lv_anim_del(pump_glow, app_pump_glow_exec_cb);
    lv_obj_set_y(pump_image, pump_image_base_y);
    lv_obj_set_style_translate_x(pump_image, 0, LV_PART_MAIN);
    lv_img_set_angle(pump_image, 0);
    lv_obj_set_style_img_opa(pump_glow, LV_OPA_TRANSP, LV_PART_MAIN);

    /* Bob cycle, shake (px) and rotation (0.1 deg) amplitude, glow peak opacity. */
    uint32_t cycle_ms;
    int32_t shake = 0;
    int32_t tilt = 0;
    int32_t glow = 0;
    switch (preset)
    {
    case APP_PRESET_OFF:
        cycle_ms = 500;
        break;
    case APP_PRESET_LOW:
        cycle_ms = 1400;
        shake = 1;
        tilt = 6;
        glow = 70;
        break;
    case APP_PRESET_MEDIUM:
        cycle_ms = 900;
        shake = 2;
        tilt = 10;
        glow = 130;
        break;
    case APP_PRESET_HIGH:
        cycle_ms = 500;
        shake = 3;
        tilt = 16;
        glow = 200;
        break;
    default:
        return;
    }

    int32_t bob = 3;
    if (settings.motion_idx == MOTION_OFF)
    {
        bob = 0;
        shake = 0;
        tilt = 0;
    }
    else if (settings.motion_idx == MOTION_GENTLE)
    {
        bob = 1;
        shake = shake / 2;
        tilt = tilt / 2;
    }

    if (bob > 0)
    {
        app_start_pump_anim(pump_image, app_pump_animation_exec_cb, pump_image_base_y,
                            pump_image_base_y - bob, cycle_ms);
    }
    if (shake > 0)
    {
        app_start_pump_anim(pump_image, app_pump_shake_exec_cb, -shake, shake, cycle_ms / 7 + 30);
    }
    if (tilt > 0)
    {
        app_start_pump_anim(pump_image, app_pump_rotate_exec_cb, -tilt, tilt, cycle_ms / 4 + 40);
    }
    if (glow > 0)
    {
        app_start_pump_anim(pump_glow, app_pump_glow_exec_cb, glow / 3, glow, cycle_ms);
    }
}
static void app_preset_button_event_cb(lv_event_t *event)
{
    app_preset_id_t *preset = lv_event_get_user_data(event);
    if (preset == NULL || *preset < APP_PRESET_OFF || *preset > APP_PRESET_AUTO)
    {
        ESP_LOGE(TAG, "Preset button event has invalid preset ID");
        return;
    }

    app_update_preset_button_styles(*preset);
    active_preset = *preset;
    if (*preset == APP_PRESET_AUTO)
    {
        auto_duty_pct = (float)preset_percent[APP_PRESET_MEDIUM];
        auto_integral = 0.0f;
    }
    app_apply_preset(*preset);
    app_update_pump_animation(*preset);
}

static void app_panel_show(lv_obj_t *panel)
{
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(panel);
}

static void app_panel_hide(lv_obj_t *panel)
{
    lv_obj_add_flag(panel, LV_OBJ_FLAG_HIDDEN);
}

static void app_update_edit_label(void)
{
    lv_label_set_text_fmt(edit_label, "%s  %d%%", preset_names[editing_preset],
                          preset_percent[editing_preset]);
}

/* Long-press: select the preset and open the duty editor over the button row. */
static void app_edit_open_cb(lv_event_t *event)
{
    app_preset_id_t *preset = lv_event_get_user_data(event);
    if (preset == NULL || *preset <= APP_PRESET_OFF || *preset > APP_PRESET_HIGH)
    {
        return;
    }
    editing_preset = *preset;
    app_update_edit_label();
    app_panel_show(edit_panel);
}

static void app_edit_adjust_cb(lv_event_t *event)
{
    const int step = (int)(intptr_t)lv_event_get_user_data(event);
    int value = app_clamp_int(preset_percent[editing_preset] + step, settings.duty_min_pct,
                              settings.duty_max_pct);
    if (value == preset_percent[editing_preset])
    {
        return;
    }
    preset_percent[editing_preset] = value;
    app_apply_preset(editing_preset);
    app_update_edit_label();
}

static void app_edit_done_cb(lv_event_t *event)
{
    (void)event;
    app_save_preset_percent(editing_preset);
    app_panel_hide(edit_panel);
}

/* Blend two 0xRRGGBB colors; weight is the share of a, 0..255. */
static uint32_t app_mix_hex(uint32_t a, uint32_t b, int weight)
{
    uint32_t out = 0;
    for (int shift = 16; shift >= 0; shift -= 8)
    {
        const int ca = (int)((a >> shift) & 0xFF);
        const int cb = (int)((b >> shift) & 0xFF);
        out |= (uint32_t)((ca * weight + cb * (255 - weight)) / 255) << shift;
    }
    return out;
}

static void app_select_theme(int index)
{
    index = app_clamp_int(index, 0, (int)THEME_COUNT - 1);
    ui_accent_bright = themes[index].accent;
    if (settings.light_mode)
    {
        ui_bg = UI_LIGHT_BACKGROUND;
        ui_tile = UI_LIGHT_TILE;
        ui_accent = app_mix_hex(themes[index].accent, 0x000000, 130);
        ui_accent_muted = app_mix_hex(ui_accent, ui_bg, 128);
        ui_tile_selected = app_mix_hex(themes[index].accent, 0xFFFFFF, 90);
    }
    else
    {
        ui_bg = UI_COLOR_BACKGROUND;
        ui_tile = UI_COLOR_TILE;
        ui_accent = themes[index].accent;
        ui_accent_muted = themes[index].muted;
        ui_tile_selected = themes[index].selected;
    }
}

static void app_recolor_tree(lv_obj_t *obj)
{
    if (lv_obj_check_type(obj, &lv_label_class))
    {
        lv_obj_set_style_text_color(obj, lv_color_hex(ui_accent), LV_PART_MAIN);
    }
    else if (lv_obj_check_type(obj, &lv_btn_class))
    {
        lv_obj_set_style_border_color(obj, lv_color_hex(ui_accent_muted), LV_PART_MAIN);
        lv_obj_set_style_bg_color(obj, lv_color_hex(lv_obj_get_parent(obj) == edit_panel ? ui_bg : ui_tile),
                                  LV_PART_MAIN);
        lv_obj_set_style_bg_color(obj, lv_color_hex(ui_tile_selected), LV_STATE_PRESSED);
    }
    const uint32_t count = lv_obj_get_child_cnt(obj);
    for (uint32_t i = 0; i < count; i++)
    {
        app_recolor_tree(lv_obj_get_child(obj, i));
    }
}

static void app_update_page_indicator(void)
{
    const uint16_t page = lv_tabview_get_tab_act(settings_tabview);
    for (size_t i = 0; i < SETTINGS_PAGE_COUNT; i++)
    {
        lv_obj_set_style_bg_color(settings_dots[i],
                                  lv_color_hex(i == page ? ui_accent : ui_accent_muted), LV_PART_MAIN);
        lv_obj_set_style_bg_opa(settings_dots[i], i == page ? LV_OPA_COVER : LV_OPA_50, LV_PART_MAIN);
    }
    lv_label_set_text(settings_title, settings_page_titles[page]);
    lv_obj_align(settings_title, LV_ALIGN_TOP_MID, 0, 8);
}

static void app_update_gset_indicator(void)
{
    const uint16_t page = lv_tabview_get_tab_act(gset_tabview);
    for (size_t i = 0; i < GSET_PAGE_COUNT; i++)
    {
        lv_obj_set_style_bg_color(gset_dots[i], lv_color_hex(i == page ? ui_accent : ui_accent_muted),
                                  LV_PART_MAIN);
        lv_obj_set_style_bg_opa(gset_dots[i], i == page ? LV_OPA_COVER : LV_OPA_50, LV_PART_MAIN);
    }
    lv_label_set_text(gset_title, gset_page_titles[page]);
    lv_obj_align(gset_title, LV_ALIGN_TOP_MID, 0, 8);
}

static lv_obj_t *app_create_text_button(lv_obj_t *parent, lv_coord_t x, lv_coord_t y, lv_coord_t width,
                                        lv_coord_t height, const char *text);

/* Temperature graph screen: swipe left from the main screen, swipe right to return. */
#define GRAPH_CHANNELS 3
#define GRAPH_BOOST 2
#define GRAPH_POINTS 90
#define GRAPH_SAMPLE_MS 2000
#define GRAPH_PANEL_WIDTH 280

static lv_obj_t *graph_panel;
static lv_obj_t *graph_chart;
static lv_obj_t *graph_empty_label;
static lv_obj_t *graph_value_labels[GRAPH_CHANNELS];
static lv_obj_t *graph_tiles[GRAPH_CHANNELS];
static lv_obj_t *graph_icons[GRAPH_CHANNELS];
static lv_chart_series_t *graph_series[GRAPH_CHANNELS];
static int graph_sample_count;
static bool graph_visible;

static const char *const graph_channel_names[GRAPH_CHANNELS] = {"Intake", "Coolant", "Boost"};

static uint32_t app_graph_channel_color(int channel)
{
    return sensor_color_hex[settings.sensor_color[channel]][settings.light_mode ? 1 : 0];
}

static adc_oneshot_unit_handle_t sensor_adc_units[2];
static adc_cali_handle_t sensor_adc_cali[2];
static int sensor_adc_unit[GRAPH_CHANNELS] = {-1, -1, -1};
static adc_channel_t sensor_adc_channel[GRAPH_CHANNELS];

/* (Re)configures the ADC input for a sensor channel from settings.sensor_gpio[]. */
static void app_sensor_configure(int channel)
{
    sensor_adc_unit[channel] = -1;
    const int gpio = settings.sensor_gpio[channel];
    adc_unit_t unit;
    adc_channel_t adc_channel;
    if (gpio == 0 || adc_oneshot_io_to_channel(gpio, &unit, &adc_channel) != ESP_OK)
    {
        return;
    }
    const int u = unit == ADC_UNIT_1 ? 0 : 1;
    if (sensor_adc_units[u] == NULL)
    {
        const adc_oneshot_unit_init_cfg_t init = {.unit_id = unit};
        if (adc_oneshot_new_unit(&init, &sensor_adc_units[u]) != ESP_OK)
        {
            sensor_adc_units[u] = NULL;
            return;
        }
        const adc_cali_curve_fitting_config_t cali = {
            .unit_id = unit, .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
        if (adc_cali_create_scheme_curve_fitting(&cali, &sensor_adc_cali[u]) != ESP_OK)
        {
            sensor_adc_cali[u] = NULL;
        }
    }
    const adc_oneshot_chan_cfg_t config = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
    if (adc_oneshot_config_channel(sensor_adc_units[u], adc_channel, &config) == ESP_OK)
    {
        sensor_adc_channel[channel] = adc_channel;
        sensor_adc_unit[channel] = u;
    }
}

/* Averaged pin voltage in millivolts, or false when the channel has no ADC pin. */
static bool app_sensor_read_mv(int channel, int *mv)
{
    const int u = sensor_adc_unit[channel];
    if (u < 0)
    {
        return false;
    }
    int sum = 0;
    int count = 0;
    for (int i = 0; i < 8; i++)
    {
        int raw = 0;
        if (adc_oneshot_read(sensor_adc_units[u], sensor_adc_channel[channel], &raw) == ESP_OK)
        {
            sum += raw;
            count++;
        }
    }
    if (count == 0)
    {
        return false;
    }
    const int raw = sum / count;
    int voltage = raw * 3100 / 4095;
    if (sensor_adc_cali[u] != NULL)
    {
        adc_cali_raw_to_voltage(sensor_adc_cali[u], raw, &voltage);
    }
    *mv = voltage;
    return true;
}

/* Temperature sensors are NTC thermistors to ground with a pull-up to 3.3 V on the pin. */
#define NTC_PULLUP_OHMS 2200.0f
#define NTC_SUPPLY_MV 3300.0f
typedef struct
{
    int temp_c;
    float ohms;
} ntc_point_t;

/* GM 25036751 intake air sensor (standard GM NTC curve). */
static const ntc_point_t ntc_gm_iat[] = {{-40, 100700}, {-20, 28680}, {0, 9420}, {20, 3520}, {40, 1459},
                                         {60, 667},     {80, 332},    {100, 177}, {120, 100}};
/* Bosch 0280130040 coolant sensor (about 2.5 k at 20 C). */
static const ntc_point_t ntc_bosch_ect[] = {{-40, 48000}, {0, 5900}, {20, 2500}, {40, 1170}, {60, 590},
                                            {80, 320},    {100, 186}, {120, 110}};

/* Pin millivolts to tenths of a degree C, interpolating log(resistance) between table points. */
static bool app_ntc_mv_to_tenths(int mv, const ntc_point_t *table, int count, int *tenths)
{
    if (mv <= 20 || mv >= (int)NTC_SUPPLY_MV - 20)
    {
        return false;
    }
    const float ohms = NTC_PULLUP_OHMS * (float)mv / (NTC_SUPPLY_MV - (float)mv);
    if (ohms >= table[0].ohms)
    {
        *tenths = table[0].temp_c * 10;
        return true;
    }
    for (int i = 1; i < count; i++)
    {
        if (ohms >= table[i].ohms)
        {
            const float f = logf(table[i - 1].ohms / ohms) / logf(table[i - 1].ohms / table[i].ohms);
            *tenths = (int)((table[i - 1].temp_c + f * (table[i].temp_c - table[i - 1].temp_c)) * 10.0f);
            return true;
        }
    }
    *tenths = table[count - 1].temp_c * 10;
    return true;
}

/* Boost sensor: 0.5-4.5 V ratiometric over 0..full-scale kPa absolute, read through a 10k/15k divider. */
#define BOOST_DIVIDER_RATIO 0.6f
static const float boost_full_scale_kpa[] = {250.0f, 300.0f, 400.0f};

static int app_boost_mv_to_tenths_psi(int mv)
{
    const float sensor_v = (float)mv / 1000.0f / BOOST_DIVIDER_RATIO;
    float kpa = (sensor_v - 0.5f) / 4.0f * boost_full_scale_kpa[settings.boost_range_idx];
    if (kpa < 0.0f)
        kpa = 0.0f;
    float psi = (kpa - 101.3f) * 0.145038f;
    if (psi < 0.0f)
        psi = 0.0f;
    return (int)(psi * 10.0f + 0.5f);
}
static bool app_sensor_read_tenths(int channel, int *tenths_c)
{
    int mv;
    if (!app_sensor_read_mv(channel, &mv))
    {
        return false;
    }
    if (channel == GRAPH_BOOST)
    {
        *tenths_c = app_boost_mv_to_tenths_psi(mv);
        return true;
    }
    return channel == 0 ? app_ntc_mv_to_tenths(mv, ntc_gm_iat, (int)(sizeof(ntc_gm_iat) / sizeof(ntc_gm_iat[0])), tenths_c)
                        : app_ntc_mv_to_tenths(mv, ntc_bosch_ect, (int)(sizeof(ntc_bosch_ect) / sizeof(ntc_bosch_ect[0])), tenths_c);
}

static bool app_graph_sample(int channel, int *tenths_c)
{
    if (app_sensor_read_tenths(channel, tenths_c))
    {
        return true;
    }
    if (!settings.demo_graph)
    {
        return false;
    }
    const float t = (float)graph_sample_count * GRAPH_SAMPLE_MS / 1000.0f;
    if (channel == GRAPH_BOOST)
    {
        *tenths_c = (int)(80 + 140 * sinf(t / 12.0f + 2.0f));
        return true;
    }
    *tenths_c = channel == 0 ? (int)(250 + 60 * sinf(t / 25.0f)) : (int)(420 + 90 * sinf(t / 40.0f + 1.0f));
    return true;
}

static void lv_obj_add_flag_if(lv_obj_t *object, bool hide)
{
    if (hide)
    {
        lv_obj_add_flag(object, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_clear_flag(object, LV_OBJ_FLAG_HIDDEN);
    }
}

#define AUTO_FAILSAFE_PRESET APP_PRESET_HIGH
#define AUTO_SLEW_PCT_PER_S 5.0f
#define AUTO_BOOST_START_TENTHS_PSI 100
#define AUTO_MIN_APPROACH_C 3

/* Every 60 s: shrink the gain if the error keeps flipping sign, grow it if the error lingers unsaturated. */
static void app_auto_learn(float error, bool unsaturated)
{
    static int ticks, flips, lag, last_sign;
    static float max_error;
    const int sign = error > 0.5f ? 1 : (error < -0.5f ? -1 : 0);
    if (sign != 0)
    {
        if (last_sign != 0 && sign != last_sign)
        {
            flips++;
        }
        last_sign = sign;
    }
    const float magnitude = error < 0 ? -error : error;
    if (magnitude > max_error)
    {
        max_error = magnitude;
    }
    if (magnitude > 1.5f && unsaturated)
    {
        lag++;
    }
    if (++ticks < 60)
    {
        return;
    }
    const float previous = auto_kp;
    if (flips >= 4 && max_error > 1.0f)
    {
        auto_kp *= 0.85f;
    }
    else if (lag >= 45)
    {
        auto_kp *= 1.15f;
    }
    if (auto_kp < 0.5f)
        auto_kp = 0.5f;
    if (auto_kp > 10.0f)
        auto_kp = 10.0f;
    ticks = flips = lag = 0;
    max_error = 0.0f;
    const int kp10 = (int)(auto_kp * 10.0f + 0.5f);
    if (auto_kp != previous && kp10 != settings.auto_kp10)
    {
        settings.auto_kp10 = kp10;
        nvs_handle_t handle;
        if (nvs_open("pump", NVS_READWRITE, &handle) == ESP_OK)
        {
            nvs_set_i32(handle, "a_kp", kp10);
            nvs_commit(handle);
            nvs_close(handle);
        }
    }
}

/* PI loop on intake temperature; runs once a second while AUTO is selected. */
static void app_auto_timer_cb(lv_timer_t *timer)
{
    (void)timer;
    if (active_preset != APP_PRESET_AUTO)
    {
        return;
    }

    const float min_pct = (float)settings.duty_min_pct;
    const float max_pct = (float)settings.duty_max_pct;
    int intake = 0;
    int coolant = 0;
    int boost = 0;
    const bool intake_ok = app_graph_sample(0, &intake);
    const bool coolant_ok = app_graph_sample(1, &coolant);
    const bool boost_ok = app_graph_sample(GRAPH_BOOST, &boost);

    float wanted;
    if (!intake_ok)
    {
        wanted = (float)preset_percent[AUTO_FAILSAFE_PRESET];
        auto_integral = 0.0f;
    }
    else
    {
        const float kp = auto_kp;
        const float ki = kp * 0.025f;
        const float error = intake / 10.0f - (float)settings.auto_target_c;
        const bool coolant_useless = coolant_ok && (intake - coolant) < AUTO_MIN_APPROACH_C * 10;

        if (!coolant_useless)
        {
            auto_integral += error;
            const float limit = ki > 0.0f ? (max_pct - min_pct) / ki : 0.0f;
            if (auto_integral > limit)
                auto_integral = limit;
            if (auto_integral < 0.0f)
                auto_integral = 0.0f;
        }
        wanted = min_pct + kp * error + ki * auto_integral;
        if (boost_ok && boost > AUTO_BOOST_START_TENTHS_PSI)
        {
            wanted += (boost - AUTO_BOOST_START_TENTHS_PSI) / 10.0f;
        }
        if (coolant_useless)
        {
            wanted = min_pct;
        }
        else
        {
            app_auto_learn(error, wanted > min_pct && wanted < max_pct);
        }
    }
    if (wanted < min_pct)
        wanted = min_pct;
    if (wanted > max_pct)
        wanted = max_pct;

    if (wanted > auto_duty_pct + AUTO_SLEW_PCT_PER_S)
        wanted = auto_duty_pct + AUTO_SLEW_PCT_PER_S;
    if (wanted < auto_duty_pct - AUTO_SLEW_PCT_PER_S)
        wanted = auto_duty_pct - AUTO_SLEW_PCT_PER_S;

    const app_preset_id_t before = app_effective_preset(APP_PRESET_AUTO);
    auto_duty_pct = wanted;
    app_apply_preset(APP_PRESET_AUTO);
    if (app_effective_preset(APP_PRESET_AUTO) != before)
    {
        app_update_pump_animation(APP_PRESET_AUTO);
    }
}

static void app_graph_timer_cb(lv_timer_t *timer)
{
    (void)timer;
    for (int ch = 0; ch < GRAPH_CHANNELS; ch++)
    {
        int tenths = 0;
        if (app_graph_sample(ch, &tenths))
        {
            lv_chart_set_next_value(graph_chart, graph_series[ch], tenths);
            lv_label_set_text_fmt(graph_value_labels[ch], "%d", (tenths + (tenths < 0 ? -5 : 5)) / 10);
        }
        else
        {
            lv_chart_set_next_value(graph_chart, graph_series[ch], LV_CHART_POINT_NONE);
            lv_label_set_text(graph_value_labels[ch], "--");
        }
    }
    graph_sample_count++;

    int low = INT_MAX;
    int high = INT_MIN;
    for (int ch = 0; ch < GRAPH_BOOST; ch++)
    {
        for (int i = 0; i < GRAPH_POINTS; i++)
        {
            const int v = graph_series[ch]->y_points[i];
            if (v == LV_CHART_POINT_NONE)
            {
                continue;
            }
            low = v < low ? v : low;
            high = v > high ? v : high;
        }
    }

    const bool any_temp = low <= high;
    if (any_temp)
    {
        low = (low / 10 - 1) * 10;
        high = (high / 10 + 2) * 10;
        if (high - low < 100)
        {
            high = low + 100;
        }
        lv_chart_set_range(graph_chart, LV_CHART_AXIS_PRIMARY_Y, low, high);
    }

    bool any_boost = false;
    for (int i = 0; i < GRAPH_POINTS && !any_boost; i++)
    {
        any_boost = graph_series[GRAPH_BOOST]->y_points[i] != LV_CHART_POINT_NONE;
    }
    lv_obj_add_flag_if(graph_empty_label, any_temp || any_boost);
    lv_chart_refresh(graph_chart);
}

static void app_page_dots_recolor(void);
static void app_paint_gradient(lv_obj_t *canvas, int width, int height);

/* The gradient only varies by row, so a 4 px wide strip (one dither period) is tiled across the panel. */
static lv_obj_t *graph_bg;
static lv_color_t graph_bg_buffer[4 * 240];

static void app_graph_icon_recolor(lv_obj_t *icon, uint32_t color)
{
    const uint32_t count = lv_obj_get_child_cnt(icon);
    for (uint32_t i = 0; i < count; i++)
    {
        lv_obj_t *part = lv_obj_get_child(icon, i);
        if (lv_obj_check_type(part, &lv_arc_class))
        {
            lv_obj_set_style_arc_color(part, lv_color_hex(color), LV_PART_MAIN);
        }
        else if (lv_obj_check_type(part, &lv_line_class))
        {
            lv_obj_set_style_line_color(part, lv_color_hex(color), LV_PART_MAIN);
        }
        else
        {
            lv_obj_set_style_bg_color(part, lv_color_hex(color), LV_PART_MAIN);
        }
    }
}

static void app_graph_apply_theme(void)
{
    app_page_dots_recolor();
    if (graph_panel == NULL)
    {
        return;
    }
    lv_obj_set_style_bg_color(graph_panel, lv_color_hex(ui_bg), LV_PART_MAIN);
    app_paint_gradient(graph_bg, 4, 240);
    lv_obj_set_style_border_color(graph_chart, lv_color_hex(ui_accent_muted), LV_PART_MAIN);
    lv_obj_set_style_bg_color(graph_chart, lv_color_hex(ui_tile), LV_PART_MAIN);
    lv_obj_set_style_border_color(graph_chart, lv_color_hex(ui_accent_muted), LV_PART_MAIN);
    lv_obj_set_style_line_color(graph_chart, lv_color_hex(ui_accent_muted), LV_PART_MAIN);
    for (int ch = 0; ch < GRAPH_CHANNELS; ch++)
    {
        lv_chart_set_series_color(graph_chart, graph_series[ch], lv_color_hex(app_graph_channel_color(ch)));
        lv_obj_set_style_text_color(graph_value_labels[ch], lv_color_hex(app_graph_channel_color(ch)),
                                    LV_PART_MAIN);
        lv_obj_set_style_bg_color(graph_tiles[ch], lv_color_hex(ui_tile), LV_PART_MAIN);
        lv_obj_set_style_border_color(graph_tiles[ch], lv_color_hex(ui_accent_muted), LV_PART_MAIN);
        app_graph_icon_recolor(graph_icons[ch], app_graph_channel_color(ch));
    }
    lv_obj_set_style_text_color(graph_empty_label, lv_color_hex(ui_accent_muted), LV_PART_MAIN);
    lv_obj_invalidate(graph_chart);
}

static void app_graph_x_exec_cb(void *object, int32_t x)
{
    lv_obj_set_x((lv_obj_t *)object, x);
}

static void app_graph_slid_out_cb(lv_anim_t *animation)
{
    (void)animation;
    lv_obj_add_flag(graph_panel, LV_OBJ_FLAG_HIDDEN);
}

static void app_graph_slide(bool show)
{
    lv_anim_del(graph_panel, app_graph_x_exec_cb);
    lv_anim_t animation;
    lv_anim_init(&animation);
    lv_anim_set_var(&animation, graph_panel);
    lv_anim_set_exec_cb(&animation, app_graph_x_exec_cb);
    lv_anim_set_values(&animation, lv_obj_get_x(graph_panel), show ? 0 : GRAPH_PANEL_WIDTH);
    lv_anim_set_time(&animation, 220);
    lv_anim_set_path_cb(&animation, lv_anim_path_ease_out);
    if (!show)
    {
        lv_anim_set_ready_cb(&animation, app_graph_slid_out_cb);
    }
    lv_anim_start(&animation);
}

static void app_graph_show(void)
{
    graph_visible = true;
    lv_obj_set_x(graph_panel, GRAPH_PANEL_WIDTH);
    lv_obj_clear_flag(graph_panel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(graph_panel);
    app_graph_slide(true);
}

static void app_graph_hide(void)
{
    graph_visible = false;
    app_graph_slide(false);
}

static lv_obj_t *graph_dots[2];
static lv_obj_t *main_dots[2];

static void app_page_dots_create(lv_obj_t *parent, lv_obj_t **dots, int active)
{
    for (int i = 0; i < 2; i++)
    {
        dots[i] = lv_obj_create(parent);
        lv_obj_remove_style_all(dots[i]);
        lv_obj_clear_flag(dots[i], LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_size(dots[i], 6, 6);
        lv_obj_set_pos(dots[i], 134 + i * 12, 234);
        lv_obj_set_style_radius(dots[i], LV_RADIUS_CIRCLE, LV_PART_MAIN);
        lv_obj_set_style_bg_color(dots[i], lv_color_hex(i == active ? ui_accent : ui_accent_muted),
                                  LV_PART_MAIN);
        lv_obj_set_style_bg_opa(dots[i], i == active ? LV_OPA_COVER : LV_OPA_50, LV_PART_MAIN);
    }
}

static void app_page_dots_recolor(void)
{
    lv_obj_t **sets[2] = {main_dots, graph_dots};
    for (int s = 0; s < 2; s++)
    {
        if (sets[s][0] == NULL)
        {
            continue;
        }
        for (int i = 0; i < 2; i++)
        {
            const bool active = (i == s);
            lv_obj_set_style_bg_color(sets[s][i], lv_color_hex(active ? ui_accent : ui_accent_muted),
                                      LV_PART_MAIN);
        }
    }
}

/* Polls the touch input directly so swipes work wherever the finger lands, in both directions. */
static void app_swipe_timer_cb(lv_timer_t *timer)
{
    (void)timer;
    static bool was_pressed;
    static lv_point_t start;
    lv_indev_t *indev = lv_indev_get_next(NULL);
    if (indev == NULL)
    {
        return;
    }
    const bool pressed = indev->proc.state == LV_INDEV_STATE_PRESSED;
    lv_point_t now = indev->proc.types.pointer.act_point;
    if (pressed && !was_pressed)
    {
        start = now;
    }
    else if (!pressed && was_pressed)
    {
        const int dx = now.x - start.x;
        const int dy = now.y - start.y;
        const bool overlay_open = !lv_obj_has_flag(settings_panel, LV_OBJ_FLAG_HIDDEN) ||
                                  !lv_obj_has_flag(gset_panel, LV_OBJ_FLAG_HIDDEN) ||
                                  !lv_obj_has_flag(edit_panel, LV_OBJ_FLAG_HIDDEN);
        if (!overlay_open && abs(dx) > 40 && abs(dx) > 2 * abs(dy))
        {
            if (dx < 0 && !graph_visible)
            {
                app_graph_show();
            }
            else if (dx > 0 && graph_visible)
            {
                app_graph_hide();
            }
        }
    }
    was_pressed = pressed;
}
static lv_obj_t *app_graph_icon_part(lv_obj_t *parent, lv_coord_t x, lv_coord_t y, lv_coord_t w, lv_coord_t h)
{
    lv_obj_t *part = lv_obj_create(parent);
    lv_obj_clear_flag(part, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_size(part, w, h);
    lv_obj_set_pos(part, x, y);
    lv_obj_set_style_radius(part, LV_RADIUS_CIRCLE, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(part, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(part, 0, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(part, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(part, 0, LV_PART_MAIN);
    return part;
}

static lv_obj_t *app_graph_icon_line(lv_obj_t *parent, const lv_point_t *points, uint16_t count)
{
    lv_obj_t *line = lv_line_create(parent);
    lv_line_set_points(line, points, count);
    lv_obj_set_pos(line, 0, 0);
    lv_obj_set_style_line_width(line, 2, LV_PART_MAIN);
    lv_obj_set_style_line_rounded(line, true, LV_PART_MAIN);
    return line;
}

/* Intake: wind, Coolant: water drop, Boost: turbocharger. */
static lv_obj_t *app_graph_icon_create(lv_obj_t *parent, int channel)
{
    static const lv_point_t wind_a[] = {{1, 8}, {13, 8}, {17, 7}, {19, 4}, {17, 1}, {14, 2}};
    static const lv_point_t wind_b[] = {{1, 13}, {21, 13}};
    static const lv_point_t wind_c[] = {{1, 18}, {11, 18}, {15, 19}, {17, 22}, {15, 24}, {12, 23}};
    static const lv_point_t drop[] = {{12, 1}, {18, 10}, {19, 15}, {17, 20}, {12, 23},
                                      {7, 20}, {5, 15}, {6, 10}, {12, 1}};
    static const lv_point_t blade_a[] = {{12, 12}, {11, 6}, {15, 2}};
    static const lv_point_t blade_b[] = {{12, 12}, {18, 14}, {19, 20}};
    static const lv_point_t blade_c[] = {{12, 12}, {7, 16}, {2, 14}};

    lv_obj_t *icon = lv_obj_create(parent);
    lv_obj_remove_style_all(icon);
    lv_obj_set_size(icon, 24, 24);
    lv_obj_clear_flag(icon, LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);

    if (channel == GRAPH_BOOST)
    {
        lv_obj_t *blades[3] = {app_graph_icon_line(icon, blade_a, 3), app_graph_icon_line(icon, blade_b, 3),
                               app_graph_icon_line(icon, blade_c, 3)};
        for (int i = 0; i < 3; i++)
        {
            lv_obj_set_style_line_width(blades[i], 5, LV_PART_MAIN);
        }
        app_graph_icon_part(icon, 9, 9, 6, 6);
    }
    else if (channel == 0)
    {
        app_graph_icon_line(icon, wind_a, 6);
        app_graph_icon_line(icon, wind_b, 2);
        app_graph_icon_line(icon, wind_c, 6);
    }
    else
    {
        app_graph_icon_line(icon, drop, 9);
    }
    return icon;
}

static void app_create_graph_ui(lv_obj_t *screen)
{
    graph_panel = lv_obj_create(screen);
    lv_obj_set_size(graph_panel, 280, 240);
    lv_obj_set_pos(graph_panel, 0, 0);
    lv_obj_clear_flag(graph_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(graph_panel, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(graph_panel, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(graph_panel, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(graph_panel, LV_OPA_COVER, LV_PART_MAIN);

    graph_bg = lv_canvas_create(graph_panel);
    lv_canvas_set_buffer(graph_bg, graph_bg_buffer, 4, 240, LV_IMG_CF_TRUE_COLOR);
    lv_obj_set_size(graph_bg, 280, 240);
    lv_obj_set_pos(graph_bg, 0, 0);
    lv_obj_clear_flag(graph_bg, LV_OBJ_FLAG_CLICKABLE);


    graph_chart = lv_chart_create(graph_panel);
    lv_obj_set_size(graph_chart, 264, 134);
    lv_obj_set_pos(graph_chart, 8, 44);
    lv_obj_clear_flag(graph_chart, LV_OBJ_FLAG_SCROLLABLE);
    lv_chart_set_type(graph_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(graph_chart, GRAPH_POINTS);
    lv_chart_set_div_line_count(graph_chart, 0, 0);
    lv_obj_set_style_radius(graph_chart, 10, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(graph_chart, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(graph_chart, 1, LV_PART_MAIN);
    lv_obj_set_style_line_opa(graph_chart, LV_OPA_40, LV_PART_MAIN);
    lv_obj_set_style_pad_all(graph_chart, 6, LV_PART_MAIN);
    lv_obj_set_style_line_width(graph_chart, 4, LV_PART_ITEMS);
    lv_obj_set_style_size(graph_chart, 0, LV_PART_INDICATOR);
    /* Series added first are drawn first (underneath), so add Boost first and Intake last. */
    for (int ch = GRAPH_CHANNELS - 1; ch >= 0; ch--)
    {
        graph_series[ch] = lv_chart_add_series(graph_chart, lv_color_hex(app_graph_channel_color(ch)),
                                               ch == GRAPH_BOOST ? LV_CHART_AXIS_SECONDARY_Y : LV_CHART_AXIS_PRIMARY_Y);
        lv_chart_set_all_value(graph_chart, graph_series[ch], LV_CHART_POINT_NONE);
    }

    graph_empty_label = lv_label_create(graph_panel);
    lv_chart_set_range(graph_chart, LV_CHART_AXIS_SECONDARY_Y, 0, 300);
    lv_label_set_text(graph_empty_label, "No sensors connected");
    lv_obj_set_style_text_font(graph_empty_label, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_align_to(graph_empty_label, graph_chart, LV_ALIGN_CENTER, 0, 0);

    for (int ch = 0; ch < GRAPH_CHANNELS; ch++)
    {
        graph_tiles[ch] = lv_obj_create(graph_panel);
        lv_obj_set_size(graph_tiles[ch], 84, 44);
        lv_obj_set_pos(graph_tiles[ch], 8 + ch * 90, 188);
        lv_obj_clear_flag(graph_tiles[ch], LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_style_radius(graph_tiles[ch], 10, LV_PART_MAIN);
        lv_obj_set_style_bg_opa(graph_tiles[ch], LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_border_width(graph_tiles[ch], 1, LV_PART_MAIN);
        lv_obj_set_style_shadow_width(graph_tiles[ch], 0, LV_PART_MAIN);
        lv_obj_set_style_pad_all(graph_tiles[ch], 0, LV_PART_MAIN);

        graph_icons[ch] = app_graph_icon_create(graph_tiles[ch], ch);
        lv_obj_set_pos(graph_icons[ch], 6, 9);

        graph_value_labels[ch] = lv_label_create(graph_tiles[ch]);
        lv_obj_set_style_text_font(graph_value_labels[ch], &lv_font_montserrat_24, LV_PART_MAIN);
        lv_obj_align(graph_value_labels[ch], LV_ALIGN_RIGHT_MID, -5, 0);
    }

    for (int ch = 0; ch < GRAPH_CHANNELS; ch++)
    {
        app_sensor_configure(ch);
    }
    app_graph_timer_cb(NULL);
    lv_timer_create(app_graph_timer_cb, GRAPH_SAMPLE_MS, NULL);
    app_page_dots_create(graph_panel, graph_dots, 1);
    lv_timer_create(app_swipe_timer_cb, 30, NULL);
    lv_obj_add_flag(graph_panel, LV_OBJ_FLAG_HIDDEN);
    app_graph_apply_theme();
}

/* Dark vertical gradient whose bottom edge carries a faint tint of the accent color. */
static lv_obj_t *background_canvas;
static lv_color_t *background_buffer;

/* Vertical gradient drawn with 4x4 ordered dithering; LVGL's built-in dither still bands on RGB565. */
static void app_apply_background(void)
{
    const int width = 280;
    const int height = 240;
    lv_obj_t *screen = lv_scr_act();
    lv_obj_set_style_bg_color(screen, lv_color_hex(ui_bg), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);
    if (background_buffer == NULL)
    {
        background_buffer = heap_caps_malloc((size_t)width * height * sizeof(lv_color_t), MALLOC_CAP_8BIT);
        if (background_buffer == NULL)
        {
            return;
        }
    }
    if (background_canvas == NULL || !lv_obj_is_valid(background_canvas))
    {
        background_canvas = lv_canvas_create(screen);
        lv_canvas_set_buffer(background_canvas, background_buffer, width, height, LV_IMG_CF_TRUE_COLOR);
        lv_obj_set_pos(background_canvas, 0, 0);
        lv_obj_clear_flag(background_canvas, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_move_background(background_canvas);
    }

    app_paint_gradient(background_canvas, width, height);
}

static void app_paint_gradient(lv_obj_t *canvas, int width, int height)
{
    static const uint8_t bayer[4][4] = {{0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}};
    const lv_color_t base = lv_color_hex(ui_bg);
    const lv_color_t top = lv_color_mix(lv_color_hex(ui_accent_bright), base, UI_BACKGROUND_TINT);
    const int c0[3] = {LV_COLOR_GET_R(base) * 255 / 31, LV_COLOR_GET_G(base) * 255 / 63, LV_COLOR_GET_B(base) * 255 / 31};
    const int c1[3] = {LV_COLOR_GET_R(top) * 255 / 31, LV_COLOR_GET_G(top) * 255 / 63, LV_COLOR_GET_B(top) * 255 / 31};
    const int levels[3] = {31, 63, 31};

    for (int y = 0; y < height; y++)
    {
        int row[3];
        for (int c = 0; c < 3; c++)
        {
            const int ramp = y * y / (height - 1);
            const int v = (c0[c] * 256) + (c1[c] - c0[c]) * 256 * ramp / (height - 1);
            row[c] = v * levels[c] / 255;
        }
        for (int x = 0; x < width; x++)
        {
            const int dither = bayer[y & 3][x & 3] * 16;
            int q[3];
            for (int c = 0; c < 3; c++)
            {
                q[c] = (row[c] + dither) >> 8;
                if (q[c] > levels[c])
                {
                    q[c] = levels[c];
                }
                q[c] = q[c] * 255 / levels[c];
            }
            lv_canvas_set_px_color(canvas, x, y, lv_color_make(q[0], q[1], q[2]));
        }
    }
    lv_obj_invalidate(canvas);
}

/* Re-apply the current accent colors to widgets that were already created. */
static void app_apply_theme(void)
{
    app_recolor_tree(lv_scr_act());
    app_graph_apply_theme();
    lv_obj_set_style_bg_color(settings_panel, lv_color_hex(ui_bg), LV_PART_MAIN);
    lv_obj_set_style_bg_color(gset_panel, lv_color_hex(ui_bg), LV_PART_MAIN);
    app_update_gset_indicator();
    lv_obj_set_style_bg_color(edit_panel, lv_color_hex(ui_tile), LV_PART_MAIN);
    app_apply_background();
    app_update_flow(active_preset);
    app_update_page_indicator();
    lv_obj_set_style_img_recolor(pump_glow, lv_color_hex(ui_accent), LV_PART_MAIN);
    lv_obj_set_style_border_color(edit_panel, lv_color_hex(ui_accent), LV_PART_MAIN);
    for (size_t i = 0; i < PRESET_COUNT; i++)
    {
        if (preset_buttons[i] == NULL)
        {
            continue;
        }
        lv_obj_set_style_border_color(preset_buttons[i], lv_color_hex(ui_accent), LV_STATE_PRESSED);
    }
    app_update_preset_button_styles(active_preset);
}

static void app_update_settings_labels(void)
{
    lv_label_set_text_fmt(settings_labels[0], "PWM freq %d Hz", settings.pwm_freq_hz);
    lv_label_set_text_fmt(settings_labels[1], "Min duty %d%%", settings.duty_min_pct);
    lv_label_set_text_fmt(settings_labels[2], "Max duty %d%%", settings.duty_max_pct);
    lv_label_set_text_fmt(settings_labels[3], "Backlight %d%%", settings.backlight_pct);
    lv_label_set_text_fmt(settings_labels[4], "Screen off %s", timeout_names[settings.timeout_idx]);
    lv_label_set_text_fmt(settings_labels[5], "Accent %s", themes[settings.accent_idx].name);
    lv_label_set_text_fmt(settings_labels[6], "Theme %s", settings.light_mode ? "Light" : "Dark");
    lv_label_set_text_fmt(settings_labels[7], "Flow %s", flow_names[settings.flow_idx]);
    lv_label_set_text_fmt(settings_labels[8], "Motion %s", motion_names[settings.motion_idx]);
        lv_label_set_text_fmt(settings_labels[9], "PWM pin GPIO%d", settings.pwm_gpio);
    lv_label_set_text_fmt(settings_labels[10], "Sound %s", sound_names[settings.sound_idx]);
    static const char *const boost_range_names[] = {"2.5 bar", "3 bar", "4 bar"};
    lv_label_set_text_fmt(settings_labels[20], "Boost sensor %s", boost_range_names[settings.boost_range_idx]);
    lv_label_set_text_fmt(settings_labels[18], "Target %d C", settings.auto_target_c);
    lv_label_set_text_fmt(settings_labels[19], "Learned gain %d.%d", (int)(auto_kp * 10.0f + 0.5f) / 10, (int)(auto_kp * 10.0f + 0.5f) % 10);
    lv_label_set_text_fmt(settings_labels[11], "Graph demo %s", settings.demo_graph ? "On" : "Off");
    for (int ch = 0; ch < GRAPH_CHANNELS; ch++)
    {
        if (settings.sensor_gpio[ch] == 0)
        {
            lv_label_set_text_fmt(settings_labels[12 + ch * 2], "%s  Off", graph_channel_names[ch]);
        }
        else
        {
            lv_label_set_text_fmt(settings_labels[12 + ch * 2], "%s  GPIO%d", graph_channel_names[ch], settings.sensor_gpio[ch]);
        }
        lv_label_set_text_fmt(settings_labels[13 + ch * 2], "%s  %s", graph_channel_names[ch], sensor_color_names[settings.sensor_color[ch]]);
    }
    }

    static void app_set_pwm_gpio(int gpio)
    {
        ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        gpio_reset_pin((gpio_num_t)settings.pwm_gpio);
        settings.pwm_gpio = gpio;
        if (app_control_init() != ESP_OK)
        {
            ESP_LOGE(TAG, "Could not move PWM output to GPIO%d", gpio);
        }
        app_apply_preset(active_preset);
    }

static void app_set_pwm_frequency(int hz)
{
    const int previous_hz = settings.pwm_freq_hz;
    if (ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, (uint32_t)hz) != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not set PWM frequency to %d Hz", hz);
        ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, (uint32_t)previous_hz);
        return;
    }
    settings.pwm_freq_hz = hz;
    app_apply_preset(active_preset);
}

/* User data encodes the row and direction: row * 2 + (increase ? 1 : 0). */
static bool app_sensor_pin_in_use(int gpio, int except_channel)
{
    for (int ch = 0; ch < GRAPH_CHANNELS; ch++)
    {
        if (ch != except_channel && settings.sensor_gpio[ch] == gpio)
        {
            return true;
        }
    }
    return false;
}

static void app_setting_adjust_cb(lv_event_t *event)
{
    const intptr_t code = (intptr_t)lv_event_get_user_data(event);
    const int row = (int)(code / 2);
    const int dir = (code % 2) ? 1 : -1;

    switch (row)
    {
    case 0:
    {
        const int hz = app_clamp_int(settings.pwm_freq_hz + dir * PUMP_PWM_FREQUENCY_STEP_HZ,
                                     PUMP_PWM_FREQUENCY_MIN_HZ, PUMP_PWM_FREQUENCY_MAX_HZ);
        if (hz != settings.pwm_freq_hz)
        {
            app_set_pwm_frequency(hz);
        }
        break;
    }
    case 1:
    case 2:
    {
        int *limit = row == 1 ? &settings.duty_min_pct : &settings.duty_max_pct;
        const int low = row == 1 ? 1 : settings.duty_min_pct + DUTY_LIMIT_MIN_GAP_PERCENT;
        const int high = row == 1 ? settings.duty_max_pct - DUTY_LIMIT_MIN_GAP_PERCENT : 100;
        const int value = app_clamp_int(*limit + dir, low, high);
        if (value != *limit)
        {
            *limit = value;
            app_clamp_presets();
            app_apply_preset(active_preset);
        }
        break;
    }
    case 3:
        settings.backlight_pct = app_clamp_int(settings.backlight_pct + dir * BACKLIGHT_STEP_PERCENT,
                                               BACKLIGHT_MIN_PERCENT, 100);
        app_backlight_set(settings.backlight_pct);
        break;
    case 4:
        settings.timeout_idx = app_clamp_int(settings.timeout_idx + dir, 0, (int)TIMEOUT_COUNT - 1);
        break;
    case 5:
        settings.accent_idx = (settings.accent_idx + dir + (int)THEME_COUNT) % (int)THEME_COUNT;
        app_select_theme(settings.accent_idx);
        app_apply_theme();
        break;
    case 6:
        settings.light_mode = !settings.light_mode;
        app_select_theme(settings.accent_idx);
        app_apply_theme();
        break;
    case 7:
        settings.flow_idx = (settings.flow_idx + dir + FLOW_STYLE_COUNT) % FLOW_STYLE_COUNT;
        app_update_flow(active_preset);
        break;
    case 8:
        settings.motion_idx = (settings.motion_idx + dir + MOTION_COUNT) % MOTION_COUNT;
        app_update_pump_animation(active_preset);
        break;
    case 9:
    {
        int index = 0;
        for (int i = 0; i < PWM_GPIO_OPTION_COUNT; i++)
        {
            if (pwm_gpio_options[i] == settings.pwm_gpio)
            {
                index = i;
            }
        }
        for (int next = index + dir; next >= 0 && next < PWM_GPIO_OPTION_COUNT; next += dir)
        {
            if (!app_sensor_pin_in_use(pwm_gpio_options[next], -1))
            {
                app_set_pwm_gpio(pwm_gpio_options[next]);
                break;
            }
        }
        break;
    }
    case 10:
        settings.sound_idx = (settings.sound_idx + dir + SOUND_COUNT) % SOUND_COUNT;
        app_buzzer_click();
        break;
    case 11:
        settings.demo_graph = !settings.demo_graph;
        break;
    case 20:
        settings.boost_range_idx = app_clamp_int(settings.boost_range_idx + dir, 0, 2);
        break;
    case 18:
        settings.auto_target_c = app_clamp_int(settings.auto_target_c + dir, 15, 60);
        break;
    case 19:
        auto_kp = 2.0f;
        settings.auto_kp10 = 20;
        break;
    case 13:
    case 15:
    case 17:
    {
        const int ch = (row - 13) / 2;
        settings.sensor_color[ch] = (settings.sensor_color[ch] + dir + SENSOR_COLOR_COUNT) % SENSOR_COLOR_COUNT;
        app_graph_apply_theme();
        break;
    }
    case 12:
    case 14:
    case 16:
    {
        const int ch = (row - 12) / 2;
        int index = 0;
        for (int i = 0; i < SENSOR_GPIO_OPTION_COUNT; i++)
        {
            if (sensor_gpio_options[i] == settings.sensor_gpio[ch])
            {
                index = i;
            }
        }
        for (int next = index + dir; next >= 0 && next < SENSOR_GPIO_OPTION_COUNT; next += dir)
        {
            const int gpio = sensor_gpio_options[next];
            if (gpio == 0 || (gpio != settings.pwm_gpio && !app_sensor_pin_in_use(gpio, ch)))
            {
                settings.sensor_gpio[ch] = gpio;
                app_sensor_configure(ch);
                break;
            }
        }
        break;
    }
    }
    app_update_settings_labels();
}

static void app_settings_open_cb(lv_event_t *event)
{
    (void)event;
    if (!lv_obj_has_flag(edit_panel, LV_OBJ_FLAG_HIDDEN))
    {
        app_save_preset_percent(editing_preset);
        app_panel_hide(edit_panel);
    }
    app_update_settings_labels();
    lv_tabview_set_act(settings_tabview, 0, LV_ANIM_OFF);
    app_update_page_indicator();
    app_panel_show(settings_panel);
}

static void app_settings_done_cb(lv_event_t *event)
{
    (void)event;
    app_save_settings();
    app_panel_hide(settings_panel);
}

static void app_gset_open_cb(lv_event_t *event)
{
    (void)event;
    app_update_settings_labels();
    lv_tabview_set_act(gset_tabview, 0, LV_ANIM_OFF);
    app_update_gset_indicator();
    app_panel_show(gset_panel);
}

static void app_gset_done_cb(lv_event_t *event)
{
    (void)event;
    app_save_settings();
    app_panel_hide(gset_panel);
}

static void app_gset_page_cb(lv_event_t *event)
{
    (void)event;
    app_update_gset_indicator();
}

/* The overlay swallows the waking touch so it doesn't also press a button underneath. */
static void app_wake_screen_cb(lv_event_t *event)
{
    (void)event;
    screen_asleep = false;
    lv_obj_add_flag(wake_overlay, LV_OBJ_FLAG_HIDDEN);
    app_backlight_set(settings.backlight_pct);
    lv_disp_trig_activity(lvgl_disp);
}

static void app_sleep_timer_cb(lv_timer_t *timer)
{
    (void)timer;
    const int timeout_s = timeout_seconds[settings.timeout_idx];
    if (timeout_s == 0 || screen_asleep)
    {
        return;
    }
    if (lv_disp_get_inactive_time(lvgl_disp) >= (uint32_t)timeout_s * 1000)
    {
        screen_asleep = true;
        app_backlight_set(0);
        lv_obj_clear_flag(wake_overlay, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(wake_overlay);
    }
}

static void app_pump_wakeup(void)
{
    /* Brief pump wake pulse, followed by an off interval before normal PWM control. */
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_LOGI(TAG, "Pump wake cycle started");
    ESP_LOGI(TAG, "Wake pulse 1: duty=255/256 for 30 ms");
    pump_set_duty(255);
    vTaskDelay(pdMS_TO_TICKS(30));
    ESP_LOGI(TAG, "Wake interval 1: duty=0 for 100 ms");
    pump_set_duty(0);
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Wake pulse 2: duty=25/256 for 30 ms");
    pump_set_duty(25);
    vTaskDelay(pdMS_TO_TICKS(30));
    ESP_LOGI(TAG, "Wake interval 2: duty=0 for 100 ms");
    pump_set_duty(0);
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Pump wake cycle complete");
}

static void app_splash_opacity_exec_cb(void *object, int32_t opacity)
{
    lv_obj_set_style_img_opa(object, (lv_opa_t)opacity, LV_PART_MAIN);
}

static void app_splash_fade_out(void)
{
    lvgl_port_lock(0);

    lv_anim_t animation;
    lv_anim_init(&animation);
    lv_anim_set_var(&animation, splash_logo);
    lv_anim_set_exec_cb(&animation, app_splash_opacity_exec_cb);
    lv_anim_set_values(&animation, LV_OPA_COVER, LV_OPA_TRANSP);
    lv_anim_set_time(&animation, APP_SPLASH_FADE_DURATION_MS);
    lv_anim_set_path_cb(&animation, lv_anim_path_ease_in_out);
    lv_anim_start(&animation);

    lvgl_port_unlock();
}

static void app_wait_until_us(int64_t deadline_us)
{
    int64_t remaining_us = deadline_us - esp_timer_get_time();

    while (remaining_us > 0)
    {
        vTaskDelay(pdMS_TO_TICKS((remaining_us + 999) / 1000));
        remaining_us = deadline_us - esp_timer_get_time();
    }
}

static esp_err_t app_lcd_init(void)
{
    esp_err_t ret = ESP_OK;

    /* Set up SPI and the ST7789 panel on the board's fixed display pins. */
    ESP_LOGD(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = {
        .sclk_io_num = EXAMPLE_LCD_GPIO_SCLK,
        .mosi_io_num = EXAMPLE_LCD_GPIO_MOSI,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_DRAW_BUFF_HEIGHT * sizeof(uint16_t),
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(EXAMPLE_LCD_SPI_NUM, &buscfg, SPI_DMA_CH_AUTO), TAG, "SPI init failed");

    ESP_LOGD(TAG, "Install panel IO");
    const esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = EXAMPLE_LCD_GPIO_DC,
        .cs_gpio_num = EXAMPLE_LCD_GPIO_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLK_HZ,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)EXAMPLE_LCD_SPI_NUM, &io_config, &lcd_io), err, TAG, "New panel IO failed");

    ESP_LOGD(TAG, "Install LCD driver");
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_LCD_GPIO_RST,
        .color_space = EXAMPLE_LCD_COLOR_SPACE,
        .bits_per_pixel = EXAMPLE_LCD_BITS_PER_PIXEL,
    };
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_st7789(lcd_io, &panel_config, &lcd_panel), err, TAG, "New panel failed");

    esp_lcd_panel_reset(lcd_panel);
    esp_lcd_panel_init(lcd_panel);
    /* These mirrors match the panel's mounting orientation in portrait mode. */
    esp_lcd_panel_mirror(lcd_panel, true, true);
    esp_lcd_panel_set_gap(lcd_panel, 0, 20);
    esp_lcd_panel_invert_color(lcd_panel, true);

    /* Panel RAM is random at power-up; fill it with black before the display and backlight turn on. */
    const int clear_rows = 20;
    uint16_t *black = heap_caps_calloc(EXAMPLE_LCD_H_RES * clear_rows, sizeof(uint16_t),
                                       MALLOC_CAP_DMA);
    if (black != NULL)
    {
        for (int y = 0; y < EXAMPLE_LCD_V_RES; y += clear_rows)
        {
            esp_lcd_panel_draw_bitmap(lcd_panel, 0, y, EXAMPLE_LCD_H_RES, y + clear_rows, black);
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        vTaskDelay(pdMS_TO_TICKS(20));
        heap_caps_free(black);
    }
    esp_lcd_panel_disp_on_off(lcd_panel, true);

    return ret;

err:
    if (lcd_panel)
    {
        esp_lcd_panel_del(lcd_panel);
    }
    if (lcd_io)
    {
        esp_lcd_panel_io_del(lcd_io);
    }
    spi_bus_free(EXAMPLE_LCD_SPI_NUM);
    return ret;
}

#if EXAMPLE_USE_TOUCH
static void example_lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    esp_lcd_touch_handle_t tp = (esp_lcd_touch_handle_t)drv->user_data;
    assert(tp);

    /* Read raw CST816S coordinates; configured mirror flags and LVGL rotation
     * transform them to match the landscape UI. */
    esp_lcd_touch_read_data(tp);
    esp_lcd_touch_point_data_t touch_points[1];
    uint8_t touch_count = 0;
    if (esp_lcd_touch_get_data(tp, touch_points, &touch_count, 1) == ESP_OK && touch_count > 0)
    {
        data->point.x = touch_points[0].x;
        data->point.y = touch_points[0].y;
        data->state = LV_INDEV_STATE_PRESSED;
        ESP_LOGD(TAG, "Touch position: %d,%d", touch_points[0].x, touch_points[0].y);
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}
#endif

static esp_err_t app_lvgl_init(void)
{
    /* Initialize LVGL */
    const lvgl_port_cfg_t lvgl_cfg = {
        .task_priority = 4,       /* LVGL task priority */
        .task_stack = 4096,       /* LVGL task stack size */
        .task_affinity = -1,      /* LVGL task pinned to core (-1 is no affinity) */
        .task_max_sleep_ms = 20,  /* Short enough that newly started animations are not slept through */
        .timer_period_ms = 5      /* LVGL timer tick period in ms */
    };
    ESP_RETURN_ON_ERROR(lvgl_port_init(&lvgl_cfg), TAG, "LVGL port initialization failed");

    /* Add LCD screen */
    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = lcd_io,
        .panel_handle = lcd_panel,
        .buffer_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_DRAW_BUFF_HEIGHT * sizeof(uint16_t),
        .double_buffer = EXAMPLE_LCD_DRAW_BUFF_DOUBLE,
        .hres = EXAMPLE_LCD_H_RES,
        .vres = EXAMPLE_LCD_V_RES,
        .monochrome = false,
        /* Rotation values must be same as used in esp_lcd for initial settings of the screen */
        .rotation = {
            /* Match app_lcd_init()'s initial controller orientation. */
            .swap_xy = false,
            .mirror_x = true,
            .mirror_y = true,
        },
        .flags = {
            .buff_dma = true,
            /* Rotate pixels in LVGL while leaving the ST7789 in its known-good mode. */
            .sw_rotate = true,
        }};
    lvgl_disp = lvgl_port_add_disp(&disp_cfg);
    lvgl_port_lock(0);
    /* 270 degrees keeps the landscape layout and turns it 180 degrees from 90. */
    lv_disp_set_rotation(lvgl_disp, LV_DISP_ROT_270);
    lvgl_port_unlock();

    return ESP_OK;
}

static lv_obj_t *app_create_text_button(lv_obj_t *parent, lv_coord_t x, lv_coord_t y, lv_coord_t width,
                                        lv_coord_t height, const char *text)
{
    lv_obj_t *button = lv_btn_create(parent);
    lv_obj_set_size(button, width, height);
    lv_obj_set_pos(button, x, y);
    lv_obj_set_style_radius(button, 10, LV_PART_MAIN);
    lv_obj_set_style_bg_color(button, lv_color_hex(ui_tile), LV_PART_MAIN);
    lv_obj_set_style_bg_color(button, lv_color_hex(ui_tile_selected), LV_STATE_PRESSED);
    lv_obj_set_style_border_color(button, lv_color_hex(ui_accent_muted), LV_PART_MAIN);
    lv_obj_set_style_border_width(button, 1, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(button, 0, LV_PART_MAIN);
    lv_obj_add_event_cb(button, app_click_sound_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_t *label = lv_label_create(button);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_color(label, lv_color_hex(ui_accent), LV_PART_MAIN);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_center(label);
    return button;
}

static void app_settings_page_cb(lv_event_t *event)
{
    (void)event;
    app_update_page_indicator();
}

static void app_create_graph_settings_ui(void)
{
    lv_obj_t *gear = app_create_text_button(graph_panel, 238, 4, 38, 34, LV_SYMBOL_SETTINGS);
    lv_obj_add_event_cb(gear, app_gset_open_cb, LV_EVENT_SHORT_CLICKED, NULL);

    gset_panel = lv_obj_create(graph_panel);
    lv_obj_set_size(gset_panel, 280, 240);
    lv_obj_set_pos(gset_panel, 0, 0);
    lv_obj_clear_flag(gset_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(gset_panel, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(gset_panel, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(gset_panel, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(gset_panel, lv_color_hex(ui_bg), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(gset_panel, LV_OPA_COVER, LV_PART_MAIN);

    gset_title = lv_label_create(gset_panel);
    lv_obj_set_style_text_font(gset_title, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(gset_title, lv_color_hex(ui_accent), LV_PART_MAIN);

    gset_tabview = lv_tabview_create(gset_panel, LV_DIR_TOP, 0);
    lv_obj_set_size(gset_tabview, 280, 150);
    lv_obj_set_pos(gset_tabview, 0, 34);
    lv_obj_set_style_bg_opa(gset_tabview, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(gset_tabview, 0, LV_PART_MAIN);
    lv_obj_add_event_cb(gset_tabview, app_gset_page_cb, LV_EVENT_VALUE_CHANGED, NULL);

    for (size_t p = 0; p < GSET_PAGE_COUNT; p++)
    {
        lv_obj_t *page = lv_tabview_add_tab(gset_tabview, gset_page_titles[p]);
        lv_obj_clear_flag(page, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_pad_all(page, 0, LV_PART_MAIN);
        for (size_t slot = 0; slot < 3; slot++)
        {
            const int row = gset_page_rows[p][slot];
            if (row < 0)
            {
                continue;
            }
            const lv_coord_t y = 4 + (lv_coord_t)slot * 48;
            settings_labels[row] = lv_label_create(page);
            lv_obj_set_style_text_color(settings_labels[row], lv_color_hex(ui_accent), LV_PART_MAIN);
            lv_obj_set_style_text_font(settings_labels[row], &lv_font_montserrat_16, LV_PART_MAIN);
            lv_obj_set_pos(settings_labels[row], 8, y + 11);
            for (int dir = 0; dir < 2; dir++)
            {
                lv_obj_t *button = app_create_text_button(page, dir ? 226 : 170, y, 50, 42, dir ? "+" : "-");
                void *code = (void *)(intptr_t)(row * 2 + dir);
                lv_obj_add_event_cb(button, app_setting_adjust_cb, LV_EVENT_SHORT_CLICKED, code);
                lv_obj_add_event_cb(button, app_setting_adjust_cb, LV_EVENT_LONG_PRESSED_REPEAT, code);
            }
        }
    }

    for (size_t i = 0; i < GSET_PAGE_COUNT; i++)
    {
        gset_dots[i] = lv_obj_create(gset_panel);
        lv_obj_remove_style_all(gset_dots[i]);
        lv_obj_set_size(gset_dots[i], 8, 8);
        lv_obj_set_pos(gset_dots[i], 122 + (lv_coord_t)i * 14, 188);
        lv_obj_set_style_radius(gset_dots[i], LV_RADIUS_CIRCLE, LV_PART_MAIN);
    }
    app_update_gset_indicator();

    lv_obj_t *done = app_create_text_button(gset_panel, 8, 202, 264, 34, "DONE");
    lv_obj_add_event_cb(done, app_gset_done_cb, LV_EVENT_SHORT_CLICKED, NULL);
    lv_obj_add_flag(gset_panel, LV_OBJ_FLAG_HIDDEN);
}

static void app_create_settings_ui(lv_obj_t *screen)
{
    lv_obj_t *gear = app_create_text_button(screen, 238, 4, 38, 34, LV_SYMBOL_SETTINGS);
    lv_obj_add_event_cb(gear, app_settings_open_cb, LV_EVENT_SHORT_CLICKED, NULL);

    settings_panel = lv_obj_create(screen);
    lv_obj_set_size(settings_panel, 280, 240);
    lv_obj_set_pos(settings_panel, 0, 0);
    lv_obj_clear_flag(settings_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(settings_panel, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(settings_panel, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(settings_panel, 0, LV_PART_MAIN);
    lv_obj_set_style_bg_color(settings_panel, lv_color_hex(ui_bg), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(settings_panel, LV_OPA_COVER, LV_PART_MAIN);

    settings_title = lv_label_create(settings_panel);
    lv_obj_set_style_text_font(settings_title, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_set_style_text_color(settings_title, lv_color_hex(ui_accent), LV_PART_MAIN);
    lv_obj_align(settings_title, LV_ALIGN_TOP_MID, 0, 8);

    /* Swipe horizontally between pages; the tab bar is hidden (height 0). */
    settings_tabview = lv_tabview_create(settings_panel, LV_DIR_TOP, 0);
    lv_obj_set_size(settings_tabview, 280, 150);
    lv_obj_set_pos(settings_tabview, 0, 34);
    lv_obj_set_style_bg_opa(settings_tabview, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(settings_tabview, 0, LV_PART_MAIN);
    lv_obj_add_event_cb(settings_tabview, app_settings_page_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_t *pages[SETTINGS_PAGE_COUNT] = {lv_tabview_add_tab(settings_tabview, "Display"),
                                            lv_tabview_add_tab(settings_tabview, "Effects"),
                                            lv_tabview_add_tab(settings_tabview, "Pump"),
                                            lv_tabview_add_tab(settings_tabview, "Output"),
                                            lv_tabview_add_tab(settings_tabview, "Auto")};
    for (size_t i = 0; i < SETTINGS_PAGE_COUNT; i++)
    {
        lv_obj_clear_flag(pages[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_pad_all(pages[i], 0, LV_PART_MAIN);
    }

    for (size_t row = 0; row < 20; row++)
    {
        if (row >= 11 && row < 18)
        {
            continue;
        }
        const size_t page_index = row >= 18 ? 4 : (row < 3 ? 2 : (row < 9 ? row / 3 - 1 : 3));
        lv_obj_t *page = pages[page_index];
        const lv_coord_t y = 4 + (lv_coord_t)(row >= 18 ? row - 18 : row % 3) * 48;
        settings_labels[row] = lv_label_create(page);
        lv_obj_set_style_text_color(settings_labels[row], lv_color_hex(ui_accent), LV_PART_MAIN);
        lv_obj_set_style_text_font(settings_labels[row], &lv_font_montserrat_16, LV_PART_MAIN);
        lv_obj_set_pos(settings_labels[row], 8, y + 11);

        for (int dir = 0; dir < 2; dir++)
        {
            lv_obj_t *button = app_create_text_button(page, dir ? 226 : 170, y, 50, 42,
                                                      dir ? "+" : "-");
            void *code = (void *)(intptr_t)(row * 2 + dir);
            lv_obj_add_event_cb(button, app_setting_adjust_cb, LV_EVENT_SHORT_CLICKED, code);
            lv_obj_add_event_cb(button, app_setting_adjust_cb, LV_EVENT_LONG_PRESSED_REPEAT, code);
        }
    }

    for (size_t i = 0; i < SETTINGS_PAGE_COUNT; i++)
    {
        settings_dots[i] = lv_obj_create(settings_panel);
        lv_obj_remove_style_all(settings_dots[i]);
        lv_obj_set_size(settings_dots[i], 8, 8);
        lv_obj_set_pos(settings_dots[i], 108 + (lv_coord_t)i * 14, 188);
        lv_obj_set_style_radius(settings_dots[i], LV_RADIUS_CIRCLE, LV_PART_MAIN);
    }
    app_update_page_indicator();

    lv_obj_t *done = app_create_text_button(settings_panel, 8, 202, 264, 34, "DONE");
    lv_obj_add_event_cb(done, app_settings_done_cb, LV_EVENT_SHORT_CLICKED, NULL);
    lv_obj_add_flag(settings_panel, LV_OBJ_FLAG_HIDDEN);

    /* Transparent full-screen layer shown while the backlight is off. */
    app_create_graph_settings_ui();
    wake_overlay = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(wake_overlay);
    lv_obj_set_size(wake_overlay, LV_PCT(100), LV_PCT(100));
    lv_obj_add_flag(wake_overlay, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(wake_overlay, app_wake_screen_cb, LV_EVENT_PRESSED, NULL);
    lv_timer_create(app_sleep_timer_cb, 1000, NULL);
}

static void app_main_display(void)
{
    lvgl_port_lock(0);

    lv_obj_t *screen = lv_scr_act();
    if (splash_logo != NULL)
    {
        lv_anim_del(splash_logo, app_splash_opacity_exec_cb);
        splash_logo = NULL;
    }
    lv_obj_clean(screen);
    app_apply_background();

    pump_glow = lv_img_create(screen);
    lv_img_set_src(pump_glow, &glow_image_asset);
    lv_obj_align(pump_glow, LV_ALIGN_TOP_MID, -5, 23);
    lv_obj_set_style_img_recolor(pump_glow, lv_color_hex(ui_accent), LV_PART_MAIN);
    lv_obj_set_style_img_recolor_opa(pump_glow, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_img_opa(pump_glow, LV_OPA_TRANSP, LV_PART_MAIN);
    pump_image = lv_img_create(screen);
    lv_img_set_pivot(pump_image, 72, 58);
    lv_img_set_src(pump_image, &pump_image_asset);
    /* Center the 116 px tall image in the 188 px of space above the buttons. */
    lv_obj_align(pump_image, LV_ALIGN_TOP_MID, 0, 35);
    pump_image_base_y = 35;

    for (int i = 0; i < 3; i++)
    {
        flow_track[i] = lv_obj_create(screen);
        lv_obj_remove_style_all(flow_track[i]);
        lv_obj_clear_flag(flow_track[i], LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_size(flow_track[i], 66, 1);
        lv_obj_set_pos(flow_track[i], FLOW_ORIGIN_X - 72, FLOW_ORIGIN_Y - 6 + i * 8);
        lv_obj_set_style_bg_opa(flow_track[i], LV_OPA_50, LV_PART_MAIN);
        lv_obj_add_flag(flow_track[i], LV_OBJ_FLAG_HIDDEN);
    }    for (size_t i = 0; i < FLOW_MAX_PARTICLES; i++)
    {
        flow_particles[i].obj = lv_obj_create(screen);
        lv_obj_remove_style_all(flow_particles[i].obj);
        lv_obj_clear_flag(flow_particles[i].obj, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_style_bg_opa(flow_particles[i].obj, LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_add_flag(flow_particles[i].obj, LV_OBJ_FLAG_HIDDEN);
    }

    const lv_coord_t button_width = 65;
    const lv_coord_t button_height = 44;
    const lv_coord_t button_x[PRESET_COUNT] = {4, 73, 0, 142, 211};
    const lv_coord_t button_y = 188;

    for (size_t i = 0; i < PRESET_COUNT; i++)
    {
        if (i == APP_PRESET_MEDIUM)
        {
            continue;
        }
        preset_buttons[i] = lv_btn_create(screen);
        lv_obj_set_size(preset_buttons[i], button_width, button_height);
        lv_obj_set_pos(preset_buttons[i], button_x[i], button_y);
        lv_obj_set_style_radius(preset_buttons[i], 10, LV_PART_MAIN);
        lv_obj_set_style_bg_opa(preset_buttons[i], LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_border_opa(preset_buttons[i], LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_bg_color(preset_buttons[i], lv_color_hex(ui_tile), LV_PART_MAIN);
        lv_obj_set_style_border_color(preset_buttons[i], lv_color_hex(ui_accent_muted), LV_PART_MAIN);
        lv_obj_set_style_border_width(preset_buttons[i], 1, LV_PART_MAIN);
        lv_obj_set_style_bg_color(preset_buttons[i], lv_color_hex(ui_tile_selected), LV_STATE_PRESSED);
        lv_obj_set_style_border_color(preset_buttons[i], lv_color_hex(ui_accent), LV_STATE_PRESSED);
        lv_obj_set_style_shadow_width(preset_buttons[i], 0, LV_PART_MAIN);
        lv_obj_add_event_cb(preset_buttons[i], app_click_sound_cb, LV_EVENT_PRESSED, NULL);

        preset_labels[i] = lv_label_create(preset_buttons[i]);
        lv_label_set_text(preset_labels[i], preset_names[i]);
        lv_obj_set_style_text_color(preset_labels[i], lv_color_hex(ui_accent), LV_PART_MAIN);
        lv_obj_set_style_text_font(preset_labels[i], &lv_font_montserrat_16, LV_PART_MAIN);
        lv_obj_center(preset_labels[i]);
        lv_obj_add_event_cb(preset_buttons[i], app_preset_button_event_cb, LV_EVENT_SHORT_CLICKED,
                            &preset_ids[i]);
        lv_obj_add_event_cb(preset_buttons[i], app_save_preset_position, LV_EVENT_SHORT_CLICKED,
                            &preset_ids[i]);
        if (i != APP_PRESET_OFF && i != APP_PRESET_AUTO)
        {
            lv_obj_add_event_cb(preset_buttons[i], app_preset_button_event_cb, LV_EVENT_LONG_PRESSED,
                                &preset_ids[i]);
            lv_obj_add_event_cb(preset_buttons[i], app_save_preset_position, LV_EVENT_LONG_PRESSED,
                                &preset_ids[i]);
            lv_obj_add_event_cb(preset_buttons[i], app_edit_open_cb, LV_EVENT_LONG_PRESSED,
                                &preset_ids[i]);
        }
    }

    /* Duty editor: covers the preset row while open. */
    edit_panel = lv_obj_create(screen);
    lv_obj_set_size(edit_panel, 272, 90);
    lv_obj_set_pos(edit_panel, 4, 142);
    lv_obj_clear_flag(edit_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(edit_panel, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(edit_panel, 10, LV_PART_MAIN);
    lv_obj_set_style_bg_color(edit_panel, lv_color_hex(ui_tile), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(edit_panel, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_color(edit_panel, lv_color_hex(ui_accent), LV_PART_MAIN);
    lv_obj_set_style_border_width(edit_panel, 2, LV_PART_MAIN);
    lv_obj_add_flag(edit_panel, LV_OBJ_FLAG_HIDDEN);

    edit_label = lv_label_create(edit_panel);
    lv_obj_set_style_text_color(edit_label, lv_color_hex(ui_accent), LV_PART_MAIN);
    lv_obj_set_style_text_font(edit_label, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_align(edit_label, LV_ALIGN_TOP_MID, 0, 8);

    static const char *edit_texts[] = {"-", "+", "DONE"};
    static const intptr_t edit_steps[] = {-1, 1, 0};
    const lv_coord_t edit_x[] = {8, 78, 148};
    const lv_coord_t edit_w[] = {62, 62, 108};
    for (size_t i = 0; i < 3; i++)
    {
        lv_obj_t *button = lv_btn_create(edit_panel);
        lv_obj_set_size(button, edit_w[i], 44);
        lv_obj_set_pos(button, edit_x[i], 36);
        lv_obj_set_style_radius(button, 10, LV_PART_MAIN);
        lv_obj_set_style_bg_color(button, lv_color_hex(ui_bg), LV_PART_MAIN);
        lv_obj_set_style_bg_color(button, lv_color_hex(ui_tile_selected), LV_STATE_PRESSED);
        lv_obj_set_style_border_color(button, lv_color_hex(ui_accent_muted), LV_PART_MAIN);
        lv_obj_set_style_border_width(button, 1, LV_PART_MAIN);
        lv_obj_set_style_shadow_width(button, 0, LV_PART_MAIN);
        lv_obj_add_event_cb(button, app_click_sound_cb, LV_EVENT_PRESSED, NULL);
        lv_obj_t *text = lv_label_create(button);
        lv_label_set_text(text, edit_texts[i]);
        lv_obj_set_style_text_color(text, lv_color_hex(ui_accent), LV_PART_MAIN);
        lv_obj_set_style_text_font(text, &lv_font_montserrat_16, LV_PART_MAIN);
        lv_obj_center(text);
        if (i < 2)
        {
            lv_obj_add_event_cb(button, app_edit_adjust_cb, LV_EVENT_SHORT_CLICKED, (void *)edit_steps[i]);
            lv_obj_add_event_cb(button, app_edit_adjust_cb, LV_EVENT_LONG_PRESSED_REPEAT,
                                (void *)edit_steps[i]);
        }
        else
        {
            lv_obj_add_event_cb(button, app_edit_done_cb, LV_EVENT_SHORT_CLICKED, NULL);
        }
    }

    lv_timer_create(app_auto_timer_cb, 1000, NULL);

    app_page_dots_create(screen, main_dots, 0);
    app_create_graph_ui(screen);
    app_create_settings_ui(screen);

    if (startup_preset == APP_PRESET_MEDIUM)
    {
        startup_preset = APP_PRESET_LOW;
    }
    active_preset = startup_preset;
    app_update_preset_button_styles(startup_preset);
    app_apply_preset(startup_preset);
    app_update_pump_animation(startup_preset);

    lvgl_port_unlock();
}

static void app_splash_display(void)
{
    lvgl_port_lock(0);

    lv_obj_t *screen = lv_scr_act();
    lv_obj_set_style_bg_color(screen, lv_color_hex(UI_COLOR_BACKGROUND), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);

    splash_logo = lv_img_create(screen);
    lv_img_set_src(splash_logo, &splash_image_asset);
    lv_obj_set_style_img_opa(splash_logo, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_align(splash_logo, LV_ALIGN_CENTER, 0, -14);

    lvgl_port_unlock();

    /* Let the background frame reach the panel, light the backlight, then start the fade on a visible screen. */
    vTaskDelay(pdMS_TO_TICKS(150));
    app_backlight_set(settings.backlight_pct);
    vTaskDelay(pdMS_TO_TICKS(250));

    lvgl_port_lock(0);
    splash_started_at_us = esp_timer_get_time();

    lv_anim_t animation;
    lv_anim_init(&animation);
    lv_anim_set_var(&animation, splash_logo);
    lv_anim_set_exec_cb(&animation, app_splash_opacity_exec_cb);
    lv_anim_set_values(&animation, LV_OPA_TRANSP, LV_OPA_COVER);
    lv_anim_set_time(&animation, APP_SPLASH_FADE_DURATION_MS);
    lv_anim_set_path_cb(&animation, lv_anim_path_ease_in_out);
    lv_anim_start(&animation);

    lvgl_port_unlock();
}

void app_main(void)
{
    /* Read settings before the wake pulse; the selected value is applied when UI is built. */
    startup_preset = app_load_preset();
    app_load_settings();
    app_select_theme(settings.accent_idx);
    app_load_preset_percents();
    app_clamp_presets();
    ESP_LOGI(TAG, "Loaded preset %s", preset_names[startup_preset]);

    /* Pump timer first so the backlight timer shares its LEDC slow clock. */
    ESP_ERROR_CHECK(app_control_init());
    pump_set_duty(0);
    ESP_ERROR_CHECK(app_backlight_init());
    ESP_ERROR_CHECK(app_buzzer_init());

    /* LCD HW initialization */
    ESP_ERROR_CHECK(app_lcd_init());

#if EXAMPLE_USE_TOUCH
    ESP_LOGI(TAG, "Initialize I2C bus");
    esp_log_level_set("lcd_panel.io.i2c", ESP_LOG_NONE);
    esp_log_level_set("CST816S", ESP_LOG_NONE);
    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = EXAMPLE_PIN_NUM_TOUCH_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = EXAMPLE_PIN_NUM_TOUCH_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100 * 1000,
    };
    i2c_param_config(TOUCH_HOST, &i2c_conf);

    i2c_driver_install(TOUCH_HOST, i2c_conf.mode, 0, 0, 0);

    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    const esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_CST816S_CONFIG();
    // Attach the TOUCH to the I2C bus
    esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)TOUCH_HOST, &tp_io_config, &tp_io_handle);

    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = EXAMPLE_LCD_H_RES,
        .y_max = EXAMPLE_LCD_V_RES,
        .rst_gpio_num = EXAMPLE_PIN_NUM_TOUCH_RST,
        .int_gpio_num = EXAMPLE_PIN_NUM_TOUCH_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 1,
            .mirror_y = 1,
        },
    };

    ESP_LOGI(TAG, "Initialize touch controller");
    esp_lcd_touch_new_i2c_cst816s(tp_io_handle, &tp_cfg, &tp);
#endif

    /* LVGL initialization */
    ESP_ERROR_CHECK(app_lvgl_init());

    app_splash_display();    /* Keep the splash visible while the pump's startup wake sequence runs. */
    app_pump_wakeup();

#if EXAMPLE_USE_TOUCH
    /* Register touch after the wake pulses so it doesn't read during pump noise. */
    static lv_indev_drv_t indev_drv; // Input device driver (Touch)
    lvgl_port_lock(0);
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.disp = lvgl_disp;
    indev_drv.read_cb = example_lvgl_touch_cb;
    indev_drv.user_data = tp;
    lv_indev_drv_register(&indev_drv);
    lvgl_port_unlock();
#endif

    app_wait_until_us(splash_started_at_us +
                      (int64_t)(APP_SPLASH_DURATION_MS - APP_SPLASH_FADE_DURATION_MS) * 1000);
    app_splash_fade_out();
    app_wait_until_us(splash_started_at_us + (int64_t)APP_SPLASH_DURATION_MS * 1000);

    ESP_LOGI(TAG, "Splash complete; showing main screen");
    app_main_display();
    ESP_LOGI(TAG, "Main screen ready");
}
