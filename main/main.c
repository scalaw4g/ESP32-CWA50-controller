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
#include <stdio.h>
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
#define EXAMPLE_LCD_BL_ON_LEVEL (1)

/* LCD pins */
#define EXAMPLE_LCD_GPIO_SCLK (GPIO_NUM_6)
#define EXAMPLE_LCD_GPIO_MOSI (GPIO_NUM_7)
#define EXAMPLE_LCD_GPIO_RST (GPIO_NUM_8)
#define EXAMPLE_LCD_GPIO_DC (GPIO_NUM_4)
#define EXAMPLE_LCD_GPIO_CS (GPIO_NUM_5)
#define EXAMPLE_LCD_GPIO_BL (GPIO_NUM_15)

#define EXAMPLE_USE_TOUCH 1

#define TOUCH_HOST I2C_NUM_0
#define PUMP_PWM_GPIO GPIO_NUM_17
#define PUMP_PWM_FREQUENCY_HZ 150
#define APP_SPLASH_DURATION_MS 2000
#define APP_SPLASH_FADE_DURATION_MS 300

/* Presets use control positions mapped onto the configured pump PWM range. */
#define PUMP_PWM_MIN_PERCENT 13
#define PUMP_PWM_MAX_PERCENT 85
#define PUMP_PRESET_OFF_POSITION 0
#define PUMP_PRESET_LOW_POSITION 10
#define PUMP_PRESET_MEDIUM_POSITION 55
#define PUMP_PRESET_HIGH_POSITION 100
#define PUMP_DEFAULT_PRESET_POSITION PUMP_PRESET_MEDIUM_POSITION

typedef enum
{
    APP_PRESET_OFF = 0,
    APP_PRESET_LOW,
    APP_PRESET_MEDIUM,
    APP_PRESET_HIGH,
} app_preset_id_t;

/* 8-bit LEDC counts rounded inward to stay within the pump's active-duty limits. */
#define PUMP_PWM_MIN_DUTY 34
#define PUMP_PWM_MAX_DUTY 216

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
static lv_obj_t *preset_buttons[4];
static lv_obj_t *preset_labels[4];
static lv_coord_t pump_image_base_y;
static app_preset_id_t startup_preset = APP_PRESET_MEDIUM;
static int64_t splash_started_at_us;
static int preset_positions[] = {
    PUMP_PRESET_OFF_POSITION,
    PUMP_PRESET_LOW_POSITION,
    PUMP_PRESET_MEDIUM_POSITION,
    PUMP_PRESET_HIGH_POSITION,
};
static app_preset_id_t preset_ids[] = {
    APP_PRESET_OFF,
    APP_PRESET_LOW,
    APP_PRESET_MEDIUM,
    APP_PRESET_HIGH,
};
static const char *preset_names[] = {"OFF", "LOW", "MED", "HIGH"};
extern const lv_img_dsc_t pump_image_asset;
extern const lv_img_dsc_t splash_image_asset;
extern const lv_img_dsc_t glow_image_asset;

#define PRESET_COUNT (sizeof(preset_positions) / sizeof(preset_positions[0]))
#define NVS_KEY_PRESET_ID "preset_id"
#define UI_COLOR_BACKGROUND 0x080A09
#define UI_COLOR_TILE 0x151918
#define UI_COLOR_TILE_SELECTED 0x18352D
#define UI_COLOR_SEAFOAM 0x8FE3C1
#define UI_COLOR_SEAFOAM_MUTED 0x4E7968

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
        .freq_hz = PUMP_PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&pwm_timer), TAG, "PWM timer initialization failed");

    const ledc_channel_config_t pwm_channel = {
        .gpio_num = PUMP_PWM_GPIO,
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
    if (saved_preset_id < APP_PRESET_OFF || saved_preset_id > APP_PRESET_HIGH)
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

static void app_save_preset_position(lv_event_t *event)
{
    app_preset_id_t *preset = lv_event_get_user_data(event);
    if (preset == NULL || *preset < APP_PRESET_OFF || *preset > APP_PRESET_HIGH)
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
        const bool selected = i == (size_t)selected_preset;
        lv_obj_set_style_bg_color(preset_buttons[i],
                                  lv_color_hex(selected ? UI_COLOR_TILE_SELECTED : UI_COLOR_TILE),
                                  LV_PART_MAIN);
        lv_obj_set_style_border_color(preset_buttons[i],
                                      lv_color_hex(selected ? UI_COLOR_SEAFOAM : UI_COLOR_SEAFOAM_MUTED),
                                      LV_PART_MAIN);
        lv_obj_set_style_border_width(preset_buttons[i], selected ? 2 : 1, LV_PART_MAIN);
        lv_obj_set_style_text_color(preset_labels[i], lv_color_hex(UI_COLOR_SEAFOAM), LV_PART_MAIN);
    }
}

/* Convert a preset's control position into a safe pump PWM duty. */
static void app_apply_preset_position(int preset_position)
{
    if (preset_position == PUMP_PRESET_OFF_POSITION)
    {
        pump_set_duty(0);
        return;
    }

    /* Map control positions 1-100 linearly onto the datasheet's 13-85% range. */
    const int requested_percent = PUMP_PWM_MIN_PERCENT +
        ((preset_position - 1) * (PUMP_PWM_MAX_PERCENT - PUMP_PWM_MIN_PERCENT) + 49) / 99;
    int duty = (requested_percent * 255 + 50) / 100;
    if (duty < PUMP_PWM_MIN_DUTY)
    {
        duty = PUMP_PWM_MIN_DUTY;
    }
    else if (duty > PUMP_PWM_MAX_DUTY)
    {
        duty = PUMP_PWM_MAX_DUTY;
    }
    /* An 8-bit LEDC period has 256 steps. */

    pump_set_duty((uint32_t)duty);
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

static void app_update_pump_animation(app_preset_id_t preset)
{
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

    app_start_pump_anim(pump_image, app_pump_animation_exec_cb, pump_image_base_y,
                        pump_image_base_y - 3, cycle_ms);
    if (shake > 0)
    {
        app_start_pump_anim(pump_image, app_pump_shake_exec_cb, -shake, shake, cycle_ms / 7 + 30);
        app_start_pump_anim(pump_image, app_pump_rotate_exec_cb, -tilt, tilt, cycle_ms / 4 + 40);
        app_start_pump_anim(pump_glow, app_pump_glow_exec_cb, glow / 3, glow, cycle_ms);
    }
}
static void app_preset_button_event_cb(lv_event_t *event)
{
    app_preset_id_t *preset = lv_event_get_user_data(event);
    if (preset == NULL || *preset < APP_PRESET_OFF || *preset > APP_PRESET_HIGH)
    {
        ESP_LOGE(TAG, "Preset button event has invalid preset ID");
        return;
    }

    app_update_preset_button_styles(*preset);
    app_apply_preset_position(preset_positions[*preset]);
    app_update_pump_animation(*preset);
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

    /* LCD backlight */
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_LCD_GPIO_BL};
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

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
    esp_lcd_panel_disp_on_off(lcd_panel, true);

    /* LCD backlight on */
    ESP_ERROR_CHECK(gpio_set_level(EXAMPLE_LCD_GPIO_BL, EXAMPLE_LCD_BL_ON_LEVEL));

    esp_lcd_panel_set_gap(lcd_panel, 0, 20);
    esp_lcd_panel_invert_color(lcd_panel, true);

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
        .task_max_sleep_ms = 500, /* Maximum sleep in LVGL task */
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
    lv_obj_set_style_bg_color(screen, lv_color_hex(UI_COLOR_BACKGROUND), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);

    pump_glow = lv_img_create(screen);
    lv_img_set_src(pump_glow, &glow_image_asset);
    lv_obj_align(pump_glow, LV_ALIGN_TOP_MID, -5, 23);
    lv_obj_set_style_img_recolor(pump_glow, lv_color_hex(UI_COLOR_SEAFOAM), LV_PART_MAIN);
    lv_obj_set_style_img_recolor_opa(pump_glow, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_img_opa(pump_glow, LV_OPA_TRANSP, LV_PART_MAIN);
    pump_image = lv_img_create(screen);
    lv_img_set_pivot(pump_image, 72, 58);
    lv_img_set_src(pump_image, &pump_image_asset);
    /* Center the 116 px tall image in the 188 px of space above the buttons. */
    lv_obj_align(pump_image, LV_ALIGN_TOP_MID, 0, 35);
    pump_image_base_y = 35;

    const lv_coord_t button_width = 65;
    const lv_coord_t button_height = 44;
    const lv_coord_t button_x[PRESET_COUNT] = {4, 73, 142, 211};
    const lv_coord_t button_y = 188;

    for (size_t i = 0; i < PRESET_COUNT; i++)
    {
        preset_buttons[i] = lv_btn_create(screen);
        lv_obj_set_size(preset_buttons[i], button_width, button_height);
        lv_obj_set_pos(preset_buttons[i], button_x[i], button_y);
        lv_obj_set_style_radius(preset_buttons[i], 10, LV_PART_MAIN);
        lv_obj_set_style_bg_opa(preset_buttons[i], LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_border_opa(preset_buttons[i], LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_bg_color(preset_buttons[i], lv_color_hex(UI_COLOR_TILE), LV_PART_MAIN);
        lv_obj_set_style_border_color(preset_buttons[i], lv_color_hex(UI_COLOR_SEAFOAM_MUTED), LV_PART_MAIN);
        lv_obj_set_style_border_width(preset_buttons[i], 1, LV_PART_MAIN);
        lv_obj_set_style_bg_color(preset_buttons[i], lv_color_hex(UI_COLOR_TILE_SELECTED), LV_STATE_PRESSED);
        lv_obj_set_style_border_color(preset_buttons[i], lv_color_hex(UI_COLOR_SEAFOAM), LV_STATE_PRESSED);
        lv_obj_set_style_shadow_width(preset_buttons[i], 0, LV_PART_MAIN);

        preset_labels[i] = lv_label_create(preset_buttons[i]);
        lv_label_set_text(preset_labels[i], preset_names[i]);
        lv_obj_set_style_text_color(preset_labels[i], lv_color_hex(UI_COLOR_SEAFOAM), LV_PART_MAIN);
        lv_obj_set_style_text_font(preset_labels[i], &lv_font_montserrat_16, LV_PART_MAIN);
        lv_obj_center(preset_labels[i]);
        lv_obj_add_event_cb(preset_buttons[i], app_preset_button_event_cb, LV_EVENT_CLICKED,
                            &preset_ids[i]);
        lv_obj_add_event_cb(preset_buttons[i], app_save_preset_position, LV_EVENT_CLICKED,
                            &preset_ids[i]);
    }

    app_update_preset_button_styles(startup_preset);
    app_apply_preset_position(preset_positions[startup_preset]);
    app_update_pump_animation(startup_preset);

    lvgl_port_unlock();
}

static void app_splash_display(void)
{
    lvgl_port_lock(0);

    splash_started_at_us = esp_timer_get_time();
    lv_obj_t *screen = lv_scr_act();
    lv_obj_set_style_bg_color(screen, lv_color_hex(UI_COLOR_BACKGROUND), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);

    splash_logo = lv_img_create(screen);
    lv_img_set_src(splash_logo, &splash_image_asset);
    lv_obj_set_style_img_opa(splash_logo, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_align(splash_logo, LV_ALIGN_CENTER, 0, -14);

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
    ESP_LOGI(TAG, "Loaded preset %s", preset_names[startup_preset]);

    /* LCD HW initialization */
    ESP_ERROR_CHECK(app_lcd_init());
    ESP_ERROR_CHECK(app_control_init());
    pump_set_duty(0);

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

    app_splash_display();
    /* Keep the splash visible while the pump's startup wake sequence runs. */
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
