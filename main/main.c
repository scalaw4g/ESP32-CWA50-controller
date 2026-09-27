/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * CWA50 pump controller for the Waveshare ESP32-S3-Touch-LCD-1.69.
 * The app uses LVGL for the 0-100 control slider and LEDC for pump PWM.
 */
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
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
#define PUMP_PWM_FREQUENCY_HZ 1000

/* The slider is a control scale: 0 is off; nonzero settings map to 13-85% PWM. */
#define PUMP_PWM_MIN_PERCENT 13
#define PUMP_PWM_MAX_PERCENT 85
#define PUMP_DEFAULT_SLIDER_POSITION 73

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
static lv_obj_t *percent_label;
static lv_obj_t *actual_pwm_label;
static lv_obj_t *pwm_slider;
static int startup_slider_position = PUMP_DEFAULT_SLIDER_POSITION;

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

/* Load the last released slider value; use the 65%-duty control default if absent. */
static int app_load_slider_position(void)
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
    err = nvs_open("pump", NVS_READONLY, &nvs_handle);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        return PUMP_DEFAULT_SLIDER_POSITION;
    }
    ESP_ERROR_CHECK(err);

    int32_t saved_position = PUMP_DEFAULT_SLIDER_POSITION;
    err = nvs_get_i32(nvs_handle, "slider", &saved_position);
    nvs_close(nvs_handle);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        return PUMP_DEFAULT_SLIDER_POSITION;
    }
    ESP_ERROR_CHECK(err);

    if (saved_position < 0 || saved_position > 100)
    {
        ESP_LOGW(TAG, "Saved slider position is invalid; using default");
        return PUMP_DEFAULT_SLIDER_POSITION;
    }
    return saved_position;
}

static void app_save_slider_position(lv_event_t *event)
{
    const int32_t position = lv_slider_get_value(lv_event_get_target(event));
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("pump", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not open NVS to save slider position: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_set_i32(nvs_handle, "slider", position);
    if (err == ESP_OK)
    {
        err = nvs_commit(nvs_handle);
    }
    nvs_close(nvs_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not save slider position: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "Saved slider position %" PRId32, position);
}

/* Convert the UI's 0-100 control scale into a safe pump PWM duty. */
static void app_apply_slider_position(int slider_position)
{
    if (slider_position == 0)
    {
        pump_set_duty(0);
        lv_label_set_text(percent_label, "OFF");
        lv_label_set_text(actual_pwm_label, "PWM DUTY 0.00%");
        printf("0 >>>>> 0.00\n");
        return;
    }

    /* Map control positions 1-100 linearly onto the datasheet's 13-85% range. */
    const int requested_percent = PUMP_PWM_MIN_PERCENT +
        ((slider_position - 1) * (PUMP_PWM_MAX_PERCENT - PUMP_PWM_MIN_PERCENT) + 49) / 99;
    int duty = (requested_percent * 255 + 50) / 100;
    if (duty < PUMP_PWM_MIN_DUTY)
    {
        duty = PUMP_PWM_MIN_DUTY;
    }
    else if (duty > PUMP_PWM_MAX_DUTY)
    {
        duty = PUMP_PWM_MAX_DUTY;
    }
    /* An 8-bit LEDC period has 256 steps, so report the applied duty accurately. */
    const int percent_hundredths = (duty * 10000 + 128) / 256;

    pump_set_duty((uint32_t)duty);
    lv_label_set_text_fmt(percent_label, "%d%%", slider_position);
    lv_label_set_text_fmt(actual_pwm_label, "PWM DUTY %d.%02d%%", percent_hundredths / 100, percent_hundredths % 100);
    printf("%d >>>>> %d.%02d\n", duty, percent_hundredths / 100, percent_hundredths % 100);
}

static void app_pump_wakeup(void)
{
    /* Brief pump wake pulse, followed by an off interval before normal PWM control. */
    pump_set_duty(PUMP_PWM_MAX_DUTY);
    vTaskDelay(pdMS_TO_TICKS(300));
    pump_set_duty(0);
    vTaskDelay(pdMS_TO_TICKS(100));
}

static void app_pwm_slider_event_cb(lv_event_t *event)
{
    lv_obj_t *slider = lv_event_get_target(event);
    app_apply_slider_position(lv_slider_get_value(slider));
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
    /* LVGL also rotates pointer coordinates for this display rotation. */
    lv_disp_set_rotation(lvgl_disp, LV_DISP_ROT_90);
    lvgl_port_unlock();

    return ESP_OK;
}

static void app_main_display(void)
{
    lvgl_port_lock(0);

    lv_obj_t *screen = lv_scr_act();
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x101820), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, LV_PART_MAIN);

    lv_obj_t *title = lv_label_create(screen);
    lv_label_set_text(title, "CWA50 PUMP CONTROL");
    lv_obj_set_style_text_color(title, lv_color_hex(0xE8F0F2), LV_PART_MAIN);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 24);

    /* The main readout shows the 0-100 control value; the second shows applied PWM. */
    percent_label = lv_label_create(screen);
    lv_label_set_text(percent_label, "0%");
    lv_obj_set_style_text_color(percent_label, lv_color_hex(0x36D6A0), LV_PART_MAIN);
    lv_obj_set_style_text_font(percent_label, &lv_font_montserrat_16, LV_PART_MAIN);
    lv_obj_align(percent_label, LV_ALIGN_TOP_MID, 0, 82);

    actual_pwm_label = lv_label_create(screen);
    lv_label_set_text(actual_pwm_label, "PWM DUTY 0.00%");
    lv_obj_set_style_text_color(actual_pwm_label, lv_color_hex(0x91A4AA), LV_PART_MAIN);
    lv_obj_set_style_text_font(actual_pwm_label, &lv_font_montserrat_14, LV_PART_MAIN);
    lv_obj_align(actual_pwm_label, LV_ALIGN_TOP_MID, 0, 112);

    pwm_slider = lv_slider_create(screen);
    lv_slider_set_range(pwm_slider, 0, 100);
    lv_slider_set_value(pwm_slider, startup_slider_position, LV_ANIM_OFF);
    lv_obj_set_size(pwm_slider, 192, 24);
    lv_obj_set_style_bg_color(pwm_slider, lv_color_hex(0x34434A), LV_PART_MAIN);
    lv_obj_set_style_bg_color(pwm_slider, lv_color_hex(0x36D6A0), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(pwm_slider, lv_color_hex(0xFFFFFF), LV_PART_KNOB);
    lv_obj_align(pwm_slider, LV_ALIGN_TOP_MID, 0, 158);
    lv_obj_add_event_cb(pwm_slider, app_pwm_slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    app_apply_slider_position(startup_slider_position);
    /* Persist only when the user releases the slider, not during every drag update. */
    lv_obj_add_event_cb(pwm_slider, app_save_slider_position, LV_EVENT_RELEASED, NULL);

    lvgl_port_unlock();
}

void app_main(void)
{
    /* Read settings before the wake pulse; the selected value is applied when UI is built. */
    startup_slider_position = app_load_slider_position();
    ESP_LOGI(TAG, "Loaded slider position %d", startup_slider_position);

    /* LCD HW initialization */
    ESP_ERROR_CHECK(app_lcd_init());
    ESP_ERROR_CHECK(app_control_init());
    pump_set_duty(0);
    /* Run the pump wake pulse before LVGL starts; no LVGL lock is needed here. */
    app_pump_wakeup();

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

#if EXAMPLE_USE_TOUCH
    static lv_indev_drv_t indev_drv; // Input device driver (Touch)
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.disp = lvgl_disp;
    indev_drv.read_cb = example_lvgl_touch_cb;
    indev_drv.user_data = tp;
    lv_indev_drv_register(&indev_drv);
#endif

    /* Show LVGL objects */
    app_main_display();
}
