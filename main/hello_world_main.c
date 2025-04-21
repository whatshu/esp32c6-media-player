/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"

// led strip include
#include "led_strip.h"

// screen driver & lvgl include
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lvgl_port.h"

// led strip define
#define LED_STRIP_USE_DMA       false
#define LED_STRIP_LED_COUNT     1 // Numbers of the LED in the strip
#define LED_STRIP_MEMORY_BLOCK_WORDS 0 // let the driver choose a proper memory block size automatically
#define LED_STRIP_GPIO_PIN      8 // GPIO assignment
#define LED_STRIP_RMT_RES_HZ    (10 * 1000 * 1000) // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)

// screen driver define
#define DISP_HOST               SPI2_HOST
#define DISP_PIXEL_CLOCK_HZ     (12 * 1000 * 1000)
#define DISP_BK_LIGHT_ON_LEVEL  1
#define DISP_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define DISP_PIN_NUM_SCLK       7
#define DISP_PIN_NUM_MOSI       6
#define DISP_PIN_NUM_CS         14
#define DISP_PIN_NUM_DC         15
#define DISP_PIN_NUM_RST        21
#define DISP_PIN_NUM_BK_LIGHT   22
#define DISP_WIDTH              172
#define DISP_HEIGHT             320
#define DISP_DRAW_BUFF_HEIGHT   16
#define DISP_DRAW_BUFF_DOUBLE   1
#define DISP_CMD_BITS           8
#define DISP_PARAM_BITS         8

static const char *TAG = "main";

led_strip_handle_t configure_led(void)
{
    // LED Strip object handle
    led_strip_handle_t led_strip;

    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO_PIN, // The GPIO that connected to the LED strip's data line
        .max_leds = LED_STRIP_LED_COUNT,      // The number of LEDs in the strip,
        .led_model = LED_MODEL_WS2812,        // LED strip model
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_RGB, // The color order of the strip: GRB
        .flags = {
            .invert_out = false, // don't invert the output signal
        }
    };

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
        .mem_block_symbols = LED_STRIP_MEMORY_BLOCK_WORDS, // the memory block size used by the RMT channel
        .flags = {
            .with_dma = LED_STRIP_USE_DMA,     // Using DMA can improve performance when driving more LEDs
        }
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    led_strip_clear(led_strip);

    ESP_LOGI(TAG, "Created LED strip object with RMT backend");
    return led_strip;
}

void app_main(void)
{
    led_strip_handle_t led_strip = configure_led();

    // lv_disp_draw_buf_t disp_buf;
    // lv_disp_drv_t disp_drv;
    esp_lcd_panel_io_handle_t lcd_io = NULL;
    esp_lcd_panel_handle_t lcd_panel = NULL;

    ESP_LOGD(TAG, "Initialize SPI bus");
    const spi_bus_config_t buscfg = {
        .sclk_io_num = DISP_PIN_NUM_SCLK,
        .mosi_io_num = DISP_PIN_NUM_MOSI,
        .miso_io_num = GPIO_NUM_NC,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = DISP_WIDTH * DISP_DRAW_BUFF_HEIGHT * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(DISP_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGD(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = DISP_PIN_NUM_DC,
        .cs_gpio_num = DISP_PIN_NUM_CS,
        .pclk_hz = DISP_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = DISP_CMD_BITS,
        .lcd_param_bits = DISP_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        // .on_color_trans_done = example_notify_lvgl_flush_ready,
        // .user_ctx = &disp_drv,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)DISP_HOST, &io_config, &lcd_io));

    ESP_LOGD(TAG, "Install LCD driver");
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = DISP_PIN_NUM_RST,
        .rgb_endian = LCD_RGB_ENDIAN_BGR,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(lcd_io, &panel_config, &lcd_panel));

    esp_lcd_panel_reset(lcd_panel);
    esp_lcd_panel_init(lcd_panel);
    esp_lcd_panel_mirror(lcd_panel, true, true);
    esp_lcd_panel_disp_on_off(lcd_panel, true);

    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    esp_err_t err = lvgl_port_init(&lvgl_cfg);

    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = lcd_io,
        .panel_handle = lcd_panel,
        .buffer_size = DISP_WIDTH * DISP_DRAW_BUFF_HEIGHT,
        .double_buffer = DISP_DRAW_BUFF_DOUBLE,
        .hres = DISP_WIDTH,
        .vres = DISP_HEIGHT,
        .monochrome = false,
#if LVGL_VERSION_MAJOR >= 9
        .color_format = LV_COLOR_FORMAT_RGB565,
#endif
        .rotation = {
            .swap_xy = false,
            .mirror_x = true,
            .mirror_y = true,
        },
        .flags = {
            .buff_dma = true,
#if LVGL_VERSION_MAJOR >= 9
            .swap_bytes = true,
#endif
        }
    };
    lv_display_t *lvgl_disp = lvgl_port_add_disp(&disp_cfg);

    lv_obj_t *scr = lv_scr_act();
    /* Label */
    lv_obj_t *label = lv_label_create(scr);
    lv_obj_set_width(label, DISP_WIDTH);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
#if LVGL_VERSION_MAJOR == 8
    lv_label_set_recolor(label, true);
    lv_label_set_text(label, "#FF0000 "LV_SYMBOL_BELL" Hello world Espressif and LVGL "LV_SYMBOL_BELL"#\n#FF9400 "LV_SYMBOL_WARNING" For simplier initialization, use BSP "LV_SYMBOL_WARNING" #");
#else
    lv_label_set_text(label, LV_SYMBOL_BELL" Hello world Espressif and LVGL "LV_SYMBOL_BELL"\n "LV_SYMBOL_WARNING" For simplier initialization, use BSP "LV_SYMBOL_WARNING);
#endif
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 20);

    while (1) {
        ESP_LOGI(TAG, "program running, current time: %02.2f\n", esp_timer_get_time() / 1000000.0);

        vTaskDelay(2 * 1000 / portTICK_PERIOD_MS);
    }
}
