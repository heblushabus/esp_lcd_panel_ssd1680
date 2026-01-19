/*
 * SPDX-FileCopyrightText: 2021-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 * 
 * Partial Refresh Demo for SSD1680 E-Paper with LVGL
 */

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lvgl.h"
#include <esp_lcd_panel_ssd1680.h>
#include <stdio.h>

static const char *TAG = "epaper_lvgl";

// Using SPI2 in the example
#define LCD_HOST SPI2_HOST

// SPI Clock - 1MHz for e-paper
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ (1 * 1000 * 1000)

// Pin definitions for your board
#define EXAMPLE_PIN_NUM_SCLK 41
#define EXAMPLE_PIN_NUM_MOSI 39
#define EXAMPLE_PIN_NUM_MISO (-1)
#define EXAMPLE_PIN_NUM_EPD_DC 38
#define EXAMPLE_PIN_NUM_EPD_RST 2
#define EXAMPLE_PIN_NUM_EPD_CS 48
#define EXAMPLE_PIN_NUM_EPD_BUSY 1

// Display dimensions - 2.9" display is 128x296
#define EXAMPLE_LCD_H_RES 128
#define EXAMPLE_LCD_V_RES 296

#define EXAMPLE_LCD_CMD_BITS 8
#define EXAMPLE_LCD_PARAM_BITS 8
#define EXAMPLE_LVGL_TICK_PERIOD_MS 2

static SemaphoreHandle_t panel_refreshing_sem = NULL;
static esp_lcd_panel_handle_t panel_handle = NULL;
static bool use_partial_refresh = false;

// Counter value for demo
static int counter_value = 0;
static lv_obj_t *counter_label = NULL;
static lv_obj_t *mode_label = NULL;

IRAM_ATTR bool epaper_flush_ready_callback(const esp_lcd_panel_handle_t handle,
                                           const void *edata, void *user_data) {
  lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_data;
  lv_disp_flush_ready(disp_driver);
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(panel_refreshing_sem, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken == pdTRUE) {
    return true;
  }
  return false;
}

static uint8_t *converted_buffer_black;

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area,
                                  lv_color_t *color_map) {
  esp_lcd_panel_handle_t ph = (esp_lcd_panel_handle_t)drv->user_data;
  int offsetx1 = area->x1;
  int offsetx2 = area->x2;
  int offsety1 = area->y1;
  int offsety2 = area->y2;

  // Convert buffer from color to monochrome bitmap
  int len_bits = (abs(offsetx1 - offsetx2) + 1) * (abs(offsety1 - offsety2) + 1);
  int len_bytes = (len_bits + 7) / 8;

  memset(converted_buffer_black, 0x00, len_bytes);

  for (int i = 0; i < len_bits; i++) {
    // 1 means BLACK, 0 means WHITE
    converted_buffer_black[i / 8] |=
        (((lv_color_brightness(color_map[i])) < 128) << (7 - (i % 8)));
  }

  // Draw bitmap
  epaper_panel_set_bitmap_color(ph, SSD1680_EPAPER_BITMAP_BLACK);
  esp_err_t ret = esp_lcd_panel_draw_bitmap(ph, offsetx1, offsety1, 
                                             offsetx2 + 1, offsety2 + 1,
                                             converted_buffer_black);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "draw_bitmap failed: %s", esp_err_to_name(ret));
  }
  
  // Refresh screen
  epaper_panel_refresh_screen(ph);
}

static void example_lvgl_wait_cb(struct _lv_disp_drv_t *disp_drv) {
  xSemaphoreTake(panel_refreshing_sem, portMAX_DELAY);
}

static void example_lvgl_port_update_callback(lv_disp_drv_t *drv) {
  esp_lcd_panel_handle_t ph = (esp_lcd_panel_handle_t)drv->user_data;
  switch (drv->rotated) {
  case LV_DISP_ROT_NONE:
    esp_lcd_panel_swap_xy(ph, false);
    esp_lcd_panel_mirror(ph, false, false);
    break;
  case LV_DISP_ROT_90:
    esp_lcd_panel_swap_xy(ph, true);
    esp_lcd_panel_mirror(ph, false, true);
    break;
  case LV_DISP_ROT_180:
    esp_lcd_panel_swap_xy(ph, false);
    esp_lcd_panel_mirror(ph, true, true);
    break;
  case LV_DISP_ROT_270:
    esp_lcd_panel_swap_xy(ph, true);
    esp_lcd_panel_mirror(ph, true, false);
    break;
  }
}

static void example_increase_lvgl_tick(void *arg) {
  lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

// Timer callback to update the counter (demonstrates partial refresh)
static void counter_timer_cb(lv_timer_t *timer) {
  counter_value++;
  if (counter_value > 999) counter_value = 0;
  
  char buf[32];
  snprintf(buf, sizeof(buf), "%03d", counter_value);
  lv_label_set_text(counter_label, buf);
}

// Button callback to toggle between full and partial refresh
static void toggle_refresh_mode_cb(lv_event_t *e) {
  use_partial_refresh = !use_partial_refresh;
  
  // Set the refresh mode in the driver
  epaper_panel_set_refresh_mode(panel_handle, !use_partial_refresh);
  
  // Update mode label
  if (use_partial_refresh) {
    lv_label_set_text(mode_label, "Mode: PARTIAL");
    ESP_LOGI(TAG, "Switched to PARTIAL refresh mode");
  } else {
    lv_label_set_text(mode_label, "Mode: FULL");
    ESP_LOGI(TAG, "Switched to FULL refresh mode");
  }
}

// Create demo UI
void example_lvgl_demo_ui(lv_disp_t *disp) {
  lv_obj_t *scr = lv_disp_get_scr_act(disp);
  
  // Set screen background to white
  lv_obj_set_style_bg_color(scr, lv_color_white(), LV_PART_MAIN);
  
  // Title
  lv_obj_t *title = lv_label_create(scr);
  lv_label_set_text(title, "Partial Refresh Demo");
  lv_obj_set_style_text_color(title, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_text_font(title, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);
  
  // Mode label
  mode_label = lv_label_create(scr);
  lv_label_set_text(mode_label, "Mode: FULL");
  lv_obj_set_style_text_color(mode_label, lv_color_black(), LV_PART_MAIN);
  lv_obj_align(mode_label, LV_ALIGN_TOP_MID, 0, 35);
  
  // Counter container
  lv_obj_t *counter_cont = lv_obj_create(scr);
  lv_obj_set_size(counter_cont, 100, 60);
  lv_obj_align(counter_cont, LV_ALIGN_CENTER, 0, -30);
  lv_obj_set_style_bg_color(counter_cont, lv_color_white(), LV_PART_MAIN);
  lv_obj_set_style_border_color(counter_cont, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_border_width(counter_cont, 2, LV_PART_MAIN);
  lv_obj_set_style_radius(counter_cont, 5, LV_PART_MAIN);
  
  // Counter label (this updates frequently to show partial refresh)
  counter_label = lv_label_create(counter_cont);
  lv_label_set_text(counter_label, "000");
  lv_obj_set_style_text_color(counter_label, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_text_font(counter_label, &lv_font_montserrat_28, LV_PART_MAIN);
  lv_obj_center(counter_label);
  
  // Toggle button
  lv_obj_t *btn = lv_btn_create(scr);
  lv_obj_set_size(btn, 110, 40);
  lv_obj_align(btn, LV_ALIGN_CENTER, 0, 50);
  lv_obj_set_style_bg_color(btn, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_radius(btn, 5, LV_PART_MAIN);
  lv_obj_add_event_cb(btn, toggle_refresh_mode_cb, LV_EVENT_CLICKED, NULL);
  
  lv_obj_t *btn_label = lv_label_create(btn);
  lv_label_set_text(btn_label, "Toggle Mode");
  lv_obj_set_style_text_color(btn_label, lv_color_white(), LV_PART_MAIN);
  lv_obj_center(btn_label);
  
  // Info text
  lv_obj_t *info = lv_label_create(scr);
  lv_label_set_text(info, "Partial is faster\nbut may ghost");
  lv_obj_set_style_text_color(info, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_text_align(info, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_align(info, LV_ALIGN_CENTER, 0, 110);
  
  // Footer
  lv_obj_t *footer = lv_label_create(scr);
  lv_label_set_text(footer, "SSD1680 128x296");
  lv_obj_set_style_text_color(footer, lv_color_black(), LV_PART_MAIN);
  lv_obj_align(footer, LV_ALIGN_BOTTOM_MID, 0, -10);
  
  // Create timer for counter update (every 2 seconds)
  lv_timer_create(counter_timer_cb, 2000, NULL);
}

void app_main(void) {
  static lv_disp_draw_buf_t disp_buf;
  static lv_disp_drv_t disp_drv;

  panel_refreshing_sem = xSemaphoreCreateBinary();
  xSemaphoreGive(panel_refreshing_sem);

  // Power enable pin
  gpio_reset_pin(GPIO_NUM_42);
  gpio_set_direction(GPIO_NUM_42, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_NUM_42, 1);

  ESP_LOGI(TAG, "Initialize SPI bus");
  spi_bus_config_t buscfg = {
      .sclk_io_num = EXAMPLE_PIN_NUM_SCLK,
      .mosi_io_num = EXAMPLE_PIN_NUM_MOSI,
      .miso_io_num = EXAMPLE_PIN_NUM_MISO,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES / 8,
  };
  ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

  ESP_LOGI(TAG, "Install panel IO");
  esp_lcd_panel_io_handle_t io_handle = NULL;
  esp_lcd_panel_io_spi_config_t io_config = {
      .dc_gpio_num = EXAMPLE_PIN_NUM_EPD_DC,
      .cs_gpio_num = EXAMPLE_PIN_NUM_EPD_CS,
      .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
      .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
      .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
      .spi_mode = 0,
      .trans_queue_depth = 10,
      .on_color_trans_done = NULL,
  };
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST,
                                           &io_config, &io_handle));

  // Create esp_lcd panel
  ESP_LOGI(TAG, "Creating SSD1680 panel...");
  esp_lcd_ssd1680_config_t epaper_ssd1680_config = {
      .busy_gpio_num = EXAMPLE_PIN_NUM_EPD_BUSY,
      .non_copy_mode = true,
  };
  esp_lcd_panel_dev_config_t panel_config = {
      .reset_gpio_num = EXAMPLE_PIN_NUM_EPD_RST,
      .flags.reset_active_high = false,
      .vendor_config = &epaper_ssd1680_config
  };
  gpio_install_isr_service(0);
  ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1680(io_handle, &panel_config, &panel_handle));

  // Reset and init display
  ESP_LOGI(TAG, "Resetting e-Paper display...");
  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
  vTaskDelay(pdMS_TO_TICKS(100));
  
  ESP_LOGI(TAG, "Initializing e-Paper display...");
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
  vTaskDelay(pdMS_TO_TICKS(100));
  
  // Register callback
  epaper_panel_callbacks_t cbs = {
      .on_epaper_refresh_done = epaper_flush_ready_callback
  };
  epaper_panel_register_event_callbacks(panel_handle, &cbs, &disp_drv);
  
  // Turn on display
  ESP_LOGI(TAG, "Turning e-Paper display on...");
  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
  vTaskDelay(pdMS_TO_TICKS(100));
  
  // Configure display
  ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, false));
  ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, false));
  esp_lcd_panel_invert_color(panel_handle, false);
  
  // Start with full refresh mode
  epaper_panel_set_refresh_mode(panel_handle, true);

  // Initialize LVGL
  ESP_LOGI(TAG, "Initialize LVGL library");
  lv_init();
  
  // Allocate LVGL buffers
  size_t buf_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES;
  lv_color_t *buf1 = heap_caps_malloc(buf_size * sizeof(lv_color_t), MALLOC_CAP_DMA);
  lv_color_t *buf2 = heap_caps_malloc(buf_size * sizeof(lv_color_t), MALLOC_CAP_DMA);
  assert(buf1 && buf2);
  
  // Allocate monochrome conversion buffer
  converted_buffer_black = heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES / 8, MALLOC_CAP_DMA);
  assert(converted_buffer_black);
  
  // Initialize LVGL display
  lv_disp_draw_buf_init(&disp_buf, buf1, buf2, buf_size);
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = EXAMPLE_LCD_H_RES;
  disp_drv.ver_res = EXAMPLE_LCD_V_RES;
  disp_drv.flush_cb = example_lvgl_flush_cb;
  disp_drv.wait_cb = example_lvgl_wait_cb;
  disp_drv.drv_update_cb = example_lvgl_port_update_callback;
  disp_drv.draw_buf = &disp_buf;
  disp_drv.user_data = panel_handle;
  disp_drv.full_refresh = true;  // Required for e-paper
  
  ESP_LOGI(TAG, "Register display driver to LVGL");
  lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

  // Init LVGL tick
  ESP_LOGI(TAG, "Install LVGL tick timer");
  const esp_timer_create_args_t lvgl_tick_timer_args = {
      .callback = &example_increase_lvgl_tick, 
      .name = "lvgl_tick"
  };
  esp_timer_handle_t lvgl_tick_timer = NULL;
  ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

  ESP_LOGI(TAG, "Display Partial Refresh Demo UI");
  example_lvgl_demo_ui(disp);

  ESP_LOGI(TAG, "Starting main loop - press button to toggle refresh mode");
  while (1) {
    lv_timer_handler();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
