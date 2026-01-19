/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_ssd1680.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <inttypes.h>
#include <string.h>

// Display dimensions for SSD1680 2.9" display
#define EPD_WIDTH 128
#define EPD_HEIGHT 296

// SPI Bus
#define EPD_PANEL_SPI_CLK 1000000
#define EPD_PANEL_SPI_CMD_BITS 8
#define EPD_PANEL_SPI_PARAM_BITS 8
#define EPD_PANEL_SPI_MODE 0
// e-Paper GPIO
#define EXAMPLE_PIN_NUM_EPD_DC 38
#define EXAMPLE_PIN_NUM_EPD_RST 2
#define EXAMPLE_PIN_NUM_EPD_CS 48
#define EXAMPLE_PIN_NUM_EPD_BUSY 1
// e-Paper SPI
#define EXAMPLE_PIN_NUM_MOSI 39
#define EXAMPLE_PIN_NUM_SCLK 41

static const char *TAG = "epaper_demo_plain";

static bool give_semaphore_in_isr(const esp_lcd_panel_handle_t handle,
                                  const void *edata, void *user_data) {
  SemaphoreHandle_t *epaper_panel_semaphore_ptr = user_data;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(*epaper_panel_semaphore_ptr, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
    return true;
  }
  return false;
}

void app_main(void) {
  esp_err_t ret;

  // Set pin 42 to HIGH (power enable?)
  gpio_reset_pin(GPIO_NUM_42);
  gpio_set_direction(GPIO_NUM_42, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_NUM_42, 1);
  
  // --- Init SPI Bus
  ESP_LOGI(TAG, "Initializing SPI Bus...");
  spi_bus_config_t buscfg = {
      .sclk_io_num = EXAMPLE_PIN_NUM_SCLK,
      .mosi_io_num = EXAMPLE_PIN_NUM_MOSI,
      .miso_io_num = -1,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = SOC_SPI_MAXIMUM_BUFFER_SIZE
  };
  ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
  
  // --- Init ESP_LCD IO
  ESP_LOGI(TAG, "Initializing panel IO...");
  esp_lcd_panel_io_handle_t io_handle = NULL;
  esp_lcd_panel_io_spi_config_t io_config = {
      .dc_gpio_num = EXAMPLE_PIN_NUM_EPD_DC,
      .cs_gpio_num = EXAMPLE_PIN_NUM_EPD_CS,
      .pclk_hz = EPD_PANEL_SPI_CLK,
      .lcd_cmd_bits = EPD_PANEL_SPI_CMD_BITS,
      .lcd_param_bits = EPD_PANEL_SPI_PARAM_BITS,
      .spi_mode = EPD_PANEL_SPI_MODE,
      .trans_queue_depth = 10,
      .on_color_trans_done = NULL
  };
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST,
                                           &io_config, &io_handle));
  
  // --- Create esp_lcd panel
  ESP_LOGI(TAG, "Creating SSD1680 panel...");
  esp_lcd_ssd1680_config_t epaper_ssd1680_config = {
      .busy_gpio_num = EXAMPLE_PIN_NUM_EPD_BUSY,
      .non_copy_mode = false,
  };
  esp_lcd_panel_dev_config_t panel_config = {
      .reset_gpio_num = EXAMPLE_PIN_NUM_EPD_RST,
      .flags.reset_active_high = false,
      .vendor_config = &epaper_ssd1680_config
  };
  esp_lcd_panel_handle_t panel_handle = NULL;
  
  // NOTE: gpio_install_isr_service() must be called before esp_lcd_new_panel_ssd1680()
  gpio_install_isr_service(0);
  ret = esp_lcd_new_panel_ssd1680(io_handle, &panel_config, &panel_handle);
  ESP_ERROR_CHECK(ret);
  
  // --- Reset the display
  ESP_LOGI(TAG, "Resetting e-Paper display...");
  ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
  vTaskDelay(pdMS_TO_TICKS(100));
  
  // --- Initialize LCD panel
  ESP_LOGI(TAG, "Initializing e-Paper display...");
  ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
  vTaskDelay(pdMS_TO_TICKS(100));
  
  // Turn on the screen
  ESP_LOGI(TAG, "Turning e-Paper display on...");
  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
  vTaskDelay(pdMS_TO_TICKS(100));

  // --- Create semaphore for refresh synchronization
  static SemaphoreHandle_t epaper_panel_semaphore;
  epaper_panel_semaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(epaper_panel_semaphore);

  // --- Register the e-Paper refresh done callback
  epaper_panel_callbacks_t cbs = {
      .on_epaper_refresh_done = give_semaphore_in_isr,
  };
  epaper_panel_register_event_callbacks(panel_handle, &cbs, &epaper_panel_semaphore);

  // --- Allocate bitmap buffer (128x296 = 4736 bytes)
  size_t bitmap_size = EPD_WIDTH * EPD_HEIGHT / 8;
  uint8_t *bitmap = heap_caps_malloc(bitmap_size, MALLOC_CAP_DMA);
  if (!bitmap) {
    ESP_LOGE(TAG, "Failed to allocate bitmap buffer!");
    return;
  }

  // === TEST 1: Clear screen to white ===
  ESP_LOGI(TAG, "TEST 1: Clearing screen to white...");
  xSemaphoreTake(epaper_panel_semaphore, portMAX_DELAY);
  memset(bitmap, 0xFF, bitmap_size);  // 0xFF = all white
  epaper_panel_set_bitmap_color(panel_handle, SSD1680_EPAPER_BITMAP_BLACK);
  ret = esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, EPD_WIDTH, EPD_HEIGHT, bitmap);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "draw_bitmap failed: %s", esp_err_to_name(ret));
  }
  ESP_ERROR_CHECK(epaper_panel_refresh_screen(panel_handle));
  ESP_LOGI(TAG, "Screen cleared to white, waiting 3 seconds...");
  vTaskDelay(pdMS_TO_TICKS(3000));

  // === TEST 2: Fill screen to black ===
  ESP_LOGI(TAG, "TEST 2: Filling screen to black...");
  xSemaphoreTake(epaper_panel_semaphore, portMAX_DELAY);
  memset(bitmap, 0x00, bitmap_size);  // 0x00 = all black
  ret = esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, EPD_WIDTH, EPD_HEIGHT, bitmap);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "draw_bitmap failed: %s", esp_err_to_name(ret));
  }
  ESP_ERROR_CHECK(epaper_panel_refresh_screen(panel_handle));
  ESP_LOGI(TAG, "Screen filled to black, waiting 3 seconds...");
  vTaskDelay(pdMS_TO_TICKS(3000));

  // === TEST 3: Draw vertical stripes ===
  ESP_LOGI(TAG, "TEST 3: Drawing vertical stripes...");
  xSemaphoreTake(epaper_panel_semaphore, portMAX_DELAY);
  memset(bitmap, 0xF0, bitmap_size);  // Vertical stripes (4 pixels on, 4 off)
  ret = esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, EPD_WIDTH, EPD_HEIGHT, bitmap);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "draw_bitmap failed: %s", esp_err_to_name(ret));
  }
  ESP_ERROR_CHECK(epaper_panel_refresh_screen(panel_handle));
  ESP_LOGI(TAG, "Vertical stripes drawn.");

  // === TEST 4: Partial Refresh Demo ===
  ESP_LOGI(TAG, "TEST 4: Partial Refresh Demo...");

  // 1. Clear to white first with full refresh
  xSemaphoreTake(epaper_panel_semaphore, portMAX_DELAY);
  epaper_panel_set_refresh_mode(panel_handle, true);
  epaper_panel_clear_screen(panel_handle, 0xFF, true);
  vTaskDelay(pdMS_TO_TICKS(1000));

  // 2. Loop for moving box
  int box_w = 40;
  int box_h = 40;
  int start_x = (EPD_WIDTH - box_w) / 2;
  
  for (int i = 0; i < 6; i++) {
      ESP_LOGI(TAG, "Partial update frame %d/6", i+1);
      int start_y = i * 40 + 10;
      
      xSemaphoreTake(epaper_panel_semaphore, portMAX_DELAY);
      memset(bitmap, 0xFF, bitmap_size); // Clear local buffer to white
      
      // Draw box in local buffer
      for (int y = start_y; y < start_y + box_h; y++) {
          for (int x = start_x; x < start_x + box_w; x++) {
              if (y < EPD_HEIGHT && x < EPD_WIDTH) {
                  int byte_idx = (y * EPD_WIDTH + x) / 8;
                  int bit_idx = 7 - ((y * EPD_WIDTH + x) % 8);
                  bitmap[byte_idx] &= ~(1 << bit_idx);
              }
          }
      }

      // Step A: Write to Current RAM (0x24) - Set partial mode (forces update to 0x24 only)
      epaper_panel_set_refresh_mode(panel_handle, false);
      esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, EPD_WIDTH, EPD_HEIGHT, bitmap);

      // Step B: Update Display (uses Partial LUT, compares 0x24 vs 0x26)
      ESP_ERROR_CHECK(epaper_panel_refresh_screen(panel_handle));

      // Step C: Update Previous RAM (0x26) - Set full mode (writes both 0x24 and 0x26)
      // This ensures 0x26 is ready for the NEXT partial update comparison.
      // We do NOT call refresh here.
      epaper_panel_set_refresh_mode(panel_handle, true);
      esp_lcd_panel_draw_bitmap(panel_handle, 0, 0, EPD_WIDTH, EPD_HEIGHT, bitmap);
      
      vTaskDelay(pdMS_TO_TICKS(500));
  }
  
  // Restore full refresh mode
  epaper_panel_set_refresh_mode(panel_handle, true);

  // === Done ===
  ESP_LOGI(TAG, "All tests complete!");
  
  // Clean up
  free(bitmap);
  
  // Put display to sleep
  vTaskDelay(pdMS_TO_TICKS(5000));
  ESP_LOGI(TAG, "Putting display to sleep...");
  esp_lcd_panel_disp_on_off(panel_handle, false);
  
  ESP_LOGI(TAG, "Demo finished.");
}
