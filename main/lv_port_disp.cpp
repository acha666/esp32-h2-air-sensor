#include "lv_port_disp.h"
#include "esp_types.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include <stdbool.h>
#include <string.h>
#include <stdexcept>

#include "esp_lcd_panel_ssd1681.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

static constexpr uint16_t DISP_HOR_RES = 200;
static constexpr uint16_t DISP_VER_RES = 200;
static constexpr uint32_t DISP_PIXEL_CLOCK_HZ = 1 * 1000 * 1000;
static constexpr gpio_num_t DISP_PIN_NUM_SCLK = GPIO_NUM_10;
static constexpr gpio_num_t DISP_PIN_NUM_MOSI = GPIO_NUM_11;
static constexpr gpio_num_t DISP_PIN_NUM_MISO = GPIO_NUM_22;
static constexpr gpio_num_t DISP_PIN_NUM_EPD_DC = GPIO_NUM_12;
static constexpr gpio_num_t DISP_PIN_NUM_EPD_RST = GPIO_NUM_4;
static constexpr gpio_num_t DISP_PIN_NUM_EPD_CS = GPIO_NUM_14;
static constexpr gpio_num_t DISP_PIN_NUM_EPD_BUSY = GPIO_NUM_5;
static constexpr char TAG[] = "lv_port_disp";

static void disp_init(void);
static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map);
static void example_lvgl_wait_cb(struct _lv_disp_drv_t *disp_drv);
static void example_lvgl_port_update_callback(lv_disp_drv_t *drv);

static SemaphoreHandle_t panel_refreshing_sem = NULL;
static esp_lcd_panel_handle_t panel_handle = NULL;
static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
static lv_disp_drv_t disp_drv;      // contains callback functions
static uint8_t *converted_buffer_black;

IRAM_ATTR bool epaper_flush_ready_callback(const esp_lcd_panel_handle_t handle, const void *edata, void *user_data)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_data;
    lv_disp_flush_ready(disp_driver);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(panel_refreshing_sem, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        return true;
    }
    return false;
}

void lv_port_disp_init(void)
{
    panel_refreshing_sem = xSemaphoreCreateBinary();

    // --- Initialize display
    disp_init();

    // --- Initialize LVGL

    // alloc draw buffers used by LVGL
    static lv_color_t buf_1[DISP_HOR_RES * DISP_VER_RES]; /*A screen sized buffer*/
    static lv_color_t buf_2[DISP_HOR_RES * DISP_VER_RES]; /*Another screen sized buffer*/
    lv_disp_draw_buf_init(&disp_buf, buf_1, buf_2,
                          DISP_VER_RES * DISP_HOR_RES); /*Initialize the display buffer*/

    // alloc bitmap buffer to draw
    converted_buffer_black = (uint8_t *)heap_caps_malloc(DISP_HOR_RES * DISP_VER_RES / 8, MALLOC_CAP_DMA);
    // converted_buffer_red = (uint8_t *)heap_caps_malloc(EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES / 8, MALLOC_CAP_DMA);

    // --- initialize LVGL display driver
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = DISP_HOR_RES;
    disp_drv.ver_res = DISP_VER_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.wait_cb = example_lvgl_wait_cb;
    disp_drv.drv_update_cb = example_lvgl_port_update_callback;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    // NOTE: The ssd1681 e-paper is monochrome and 1 byte represents 8 pixels
    // so full_refresh is MANDATORY because we cannot set position to bitmap at pixel level
    disp_drv.full_refresh = true;

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_register(&disp_drv);
}

// void display_refresh(void)
// {
//     ESP_ERROR_CHECK(epaper_panel_refresh_screen(panel_handle));
// }

static void disp_init(void)
{
    spi_bus_config_t bus_config = {
        .mosi_io_num = DISP_PIN_NUM_MOSI,
        .miso_io_num = DISP_PIN_NUM_MISO,
        .sclk_io_num = DISP_PIN_NUM_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 5005,
        .flags = SPICOMMON_BUSFLAG_MASTER,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = DISP_PIN_NUM_EPD_CS,
        .dc_gpio_num = DISP_PIN_NUM_EPD_DC,
        .spi_mode = 0,
        .pclk_hz = DISP_PIXEL_CLOCK_HZ,
        .trans_queue_depth = 10,
        .on_color_trans_done = NULL,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    // --- Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &io_handle));

    esp_lcd_ssd1681_config_t epaper_ssd1681_config = {
        .busy_gpio_num = DISP_PIN_NUM_EPD_BUSY,
        .non_copy_mode = true,
    };
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = DISP_PIN_NUM_EPD_RST,
        .flags = {.reset_active_high = false},
        .vendor_config = &epaper_ssd1681_config,
    };

    gpio_install_isr_service(0);
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1681(io_handle, &panel_config, &panel_handle));

    // --- Reset the display
    ESP_LOGI(TAG, "Resetting e-Paper display...");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // --- Initialize panel
    ESP_LOGI(TAG, "Initializing e-Paper display...");
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    // --- Register the e-Paper refresh done callback
    epaper_panel_callbacks_t cbs = {
        .on_epaper_refresh_done = epaper_flush_ready_callback};
    epaper_panel_register_event_callbacks(panel_handle, &cbs, &disp_drv);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // --- Turn on display
    ESP_LOGI(TAG, "Turning e-Paper display on...");
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // --- Configurate the screen
    // NOTE: the configurations below are all FALSE by default
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, false, false));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, false));
    esp_lcd_panel_invert_color(panel_handle, false);
    // NOTE: Calling esp_lcd_panel_disp_on_off(panel_handle, true) will reset the LUT to the panel built-in one,
    // custom LUT will not take effect any more after calling esp_lcd_panel_disp_on_off(panel_handle, true)
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // Used to vertical traverse lvgl framebuffer
    // int len_x = abs(offsetx1 - offsetx2) + 1;
    // int len_y = abs(offsety1 - offsety2) + 1;
    // --- Convert buffer from color to monochrome bitmap
    int len_bits = (abs(offsetx1 - offsetx2) + 1) * (abs(offsety1 - offsety2) + 1);

    memset(converted_buffer_black, 0x00, len_bits / 8);
    // memset(converted_buffer_red, 0x00, len_bits / 8);
    for (int i = 0; i < len_bits; i++)
    {
        // NOTE: Set bits of converted_buffer[] FROM LOW ADDR TO HIGH ADDR, FROM HSB TO LSB
        // NOTE: 1 means BLACK/RED, 0 means WHITE
        // Horizontal traverse lvgl framebuffer (by row)
        converted_buffer_black[i / 8] |= (((lv_color_brightness(color_map[i])) < 251) << (7 - (i % 8)));
        // converted_buffer_red[i / 8] |= ((((color_map[i].ch.red) > 3) && ((lv_color_brightness(color_map[i])) < 251)) << (7 - (i % 8)));
        // Vertical traverse lvgl framebuffer (by column), needs to uncomment len_x and len_y
        // NOTE: If your screen rotation requires setting the pixels vertically, you could use the code below
        // converted_buffer[i/8] |= (((lv_color_brightness(color_map[((i*len_x)%len_bits) + i/len_y])) > 250) << (7-(i % 8)));
    }
    // --- Draw bitmap

    ESP_ERROR_CHECK(epaper_panel_set_bitmap_color(panel_handle, SSD1681_EPAPER_BITMAP_BLACK));
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, converted_buffer_black));
    // ESP_ERROR_CHECK(epaper_panel_set_bitmap_color(panel_handle, SSD1681_EPAPER_BITMAP_RED));
    // ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, converted_buffer_red));
    ESP_ERROR_CHECK(epaper_panel_refresh_screen(panel_handle));
}

static void example_lvgl_wait_cb(struct _lv_disp_drv_t *disp_drv)
{
    xSemaphoreTake(panel_refreshing_sem, portMAX_DELAY);
}

static void example_lvgl_port_update_callback(lv_disp_drv_t *drv)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;

    switch (drv->rotated)
    {
    case LV_DISP_ROT_NONE:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, false);
        break;
    case LV_DISP_ROT_90:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, true);
        break;
    case LV_DISP_ROT_180:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, true);
        break;
    case LV_DISP_ROT_270:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, false);
        break;
    }
}