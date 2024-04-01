#include "main.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "string.h"

#include "gui_guider.h"
#include "lv_port_disp.h"

void vTaskLvglTimer(void *pvParameters);

lv_ui guider_ui;

static void lvgl_tick_cb(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(CONFIG_LVGL_TICK_PERIOD);
}

extern "C" void vTaskDisplayInit(void *pvParameters)
{
    lv_init();
    lv_port_disp_init();

    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &lvgl_tick_cb,
        .name = "lvgl_tick"};
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, CONFIG_LVGL_TICK_PERIOD * 1000));

    xTaskCreate(vTaskLvglTimer, "vTaskLvglTimer", 4096, NULL, 1, NULL);

    setup_ui(&guider_ui);

    vTaskDelete(NULL);
}

void vTaskLvglTimer(void *pvParameters)
{
    while (1)
    {
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
