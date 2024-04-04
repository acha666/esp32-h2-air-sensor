#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string>
#include <format>

#include "gui_guider.h"
#include "lv_port_disp.h"
#include "main.h"
#include "power.h"
#include "display.h"

static void lvgl_timer_task(void *pvParameters);
static void display_update_task(void *pvParameters);

lv_ui guider_ui;

static void lvgl_tick_cb(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(CONFIG_LVGL_TICK_PERIOD);
}

extern "C" void display_init_task(void *pvParameters)
{
    // Initialize lvgl and the display
    lv_init();
    lv_port_disp_init();

    // Create a periodic timer to call lv_tick_inc
    esp_timer_create_args_t lvgl_tick_timer_args;
    lvgl_tick_timer_args.callback = &lvgl_tick_cb;
    lvgl_tick_timer_args.name = "lvgl_tick";
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, CONFIG_LVGL_TICK_PERIOD * 1000));

    // Create a task to handle lvgl timers
    xTaskCreate(lvgl_timer_task, "lvgl_timer_task", 4096, NULL, 1, NULL);

    setup_ui(&guider_ui);

    // Create a task to update the display
    xTaskCreate(display_update_task, "display_update_task", 4096, NULL, 1, NULL);

    vTaskDelete(NULL);
}

static void display_update_task(void *pvParameters)
{
    display_main_data_t data;
    while(1)
    {
        if(xQueueReceive(displayMainDataQueue, &data, portMAX_DELAY))
        {
            lv_label_set_text(guider_ui.main_label_1,std::format("Batt SOC: {}%  {:.2f}V", data.battery_soc, data.battery_voltage/1000.0).c_str());
            std::string state, charging_state;
            switch(data.battery_state)
            {
                case display_main_data_t::BATTERY_FAULT:
                    state = "Fault";
                    break;
                case display_main_data_t::BATTERY_LOW:
                    state = "Low";
                    break;
                case display_main_data_t::BATTERY_OK:
                    state = "OK";
                    break;
                case display_main_data_t::BATTERY_FULL:
                    state = "Full";
                    break;
            }
            switch(data.battery_charging_state)
            {
                case display_main_data_t::BATTERY_NOT_CHARGING:
                    charging_state = "Not Charging";
                    break;
                case display_main_data_t::BATTERY_CHARGING:
                    charging_state = "Charging";
                    break;
            }
            lv_label_set_text(guider_ui.main_label_2,std::format("Batt State: {} {}", state, charging_state).c_str());
            lv_label_set_text(guider_ui.main_label_3,std::format("Temp: {:.2f}C  Hum: {:.2f}%", data.temperature, data.humidity).c_str());
            lv_label_set_text(guider_ui.main_label_4,std::format("Pressure: {:.2f}hPa", data.pressure).c_str());
        }
    }
}

static void lvgl_timer_task(void *pvParameters)
{
    while (1)
    {
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
