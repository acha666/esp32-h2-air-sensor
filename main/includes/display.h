#ifndef __DISPLAY_H
#define __DISPLAY_H

#include "esp_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void display_init_task(void *pvParameters);
    // void display_update_task(void *pvParameters);

    typedef struct
    {
        uint8_t battery_soc;      // in %
        uint16_t battery_voltage; // in mV
        enum BATTERY_CHARING_STATE
        {
            BATTERY_NOT_CHARGING =0,
            BATTERY_CHARGING =1,
        } battery_charging_state;
        enum BATTERY_STATE
        {
            BATTERY_FAULT = 0,
            BATTERY_LOW = 1,
            BATTERY_OK = 2,
            BATTERY_FULL = 3
        } battery_state;
        float temperature;
        float humidity;
        float pressure;
        uint32_t timestamp;
    } display_main_data_t;

#ifdef __cplusplus
}
#endif

#endif