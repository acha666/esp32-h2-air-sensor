#ifndef __POWER_H
#define __POWER_H

#ifdef __cplusplus
extern "C"
{
#endif

    void power_task(void *pvParameters);

    typedef struct
    {
        // data from gauge
        uint8_t gauge_battery_soc;           // in %
        uint8_t gauge_battery_soh;           // in %
        uint16_t gauge_battery_voltage;      // in mV
        uint16_t gauge_temperature;          // in 0.1 Kelvin
        uint16_t gauge_average_current;      // in mA
        uint16_t gauge_average_power;        // in mW
        uint16_t gauge_remaining_capacity;   // in %
        uint16_t gauge_full_charge_capacity; // in %

        enum
        {
            UNDER_TEMP,
            NORMAL_TEMP,
            OVER_TEMP
        } gauge_temperature_state;
        bool gauge_fc;  // full charge
        bool gauge_chg; // fast charging allowed

        // data from charger
        uint16_t charger_current_limit;
        uint16_t charger_vbus;
        uint16_t charger_ibus;
        uint16_t charger_vbat;
        uint16_t charger_ibat;

        bool charger_charging;

        // fault flag
        bool charger_fault;
        bool gauge_fault;

        uint32_t timestamp;
    } power_data_t;

#ifdef __cplusplus
}
#endif

#endif
