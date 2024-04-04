#ifndef SENSOR_H
#define SENSOR_H

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        float temperature;
        float humidity;
        float pressure;
        uint32_t timestamp;
    } sensor_data_t;

    void sensor_init_task(void *pvParameters);
    void sensor_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif
