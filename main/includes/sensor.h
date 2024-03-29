#ifndef SENSOR_H
#define SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float temperature;
    float humidity;
    float pressure;
} sensorData_t;

void SensorInitTask(void *pvParameters);
void SensorTask(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif
