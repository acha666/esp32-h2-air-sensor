#ifndef SENSOR_H
#define SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float temperature;
    float humidity;
} TempSensorData_t;

void TempSensorInitTask(void *pvParameters);
void TempSensorTask(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif
