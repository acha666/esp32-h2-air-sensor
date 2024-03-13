#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

QueueHandle_t tempSensorDataQueue;

typedef struct {
    float temperature;
    float humidity;
} TempSensorData_t;

void TempSensorInitTask(void);
void TempSensorTask(void);

void sht4x_init(void);

#ifdef __cplusplus
}
#endif
