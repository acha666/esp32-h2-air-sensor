#ifndef __MAIN_H
#define __MAIN_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

extern TaskHandle_t tempSensorTaskHandle;
extern QueueHandle_t sensorDataQueue;

void MainTask(void *pvParameters);

#endif
