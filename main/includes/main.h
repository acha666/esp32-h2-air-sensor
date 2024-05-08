#ifndef __MAIN_H
#define __MAIN_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

extern EventGroupHandle_t xZigbeeEvents;
extern QueueHandle_t sensorDataQueue;
extern QueueHandle_t powerDataQueue;
extern QueueHandle_t displayMainDataQueue;
extern QueueHandle_t zigbeeReportDataQueue;
extern TaskHandle_t sensorTaskHandle;
extern TaskHandle_t powerTaskHandle;

void main_task(void *pvParameters);

#endif
