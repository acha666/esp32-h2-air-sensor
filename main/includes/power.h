#ifndef __POWER_H
#define __POWER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "main.h"

    void PowerTask(void *pvParameters);

    void BQScanTask();

    void ChargerTestTask();

#ifdef __cplusplus
}
#endif

#endif
