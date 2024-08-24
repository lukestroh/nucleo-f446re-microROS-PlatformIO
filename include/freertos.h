#ifndef __FREERTOS_TASKS_H__
#define __FREERTOS_TASKS_H__

#include "cmsis_os.h"

void StartBlinkLEDTask(void *argument);
void StartmicroROSTask(void *argument);

#endif // __FREERTOS_TASKS_H__
