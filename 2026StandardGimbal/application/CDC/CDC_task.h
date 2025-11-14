#ifndef __CDC_task_H__
#define __CDC_task_H__

#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "main.h"
#include "semphr.h"

#include "INS.h"


extern osThreadId_t cdc_task_handel;
extern SemaphoreHandle_t g_xSemCDC;


extern void CDC_Task_Init(void);
extern void CDC_Task(void *argument);




#endif