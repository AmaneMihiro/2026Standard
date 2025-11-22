#ifndef __VPC_task_H__
#define __VPC_task_H__

#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "main.h"
#include "semphr.h"

#include "INS.h"


extern osThreadId_t VPC_task_handel;
extern SemaphoreHandle_t g_xSemVPC;


extern void VPC_Task_Init(void);
extern void VPC_Task(void *argument);




#endif