#include "CDC.h"
#include "CDC_task.h"

#include "Serial.h"

#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "main.h"
#include "semphr.h"
#include "usbd_cdc_if.h"

#include "INS.h"

osThreadId_t cdc_task_handel;




void CDC_Task_Init(void)
{
    g_xSemCDC = xSemaphoreCreateBinary();
     if (g_xSemCDC == NULL) {
    }
	osThreadAttr_t task_attr = {
			.name = "CDC_Task",
			.stack_size = 128 * 4 ,
			.priority = (osPriority_t) osPriorityAboveNormal1 ,
	};
	cdc_task_handel = osThreadNew (CDC_Task, NULL, &task_attr);
}



void CDC_Task(void *argument)
{
    CDC_Init();
    for(;;)
    {
        CDC_Receive();
        Pack_And_Send_Data_ROS2(&aim_packet_to_nuc);
        vTaskDelay(pdMS_TO_TICKS(100));//原本为1，现调试需要改为100
    }
}