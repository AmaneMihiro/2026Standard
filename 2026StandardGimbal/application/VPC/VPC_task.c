#include "VPC.h"
#include "VPC_task.h"

#include "Serial.h"

#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "main.h"
#include "semphr.h"
#include "usbd_CDC_if.h"

#include "INS.h"

osThreadId_t VPC_task_handel;




void VPC_Task_Init(void)
{
    g_xSemVPC = xSemaphoreCreateBinary();
     if (g_xSemVPC == NULL) {
    }
	osThreadAttr_t task_attr = {
			.name = "VPC_Task",
			.stack_size = 128 * 4 ,
			.priority = (osPriority_t) osPriorityAboveNormal1 ,
	};
	VPC_task_handel = osThreadNew (VPC_Task, NULL, &task_attr);
}



void VPC_Task(void *argument)
{
    VPC_Init();
    for(;;)
    {
        VPC_Receive();
        Pack_And_Send_Data_ROS2(&aim_packet_to_nuc);
        //VPC_SendTarget();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}