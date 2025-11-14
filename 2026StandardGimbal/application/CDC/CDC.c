#include "CDC.h"
#include "CDC_task.h"
#include "Serial.h"
#include "INS.h"

#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "main.h"
#include "semphr.h"


void CDC_Init(void)
{
    aim_packet_to_nuc.detect_color=0;
    Send_Packet_Init(&aim_packet_to_nuc);
}


void CDC_Receive(void)
{
    if (g_xSemCDC == NULL) 
    {
    /* semaphore not ready yet; avoid calling null handle */
    vTaskDelay(pdMS_TO_TICKS(10));
    return;
    }
  xSemaphoreTake(g_xSemCDC, portMAX_DELAY);

  /* Serial already copied validated frame into aim_packet_from_nuc in UnPack_Data_ROS2 */
  /* Copy relevant fields into outgoing packet so we reply with updated data */
  aim_packet_to_nuc.yaw = INS.Yaw;
  aim_packet_to_nuc.pitch = INS.Pitch;;
}