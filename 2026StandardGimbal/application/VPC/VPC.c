#include "VPC.h"
#include "Serial.h"
#include "INS.h"
#include "gimbal.h"

#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "main.h"
#include "semphr.h"

void VPC_Init(void)
{
   NV_Send_Packet_Init(&nv_aim_packet_to_nuc);
  // VS_Send_Packet_Init(&vs_aim_packet_to_nuc);
  //Send_Packet_Init(&aim_packet_to_nuc);
}

void VPC_Receive(void) // 并非接收函数，这里的receive指的是接收下位机的数据存入发送数组准备传给上位机
{
  //   if (g_xSemVPC == NULL)
  //   {
  //   /* semaphore not ready yet; avoid calling null handle */
  //   vTaskDelay(pdMS_TO_TICKS(10));
  //   return;
  //   }
  // xSemaphoreTake(g_xSemVPC, portMAX_DELAY);

  /* Serial already copied validated frame into aim_packet_from_nuc in UnPack_Data_ROS2 */
  /* Copy relevant fields into outgoing packet so we reply with updated data */

  // /*导航传输数据区*/
  nv_aim_packet_to_nuc.yaw = INS.Yaw;
  nv_aim_packet_to_nuc.pitch = INS.Pitch;

  // /*视觉传输数据区*/
  // vs_aim_packet_to_nuc.yaw = INS.Yaw;
  // vs_aim_packet_to_nuc.pitch = INS.Pitch;

//   aim_packet_to_nuc.detect_color = 1;
//   aim_packet_to_nuc.task_mode = 1; // 0-auto 1-aim 2-buff
//   aim_packet_to_nuc.reset_tracker = 1;
//   aim_packet_to_nuc.is_play = 1;
//   aim_packet_to_nuc.change_target = 1;
//   aim_packet_to_nuc.reserved = 1;
//   aim_packet_to_nuc.roll = INS.Roll;
//   aim_packet_to_nuc.pitch = INS.Pitch;
//   aim_packet_to_nuc.yaw = INS.Yaw;
//   aim_packet_to_nuc.aim_x = 1.23;
//   aim_packet_to_nuc.aim_y = 4.56;
//   aim_packet_to_nuc.aim_z = 7.89;
//   aim_packet_to_nuc.game_time = 0.0f;
//   aim_packet_to_nuc.timestamp = 0.0f;
 }

// void VPC_SendTarget(void)
// {
//   if(gimbal_motor_pitch!=NULL)
//   {
//     gimbal_motor_pitch->measure.rad = aim_packet_from_nuc.pitch;
//     //yaw轴目标值不知道设定哪个，暂时不写
//   }

// }