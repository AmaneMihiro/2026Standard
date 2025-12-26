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
#include "usbd_cdc_if.h"

uint8_t frame_buf[1024];

void VPC_Init(void)
{
  NV_Send_Packet_Init(&nv_aim_packet_to_nuc);
  VS_Send_Packet_Init(&vs_aim_packet_to_nuc);
  VS_Receive_Packet_Init(&vs_aim_packet_from_nuc);
  // Send_Packet_Init(&aim_packet_to_nuc);
}

void VPC_Receive(void) // 并非接收函数，这里的receive指的是接收下位机的数据存入发送数组准备传给上位机
{
  /* Serial already copied validated frame into aim_packet_from_nuc in UnPack_Data_ROS2 */
  /* Copy relevant fields into outgoing packet so we reply with updated data */

  // /*导航传输数据区*/
  nv_aim_packet_to_nuc.yaw = INS.Yaw;
  nv_aim_packet_to_nuc.pitch = INS.Pitch;

  // /*视觉传输数据区*/
  vs_aim_packet_to_nuc.frame_header.sof = 0xA6; // 帧头赋值
  vs_aim_packet_to_nuc.frame_header.crc8 = 1;
  vs_aim_packet_to_nuc.output_data.config = 1.0f;
  vs_aim_packet_to_nuc.output_data.target_pose[0] = 1.0f;
  vs_aim_packet_to_nuc.output_data.target_pose[1] = 1.0f;
  vs_aim_packet_to_nuc.output_data.target_pose[2] = 0.0f;
  vs_aim_packet_to_nuc.output_data.curr_yaw = INS.Yaw;
  vs_aim_packet_to_nuc.output_data.curr_pitch = INS.Pitch;
  vs_aim_packet_to_nuc.output_data.enemy_color = 0;
  vs_aim_packet_to_nuc.output_data.shoot_config = 0;

  // vs_aim_packet_to_nuc.sof = 0xA6; // 帧头赋值
  // vs_aim_packet_to_nuc.crc8 = 0;
  // vs_aim_packet_to_nuc.config = 0.0f;
  // vs_aim_packet_to_nuc.target_pose0 = 1.1f;
  // vs_aim_packet_to_nuc.target_pose1 = 2.2f;
  // vs_aim_packet_to_nuc.target_pose2 = 3.3f;
  // vs_aim_packet_to_nuc.curr_yaw = INS.Yaw;
  // vs_aim_packet_to_nuc.curr_pitch = INS.Pitch;
  // vs_aim_packet_to_nuc.enemy_color = 0;
  // vs_aim_packet_to_nuc.shoot_config = 0;
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

void Choose_VPC_Type(void)
{
  uint16_t frame_len = sizeof(cdc_rx_cache);

  /* 拷贝完整帧 */
  memcpy(frame_buf, cdc_rx_cache, frame_len);

  // /* 解析 */
  // if (frame_buf[0] == 0xA5)
  //   UnPack_Data_ROS2(frame_buf, &aim_packet_from_nuc, frame_len);
  // else
  //   VS_UnPack_Data_ROS2(frame_buf, &vs_aim_packet_from_nuc, frame_len);

  /*根据帧头来判断接收到的是哪个数据包*/
  if (frame_buf[0] == 0xA5)
  {
    /*导航部分*/
    // uint32_t copyLen = (*Len > sizeof(buf_receive_from_nuc)) ? sizeof(buf_receive_from_nuc) : *Len;
    // memcpy(buf_receive_from_nuc, Buf, copyLen);
    memcpy(buf_receive_from_nuc, frame_buf, sizeof(nv_receive_packet_t));

    /* Call UnPack which performs CRC check and will notify VPC (uses FromISR when appropriate) */
    UnPack_Data_ROS2(buf_receive_from_nuc, &aim_packet_from_nuc, sizeof(nv_receive_packet_t));
  }
  /*视觉部分*/
  else if (frame_buf[0] == 0xA6)
  {
    /* Copy received bytes into application buffer (size of receive_packet_t) */
    // uint32_t copyLen = (*Len > sizeof(vs_buf_receive_from_nuc)) ? sizeof(vs_buf_receive_from_nuc) : *Len;
    // memcpy(vs_buf_receive_from_nuc, Buf, copyLen);
     memcpy(vs_buf_receive_from_nuc, frame_buf, sizeof(vs_receive_packet_t));
    /* Call UnPack which performs CRC check and will notify VPC (uses FromISR when appropriate) */
     VS_UnPack_Data_ROS2(vs_buf_receive_from_nuc, &vs_aim_packet_from_nuc, sizeof(vs_receive_packet_t));
  }
  /* 移除已处理数据 */
  // memmove(cdc_rx_cache, cdc_rx_cache + frame_len, cdc_rx_len - frame_len);
  // cdc_rx_len -= frame_len;
}