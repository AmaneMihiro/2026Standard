/*
 * @file		Serial.c/h
 * @brief		usb-CDC通信程序，用于上位机和stm32的通信，需配合上位机代码使用，现采用zy版本
 * @history
 * 版本			作者			编写日期
 * v1.0.0		许金帅		2023/4/1
 * v2.0.0		冯俊玮		2024/10/29
 * v3.0.0   miracle    2025/12/6  （将传统的一个发送包拆为两个包发送，一个给视觉，一个给导航）
 */
#ifndef __Serial_h__
#define __Serial_h__
#include "stm32h7xx_hal.h"
#include "stdio.h"
#include "string.h"
#include "stdint.h"
#include "stdbool.h"
// 下面是为和ROS2上位机沟通而定制的结构体(zy)

/*导航的发送结构体aim to nuc*/
typedef struct
{
  // uint8_t header; // 0x5A   帧头

  // float roll;
  // float pitch;
  // float yaw;

  // uint16_t timestamp; // 时间戳
  // uint16_t robot_hp;  // 血量
  // uint16_t game_time; // 比赛时间

  // uint16_t checksum;
  uint8_t header;           // 0x5A
  uint8_t detect_color : 1; // 0-red 1-blue
  uint8_t task_mode : 2;    // 0-auto 1-aim 2-buff
  bool reset_tracker : 1;
  uint8_t is_play : 1;
  uint8_t change_target : 1;
  uint8_t reserved : 2;
  float roll;
  float pitch;
  float yaw;
  float aim_x;
  float aim_y;
  float aim_z;
  
  uint16_t timestamp; // (ms) board time
  uint16_t robot_hp;  // 血量
  uint16_t game_time; // (s) game time [0, 450]
  uint16_t checksum;
} __attribute__((packed)) nv_send_packet_t;

/*导航的接收结构体aim from nuc*/
typedef struct
{
  uint8_t header; // 0xA5   帧头

  uint8_t state : 2;      // 0-untracking 1-tracking-aim 2-tracking-buff
  uint8_t id : 3;         // aim: 0-outpost 6-guard 7-base
  uint8_t armors_num : 3; // 2-balance 3-outpost 4-normal

  float yaw_diff;
  float pitch_diff;

  int fire_advice;

  float vx; // x轴速度指令
  float vy; // y轴速度指令

  uint16_t checksum;
} __attribute__((packed)) nv_receive_packet_t;

/*视觉的发送结构体aim to nuc*/
typedef struct
{
  uint8_t header;           // 0x5A
  uint8_t detect_color : 1; // 0-red 1-blue
  uint8_t task_mode : 2;    // 0-auto 1-aim 2-buff
  uint8_t reset_tracker : 1;
  uint8_t is_play : 1;
  uint8_t change_target : 1;
  uint8_t reserved : 2;
  float roll;
  float pitch;
  float yaw;
  float aim_x;
  float aim_y;
  float aim_z;
  uint16_t game_time; // (s) game time [0, 450]
  uint32_t timestamp; // (ms) board time
  uint16_t checksum;
} __attribute__((packed)) vs_send_packet_t;

/*视觉的接收结构体aim from nuc*/
typedef struct
{
  uint8_t header;         // 0xA5
  uint8_t state : 2;      // 0-untracking 1-tracking-aim 2-tracking-buff
  uint8_t id : 3;         // aim: 0-outpost 6-guard 7-base
  uint8_t armors_num : 3; // 2-balance 3-outpost 4-normal

  // o-auto
  float yaw;
  float pitch;

  float yaw_diff;
  float pitch_diff;
  int fire_advice;

  uint16_t checksum;
} __attribute__((packed)) vs_receive_packet_t;

typedef struct
{
  uint8_t header;           // 0x5A
  uint8_t detect_color : 1; // 0-red 1-blue
  uint8_t task_mode : 2;    // 0-auto 1-aim 2-buff
  uint8_t reset_tracker : 1;
  uint8_t is_play : 1;
  uint8_t change_target : 1;
  uint8_t reserved : 2;
  float roll;
  float pitch;
  float yaw;
  float aim_x;
  float aim_y;
  float aim_z;

  uint16_t game_time; // (s) game time [0, 450]
  uint32_t timestamp; // (ms) board time
  uint16_t checksum;

} __attribute__((packed)) send_packet_t;

typedef struct
{
  uint8_t header;         // 0xA5
  uint8_t state : 2;      // 0-untracking 1-tracking-aim 2-tracking-buff
  uint8_t id : 3;         // aim: 0-outpost 6-guard 7-base
  uint8_t armors_num : 3; // 2-balance 3-outpost 4-normal

  // o-auto
  float yaw;
  float pitch;

  float yaw_diff;
  
  float pitch_diff;
  int fire_advice;

  uint16_t checksum;
} __attribute__((packed)) receive_packet_t;

extern nv_receive_packet_t nv_aim_packet_from_nuc;
extern nv_send_packet_t nv_aim_packet_to_nuc;

extern vs_receive_packet_t vs_aim_packet_from_nuc;
extern vs_send_packet_t vs_aim_packet_to_nuc;

extern receive_packet_t aim_packet_from_nuc;
extern send_packet_t aim_packet_to_nuc;

extern uint8_t nv_buf_receive_from_nuc[sizeof(nv_receive_packet_t)]; // 具体应用在usb接收函数中（在CDC_if.c中）
extern uint8_t vs_buf_receive_from_nuc[sizeof(vs_receive_packet_t)]; // 具体应用在usb接收函数中（在CDC_if.c中）
extern uint8_t buf_receive_from_nuc[sizeof(receive_packet_t)];       // 具体应用在usb接收函数中（在CDC_if.c中）

// extern protocol_parser_t nv_parser;
// extern protocol_parser_t vs_parser;

// extern uint8_t com_rx_buffer[];
// extern uint16_t com_rx_write_index; // 写指针
// extern uint16_t com_rx_read_index;  // 读指针（由解析器移动）

// #define COM_RX_BUFFER_SIZE 1024 // 根据数据量调整，必须能容纳至少2个最大数据包

int CDC_SendFeed(uint8_t *Fed, uint16_t Len); // CDC发送反馈数据的函数
// ROS2发送代码
void NV_Pack_And_Send_Data_ROS2(nv_send_packet_t *send_packet);
void NV_UnPack_Data_ROS2(uint8_t *receive_buf, nv_receive_packet_t *receive_packet, uint16_t Len);
void NV_Send_Packet_Init(nv_send_packet_t *send_packet);

void VS_Pack_And_Send_Data_ROS2(vs_send_packet_t *send_packet);
void VS_UnPack_Data_ROS2(uint8_t *receive_buf, vs_receive_packet_t *receive_packet, uint16_t Len);
void VS_Send_Packet_Init(vs_send_packet_t *send_packet);

void Pack_And_Send_Data_ROS2(send_packet_t *send_packet);
void UnPack_Data_ROS2(uint8_t *receive_buf, receive_packet_t *receive_packet, uint16_t Len);
void Send_Packet_Init(send_packet_t *send_packet);

// void Analysis_VPC_Buffer(void);

// int CDC_Receive_ROS2(uint8_t *receive_buf, receive_packet_t *receive_packet, uint32_t Len);

#endif
