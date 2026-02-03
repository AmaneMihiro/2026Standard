/**
******************************************************************************
* @file    gimbal_task.c
* @brief
* @author
******************************************************************************
* Copyright (c) 2023 Team
* All rights reserved.
******************************************************************************
*/

#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"

#include "gimbal_task.h"
#include "gimbal.h"

#include "message_center.h"

#include "DJI_motor.h"
#include "DM_motor.h"
#include "rs485.h"
#include "remote_control.h"

#include "VPC.h"
#include "Serial.h"
#include "shoot.h"
#include "shoot_motor.h"

#include "lpf.h"
#include "bsp_dwt.h"
#define GIMBAL_TASK_PERIOD 1 // ms

osThreadId_t gimbal_task_handel;

static publisher_t *gimbal_publisher;
static subscriber_t *gimbal_subscriber;

static void Gimbal_Task(void *argument);

void Gimbal_Task_Init(void)
{
    LowPass_Filter1p_Init(&pitch_lpf_filter, 0.8f);
    LowPass_Filter1p_Init(&yaw_lpf_filter, 0.8f);
    if (g_xSemVPC == NULL)
    {
        g_xSemVPC = xSemaphoreCreateBinary();
    }
    const osThreadAttr_t attr = {
        .name = "Gimbal_Task",
        .stack_size = 128 * 8,
        .priority = (osPriority_t)osPriorityRealtime4,
    };
    gimbal_task_handel = osThreadNew(Gimbal_Task, NULL, &attr);

    gimbal_publisher = Publisher_Register("gimbal_transmit_feed", sizeof(gimbal_behaviour_t));
    gimbal_subscriber = Subscriber_Register("gimbal_receive_cmd", sizeof(gimbal_cmd_t));
}

uint32_t gimbal_task_diff;

// float TTT = 0;
// float AAA = 0;
// float gimbal_t_ms = 0;
// float gimbal_delta_t = 0;
// float gimbal_freq = 0;

static void Gimbal_Task(void *argument)
{

    static uint8_t shoot_cnt = 0;
    HAL_UART_Receive_IT(&huart2, &uart2_current_byte, 1);
    uint32_t time = osKernelGetTickCount();

    for (;;)
    {
        //uint32_t Last_time = DWT->CYCCNT;
        Remote_Deadzone_Control();
        Get_Gimbal_Mode();
        Gimbal_State_Machine();
        // TTT = target_angle_pitch;
        // AAA = gimbal_motor_pitch->measure.rad;
        Chassis_Control();
        uart2_online_check();

        VPC_UpdatePackets();
        //NV_Pack_And_Send_Data_ROS2(&nv_aim_packet_to_nuc); // 导航数据包发送
        VS_Pack_And_Send_Data_ROS2(&vs_aim_packet_to_nuc); // 新视觉数据包发送
        // Pack_And_Send_Data_ROS2(&aim_packet_to_nuc);    //旧视觉数据包发送
        gimbal_task_diff = osKernelGetTickCount() - time;
        time = osKernelGetTickCount();
        osDelayUntil(time + GIMBAL_TASK_PERIOD);

        // gimbal_t_ms = DWT_GetTimeline_ms();
        // gimbal_delta_t = DWT_GetDeltaT64(&Last_time);
        // gimbal_freq = 1 / gimbal_delta_t;
    }
}