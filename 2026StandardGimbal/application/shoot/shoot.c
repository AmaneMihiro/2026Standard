/**
******************************************************************************
* @file    shoot.c
* @brief
* @author
******************************************************************************
* Copyright (c) 2023 Team
* All rights reserved.
******************************************************************************
*/

#include <string.h>
#include <stdlib.h>

#include "shoot.h"
#include "gimbal.h"
#include "remote_control.h"
#include "rs485.h"
#include "shoot_motor.h"

#define BUllET_V 35   //弹速 m/s
#define SHOOT_V 5000  //摩擦轮转速 rpm

uint8_t shoot_mode = 0;
uint8_t shoot_mode_last = 0;

PID_t gimbal_test_angle_pid = {
    .kp = 12.0f,
    .ki = 0.0f,
    .kd = 480.0f,
    .output_limit = 10.0f,
    .integral_limit = 0.0f,
    .dead_band = 0.0f,
};

PID_t gimbal_test_speed_pid = {
    .kp = 400.0f,
    .ki = 400.0f,
    .kd = 0.0f,
    .output_limit = 25000.0f,
    .integral_limit = 25000.0f,
    .dead_band = 0.0f,
};

motor_init_config_t shoot_motor1_init = {
    .controller_param_init_config = {
        .angle_PID = &gimbal_test_angle_pid,
        .speed_PID = &gimbal_test_speed_pid,
        .current_PID = NULL,
        .torque_PID = NULL,

        .other_angle_feedback_ptr = NULL,
        .other_speed_feedback_ptr = NULL,

        .angle_feedforward_ptr = NULL,
        .speed_feedforward_ptr = NULL,
        .current_feedforward_ptr = NULL,
        .torque_feedforward_ptr = NULL,

        .pid_ref = 0.0f,
    },
    .controller_setting_init_config = {
        .outer_loop_type = ANGLE_LOOP,
        .close_loop_type = ANGLE_AND_SPEED_LOOP,

        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

        .angle_feedback_source = MOTOR_FEED,
        .speed_feedback_source = MOTOR_FEED,

        .feedforward_flag = FEEDFORWARD_NONE,
    },

    .motor_type = shoot,

    .can_init_config = {
        .can_handle = &hfdcan2,
        .tx_id = 0x01,
        .rx_id = 0x011,
    },
};

motor_init_config_t shoot_motor2_init = {
    .controller_param_init_config = {
        .angle_PID = &gimbal_test_angle_pid,
        .speed_PID = &gimbal_test_speed_pid,
        .current_PID = NULL,
        .torque_PID = NULL,

        .other_angle_feedback_ptr = NULL,
        .other_speed_feedback_ptr = NULL,

        .angle_feedforward_ptr = NULL,
        .speed_feedforward_ptr = NULL,
        .current_feedforward_ptr = NULL,
        .torque_feedforward_ptr = NULL,

        .pid_ref = 0.0f,
    },
    .controller_setting_init_config = {
        .outer_loop_type = ANGLE_LOOP,
        .close_loop_type = ANGLE_AND_SPEED_LOOP,

        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

        .angle_feedback_source = MOTOR_FEED,
        .speed_feedback_source = MOTOR_FEED,

        .feedforward_flag = FEEDFORWARD_NONE,
    },

    .motor_type = shoot,

    .can_init_config = {
        .can_handle = &hfdcan2,
        .tx_id = 0x02,
        .rx_id = 0x012,
    },
};

motor_init_config_t shoot_motor3_init = {
    .controller_param_init_config = {
        .angle_PID = &gimbal_test_angle_pid,
        .speed_PID = &gimbal_test_speed_pid,
        .current_PID = NULL,
        .torque_PID = NULL,

        .other_angle_feedback_ptr = NULL,
        .other_speed_feedback_ptr = NULL,

        .angle_feedforward_ptr = NULL,
        .speed_feedforward_ptr = NULL,
        .current_feedforward_ptr = NULL,
        .torque_feedforward_ptr = NULL,

        .pid_ref = 0.0f,
    },
    .controller_setting_init_config = {
        .outer_loop_type = ANGLE_LOOP,
        .close_loop_type = ANGLE_AND_SPEED_LOOP,

        .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,

        .angle_feedback_source = MOTOR_FEED,
        .speed_feedback_source = MOTOR_FEED,

        .feedforward_flag = FEEDFORWARD_NONE,
    },

    .motor_type = shoot,

    .can_init_config = {
        .can_handle = &hfdcan2,
        .tx_id = 0x03,
        .rx_id = 0x013,
    },
};

shoot_motor_instance_t *shoot_motor_1;
shoot_motor_instance_t *shoot_motor_2;
shoot_motor_instance_t *shoot_motor_3;
void Shoot_Init(void)
{
    shoot_motor_1 = Shoot_Motor_Init(&shoot_motor1_init);
    shoot_motor_2 = Shoot_Motor_Init(&shoot_motor2_init);
    shoot_motor_3 = Shoot_Motor_Init(&shoot_motor3_init);
}

void Shoot_Enable(void)
{
   Shoot_Motor_Enable(shoot_motor_1);
   Shoot_Motor_Enable(shoot_motor_2);
   Shoot_Motor_Enable(shoot_motor_3);
}

void Shoot_Stop(void)
{
    Shoot_Motor_Stop(shoot_motor_1);
    Shoot_Motor_Stop(shoot_motor_2);
    Shoot_Motor_Stop(shoot_motor_3);
}

void Shoot_SetAll(float speed)
{
    Shoot_Motor_SetTar(shoot_motor_1,speed);
    Shoot_Motor_SetTar(shoot_motor_2,speed);
    Shoot_Motor_SetTar(shoot_motor_3,-speed);
}

void Get_Shoot_Mode(void)
{
    if (switch_is_up(rc_data->rc.switch_right))
    {
        shoot_mode = SHOOT_MODE_FIRE;
    }
    else if (switch_is_mid(rc_data->rc.switch_right))
    {
        shoot_mode = SHOOT_MODE_READY;
    }
    else if (switch_is_down(rc_data->rc.switch_right))
    {
        shoot_mode = SHOOT_MODE_STOP;
    }
    else
    {
        shoot_mode = SHOOT_MODE_STOP;
    }
}

void Shoot_State_Machine(void)
{
    static uint16_t init_count = 0;
    if (init_count < 1000)
    {
        init_count++;
        shoot_mode = SHOOT_MODE_STOP;
    }
    if (gimbal_mode)
    {
        switch (shoot_mode)
        {
        case SHOOT_MODE_FIRE:
            uart2_tx_message.shoot_mode = 1;
            Shoot_Enable();
            Shoot_SetAll(SHOOT_V);
            break;

        case SHOOT_MODE_READY:
            uart2_tx_message.shoot_mode = 0;
            //Shoot_Stop();
            Shoot_Enable();
            Shoot_SetAll(SHOOT_V);
            break;

        case SHOOT_MODE_STOP:
            uart2_tx_message.shoot_mode = 0;
            Shoot_Stop();
            break;

        default:
            uart2_tx_message.shoot_mode = 0;
            Shoot_Stop();
            break;
        }
    }
    else
    {
        uart2_tx_message.shoot_mode = 0;
        Shoot_Stop();
    }
}