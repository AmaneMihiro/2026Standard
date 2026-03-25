/*
 * @file		Keyboard_Control.c/h
 * @brief		键盘控制程序，用于通过键盘输入控制机器人
 * @history
 * 版本			作者			编写日期
 * v1.0.0		miracle		2026/3/18
 */

// application
#include "robot_frame_init.h"

// module
#include "Keyboard_Control.h"
#include "remote_control.h"

#include "stdbool.h"
#define PITCH_SENSITIVITY 0.0003f // 键盘输入转换为俯仰角的灵敏度调整参数
#define YAW_SENSITIVITY 0.001f	  // 键盘输入转换为底盘速度的灵敏度调整参数

static const float chassis_gear_speed[SPEED_GEAR_COUNT] = {0.5f, 2.0f, 3.0f}; // 不同档位（低，中，高）对应的底盘最大速度
Speed_Gear_e current_gear = GEAR_PRECISION;									  // 默认低速档

float keyboard_speed_x = 0.0f;		// 键盘输入的底盘X方向速度
float keyboard_speed_y = 0.0f;		// 键盘输入的底盘Y方向速度
float keyboard_target_pitch = 0.0f; // 键盘输入的目标俯仰角
float keyboard_target_yaw = 0.0f;	// 键盘输入的目标偏航角
uint8_t Control_Mode = 0;			// 0：遥控器控制 1：键盘控制

/*
 *@brief 更新控制模式
 */
void Update_Control_Mode()
{
	uint8_t current_mode_state = rc_data->key_count[KEY_PRESS][Key_B]; // 'b'键作为控制模式切换的触发键

	if (current_mode_state % 2 == 1)
	{
		Control_Mode = Mode_Keyboard; // 键盘控制模式
	}
	else
	{
		Control_Mode = Mode_Remote; // 遥控器控制模式
	}
}

void Update_Chassis_Gear()
{
	static uint8_t last_gear_state = 0;
	uint8_t current_gear_state = rc_data->key_count[KEY_PRESS][Key_Shift]; // shift键作为档位切换的触发键

	if (current_gear_state > last_gear_state) // 检测到shift键的上升沿
	{
		// 档位切换逻辑
		current_gear++;
		if (current_gear >= SPEED_GEAR_COUNT)
		{
			current_gear = GEAR_PRECISION; // 循环回到最低档
		}
	}
	last_gear_state = current_gear_state;
}

static float inline Get_Current_Chassis_Speed_X()
{
	return (rc_data->key[KEY_PRESS].w - rc_data->key[KEY_PRESS].s) * chassis_gear_speed[current_gear];
}

static float inline Get_Current_Chassis_Speed_Y()
{
	return (rc_data->key[KEY_PRESS].a - rc_data->key[KEY_PRESS].d) * chassis_gear_speed[current_gear];
}

static float inline Get_Current_Target_Pitch()
{
	return (float)rc_data->mouse.y * PITCH_SENSITIVITY; // 将鼠标竖直移动转换为pitch，乘以灵敏度调整参数
}

static float inline Get_Current_Target_Yaw()
{
	return (float)rc_data->mouse.x * YAW_SENSITIVITY; // 将鼠标水平移动转换为yaw，乘以灵敏度调整参数
}

/*
 *@brief 键盘控制主函数
 */
void Keyboard_Control(void)
{

	Update_Control_Mode();
	Update_Chassis_Gear();

	keyboard_speed_x = Get_Current_Chassis_Speed_X();
	keyboard_speed_y = Get_Current_Chassis_Speed_Y();

	keyboard_target_pitch = Get_Current_Target_Pitch();
	keyboard_target_yaw = Get_Current_Target_Yaw();
}