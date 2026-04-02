#ifndef __KEYBOARD_CONTROL_H__
#define __KEYBOARD_CONTROL_H__

#include "remote_control.h"

typedef enum
{
    GEAR_PRECISION,
    GEAR_NORMAL,
    GEAR_AGILE,
    SPEED_GEAR_COUNT,
}Speed_Gear_e;

#define Mode_Keyboard 1
#define Mode_Remote 0

extern uint8_t Control_Mode;
extern uint8_t Omega_flag;
extern uint8_t FRIC_flag;
extern float keyboard_speed_x;		// 键盘输入的底盘速度
extern float keyboard_speed_y;		// 键盘输入的底盘速度
extern float keyboard_target_pitch; // 键盘输入的目标俯仰角
extern float keyboard_target_yaw;	// 键盘输入的目标偏航角


extern Speed_Gear_e current_gear;
extern void Keyboard_Control(void);

#endif