#ifndef SERIAL_TEST_UI_INTERFACE_H
#define SERIAL_TEST_UI_INTERFACE_H

#include "main.h"

#include "stm32h7xx_hal.h"
#include "ui_types.h"
#include "usart.h"
#include "bsp_usart.h"

typedef struct
{
    uint8_t revive_confirm : 1; // bit 0：哨兵机器人是否确认复活(0表示哨兵机器人确认不复活)
    uint16_t exch_revive;       // 哨兵机器人是否确认兑换立即复活(0表示哨兵机器人确认不兑换)
    uint32_t exch_bullet;       // bit 2-12：哨兵将要兑换的发弹量值，开局为0，修改此值后，哨兵在补血点即可兑换允许发弹量,此值的变化需要单调递增
    uint16_t exch_bullet_cnt;   // bit 13-16：哨兵远程兑换发弹量的请求次数，开局为0, 此值的变化需要单调递增且每次仅能增加1
    uint32_t exch_blood_cnt;    // bit 17-20：哨兵远程兑换血量的请求次数，开局为0，修改此值即可请求远程兑换血量,此值的变化需要单调递增且每次仅能增加1
    uint16_t posture;           // bit 21-22：哨兵修改当前姿态指令，1为进攻姿态，2为防御姿态，3为移动姿态，默认为3；修改此值即可改变哨兵姿态
    uint16_t buff_confirm;      // bit 23：哨兵机器人是否确认使能量机关进入正在激活状态，1为确认。默认为0
} __attribute__((packed)) sentry_msg_t;

extern USART_t *referee_usart_instance;
extern int ui_self_id;
extern uint8_t seq;

extern sentry_msg_t sentry_msg; // 哨兵指令结构体实例
#define SEND_MESSAGE(message, len) USART_Send(referee_usart_instance, message, len, USART_TRANSFER_DMA);
// #define SEND_MESSAGE(message, len)  HAL_UART_Transmit_DMA(&huart1, message, len);

// 由于裁判系统接收限制，你可能需要使用队列或延时来控制发送频率，此时需要自己建立一个新的发送函数替换掉HAL_UART_Transmit_DMA
unsigned char calc_crc8(unsigned char *pchMessage, unsigned int dwLength);
uint16_t calc_crc16(uint8_t *pchMessage, uint32_t dwLength);
void ui_proc_1_frame(ui_1_frame_t *msg);
void ui_proc_2_frame(ui_2_frame_t *msg);
void ui_proc_5_frame(ui_5_frame_t *msg);
void ui_proc_7_frame(ui_7_frame_t *msg);
void ui_proc_string_frame(ui_string_frame_t *msg);
void autonomous_sentinel_decision(sentry_cmd_t *msg);
void autonomous_radar_decision(radar_cmd_t *msg);

#endif // SERIAL_TEST_UI_INTERFACE_H
