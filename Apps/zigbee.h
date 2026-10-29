#ifndef __ZIGBEE_H
#define __ZIGBEE_H

#include <stdint.h>

extern volatile uint32_t zigbee_send_cnt;
extern volatile uint32_t zigbee_tx_ok_cnt;
extern volatile uint32_t zigbee_tx_err_cnt;
extern volatile uint32_t zigbee_rx_err_cnt;
extern volatile uint32_t zigbee_frame_err_cnt;
extern volatile uint32_t custom_ctrl_rx_byte_cnt;
extern volatile uint32_t custom_ctrl_valid_frame_cnt;
extern volatile uint8_t custom_ctrl_rx_raw[15];
extern volatile uint32_t zb_tx_gap_max_ms;

/* 夹爪命令：0 = 开，1 = 合。由主手 PE0 的按键经 30ms 消抖后写入，
   随 Zigbee 帧的第 14 字节一起发出去。可在 Watch 里观察。 */
extern volatile uint8_t g_grip_cmd;

/* 发帧周期（ms）。10 = 100Hz（默认）；20 = 50Hz，可显著降低无线链路负载。
   注意：改这里要同步从手 27_rm_arm 的 Modules/arm.c 里 ARM_INTERP_TICKS
   —— 它把一帧的增量摊到 5 个 2ms tick（正好 10ms），改成 50Hz 时应同步改 10。 */
#define ZIGBEE_TX_PERIOD_MS   10

/* USART6 RX on PG9 reads the custom controller; TX on PG14 forwards its pose. */
void Zigbee_Init(void);

/* 发送一帧数据到 Zigbee —— 阻塞版（HAL_UART_Transmit，最长 20ms）。
   只用于初始化/调试，不要在控制任务里调用；控制任务走 Zigbee_SendJoints（中断发送）。 */
void Zigbee_Send(uint8_t *data, uint8_t len);

/* 转发自定义控制器的关节目标角度给从手 Zigbee */
void Zigbee_SendJoints(void);

/* 收到的数据回调（在中断里调用） */
void Zigbee_OnRx(uint8_t byte);

#endif
