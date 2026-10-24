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

/* USART6 RX on PG9 reads the custom controller; TX on PG14 forwards its pose. */
void Zigbee_Init(void);

/* 发送一帧数据到 Zigbee */
void Zigbee_Send(uint8_t *data, uint8_t len);

/* 转发自定义控制器的关节目标角度给从手 Zigbee */
void Zigbee_SendJoints(void);

/* 收到的数据回调（在中断里调用） */
void Zigbee_OnRx(uint8_t byte);

#endif
