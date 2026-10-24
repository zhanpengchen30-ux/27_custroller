#ifndef BSP_CAN2_H
#define BSP_CAN2_H

#include "main.h"
#include "can.h"

extern volatile uint32_t can2_rx_frame_cnt;
extern volatile uint32_t can2_rx_arm_id_cnt;

void BSP_CAN2_Init(void);
uint8_t BSP_CAN2_SendMsg(uint32_t std_id, uint8_t *data, uint8_t len);

#endif
