#ifndef BSP_CAN2_H
#define BSP_CAN2_H

#include "main.h"
#include "can.h"

void BSP_CAN2_Init(void);
uint8_t BSP_CAN2_SendMsg(uint32_t std_id, uint8_t *data, uint8_t len);

#endif
