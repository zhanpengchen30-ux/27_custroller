#include "bsp_can2.h"
#include "dm_j4310.h"
#include "arm_state.h"

extern CAN_HandleTypeDef hcan2; // 只使用 hcan2

void BSP_CAN2_Init(void) {
    // STM32F4 的 CAN 过滤器物理寄存器挂在 CAN1 总线上，必须开启 CAN1 和 CAN2 时钟
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_CAN2_CLK_ENABLE();

    CAN_FilterTypeDef can_filter;
    can_filter.FilterBank = 14;                  // CAN2 使用 14~27 号过滤器
    can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter.FilterIdHigh = 0x0000;
    can_filter.FilterIdLow = 0x0000;
    can_filter.FilterMaskIdHigh = 0x0000;
    can_filter.FilterMaskIdLow = 0x0000;
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter.FilterActivation = ENABLE;
    can_filter.SlaveStartFilterBank = 14;         // 14 往后分配给 CAN2

    HAL_CAN_ConfigFilter(&hcan2, &can_filter);   // 改回 &hcan2
    HAL_CAN_Start(&hcan2);                       // 启动 CAN2
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING); // 开启接收中断
}

uint8_t BSP_CAN2_SendMsg(uint32_t std_id, uint8_t *data, uint8_t len) {
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;

    // 核心自愈逻辑：如果 CAN2 处于错误锁死状态，强制复位恢复为 LISTENING
    if (hcan2.State == HAL_CAN_STATE_ERROR) {
        hcan2.State = HAL_CAN_STATE_LISTENING;
        hcan2.ErrorCode = HAL_CAN_ERROR_NONE;
    }

    tx_header.StdId = std_id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = len;

    // 等待空闲邮箱（最多等 1ms）
    uint32_t timeout = 500;
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) == 0 && timeout--) {}

    if (HAL_CAN_AddTxMessage(&hcan2, &tx_header, data, &tx_mailbox) != HAL_OK) {
        // 如果邮箱被占死，强行撤销挂死请求，保证下一帧畅通
        HAL_CAN_AbortTxRequest(&hcan2, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
        return 0;
    }
    return 1;
}

// CAN2 接收中断回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan->Instance == CAN2) {
        CAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
            // 通过 Master ID (0x17 ~ 0x22) 匹配 6 个电机
            DM_J4310_DecodeByMasterId(g_arm.motors, 6, rx_header.StdId, rx_data);
        }
    }
}
