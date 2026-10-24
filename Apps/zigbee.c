#include "zigbee.h"
#include "usart.h"
#include "arm_state.h"
#include "custom_ctrl.h"

extern UART_HandleTypeDef huart6;

static uint8_t rx_buf[CUSTOM_FRAME_LEN];
static uint8_t rx_byte = 0;
static uint8_t rx_idx = 0;
static volatile uint8_t tx_busy = 0;
#define CUSTOM_CTRL_TIMEOUT_MS 200u
volatile uint32_t zigbee_send_cnt = 0;
volatile uint32_t zigbee_tx_ok_cnt = 0;
volatile uint32_t zigbee_tx_err_cnt = 0;
volatile uint32_t zigbee_rx_err_cnt = 0;
volatile uint32_t zigbee_frame_err_cnt = 0;
volatile uint32_t custom_ctrl_rx_byte_cnt = 0;
volatile uint32_t custom_ctrl_valid_frame_cnt = 0;
volatile uint8_t custom_ctrl_rx_raw[15] = {0};
volatile uint32_t zb_tx_gap_max_ms = 0;
volatile uint8_t zigbee_tx_buf[16] = {0};
static uint32_t last_tx_tick = 0;

void Zigbee_Init(void)
{
    /* USART6 全双工：PG14(TX) 把主手关节角发给从手；PG9(RX) 保留用于
       接有线备份控制器 / 调试回读，收到的数据存进 custom_ctrl，但不参与发送。 */
    Custom_Ctrl_Init();
    rx_idx = 0;
    last_tx_tick = HAL_GetTick();
    if (HAL_UART_Receive_IT(&huart6, &rx_byte, 1) != HAL_OK)
    {
        zigbee_rx_err_cnt++;
    }
}

void Zigbee_Send(uint8_t *data, uint8_t len)
{
    if (HAL_UART_Transmit(&huart6, data, len, 20) == HAL_OK)
        zigbee_tx_ok_cnt++;
    else
        zigbee_tx_err_cnt++;
}

/* 发送主手自己的 6 个关节角给从手（15字节：帧头 + 6xint16角度 + active + 帧尾）。
   本板是主手（自定义控制器），发的是 g_arm.q —— 即电机实测、已扣 zero_offset、
   已按 dir 定向后的关节角，不是收到的外部数据。 */

void Zigbee_SendJoints(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t gap = now - last_tx_tick;
    if (zigbee_send_cnt > 0 && gap > zb_tx_gap_max_ms)
        zb_tx_gap_max_ms = gap;
    last_tx_tick = now;
    zigbee_send_cnt++;

    if (tx_busy)
    {
        zigbee_tx_err_cnt++;
        return;
    }

    /* g_arm.q 由同一个任务里的 Arm_Control_Loop() 更新，和本函数串行执行，
       所以这里不需要关中断做快照。 */
    uint8_t active = (g_arm.mode == ARM_SYS_ZERO_FORCE) ? 1u : 0u;

    uint8_t idx = 0;

    zigbee_tx_buf[idx++] = 0xAA;  // 帧头

    for (int i = 0; i < 6; i++) {
        /* 编码器量程 ±12.57 rad，×1000 后 ±12570，int16 装得下。
           仍然钳一次：q 若为 NaN，float→int16 的转换是未定义行为。 */
        float q = g_arm.q[i];
        if (!(q == q))        q =   0.0f;   /* 挡住 NaN */
        else if (q >  30.0f)  q =  30.0f;
        else if (q < -30.0f)  q = -30.0f;

        int16_t angle = (int16_t)(q * 1000.0f);
        zigbee_tx_buf[idx++] = (uint8_t)(angle & 0xFF);
        zigbee_tx_buf[idx++] = (uint8_t)((angle >> 8) & 0xFF);
    }

    zigbee_tx_buf[idx++] = active;

    zigbee_tx_buf[idx++] = 0x55;  // 帧尾

    tx_busy = 1;
    if (HAL_UART_Transmit_IT(&huart6, (uint8_t *)zigbee_tx_buf, idx) != HAL_OK)
    {
        tx_busy = 0;
        zigbee_tx_err_cnt++;
    }
}

void Zigbee_OnRx(uint8_t byte)
{
    /* Fixed-length parser: seek 0xAA, then consume exactly one frame. This
       prevents stale bytes in a rolling window from refreshing online state. */
    if (rx_idx == 0)
    {
        if (byte == CUSTOM_FRAME_HEAD1) rx_buf[rx_idx++] = byte;
        return;
    }
    rx_buf[rx_idx++] = byte;
    if (rx_idx == CUSTOM_FRAME_LEN)
    {
        if (rx_buf[0] == CUSTOM_FRAME_HEAD1 &&
            rx_buf[CUSTOM_FRAME_LEN - 1] == CUSTOM_FRAME_TAIL)
        {
            Custom_Ctrl_Parse(rx_buf, rx_idx);
            custom_ctrl_valid_frame_cnt++;
            rx_idx = 0;
        }
        else
        {
            zigbee_frame_err_cnt++;
            if (rx_buf[CUSTOM_FRAME_LEN - 1] == CUSTOM_FRAME_HEAD1)
            {
                rx_buf[0] = CUSTOM_FRAME_HEAD1;
                rx_idx = 1;
            }
            else
            {
                rx_idx = 0;
            }
        }
    }
}

// RX interrupt callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6) {
        custom_ctrl_rx_byte_cnt++;
        for (uint8_t i = 0; i < 14; i++)
            custom_ctrl_rx_raw[i] = custom_ctrl_rx_raw[i + 1];
        custom_ctrl_rx_raw[14] = rx_byte;
        Zigbee_OnRx(rx_byte);
        if (HAL_UART_Receive_IT(&huart6, &rx_byte, 1) != HAL_OK)
            zigbee_rx_err_cnt++;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        zigbee_rx_err_cnt++;
        rx_idx = 0;
#if defined(USART_SR_ORE)
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
#endif
        (void)HAL_UART_AbortReceive(huart);
        if (HAL_UART_Receive_IT(&huart6, &rx_byte, 1) != HAL_OK)
            zigbee_rx_err_cnt++;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        tx_busy = 0;
        zigbee_tx_ok_cnt++;
    }
}

