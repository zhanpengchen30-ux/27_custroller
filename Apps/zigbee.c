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
volatile uint8_t zigbee_tx_buf[19] = {0};

/* 帧格式（18 字节）：AA | q0..q5 (12B) | active | grip | seq | crc8 | 55
   crc8 = CRC-8/ATM(poly 0x07)，覆盖 buf[0..15]。
   ⚠ 从手 27_rm_arm/Modules/arm.c 里的 Zigbee_Crc8 与 ZIGBEE_FRAME_LEN 必须与此完全一致。 */
static uint8_t zigbee_tx_seq = 0;

/* ===== 夹爪按键（2026-10-29 新增）=====
 * 硬件：主手 PE0，内部下拉，按键另一端接 3V3 —— 按下读到 1 = 夹爪闭合。
 *       选下拉而不是上拉：断线时读 0（夹爪张开），比读 1（误闭合）安全。
 * 消抖：Zigbee_SendJoints 每 10ms 调一次，连续 3 次一致才认（30ms）。
 * 引脚：CubeMX 里把 PE0 命名为 GRIP_BTN、模式 GPIO_Input、下拉。
 *       没配也能编译（下面的 fallback 宏兜底），只是永远读到 0 = 夹爪张开。 */
#ifndef GRIP_BTN_Pin
#define GRIP_BTN_Pin        GPIO_PIN_0
#define GRIP_BTN_GPIO_Port  GPIOE
#endif

volatile uint8_t g_grip_cmd = 0;      /* 0=开  1=合（给 Watch 看） */
static uint8_t s_grip_cnt = 0;
static uint8_t s_grip_raw_last = 0;

static void Gripper_Scan(void)
{
    uint8_t raw = (HAL_GPIO_ReadPin(GRIP_BTN_GPIO_Port, GRIP_BTN_Pin) == GPIO_PIN_SET) ? 1u : 0u;

    if (raw == s_grip_raw_last)
    {
        if (s_grip_cnt < 3u) s_grip_cnt++;
        if (s_grip_cnt >= 3u) g_grip_cmd = raw;   /* 稳定 30ms 才生效 */
    }
    else
    {
        s_grip_raw_last = raw;
        s_grip_cnt = 0;
    }
}

static uint8_t Zigbee_Crc8(const uint8_t *d, uint8_t n)
{
    uint8_t crc = 0x00u;
    for (uint8_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (uint8_t b = 0; b < 8u; b++)
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
    }
    return crc;
}
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

/* 发送主手自己的 6 个关节角给从手（17字节：AA + 6xint16角度 + active + seq + crc8 + 55）。
   本板是主手（自定义控制器），发的是 g_arm.q —— 即电机实测、已扣 zero_offset、
   已按 dir 定向后的关节角，不是收到的外部数据。 */

void Zigbee_SendJoints(void)
{
    Gripper_Scan();                  /* 10ms 一次，30ms 消抖 */

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
    zigbee_tx_buf[idx++] = g_grip_cmd;                                   /* 夹爪：0=开 1=合 */
    zigbee_tx_buf[idx++] = zigbee_tx_seq++;                              /* 序号：丢包可数 */
    zigbee_tx_buf[idx++] = Zigbee_Crc8((const uint8_t *)zigbee_tx_buf, idx); /* 校验 [0..15] */

    zigbee_tx_buf[idx++] = 0x55;  // 帧尾（共 18 字节）

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

