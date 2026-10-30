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
   ⚠ 从手 27_rm_arm/Modules/arm.c 里的 Zigbee_Crc8 必须与此完全一致。 */
static uint8_t zigbee_tx_seq = 0;

/* ===== 夹爪按键 PD14（2026-10-30）=====
 * CubeMX: PD14 = GPIO_Input / Pull-down / User Label = GRIP_BTN
 *         → main.h 生成 GRIP_BTN_Pin / GRIP_BTN_GPIO_Port
 * 电气：下拉输入，按键另一端接 3V3，按下读到 1。
 *       选下拉：按键线断了读 0，翻转模式下等于"保持原状"，不会误动作。
 * 消抖用 HAL_GetTick 计时而不是数调用次数 —— 改 ZIGBEE_TX_PERIOD_MS 时
 * 消抖窗口不会跟着一起变。 */
#define GRIP_DEBOUNCE_MS 30u

volatile uint8_t  g_grip_cmd = 0;          /* 发给从手：0=开 1=合 */
volatile uint8_t  g_grip_toggle_mode = 1;  /* 1=按一下翻转  0=按住才合 */
volatile uint32_t g_grip_press_cnt = 0;    /* 有效按下次数，排查按键接线用 */
volatile uint8_t  g_grip_raw = 0xFF;       /* PD14 原始电平 0/1，不经过消抖。
                                              排查时先看这个：它不动就是线/引脚的问题，
                                              它动而 press_cnt 不涨才是消抖逻辑的问题。 */

static void Gripper_Scan(void)
{
    static uint8_t  stable   = 0;   /* 已确认的按键电平 */
    static uint8_t  pending  = 0;   /* 正在计时的候选电平 */
    static uint32_t t_change = 0;

    uint8_t raw = (HAL_GPIO_ReadPin(GRIP_BTN_GPIO_Port, GRIP_BTN_Pin) == GPIO_PIN_SET) ? 1u : 0u;
    g_grip_raw = raw;                    /* 原始电平，Watch 里直接看这个 */

    if (raw != pending)
    {
        pending  = raw;
        t_change = HAL_GetTick();
        return;
    }
    if (raw == stable) return;
    if ((HAL_GetTick() - t_change) < GRIP_DEBOUNCE_MS) return;

    stable = raw;
    if (stable) g_grip_press_cnt++;

    if (g_grip_toggle_mode) { if (stable) g_grip_cmd = (uint8_t)!g_grip_cmd; }
    else                    g_grip_cmd = stable;
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

/* 发送主手自己的 6 个关节角给从手（18 字节，布局见文件顶部）。
   本板是主手（自定义控制器），发的是 g_arm.q —— 即电机实测、已扣 zero_offset、
   已按 dir 定向后的关节角，不是收到的外部数据。 */

void Zigbee_SendJoints(void)
{
    Gripper_Scan();                  /* 消抖窗口见 GRIP_DEBOUNCE_MS */

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
    zigbee_tx_buf[idx++] = g_grip_cmd;                                   /* buf[14] 夹爪 0=开 1=合 */
    zigbee_tx_buf[idx++] = zigbee_tx_seq++;                              /* buf[15] 序号：丢包可数 */
    zigbee_tx_buf[idx++] = Zigbee_Crc8((const uint8_t *)zigbee_tx_buf, idx); /* buf[16] 校验 [0..15] */

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

