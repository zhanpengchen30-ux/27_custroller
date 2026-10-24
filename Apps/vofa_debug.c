#include "vofa_debug.h"
#include "arm_state.h"
#include "gravity_model.h"
#include "zigbee.h"
#include "bsp_can2.h"
#include "custom_ctrl.h"
#include <stdio.h>
#include <math.h>

extern UART_HandleTypeDef huart3;

void VOFA_SendData(void)
{
    /*
     * 输出：主手反馈q1..q6、USART6控制器目标q1..q6、链路状态及最近15个USART6原始字节。
     */
    uint8_t online_mask = 0;
    for (int i = 0; i < 6; i++)
        if (g_arm.is_online[i]) online_mask |= (uint8_t)(1u << i);

    Custom_Ctrl_t ctrl_snapshot;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    for (int i = 0; i < CUSTOM_FRAME_ANGLES; i++)
        ctrl_snapshot.target_joint[i] = custom_ctrl.target_joint[i];
    ctrl_snapshot.active = custom_ctrl.active;
    ctrl_snapshot.last_update_time = custom_ctrl.last_update_time;
    ctrl_snapshot.online = custom_ctrl.online;
    __set_PRIMASK(primask);

    uint32_t now = HAL_GetTick();
    uint32_t ctrl_age_ms = ctrl_snapshot.online ? now - ctrl_snapshot.last_update_time : 0xFFFFFFFFu;
    /* 一整行最坏约 268 字符（12 个 %.3f + 24 个整数 + 15 个原始字节），
       留到 320 够用。别再加到 448——那是放在 CommTask 栈上的。 */
    char buf[320];
    int len = snprintf(buf, sizeof(buf),
        "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u,%u,%u,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
        g_arm.q[0], g_arm.q[1], g_arm.q[2],
        g_arm.q[3], g_arm.q[4], g_arm.q[5],
        ctrl_snapshot.target_joint[0], ctrl_snapshot.target_joint[1],
        ctrl_snapshot.target_joint[2], ctrl_snapshot.target_joint[3],
        ctrl_snapshot.target_joint[4], ctrl_snapshot.target_joint[5],
        (unsigned)g_arm.mode,
        (unsigned)online_mask,
        (unsigned)((ctrl_snapshot.online ? 1u : 0u) | (ctrl_snapshot.active ? 2u : 0u)),
        (unsigned long)zigbee_send_cnt,
        (unsigned long)zigbee_tx_ok_cnt,
        (unsigned long)zigbee_tx_err_cnt,
        (unsigned long)custom_ctrl_rx_byte_cnt,
        (unsigned long)custom_ctrl_valid_frame_cnt,
        (unsigned long)zigbee_frame_err_cnt,
        (unsigned long)zigbee_rx_err_cnt,
        (unsigned long)ctrl_age_ms,
        (unsigned long)can2_rx_frame_cnt,
        (unsigned long)can2_rx_arm_id_cnt,
        (unsigned)custom_ctrl_rx_raw[0],
        (unsigned)custom_ctrl_rx_raw[1],
        (unsigned)custom_ctrl_rx_raw[2],
        (unsigned)custom_ctrl_rx_raw[3],
        (unsigned)custom_ctrl_rx_raw[4],
        (unsigned)custom_ctrl_rx_raw[5],
        (unsigned)custom_ctrl_rx_raw[6],
        (unsigned)custom_ctrl_rx_raw[7],
        (unsigned)custom_ctrl_rx_raw[8],
        (unsigned)custom_ctrl_rx_raw[9],
        (unsigned)custom_ctrl_rx_raw[10],
        (unsigned)custom_ctrl_rx_raw[11],
        (unsigned)custom_ctrl_rx_raw[12],
        (unsigned)custom_ctrl_rx_raw[13],
        (unsigned)custom_ctrl_rx_raw[14]);

    if (len > 0 && len < (int)sizeof(buf))
        /* 一行约 268 字节 @115200 = 23.3ms，超时必须大于它，否则每帧都被截断 */
        HAL_UART_Transmit(&huart3, (uint8_t *)buf, (uint16_t)len, 30);
}
