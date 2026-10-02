#include "task_telemetry.h"
#include "arm_state.h"
#include "usart.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>

void Task_Telemetry(void const * argument) {
    char buf[96];
    for (;;) {
        // 20Hz 遥测，USART6 @ 115200
        // 直接 HAL_UART_Transmit 发送，不依赖 printf retarget（避免 semihosting 卡任务）
        int len = snprintf(buf, sizeof(buf),
            "J2:%.2f,T2:%.2f,J3:%.2f,T3:%.2f,MD:%d,ON:%d%d%d%d%d%d\r\n",
            g_arm.q[1], g_arm.tau_final[1], g_arm.q[2], g_arm.tau_final[2],
            (int)g_arm.mode,
            g_arm.is_online[0], g_arm.is_online[1], g_arm.is_online[2],
            g_arm.is_online[3], g_arm.is_online[4], g_arm.is_online[5]);
        if (len > 0) {
            HAL_UART_Transmit(&huart6, (uint8_t *)buf, (uint16_t)len, 100);
        }
        osDelay(50);
    }
}
