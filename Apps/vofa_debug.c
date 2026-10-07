#include "vofa_debug.h"
#include "usart.h"
#include "arm_state.h"
#include <stdio.h>

extern volatile float g_scale_j2;
extern volatile float g_scale_j3;

void VOFA_Init(void)
{
}

void VOFA_SendData(void)
{
    char buf[256];
    int len = snprintf(buf, sizeof(buf),
        "%.4f,%.4f,"
        "%.4f,%.4f,"
        "%.4f,%.4f,"
        "%.4f,%.4f,"
        "%.3f,%.3f\n",
        g_arm.q[1],
        g_arm.q[2],
        g_arm.tau_gravity[1],
        g_arm.tau_gravity[2],
        g_arm.tau_final[1],
        g_arm.tau_final[2],
        g_arm.v[1],
        g_arm.v[2],
        g_scale_j2,
        g_scale_j3);

    HAL_UART_Transmit(&huart7, (uint8_t *)buf, len, 10);
}