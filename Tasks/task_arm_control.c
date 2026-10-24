#include "task_arm_control.h"
#include "arm_control.h"
#include "arm_state.h"
#include "bsp_can2.h"
#include "cmsis_os.h"
#include "vofa_debug.h"
#include "zigbee.h"

// 设为 1 触发一键水平校准
uint8_t g_arm_zero_cali_cmd = 0;

void Task_ArmControl(void const * argument) {
    BSP_CAN2_Init();
    Arm_Control_Init();
    g_arm.mode = ARM_SYS_INIT;

    // 关键：给电机 2.5 秒的充分开机自检时间！
    // 观察电机红灯亮起并稳定后，单片机才发起第一次通信
    osDelay(2500);

    // 依次使能 6 个电机（每台间隔 20ms，给总线充裕的建链时间）
    for (int i = 0; i < 6; i++) {
        DM_J4310_Enable(g_arm.motors[i].id);
        osDelay(20);
    }

    // 此时正常情况下，电机的红灯会瞬间刷刷刷全变成【绿灯】！
    g_arm.mode = ARM_SYS_ZERO_FORCE;

   uint32_t zigbee_tick = HAL_GetTick();

   for (;;) {
        if (g_arm_zero_cali_cmd == 1) {
            Arm_Calibrate_Zero();
            g_arm_zero_cali_cmd = 0;
        }

        uint32_t loop_start = HAL_GetTick();
        Arm_Control_Loop();

        // 固定间隔发帧（周期见 zigbee.h 的 ZIGBEE_TX_PERIOD_MS，默认 10ms ≈ 100Hz）
        uint32_t send_now = HAL_GetTick();
        if ((int32_t)(send_now - zigbee_tick) >= ZIGBEE_TX_PERIOD_MS) {
            zigbee_tick += ((send_now - zigbee_tick) / ZIGBEE_TX_PERIOD_MS) * ZIGBEE_TX_PERIOD_MS;
            Zigbee_SendJoints();
        }

        uint32_t elapsed = HAL_GetTick() - loop_start;
        if (elapsed < 3) osDelay(3 - elapsed);
    }
}
