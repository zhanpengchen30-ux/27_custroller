#include "task_arm_control.h"
#include "arm_control.h"
#include "arm_state.h"
#include "bsp_can2.h"
#include "cmsis_os.h"
#include "vofa_debug.h"

// 设为 1 触发一键水平校准
uint8_t g_arm_zero_cali_cmd = 0;

void Task_ArmControl(void const * argument) {
    BSP_CAN2_Init();
    Arm_Control_Init();
    VOFA_Init();
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

    for (;;) {
        if (g_arm_zero_cali_cmd == 1) {
            Arm_Calibrate_Zero();
            g_arm_zero_cali_cmd = 0;
        }

        Arm_Control_Loop();

        // VOFA 串口 100Hz 发送（33ms * 3 ≈ 100ms）
        static uint8_t vofa_div = 0;
        if (++vofa_div >= 3) {
            vofa_div = 0;
            VOFA_SendData();
        }

        osDelay(3);
    }
}
