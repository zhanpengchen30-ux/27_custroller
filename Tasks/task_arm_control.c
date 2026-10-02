#include "task_arm_control.h"
#include "arm_control.h"
#include "arm_state.h"
#include "bsp_can2.h"
#include "cmsis_os.h"

// 设为 1 触发一键水平校准
uint8_t g_arm_zero_cali_cmd = 0;

void Task_ArmControl(void const * argument) {
    // 1. 硬件初始化：必须显式启动 CAN2 及过滤器
    BSP_CAN2_Init();

    // 2. 机械臂控制与状态初始化
    Arm_Control_Init();

    // 3. 上电宽限期：给电机 1 秒时间上线与发送心跳，不触发急停
    osDelay(1000);

    // 4. 依次使能 6 个电机
    for (int i = 0; i < 6; i++) {
        DM_J4310_Enable(g_arm.motors[i].id);
        osDelay(10);
    }

    g_arm.mode = ARM_SYS_ZERO_FORCE;

    for (;;) {
        // 外部（Keil 变量/按键）请求校准零点
        if (g_arm_zero_cali_cmd == 1) {
            Arm_Calibrate_Zero();
            g_arm_zero_cali_cmd = 0;
        }

        // 执行控制解算与力矩下发
        Arm_Control_Loop();

        osDelay(3); // 刷新率约为 300Hz
    }
}
