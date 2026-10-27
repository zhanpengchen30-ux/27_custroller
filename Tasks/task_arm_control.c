#include "task_arm_control.h"
#include "arm_control.h"
#include "arm_state.h"
#include "bsp_can2.h"
#include "cmsis_os.h"
#include "vofa_debug.h"
#include "zigbee.h"

// 设为 1 触发一键水平校准
// ⚠ 已停用：Arm_Calibrate_Zero() 会同时改电机 zero_offset 和累加 g_gravity_zero，
//   一旦按下，之前写死在 gravity_model.c 里的重力零位就全废了，还会让 q 瞬间跳变。
//   重力零位现在统一走 g_grav_zero_cmd（只改 g_gravity_zero，不动电机零点）。
//   留着这个变量只是为了 Watch 里还能看到它，写 1 不会有任何动作。
uint8_t g_arm_zero_cali_cmd = 0;

/* 启动阶段指示：定位"电机使能不了"到底卡在哪一步
     0 = 还没进任务
     1 = CAN2 初始化完
     2 = 2.5s 电机自检等待结束
     3 = 6 台使能帧已发完，进入主循环      ← 正常应该一直是 3
   配合 g_loop_cnt（控制循环累计次数）一起看：
     阶段 3 但 loop_cnt 不涨 → 卡在循环里（很难，一般是死等 CAN 邮箱）
     阶段停在 0/1/2       → 卡在启动序列                                        */
volatile uint8_t g_boot_stage = 0;

void Task_ArmControl(void const * argument) {
    BSP_CAN2_Init();
    Arm_Control_Init();
    g_arm.mode = ARM_SYS_INIT;
    g_boot_stage = 1;

    // 关键：给电机 2.5 秒的充分开机自检时间！
    // 观察电机红灯亮起并稳定后，单片机才发起第一次通信
    osDelay(2500);
    g_boot_stage = 2;

    // 依次使能 6 个电机（每台间隔 20ms，给总线充裕的建链时间）
    for (int i = 0; i < 6; i++) {
        DM_J4310_Enable(g_arm.motors[i].id);
        osDelay(20);
    }

    // 此时正常情况下，电机的红灯会瞬间刷刷刷全变成【绿灯】！
    g_arm.mode = ARM_SYS_ZERO_FORCE;
    g_boot_stage = 3;

   uint32_t zigbee_tick = HAL_GetTick();

   for (;;) {
        /* 一键校准已停用，见文件开头说明。
           想重标重力零位请用 g_grav_zero_cmd = 1（只改 g_gravity_zero）。 */
        if (g_arm_zero_cali_cmd == 1) {
            g_arm_zero_cali_cmd = 0;   /* 吃掉的只是命令，不做任何动作 */
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
