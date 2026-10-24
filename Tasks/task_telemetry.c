#include "task_telemetry.h"
#include "arm_state.h"
#include "zigbee.h"
#include "vofa_debug.h"
#include "usart.h"
#include "cmsis_os.h"

void Task_Telemetry(void const * argument) {
    // USART6 在 PG9 接收自定义控制器，在 PG14 转发到从手 Zigbee。
    Zigbee_Init();

    for (;;) {
        // 2. 发送 VOFA 调试数据到 USART3 (20Hz)
        VOFA_SendData();

        // 3. Zigbee 关节数据由 Task_ArmControl 统一发送（10ms 一次）
        //    这里不再发，否则两个任务并发写 zigbee_tx_buf 会导致帧错乱
        // Zigbee_SendJoints();

        osDelay(50);
    }
}
