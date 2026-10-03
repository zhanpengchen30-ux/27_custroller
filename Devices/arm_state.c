#include "arm_state.h"

ArmState_t g_arm;

// 6 个电机的发送 ID 与 回传 Master ID
static const uint16_t TX_IDS[6] = {0x07, 0x08, 0x09, 0x10, 0x11, 0x12};
static const uint16_t RX_IDS[6] = {0x17, 0x18, 0x19, 0x20, 0x21, 0x22};
static const int8_t DIRS[6] = {1, -1, -1, 1, -1, 1};

void Arm_State_Init(void) {
    g_arm.mode = ARM_SYS_INIT;
    for (int i = 0; i < 6; i++) {
        DM_J4310_Init(&g_arm.motors[i], TX_IDS[i], RX_IDS[i], DIRS[i]);
        g_arm.is_online[i] = 0;
        g_arm.q[i] = 0.0f;
        g_arm.v[i] = 0.0f;
        g_arm.tau_gravity[i] = 0.0f;
        g_arm.tau_final[i] = 0.0f;
    }
}
