#ifndef ARM_STATE_H
#define ARM_STATE_H

#include "dm_j4310.h"

typedef enum {
    ARM_SYS_INIT = 0,
    ARM_SYS_CALIBRATING,
    ARM_SYS_ZERO_FORCE,
    ARM_SYS_EMERGENCY
} ArmSystemMode_e;

typedef struct {
    ArmSystemMode_e mode;
    DM_J4310_t motors[6];
    float q[6];           // 当前关节角
    float v[6];           // 当前关节角速度
    float tau_gravity[6]; // 重力前馈力矩
    float tau_final[6];   // 最终下发力矩
    uint8_t is_online[6]; // 在线标志
} ArmState_t;

extern ArmState_t g_arm;

void Arm_State_Init(void);

#endif
