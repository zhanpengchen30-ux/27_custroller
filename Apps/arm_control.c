#include "arm_control.h"
#include "arm_state.h"
#include "gravity_model.h"
#include "joint_mapping.h"
#include "limit_filter.h"
#include "safety.h"

// 标定零点：将当前水平拉直的角度记录为偏置
void Arm_Calibrate_Zero(void) {
    for (int i = 0; i < 6; i++) {
        g_arm.motors[i].zero_offset = g_arm.motors[i].pos_raw;
    }
}

// 各轴力矩限幅 (Nm)：
// J2 满伸水平时重力矩需求 ≈ 3.97Nm，一刀切 3.5Nm 会截断大臂力矩导致水平下沉，单独放开到 4.2Nm
// （J4310 配置力矩范围 ±10Nm，4.2Nm 静态悬停远低于峰值，安全）
static const float TAU_LIMIT[6] = {3.5f, 4.2f, 3.5f, 3.5f, 3.5f, 3.5f};

void Arm_Control_Init(void) {
    Arm_State_Init();
}

void Arm_Control_Loop(void) {
    // 1. 安全检查
    if (!Safety_CheckArm(&g_arm)) {
        // 急停模式：力矩清零
        for (int i = 0; i < 6; i++) {
            g_arm.motors[i].t_ff = 0.0f;
            g_arm.motors[i].kp = 0.0f;
            g_arm.motors[i].kd = 0.05f; // 轻微阻尼缓慢下垂
            DM_J4310_SendMIT(&g_arm.motors[i]);
        }
        return;
    }

    // 2. 更新关节物理角度与速度
    Joint_UpdateState(g_arm.motors, g_arm.q, g_arm.v);

    // 3. 计算重力补偿前馈
    Arm_CalcGravityTorque(g_arm.q, g_arm.tau_gravity);

    // 4. 滤波与力矩保护
    for (int i = 0; i < 6; i++) {
        // 各轴独立限幅（J2 放开到 4.2Nm 以支持满伸水平姿态）
        float target_tor = Math_Clamp(g_arm.tau_gravity[i], -TAU_LIMIT[i], TAU_LIMIT[i]);
        
        // 斜坡平滑，防止突变打手（每次循环最多变化 0.08Nm）
        g_arm.tau_final[i] = Math_Ramp(target_tor, g_arm.tau_final[i], 0.08f);

        // MIT 零力拖动配置
        g_arm.motors[i].p_des = 0.0f;
        g_arm.motors[i].v_des = 0.0f;
        g_arm.motors[i].kp = 0.0f;
        g_arm.motors[i].kd = 0.12f; // 虚拟阻尼，消除零力手感抖动
        g_arm.motors[i].t_ff = g_arm.tau_final[i];

        // 发送至 CAN2
        DM_J4310_SendMIT(&g_arm.motors[i]);
    }
}
