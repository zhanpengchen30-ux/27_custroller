#include "arm_control.h"
#include "arm_state.h"
#include "gravity_model.h"
#include "joint_mapping.h"
#include "limit_filter.h"
#include "safety.h"
#include <math.h>

// Watch 实时调方向
volatile float g_grav_dir_j2 = 1.0f;
volatile float g_grav_dir_j3 = -1.0f;
volatile float g_grav_dir_j5 = 1.0f;
volatile float g_grav_dir_j4 = 1.0f;
// 比例
volatile float g_scale_j2 = 0.94f;
volatile float g_scale_j3 = 0.90f;
volatile float g_scale_j4 = 0.30f;
volatile float g_drag_kd  = 0.08f;
// J5 补偿开关
volatile uint8_t g_j5_comp_enable = 0;

// 力矩限幅
static const float TAU_LIMITS[6] = {1.5f, 3.8f, 1.8f, 1.5f, 1.0f, 0.8f};

void Arm_Calibrate_Zero(void)
{
    // 刷新反馈
    Joint_UpdateState(g_arm.motors, g_arm.q, g_arm.v);

    for (int i = 0; i < 6; i++) {
        // 存重力零位

        g_gravity_zero[i] += g_arm.q[i];
        g_arm.motors[i].zero_offset = g_arm.motors[i].pos_raw;
    }
}

void Arm_Control_Init(void)
{
    Arm_State_Init();
}

void Arm_Control_Loop(void)
{
    Joint_UpdateState(g_arm.motors, g_arm.q, g_arm.v);

    // 重力零位
    float q_gravity[6];
    for (int i = 0; i < 6; i++) {
        q_gravity[i] = g_arm.q[i] + g_gravity_zero[i];
    }
    Arm_CalcGravityTorque(q_gravity, g_arm.tau_gravity);

    // 安全检查
    // Safety_CheckArm 返回 false 时仍是“低刚度重力托举”，并非硬急停。
    uint8_t safety_ok = Safety_CheckArm(&g_arm);

    for (int i = 0; i < 6; i++) {
        g_arm.motors[i].p_des = g_arm.q[i];
        g_arm.motors[i].v_des = 0.0f;
        g_arm.motors[i].kp = 0.0f;  // 零刚度

        if (!g_arm.is_online[i] || !safety_ok) {
            g_arm.motors[i].kd = 0.0f;
            g_arm.motors[i].t_ff = 0.0f;
            g_arm.tau_final[i] = 0.0f;
            DM_J4310_SendMIT(&g_arm.motors[i]);
            continue;
        }

        g_arm.motors[i].kd = safety_ok ? g_drag_kd : 0.05f;

        // 方向+比例
        float comp_tor = g_arm.tau_gravity[i];
        if (i == 1) {
            comp_tor *= g_grav_dir_j2 * g_scale_j2;
        } else if (i == 2) {
            comp_tor *= g_grav_dir_j3 * g_scale_j3;
        } else if (i == 3) {
            comp_tor *= g_grav_dir_j4 * g_scale_j4;
        } else if (i == 4) {
            comp_tor = g_j5_comp_enable ? comp_tor * g_grav_dir_j5 : 0.0f;
        } else {
            comp_tor = 0.0f;
        }

        float clamped = Math_Clamp(comp_tor, -TAU_LIMITS[i], TAU_LIMITS[i]);
        g_arm.tau_final[i] = Math_Ramp(clamped, g_arm.tau_final[i], 0.03f);
        g_arm.motors[i].t_ff = g_arm.tau_final[i];

        DM_J4310_SendMIT(&g_arm.motors[i]);
    }
}
