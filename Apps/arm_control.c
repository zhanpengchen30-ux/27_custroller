#include "arm_control.h"
#include "arm_state.h"
#include "gravity_model.h"
#include "joint_mapping.h"
#include "limit_filter.h"
#include "safety.h"
#include <math.h>

// Keil Watch 可实时调整。必须在支撑机械臂的情况下，从低比例开始标定。
volatile float g_scale_j2 = 0.20f;
volatile float g_scale_j3 = 0.20f;
volatile float g_grav_dir = 1.0f;
volatile float g_drag_kd  = 0.05f;

// 每个关节的最终力矩限幅（N·m）。这是软件限幅，不代表机构绝对安全。
static const float TAU_LIMITS[6] = {1.5f, 3.8f, 1.8f, 1.5f, 1.0f, 0.8f};

void Arm_Calibrate_Zero(void)
{
    // 先刷新当前反馈，避免使用上一拍的 q。
    Joint_UpdateState(g_arm.motors, g_arm.q, g_arm.v);

    for (int i = 0; i < 6; i++) {
        // 保存调零前的重力模型角度。编码器调零后 q 会变成 0，
        // 因此把原角度转存到独立 gravity_zero，避免调零瞬间重力模型跳变。
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

    // 编码器相对角 q 与重力模型角度分离。
    float q_gravity[6];
    for (int i = 0; i < 6; i++) {
        q_gravity[i] = g_arm.q[i] + g_gravity_zero[i];
    }
    Arm_CalcGravityTorque(q_gravity, g_arm.tau_gravity);

    // 无论正常还是异常分支，都使用同一套比例、限幅、斜坡。
    // Safety_CheckArm 返回 false 时仍是“低刚度重力托举”，并非硬急停。
    uint8_t safety_ok = Safety_CheckArm(&g_arm);

    for (int i = 0; i < 6; i++) {
        g_arm.motors[i].p_des = g_arm.q[i];
        g_arm.motors[i].v_des = 0.0f;
        g_arm.motors[i].kp = 0.0f;  // 永久零刚度：不保留位置锁止分支

        if (!g_arm.is_online[i] && !safety_ok) {
            g_arm.motors[i].kd = 0.0f;
            g_arm.motors[i].t_ff = 0.0f;
            g_arm.tau_final[i] = 0.0f;
            DM_J4310_SendMIT(&g_arm.motors[i]);
            continue;
        }

        g_arm.motors[i].kd = safety_ok ? g_drag_kd : 0.05f;

        float comp_tor = g_arm.tau_gravity[i] * g_grav_dir;
        if (i == 1) {
            comp_tor *= g_scale_j2;
        } else if (i == 2) {
            comp_tor *= g_scale_j3;
        }

        float clamped = Math_Clamp(comp_tor, -TAU_LIMITS[i], TAU_LIMITS[i]);
        g_arm.tau_final[i] = Math_Ramp(clamped, g_arm.tau_final[i], 0.03f);
        g_arm.motors[i].t_ff = g_arm.tau_final[i];

        DM_J4310_SendMIT(&g_arm.motors[i]);
    }
}
