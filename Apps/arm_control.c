#include "arm_control.h"
#include "arm_state.h"
#include "gravity_model.h"
#include "joint_mapping.h"
#include "limit_filter.h"
#include "safety.h"
#include <math.h>

// Keil Watch 可实时调整。方向按实机标定：J2 实测正确(+)，J3 实测反了(-)，J5 暂默认(+)待测
volatile float g_grav_dir_j2 = 1.0f;
volatile float g_grav_dir_j3 = -1.0f;
volatile float g_grav_dir_j4 = 1.0f;
volatile float g_grav_dir_j5 = 1.0f;
// 默认 0.10：J2 模型约 3.3~3.5 N·m，0.10≈0.35 N·m 安全起步；方向对再 0.15→0.20 加
volatile float g_scale_j2 = 0.90f;
volatile float g_scale_j3 = 0.90f;
volatile float g_scale_j4 = 0.30f;
volatile float g_drag_kd  = 0.05f;
// J5 重力补偿总开关（Watch：1=开，0=关）。排查期默认 0：烧录即关 J5，先做解耦实验。
volatile uint8_t g_j5_comp_enable = 0;
// J3 固定力矩测试（Watch：0=正常重力补偿；非0=J3 直接输出此力矩，绕过模型）。
// 用途：g_scale_j3=0 时设 +0.10/-0.10，测电机物理推力方向。
volatile float g_test_j3_torque = 0.0f;

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

        // 逐关节方向（Watch 可单独翻）+ 比例；不动 DIRS 电机方向，避免反馈/编码器方向被打乱
        float comp_tor = g_arm.tau_gravity[i];
        if (i == 1) {
            comp_tor *= g_grav_dir_j2 * g_scale_j2;
        } else if (i == 2) {
            if (g_test_j3_torque != 0.0f) {
                comp_tor = g_test_j3_torque;  // 固定力矩测试，绕过重力模型
            } else {
                comp_tor *= g_grav_dir_j3 * g_scale_j3;
            }
        } else if (i == 3) {
            comp_tor *= g_grav_dir_j4 * g_scale_j4;
        } else if (i == 4) {
            comp_tor = g_j5_comp_enable ? comp_tor * g_grav_dir_j5 : 0.0f;
        } else {
            comp_tor = 0.0f;  // J1/J6 模型本就为 0，显式归零防残留
        }

        float clamped = Math_Clamp(comp_tor, -TAU_LIMITS[i], TAU_LIMITS[i]);
        g_arm.tau_final[i] = Math_Ramp(clamped, g_arm.tau_final[i], 0.03f);
        g_arm.motors[i].t_ff = g_arm.tau_final[i];

        DM_J4310_SendMIT(&g_arm.motors[i]);
    }
}
