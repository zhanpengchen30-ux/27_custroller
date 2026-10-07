#include "gravity_model.h"
#include <math.h>

#define G_CONST 9.80665f

// 这些质量/质心/连杆参数仍需按实物测量校准，当前值只是起始估计。
volatile float g_grav_m2     = 0.38f;
volatile float g_grav_lc2    = 0.09f;
volatile float g_grav_l2     = 0.16f;

volatile float g_grav_m3     = 0.65f;
volatile float g_grav_lc3    = 0.10f;
volatile float g_grav_l3     = 0.16f;

volatile float g_grav_m_end  = 0.45f;
volatile float g_grav_lc_end = 0.08f;

// 独立于编码器 zero_offset 的重力模型角度偏置。
// 初始值 0 不是自动完成了物理标定；必须根据已知参考姿态校准。
volatile float g_gravity_zero[6] = {0, 0, 0, 0, 0, 0};

// 角度组合符号（Watch 实时切）：J3 实测 th23=q2-q3（g_sign_q3=-1），J5 待实测。
volatile float g_sign_q3 = -1.0f;   // th23 = q2 + g_sign_q3 * q3
volatile float g_sign_q5 = 1.0f;    // th235 = th23 + g_sign_q5 * q5

void Arm_CalcGravityTorque(float q[6], float tau_g[6])
{
    const float th2   = q[1];
    const float th23  = q[1] + g_sign_q3 * q[2];
    const float th235 = q[1] + g_sign_q3 * q[2] + g_sign_q5 * q[4];

    // 使用正常的有符号 cosf。真实力矩方向由模型和关节坐标定义决定，必须低力矩验证。
    const float c2   = cosf(th2);
    const float c23  = cosf(th23);
    const float c235 = cosf(th235);

    tau_g[0] = 0.0f;

    tau_g[1] = g_grav_m2 * G_CONST * g_grav_lc2 * c2
             + g_grav_m3 * G_CONST * (g_grav_l2 * c2 + g_grav_lc3 * c23)
             + g_grav_m_end * G_CONST * (g_grav_l2 * c2 + g_grav_l3 * c23
                                         + g_grav_lc_end * c235);

    // 大臂在前半球（th2 在 -90°~+90°）时，重力一定向下拉 J2，
    // 模型不能因 th23 钝角 cos 变负而倒扣成负力矩（那会变成电机往下压臂）。
    if (c2 > 0.0f && tau_g[1] < 0.0f) {
        tau_g[1] = 0.0f;
    }

    tau_g[2] = g_grav_m3 * G_CONST * g_grav_lc3 * c23
             + g_grav_m_end * G_CONST * (g_grav_l3 * c23 + g_grav_lc_end * c235);

    tau_g[3] = 0.0f;
    tau_g[4] = g_grav_m_end * G_CONST * g_grav_lc_end * c235;
    tau_g[5] = 0.0f;
}
