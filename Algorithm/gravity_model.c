#include "gravity_model.h"
#include <math.h>

#define G_CONST 9.80665f

// 动力学真实物理质量分布（volatile：Keil Watch 实时改，不用反复烧录）
// 当前值为拍脑袋估计，实机务必按手感/称重修正
volatile float g_grav_m2     = 0.38f;   // J2 大臂连杆+中间电机 (约 380g)
volatile float g_grav_lc2    = 0.09f;
volatile float g_grav_l2     = 0.16f;

volatile float g_grav_m3     = 0.65f;   // J3 小臂连杆 + J4 + J5 电机群 (约 650g)
volatile float g_grav_lc3    = 0.10f;
volatile float g_grav_l3     = 0.16f;

volatile float g_grav_m_end  = 0.45f;   // J6 电机 + 夹爪 (约 450g)
volatile float g_grav_lc_end = 0.08f;

void Arm_CalcGravityTorque(float q[6], float tau_g[6]) {
    float th2   = q[1];                  // J2 俯仰角
    float th23  = q[1] + q[2];          // J3 绝对俯仰角
    float th235 = q[1] + q[2] + q[4];  // J5 腕部俯仰角

    tau_g[0] = 0.0f;

    // J2 大臂俯仰（承担全臂力矩）
    tau_g[1] = g_grav_m2 * G_CONST * g_grav_lc2 * cosf(th2)
             + g_grav_m3 * G_CONST * (g_grav_l2 * cosf(th2) + g_grav_lc3 * cosf(th23))
             + g_grav_m_end * G_CONST * (g_grav_l2 * cosf(th2) + g_grav_l3 * cosf(th23) + g_grav_lc_end * cosf(th235));

    // J3 小臂俯仰（托起末端 3 个电机）
    tau_g[2] = g_grav_m3 * G_CONST * g_grav_lc3 * cosf(th23)
             + g_grav_m_end * G_CONST * (g_grav_l3 * cosf(th23) + g_grav_lc_end * cosf(th235));

    tau_g[3] = 0.0f;
    tau_g[4] = g_grav_m_end * G_CONST * g_grav_lc_end * cosf(th235);
    tau_g[5] = 0.0f;
}
