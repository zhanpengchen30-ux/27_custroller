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

// 安全门限：重力补偿只在"负载为正"的区间生效（cos ≥ 0）
// cos<0 = 关节已转过死点（下垂/过顶），此时按公式补偿会反向推臂 → 正反馈"突然发力"甩臂
// 钳 0 后，死点外只保留阻尼+位置托底，不再主动推；水平/下方区间补偿完全不受影响
static float safe_cos(float rad) {
    float c = cosf(rad);
    return (c > 0.0f) ? c : 0.0f;
}

void Arm_CalcGravityTorque(float q[6], float tau_g[6]) {
    float th2   = q[1];                  // J2 俯仰角
    float th23  = q[1] + q[2];          // J3 绝对俯仰角
    float th235 = q[1] + q[2] + q[4];  // J5 腕部俯仰角

    float c2   = safe_cos(th2);
    float c23  = safe_cos(th23);
    float c235 = safe_cos(th235);

    tau_g[0] = 0.0f;

    // J2 大臂俯仰（承担全臂力矩；死点外相应项自动为 0）
    tau_g[1] = g_grav_m2 * G_CONST * g_grav_lc2 * c2
             + g_grav_m3 * G_CONST * (g_grav_l2 * c2 + g_grav_lc3 * c23)
             + g_grav_m_end * G_CONST * (g_grav_l2 * c2 + g_grav_l3 * c23 + g_grav_lc_end * c235);

    // J3 小臂俯仰（托起末端 3 个电机；死点外自动为 0）
    tau_g[2] = g_grav_m3 * G_CONST * g_grav_lc3 * c23
             + g_grav_m_end * G_CONST * (g_grav_l3 * c23 + g_grav_lc_end * c235);

    tau_g[3] = 0.0f;
    tau_g[4] = g_grav_m_end * G_CONST * g_grav_lc_end * c235;
    tau_g[5] = 0.0f;
}
