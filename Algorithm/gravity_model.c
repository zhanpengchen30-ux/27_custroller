#include "gravity_model.h"
#include <math.h>

#define G_CONST 9.80665f

// 动力学真实物理质量分布（M2 恢复正常，J3/末端覆盖 3 个电机重量）
static const float M2     = 0.38f;   // J2 大臂连杆+中间电机 (约 380g，之前误填成 0.95 导致浮力严重超载)
static const float LC2    = 0.09f;
static const float L2     = 0.16f;

static const float M3     = 0.65f;   // J3 小臂连杆 + J4 + J5 电机群 (约 650g)
static const float LC3    = 0.10f;
static const float L3     = 0.16f;

static const float M_END  = 0.45f;   // J6 电机 + 夹爪 (约 450g)
static const float LC_END = 0.08f;

void Arm_CalcGravityTorque(float q[6], float tau_g[6]) {
    float th2   = q[1];                  // J2 俯仰角
    float th23  = q[1] + q[2];          // J3 绝对俯仰角
    float th235 = q[1] + q[2] + q[4];  // J5 腕部俯仰角

    tau_g[0] = 0.0f;

    // J2 大臂俯仰（承担全臂力矩）
    tau_g[1] = M2 * G_CONST * LC2 * cosf(th2)
             + M3 * G_CONST * (L2 * cosf(th2) + LC3 * cosf(th23))
             + M_END * G_CONST * (L2 * cosf(th2) + L3 * cosf(th23) + LC_END * cosf(th235));

    // J3 小臂俯仰（托起末端 3 个电机）
    tau_g[2] = M3 * G_CONST * LC3 * cosf(th23)
             + M_END * G_CONST * (L3 * cosf(th23) + LC_END * cosf(th235));

    tau_g[3] = 0.0f;
    tau_g[4] = M_END * G_CONST * LC_END * cosf(th235);
    tau_g[5] = 0.0f;
}
