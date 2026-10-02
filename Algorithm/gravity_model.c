#include "gravity_model.h"
#include <math.h>

#define G_CONST 9.80665f

// 动力学等效质量分布（根据实测 4310 电机 300g 与 3D 打印件配置）
static const float M2     = 0.45f;   // J2 连杆总重 (kg)
static const float LC2    = 0.11f;   // J2 质心到轴心距 (m)
static const float L2     = 0.16f;   // J2-J3 轴距 (m)

static const float M3     = 0.75f;   // J3 连杆+中间电机群总重 (kg)
static const float LC3    = 0.10f;   // J3 质心距 (m)
static const float L3     = 0.16f;   // J3-J5 轴距 (m)

static const float M_END  = 0.40f;   // 末端夹爪及负载 (kg)
static const float LC_END = 0.08f;   // 末端质心距 (m)

/**
 * @brief 水平基准解析重力补偿
 * @param q 6个轴的绝对物理角度 (rad)，水平拉直时全部为 0
 * @param tau_g 输出补偿力矩 (Nm)
 */
void Arm_CalcGravityTorque(float q[6], float tau_g[6]) {
    float th2   = q[1];                  // J2 俯仰角
    float th23  = q[1] + q[2];          // J3 绝对俯仰角
    float th235 = q[1] + q[2] + q[4];  // J5 腕部俯仰角

    // J1: Yaw 轴，无重力分量
    tau_g[0] = 0.0f;

    // J2: 大臂俯仰（承担全臂重力矩）
    tau_g[1] = M2 * G_CONST * LC2 * cosf(th2)
             + M3 * G_CONST * (L2 * cosf(th2) + LC3 * cosf(th23))
             + M_END * G_CONST * (L2 * cosf(th2) + L3 * cosf(th23) + LC_END * cosf(th235));

    // J3: 小臂俯仰
    tau_g[2] = M3 * G_CONST * LC3 * cosf(th23)
             + M_END * G_CONST * (L3 * cosf(th23) + LC_END * cosf(th235));

    // J4: 腕翻滚 Roll 轴
    tau_g[3] = 0.0f;

    // J5: 腕俯仰 Pitch 轴
    tau_g[4] = M_END * G_CONST * LC_END * cosf(th235);

    // J6: 夹爪旋转 Roll 轴
    tau_g[5] = 0.0f;
}
