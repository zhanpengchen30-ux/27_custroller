#ifndef GRAVITY_MODEL_H
#define GRAVITY_MODEL_H

// 重力模型参数（volatile，Keil Watch 可直接实时修改，无需重新烧录）
// 标定口诀：把手臂推到水平伸展姿态，调 g_grav_* 使 tau_final 恰好托住不沉不顶
extern volatile float g_grav_m2;      // J2 大臂连杆质量 (kg)
extern volatile float g_grav_lc2;     // J2 大臂质心距 (m)
extern volatile float g_grav_l2;      // J2 大臂长度 (m)
extern volatile float g_grav_m3;      // J3 小臂+J4/J5 电机群质量 (kg)
extern volatile float g_grav_lc3;     // J3 质心距 (m)
extern volatile float g_grav_l3;      // J3 长度 (m)
extern volatile float g_grav_m_end;   // J6+夹爪质量 (kg)
extern volatile float g_grav_lc_end;  // 末端质心距 (m)

void Arm_CalcGravityTorque(float q[6], float tau_g[6]);

#endif
