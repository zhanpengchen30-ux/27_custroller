#ifndef GRAVITY_MODEL_H
#define GRAVITY_MODEL_H

extern volatile float g_grav_m2;
extern volatile float g_grav_lc2;
extern volatile float g_grav_l2;
extern volatile float g_grav_m3;
extern volatile float g_grav_lc3;
extern volatile float g_grav_l3;
extern volatile float g_grav_m_end;
extern volatile float g_grav_lc_end;

// 与电机编码器 zero_offset 独立的重力模型角度偏置（rad）。
// 需要按机械臂已知参考姿态校准，不能仅凭“按下调零”自动推断物理零度。
extern volatile float g_gravity_zero[6];

void Arm_CalcGravityTorque(float q[6], float tau_g[6]);

#endif
