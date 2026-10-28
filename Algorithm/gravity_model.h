#ifndef GRAVITY_MODEL_H
#define GRAVITY_MODEL_H

#include <stdint.h>

/*
 * 六轴主手（27_custroller）重力前馈模型 —— 逐段合并版
 *
 * 与旧版的区别：
 *   旧版只有 3 个质量（m2 / m3 / m_end），且把末端钉死在 J3 末端，
 *   J5 弯曲后末端质心的位移体现不出来；J4 的重力项从来没被辨识过
 *   （采数据时 CSV 里根本没录 J4 角），只能靠 g_scale_j4 = 0.30 硬压。
 *   新版按实际刚体分段，模型自洽，不再需要任何方向乘子，
 *   也不该再出现 0.3 ~ 0.94 这种比例系数。
 *
 * 分段（实测 2026-10-24）：
 *   段A = J2 轴 → J3 轴，                              354 g / 150 mm
 *   段B = J3 轴 → J5 轴（含 J4 滚转，与连杆共线，合并） 811 g / 150 mm
 *   段C = J5 轴 → 末端（含 J6 滚转，与末端轴共线，合并） 445 g /  82 mm
 *   段1 = J1 轴 → J2 轴：不建模。J1 是竖直偏航轴不产生重力矩，
 *         且该段完全位于 J2 近端，对任何关节的重力矩都无贡献。
 */

/* 重力零位偏置（rad）：把编码器零位对齐到「臂水平伸直沿 +x」这个物理参考。
   由 Arm_Calibrate_Zero() 写入，和电机的 zero_offset 是两回事，别混用。
   注意 Arm_Control_Loop 已经在调用前把它加进 q 里了，模型内部不再重复加。 */
extern volatile float g_gravity_zero[6];

/* 整条臂的总符号开关：如果补偿力矩整体方向反了（越补越往下掉），改成 -1.0f。
   单个关节的符号由 gravity_model.c 里的 g_sign_q* 负责，这里只是最后一道总闸。 */
extern volatile float g_grav_sign;

/* J4 偏心重力补偿增益：0=关（默认），Watch 里试 +1.0 / -1.0 定方向 */
extern volatile float g_j4_gain;

/* 重力补偿总开关：
     0 = 只算不下发。tau_gravity 仍然每拍更新（可以照常观察），但 t_ff 恒为 0，
         臂退化成纯阻尼，可以自由拖动。调符号 / 找零点 / 量质心时必须置 0。
     1 = 正常补偿。 */
extern volatile uint8_t g_grav_enable;

/* 模型当前"认为"的三段仰角（单位：度）。
   拿来和肉眼看到的姿态对：段A=J2→J3，段B=J3→J5，段C=J5→末端。
   对不上就说明 g_sign_q* 或 g_gravity_zero 错了。 */
extern volatile float g_thA_deg;
extern volatile float g_thB_deg;
extern volatile float g_thC_deg;

/* 三段仰角的余弦（每拍刷新）。标定拟合用，见 gravity_model.c 的注释。 */
extern volatile float g_cosA;
extern volatile float g_cosB;
extern volatile float g_cosC;

void Arm_CalcGravityTorque(float q[6], float tau_g[6]);

/* 把当前姿态登记为"全段水平伸直、指向 +x"的基准。
   只改写 g_gravity_zero，不动电机的 zero_offset —— 两者解耦，别混。
   在 Watch 里把 g_grav_zero_cmd 写 1 触发，它会变成 2 表示已执行。 */
void Arm_GravityZeroCapture(float q[6]);

/* 标零命令，3 个状态：
     0 = 空闲
     1 = 请求（在 Watch 里写 1）
     2 = 已执行（看到它变成 2，就说明标零真的跑过了）
   要再标一次就再写 1，不要写 0。 */
extern volatile uint8_t g_grav_zero_cmd;

#endif
