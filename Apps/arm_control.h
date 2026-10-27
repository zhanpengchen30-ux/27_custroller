#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

#include <stdint.h>

void Arm_Control_Init(void);
void Arm_Control_Loop(void);
void Arm_Calibrate_Zero(void);

/* ---- 调试捕捉（Watch 里看/写，用法见 arm_control.c 注释） ---- */

/* 补偿期间掉进安全降级的次数："突然一沉"若由它引起，这个数会涨 */
extern volatile uint16_t g_dbg_sag_cnt;

/* 姿态快照表：标定模式(g_calib_kp>0)下摆好姿态后写 g_dbg_pose_cmd=1 记一行，
   最多 8 行，写 g_dbg_pose_cnt=0 清空重记。记满后把表抄出来做最小二乘。 */
#define DBG_POSE_N 16
extern volatile uint8_t  g_dbg_pose_cmd;          /* 1=记录一行, 跑完变 2 */
extern volatile uint8_t  g_dbg_pose_cnt;          /* 已记录行数, 写 0 清空 */
extern volatile float    g_dbg_pose_th[DBG_POSE_N][3];  /* 仰角(度) thA/B/C */
extern volatile float    g_dbg_pose_c[DBG_POSE_N][3];   /* cos(thA/B/C) 拟合用 */
extern volatile float    g_dbg_pose_tm[DBG_POSE_N][3];  /* 模型力矩 [J2,J3,J5] */
extern volatile float    g_dbg_pose_te[DBG_POSE_N][3];  /* 实测-模型 [J2,J3,J5] */
extern volatile uint8_t  g_dbg_pose_qual[DBG_POSE_N];   /* bit0静止 bit1标定模式 */

/* 实时模型误差(每拍刷新)：标定模式下 J2/J3/J5 的 tor - tau_gravity。
   正=欠补(松手下沉) 负=过补(松手上顶)。拖动模式下无意义。 */
extern volatile float g_dbg_err_j2;
extern volatile float g_dbg_err_j3;
extern volatile float g_dbg_err_j5;

/* "电机应出力矩" = kp·(p_des−q) + kd·(−v) + t_ff（关节系，不依赖 tor 反馈）。
   标定模式停稳时 ≈ 电机真实出力 ≈ 真实重力力矩。tor 解码修好前，标定用它。 */
extern volatile float g_dbg_tau_cmd_j2;
extern volatile float g_dbg_tau_cmd_j3;
extern volatile float g_dbg_tau_cmd_j5;

/* 姿态快照表的第三组力矩：tc = tau_cmd（kp 推算）。拟合优先用这组。 */
extern volatile float g_dbg_pose_tc[DBG_POSE_N][3];

/* 记录时的 J4 角度（q[3]）—— 用来判断这一行有没有被 J4 耦合污染，模型要求它≈0 */
extern volatile float g_dbg_pose_q4[DBG_POSE_N];

/* 记录时电机直接反馈的力矩 motors[1/2/4].tor（比 tm+te 更不容易抄错） */
extern volatile float g_dbg_pose_tor[DBG_POSE_N][3];

#endif
