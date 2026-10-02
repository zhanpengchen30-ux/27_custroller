#ifndef DM_J4310_H
#define DM_J4310_H

#include "main.h"

#define DM_P_MIN   -12.56637f
#define DM_P_MAX    12.56637f
#define DM_V_MIN   -30.0f
#define DM_V_MAX    30.0f
#define DM_KP_MIN   0.0f
#define DM_KP_MAX   500.0f
#define DM_KD_MIN   0.0f
#define DM_KD_MAX   5.0f
#define DM_T_MIN   -10.0f
#define DM_T_MAX    10.0f

typedef struct {
    uint16_t id;         // 发送控制 ID: 0x07, 0x08, 0x09, 0x10, 0x11, 0x12
    uint16_t master_id;  // 接收回传 ID: 0x17, 0x18, 0x19, 0x20, 0x21, 0x22
    uint8_t  id_low4;    // ID 低4位
    
    // 反馈状态
    float pos_raw;       // 编码器原始绝对弧度
    float pos;           // 扣除零点后的关节有效角度 (rad)
    float vel;           // 速度 (rad/s)
    float tor;           // 实际力矩 (Nm)
    float t_mos;         // MOS 温度
    float t_rotor;       // 转子温度
    uint32_t last_update_time; // 心跳计时器 (ms)

    // 零位与方向
    float zero_offset;   // 软件零点偏置
    int8_t dir;          // 旋转方向极性 (1 或 -1)

    // 控制目标 (MIT)
    float p_des;
    float v_des;
    float kp;
    float kd;
    float t_ff;          // 前馈力矩 (重力补偿输出)
} DM_J4310_t;

void DM_J4310_Init(DM_J4310_t *motor, uint16_t can_id, uint16_t master_id, int8_t dir);
void DM_J4310_Enable(uint16_t can_id);
void DM_J4310_Disable(uint16_t can_id);
void DM_J4310_SendMIT(DM_J4310_t *motor);
void DM_J4310_DecodeByMasterId(DM_J4310_t *motors, uint8_t motor_num, uint32_t std_id, uint8_t *rx_data);

#endif
