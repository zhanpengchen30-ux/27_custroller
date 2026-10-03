#include "dm_j4310.h"
#include "bsp_can2.h"

static int float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    if (x < x_min) x = x_min;
    else if (x > x_max) x = x_max;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

static float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void DM_J4310_Init(DM_J4310_t *motor, uint16_t can_id, uint16_t master_id, int8_t dir) {
    motor->id = can_id;
    motor->master_id = master_id;
    motor->id_low4 = (uint8_t)(can_id & 0x0F);
    motor->dir = dir;
    motor->zero_offset = 0.0f;
    motor->pos_raw = 0.0f;
    motor->pos = 0.0f;
    motor->vel = 0.0f;
    motor->tor = 0.0f;
    motor->p_des = 0.0f;
    motor->v_des = 0.0f;
    motor->kp = 0.0f;
    motor->kd = 0.0f;
    motor->t_ff = 0.0f;
    motor->last_update_time = 0;
}

void DM_J4310_Enable(uint16_t can_id) {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    BSP_CAN2_SendMsg(can_id, data, 8);
}

void DM_J4310_Disable(uint16_t can_id) {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    BSP_CAN2_SendMsg(can_id, data, 8);
}

void DM_J4310_SendMIT(DM_J4310_t *motor) {
    float send_t_ff = motor->t_ff * (float)motor->dir;

    // ★★★ 核心修复：把关节逻辑目标角度逆变换为电机底层的 raw 编码器绝对弧度 ★★★
    float send_p_des = motor->p_des * (float)motor->dir + motor->zero_offset;

    uint16_t p_int  = float_to_uint(send_p_des, DM_P_MIN, DM_P_MAX, 16);
    uint16_t v_int  = float_to_uint(motor->v_des, DM_V_MIN, DM_V_MAX, 12);
    uint16_t kp_int = float_to_uint(motor->kp,    DM_KP_MIN, DM_KP_MAX, 12);
    uint16_t kd_int = float_to_uint(motor->kd,    DM_KD_MIN, DM_KD_MAX, 12);
    uint16_t t_int  = float_to_uint(send_t_ff,    DM_T_MIN, DM_T_MAX, 12);

    uint8_t tx_data[8];
    tx_data[0] = (p_int >> 8);
    tx_data[1] = (p_int & 0xFF);
    tx_data[2] = (v_int >> 4);
    tx_data[3] = ((v_int & 0x0F) << 4) | (kp_int >> 8);
    tx_data[4] = (kp_int & 0xFF);
    tx_data[5] = (kd_int >> 4);
    tx_data[6] = ((kd_int & 0x0F) << 4) | (t_int >> 8);
    tx_data[7] = (t_int & 0xFF);

    BSP_CAN2_SendMsg(motor->id, tx_data, 8);
}

// 采用独特的 Master ID (0x17~0x22) 解码，彻底避免 D[0] 位截断
void DM_J4310_DecodeByMasterId(DM_J4310_t *motors, uint8_t motor_num, uint32_t std_id, uint8_t *rx_data) {
    for (int i = 0; i < motor_num; i++) {
        if (motors[i].master_id == std_id) {
            uint16_t p_raw = (rx_data[1] << 8) | rx_data[2];
            uint16_t v_raw = (rx_data[3] << 4) | (rx_data[4] >> 4);
            uint16_t t_raw = ((rx_data[4] & 0x0F) << 8) | rx_data[5];

            motors[i].pos_raw = uint_to_float(p_raw, DM_P_MIN, DM_P_MAX, 16);
            motors[i].pos     = (motors[i].pos_raw - motors[i].zero_offset) * (float)motors[i].dir;
            motors[i].vel     = uint_to_float(v_raw, DM_V_MIN, DM_V_MAX, 12) * (float)motors[i].dir;
            motors[i].tor     = uint_to_float(t_raw, DM_T_MIN, DM_T_MAX, 12) * (float)motors[i].dir;
            motors[i].t_mos   = (float)rx_data[6];
            motors[i].t_rotor = (float)rx_data[7];
            motors[i].last_update_time = HAL_GetTick();
            break;
        }
    }
}
