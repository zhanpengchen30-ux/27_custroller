#include "arm_control.h"
#include "arm_state.h"
#include "gravity_model.h"
#include "joint_mapping.h"
#include "limit_filter.h"
#include "safety.h"
#include <math.h>

// ---- 现场调参（Keil Watch 实时改，立即生效）----
volatile float g_scale_j2 = 0.5f;   // J2 重力补偿比例：0.5 起步，现场慢慢加，加到"不沉不顶"为止
volatile float g_scale_j3 = 0.5f;   // J3 同上
volatile float g_grav_dir = 1.0f;   // 重力补偿极性：1 或 -1。水平放手臂，若翻转后能托住说明原来反了
volatile float g_drag_kd  = 0.15f;  // 拖动阻尼：0.10~0.20 合适，超过 0.3 会有明显"刹车感"

// ★★★ 模式：1 = 零刚度重力拖动（默认，推荐）；0 = 位置锁定保持（备选：示教完需要锁住时用）★★★
volatile uint8_t g_drag_mode = 1;

// 锁定保持模式参数（仅 g_drag_mode=0 时生效；纯位置保持，无拖动检测、无自动锁止）
static const float KP_HOLD[6] = {10.f, 25.f, 20.f, 10.f, 10.f, 10.f};
static const float KD_HOLD[6] = {0.8f, 2.0f, 1.2f, 0.5f, 0.3f, 0.4f};

static float hold_pos[6] = {0};
static uint8_t is_initialized = 0;

void Arm_Calibrate_Zero(void) {
    for (int i = 0; i < 6; i++) {
        g_arm.motors[i].zero_offset = g_arm.motors[i].pos_raw;
        hold_pos[i] = g_arm.q[i];   // 锁定模式锁在校零姿态，而不是角度 0
    }
}

void Arm_Control_Init(void) {
    Arm_State_Init();
    is_initialized = 0;
}

void Arm_Control_Loop(void) {
    Joint_UpdateState(g_arm.motors, g_arm.q, g_arm.v);
    Arm_CalcGravityTorque(g_arm.q, g_arm.tau_gravity);

    // ★ 急停/掉线：在线电机保留重力托举但放弃闭环（可被人手扶住），掉线电机完全松手 ★
    if (!Safety_CheckArm(&g_arm)) {
        for (int i = 0; i < 6; i++) {
            g_arm.motors[i].p_des = g_arm.q[i];   // 目标=当前位置，防 p 跳变
            g_arm.motors[i].v_des = 0.0f;
            if (g_arm.is_online[i]) {
                g_arm.motors[i].kp = 0.0f;
                g_arm.motors[i].kd = 0.05f;
                // 急停也按当前 g_scale 补偿，避免满值"顶手"（原来漏乘 g_scale 导致急停时 J2/J3 突然满力）
                float comp = g_arm.tau_gravity[i] * g_grav_dir;
                if (i == 1)      comp *= g_scale_j2;
                else if (i == 2) comp *= g_scale_j3;
                g_arm.motors[i].t_ff = comp;
            } else {
                g_arm.motors[i].kp = 0.0f;
                g_arm.motors[i].kd = 0.0f;
                g_arm.motors[i].t_ff = 0.0f;      // 掉线电机严禁继续驱动
            }
            DM_J4310_SendMIT(&g_arm.motors[i]);
        }
        return;
    }

    // 开机记录保持点（仅锁定模式用）
    if (!is_initialized && g_arm.mode == ARM_SYS_ZERO_FORCE) {
        for (int i = 0; i < 6; i++) {
            hold_pos[i] = g_arm.q[i];
        }
        is_initialized = 1;
    }

    const float TAU_LIMITS[6] = {1.5f, 3.8f, 1.8f, 1.5f, 1.0f, 0.8f};

    for (int i = 0; i < 6; i++) {
        // ---- 重力前馈：极性 + J2/J3 比例 + 限幅 + 平滑 ----
        float comp_tor = g_arm.tau_gravity[i] * g_grav_dir;

        if (i == 1)      comp_tor *= g_scale_j2;
        else if (i == 2) comp_tor *= g_scale_j3;

        float clamped = Math_Clamp(comp_tor, -TAU_LIMITS[i], TAU_LIMITS[i]);
        g_arm.tau_final[i] = Math_Ramp(clamped, g_arm.tau_final[i], 0.03f);

        if (g_drag_mode == 1) {
            // ★ 零刚度重力拖动：Kp=0（无位置弹簧），Kd=小阻尼，Tff=重力托举
            //   手随便推 → 跟手走；松手 → 重力被抵消处停在原位，不弹回、不锁定
            g_arm.motors[i].kp    = 0.0f;
            g_arm.motors[i].kd    = g_drag_kd;
            g_arm.motors[i].p_des = g_arm.q[i];   // 仅防 MIT 目标跳变（Kp=0，无吸力）
            g_arm.motors[i].v_des = 0.0f;
        } else {
            // 位置锁定保持（备选）：锁在校零/开机姿态，不做拖动跟随
            g_arm.motors[i].kp    = KP_HOLD[i];
            g_arm.motors[i].kd    = KD_HOLD[i];
            g_arm.motors[i].p_des = hold_pos[i];
            g_arm.motors[i].v_des = 0.0f;
        }

        g_arm.motors[i].t_ff = g_arm.tau_final[i];   // 重力补偿托底
        DM_J4310_SendMIT(&g_arm.motors[i]);
    }
}
