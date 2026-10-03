#include "arm_control.h"
#include "arm_state.h"
#include "gravity_model.h"
#include "joint_mapping.h"
#include "limit_filter.h"
#include "safety.h"
#include <math.h>

volatile float g_scale_j2 = 1.0f; 
volatile float g_scale_j3 = 1.0f; 

// ★★★ 调参秘籍：1 = 开启纯零力拖动模式(随便推)；0 = 锁定保持模式(稳稳悬停) ★★★
volatile uint8_t g_drag_mode = 0; 

// ---- 拖/锁状态机参数（Live Watch 可实时改 g_drag_*，改后立即生效）----
volatile float g_drag_v_unlock = 0.15f;  // 进入拖动阈值：全臂最大滤波速度 (rad/s)
volatile float g_drag_kp_ramp  = 1.5f;   // kp 斜坡步长/拍（1 拍≈3ms），决定锁止/解锁柔顺度
#define V_LOCK 0.04f            // 退出拖动速度阈值（迟滞；全臂低于此且持续 LOCK_HOLD_CYCLES 拍才锁）
#define LOCK_HOLD_CYCLES 15     // 全臂停稳 ~45ms 才锁死，防松手瞬间误锁

// 锁止刚度/阻尼（J2/J3 按 ζ≈0.7~1.0 估算，其余按防漂移；实机手感为准）
static const float KP_HOLD[6] = {10.f, 25.f, 20.f, 10.f, 10.f, 10.f};
static const float KD_HOLD[6] = {0.8f, 2.0f, 1.2f, 0.5f, 0.3f, 0.4f};

static float hold_pos[6] = {0};
static uint8_t is_initialized = 0;

void Arm_Calibrate_Zero(void) {
    for (int i = 0; i < 6; i++) {
        g_arm.motors[i].zero_offset = g_arm.motors[i].pos_raw;
        hold_pos[i] = 0.0f;
    }
}

void Arm_Control_Init(void) {
    Arm_State_Init();
    is_initialized = 0;
}

void Arm_Control_Loop(void) {
    Joint_UpdateState(g_arm.motors, g_arm.q, g_arm.v);
    Arm_CalcGravityTorque(g_arm.q, g_arm.tau_gravity);

    // ★ 急停/掉线真正生效：在线电机保留重力托举但放弃锁止（可被人手扶住），掉线电机完全松手 ★
    if (!Safety_CheckArm(&g_arm)) {
        for (int i = 0; i < 6; i++) {
            g_arm.motors[i].p_des = g_arm.q[i];   // 目标=当前位置，防 p 跳变
            g_arm.motors[i].v_des = 0.0f;
            if (g_arm.is_online[i]) {
                g_arm.motors[i].kp = 0.0f;
                g_arm.motors[i].kd = 0.05f;
                g_arm.motors[i].t_ff = g_arm.tau_gravity[i];
            } else {
                g_arm.motors[i].kp = 0.0f;
                g_arm.motors[i].kd = 0.0f;
                g_arm.motors[i].t_ff = 0.0f;      // 掉线电机严禁继续驱动
            }
            DM_J4310_SendMIT(&g_arm.motors[i]);
        }
        return;
    }

    // 开机初始化当前位置为保持点
    if (!is_initialized && g_arm.mode == ARM_SYS_ZERO_FORCE) {
        for (int i = 0; i < 6; i++) {
            hold_pos[i] = g_arm.q[i];
        }
        is_initialized = 1;
    }

    const float TAU_LIMITS[6] = {1.5f, 3.8f, 1.8f, 1.5f, 1.0f, 0.8f};

    // ---- 拖/锁状态机：全臂速度低通 + 迟滞 + 全局解锁 + kp 斜坡 ----
    static float   v_filt[6] = {0};
    static uint8_t drag_state = 0;
    static uint16_t lock_cnt  = 0;
    static float   kp_cur[6]  = {0};

    float v_drag = 0.0f;
    for (int i = 0; i < 6; i++) {
        v_filt[i] = Math_LowPass(g_arm.v[i], v_filt[i], 0.3f);
        if (fabsf(v_filt[i]) > v_drag) v_drag = fabsf(v_filt[i]);
    }

    if (g_drag_mode == 1 || v_drag > g_drag_v_unlock) {
        drag_state = 1;        // 任一关节被推 → 全臂解锁（拖动示教标准做法）
        lock_cnt = 0;
    } else if (drag_state == 1) {
        if (++lock_cnt >= LOCK_HOLD_CYCLES) drag_state = 0;  // 全臂停稳后才锁
    }

    for (int i = 0; i < 6; i++) {
        float comp_tor = g_arm.tau_gravity[i];

        if (i == 1)      comp_tor *= g_scale_j2;
        else if (i == 2) comp_tor *= g_scale_j3;

        float clamped = Math_Clamp(comp_tor, -TAU_LIMITS[i], TAU_LIMITS[i]);
        g_arm.tau_final[i] = Math_Ramp(clamped, g_arm.tau_final[i], 0.08f);

        if (drag_state) {
            hold_pos[i] = g_arm.q[i];                       // 目标点跟随手
            kp_cur[i]   = Math_Ramp(0.0f, kp_cur[i], g_drag_kp_ramp);  // 放软
        } else {
            kp_cur[i]   = Math_Ramp(KP_HOLD[i], kp_cur[i], g_drag_kp_ramp); // 柔顺锁紧
        }

        g_arm.motors[i].kp    = kp_cur[i];
        g_arm.motors[i].kd    = drag_state ? 0.20f : KD_HOLD[i];
        g_arm.motors[i].p_des = hold_pos[i];                // 锁止目标
        g_arm.motors[i].v_des = 0.0f;
        g_arm.motors[i].t_ff  = g_arm.tau_final[i];         // 重力前馈托底

        DM_J4310_SendMIT(&g_arm.motors[i]);
    }
}
