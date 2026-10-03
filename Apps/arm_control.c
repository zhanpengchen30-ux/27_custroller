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
    Safety_CheckArm(&g_arm);

    Joint_UpdateState(g_arm.motors, g_arm.q, g_arm.v);

    // 开机初始化当前位置为保持点
    if (!is_initialized && g_arm.mode == ARM_SYS_ZERO_FORCE) {
        for (int i = 0; i < 6; i++) {
            hold_pos[i] = g_arm.q[i];
        }
        is_initialized = 1;
    }

    // 基础重力矩计算
    Arm_CalcGravityTorque(g_arm.q, g_arm.tau_gravity);

    const float TAU_LIMITS[6] = {1.5f, 2.6f, 1.8f, 1.5f, 1.0f, 0.8f};

    for (int i = 0; i < 6; i++) {
        float comp_tor = g_arm.tau_gravity[i];

        if (i == 1)      comp_tor *= g_scale_j2;
        else if (i == 2) comp_tor *= g_scale_j3;

        float clamped = Math_Clamp(comp_tor, -TAU_LIMITS[i], TAU_LIMITS[i]);
        g_arm.tau_final[i] = Math_Ramp(clamped, g_arm.tau_final[i], 0.08f);

        // ★★★ 核心升级：真·悬停自锁状态机（杜绝自动溜车撞限位）★★★
        
        // 判断手是否在快速推它（速度大于 0.12 rad/s，确认是人手在故意推，而不是微弱溜车）
        if (fabsf(g_arm.v[i]) > 0.12f || g_drag_mode == 1) {
            // 人在推：目标点跟随手走，刚度放软
            hold_pos[i] = g_arm.q[i];
            g_arm.motors[i].kp = 0.0f;  // 拖动时 0 刚度，极其轻巧
            g_arm.motors[i].kd = 0.20f;
        } else {
            // 人手松开 / 停止移动：目标点【绝对死锁冻结】！绝不跟随！
            // 强刚度瞬间接管，死死钉在原地
            if (i == 1) {
                g_arm.motors[1].kp = 25.0f; // J2 锁死刚度
                g_arm.motors[1].kd = 0.80f;
            } else if (i == 2) {
                g_arm.motors[2].kp = 20.0f; // J3 锁死刚度
                g_arm.motors[2].kd = 0.60f;
            } else {
                g_arm.motors[i].kp = 10.0f;
                g_arm.motors[i].kd = 0.30f;
            }
        }

        g_arm.motors[i].p_des = hold_pos[i]; // 锁止目标
        g_arm.motors[i].v_des = 0.0f;
        g_arm.motors[i].t_ff  = g_arm.tau_final[i]; // 重力前馈托底

        DM_J4310_SendMIT(&g_arm.motors[i]);
    }
}
