#include "safety.h"

#define TIMEOUT_THRESHOLD_MS 200   // 掉线检测阈值 (ms)
#define OVERTEMP_TRIP_C       85.0f // 过温触发阈值
#define OVERTEMP_RELEASE_C    80.0f // 过温释放阈值（5°C 迟滞，防 85°C 附近反复进出急停）
#define RECOVERY_GOOD_FRAMES  50    // 连续全在线 50 拍（约 150ms）才允许退出急停，防总线丢帧抽搐

uint8_t Safety_CheckArm(ArmState_t *arm) {
    uint32_t now = HAL_GetTick();
    static uint8_t  overtemp_latch = 0; // 过温闩锁
    static uint16_t good_frames    = 0; // 连续全在线计数

    // 1. 系统刚启动的前 2.5 秒内，处于硬件建链期，不触发任何急停
    //    达妙电机是"一问一答"机制：必须先发使能/控制帧才会有回包
    if (now < 2500) {
        return 1;
    }

    // 2. 过温迟滞：任一电机 >85°C 触发闩锁，全部 <80°C 才解除
    uint8_t hot = 0;
    for (int i = 0; i < 6; i++) {
        if (arm->motors[i].t_mos > OVERTEMP_TRIP_C || arm->motors[i].t_rotor > OVERTEMP_TRIP_C) {
            hot = 1;
            break;
        }
    }
    if (hot) {
        overtemp_latch = 1;
    } else {
        uint8_t cool = 1;
        for (int i = 0; i < 6; i++) {
            if (arm->motors[i].t_mos > OVERTEMP_RELEASE_C || arm->motors[i].t_rotor > OVERTEMP_RELEASE_C) {
                cool = 0;
                break;
            }
        }
        if (cool) {
            overtemp_latch = 0;
        }
    }
    if (overtemp_latch) {
        arm->mode = ARM_SYS_EMERGENCY;
        good_frames = 0;
        return 0;
    }

    // 3. 掉线检测：从未收到包，或超过 200ms 没收到包，判掉线
    uint8_t all_online = 1;
    for (int i = 0; i < 6; i++) {
        if (arm->motors[i].last_update_time == 0 ||
            (now - arm->motors[i].last_update_time) > TIMEOUT_THRESHOLD_MS) {
            arm->is_online[i] = 0;
            all_online = 0;
        } else {
            arm->is_online[i] = 1;
        }
    }

    // 4. 掉线恢复迟滞：连续 50 拍全在线才从急停切回控臂
    if (all_online) {
        if (good_frames < RECOVERY_GOOD_FRAMES) {
            good_frames++;
        }
        if (good_frames >= RECOVERY_GOOD_FRAMES) {
            if (arm->mode == ARM_SYS_EMERGENCY) {
                arm->mode = ARM_SYS_ZERO_FORCE; // 稳定后才复位控臂
            }
            return 1;
        }
        if (arm->mode == ARM_SYS_EMERGENCY) {
            return 0; // 处于急停等待期，暂不切回
        }
        return 1;
    } else {
        good_frames = 0;
        arm->mode = ARM_SYS_EMERGENCY;
        return 0;
    }
}
