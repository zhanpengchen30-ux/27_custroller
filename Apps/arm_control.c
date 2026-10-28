#include "arm_control.h"
#include "arm_state.h"
#include "gravity_model.h"
#include "joint_mapping.h"
#include "limit_filter.h"
#include "safety.h"
#include <math.h>

// 方向乘子
//
// 这几个乘子不是"应急开关"，它们是模型坐标系和电机坐标系之间的换算，
// 必须和 gravity_model.c 里的 g_sign_q* 配对，推导如下：
//
//   J2：g_sign_q2 = +1  →  thA = +q2，q2 增大 = 大臂抬起
//        所以"正力矩 = 抬起"，和模型算出的方向一致 → g_grav_dir_j2 = +1
//
//   J3：g_sign_q3 = -1  →  thB = thA - q3，q3 增大 = 前臂下落
//        所以"正力矩 = 下落"，而模型算的是"托住所需的力矩"（方向朝上）
//        → 必须取反：g_grav_dir_j3 = -1        ★ 漏了这个就会出现
//          "平行时 J2 往上翘、J3 往下掉"——两个关节互相打架
//
//   J5：g_sign_q5 = +1  →  thC = thB + q5，q5 增大 = 末端抬起 → +1
//
// 旧工程（27_custroll）里也是 j2=+1 / j3=-1 / j5=+1，与此完全一致。
volatile float g_grav_dir_j2 = 1.0f;
volatile float g_grav_dir_j3 = -1.0f;    /* 必须 -1，见上面推导 */
volatile float g_grav_dir_j5 = 1.0f;
volatile float g_grav_dir_j4 = 1.0f;
/* 补偿比例（2026-10-28 实机最终整定）
 *
 * 起点来自干净数据（thA=48.69/thB=49.35/thC=-95.13, qual=7）：
 *     J2 实测/模型 = 0.931      J3 实测/模型 = 0.963
 * 但按 0.93/0.96 上机后：J2 在水平附近弹起（过补）、J3 仍下滑（欠补），
 * 说明两个关节的真实比例不同。实机逐步微调，最终定格 0.90 / 1.10。
 *
 * 这两个数相差 22%，不是整体缩放能解释的 —— 本质是「质量分配」的误差：
 * 如果反算 lc，`lcB` 要 0.091（略大于 L/2，合理），但 `lcA` 要 ≈0.002（等于
 * 质心贴在 J2 轴上，物理上不可能）。说明 K1 里 (mB+mC)·LA 这一项偏大。
 *
 * 根因是平面模型要求 K2、K3 在 J2/J3 之间「共享」，而实机要求不同 ——
 * 所以这两个比例是模型缺的那个自由度，不是临时补丁。
 * 想根治只能靠 URDF + RNEA（Pinocchio）重新出参数，见项目笔记。
 *
 * J5：力矩仅 0.18 N·m，与静摩擦同量级，标不出来，关闭。 */
volatile float g_scale_j2 = 0.90f;
volatile float g_scale_j3 = 1.10f;
volatile float g_scale_j4 = 1.0f;   /* J4 项模型里是 0，这个值暂时无效 */
volatile float g_scale_j5 = 0.0f;   /* 关掉 J5 补偿 */
// 拖动阻尼（kd）。
// kp=0 时没有位置刚度：力矩误差 τe 不会被位置环纠正，只会一直加速，直到阻尼平衡。
//     稳态角速度 ≈ τe / kd
//   太小（0.05）：竖直附近一点误差就是 200°/s，会"弹飞"
//   太大（0.40）：正常拖也像推在泥里，控不动
// 旧工程磨到最后用的是 0.08，这里沿用。若还能感到"弹飞"再往 0.12~0.15 加。
volatile float g_drag_kd  = 0.08f;

/* J4 单独阻尼（2026-10-27 加）
 *
 * J4 是绕前臂的滚转轴。它的重力力矩最大只有 g·mC·lcC = 0.18 N·m，
 * 比达妙 4310 自身的静摩擦（0.2~0.4 N·m）还小 —— 所以：
 *   · 喂重力补偿推不动它（被摩擦吃掉），实测 g_j4_gain=±1.0 均无效果；
 *   · 但末端偏载一超过摩擦，它就会自己转到最低点，即"总是掉"。
 * 结论：这不是补偿能解决的物理问题，配平才是根治。若暂不配平，
 * 就单独给 J4 加大阻尼，把"掉"变成"缓慢漂"，代价是手转它也变沉。
 *
 * 调法（Watch 实时改，不用烧）：
 *   0.5  → 缓慢漂（约 10°/s）  ← 实机选用这个
 *   1.5  → 很慢（约 4°/s），但手转也变沉
 *   3~5  → 几乎不动，手转明显发涩
 * 其他关节仍用 g_drag_kd，不受影响。 */
volatile float g_j4_kd = 0.5f;
// J5 补偿开关：与旧工程一致，默认关。J5 满量程仅 0.18 N·m，和静摩擦同量级，
// 开了也只是引入噪声。等 J2/J3 调稳、且换了低摩擦方案再考虑。
volatile uint8_t g_j5_comp_enable = 0;

/* 标定用刚度（kp）。
 *
 * 平时必须是 0（零刚度拖动）。把它临时设成 8~15，电机会"站住"在当前姿态，
 * 停稳之后它自己输出的力矩就等于该姿态下的重力力矩，可以直接读
 * `g_arm.motors[i].tor`（电机反馈的实测力矩）来核对模型算的 `tau_gravity[i]`。
 *
 * 这是唯一不靠手感、纯数字的验符号办法：
 *   tor 与 tau_gravity 同号同量级 → 模型对
 *   tor 与 tau_gravity 反号        → g_grav_sign 改成 -1
 *   同号但差很多                   → 段参数（lc / 质量）不对
 *
 * 用完记得改回 0，否则就不是拖动示教了。 */
volatile float g_calib_kp = 0.0f;

// 力矩限幅
static const float TAU_LIMITS[6] = {1.5f, 3.8f, 1.8f, 1.5f, 1.0f, 0.8f};

/* ==========================================================================
 * 诊断标志位 —— Watch 里直接看这几个，比描述手感靠谱得多
 * ========================================================================== */

/* 状态位（实时刷新，每拍重算） */
volatile uint16_t g_grav_stat = 0;

#define GRAV_STAT_ENABLE    (1u << 0)   /* 重力补偿已使能                     */
#define GRAV_STAT_CLAMPED   (1u << 1)   /* 有轴的力矩被 TAU_LIMITS 削平       */
#define GRAV_STAT_RAMPING   (1u << 2)   /* 有轴目标跳变 >0.1N·m（正被斜坡平滑）*/
#define GRAV_STAT_DEAD_A    (1u << 3)   /* 段A 越过死点（|thA| > 90°）        */
#define GRAV_STAT_DEAD_B    (1u << 4)   /* 段B 越过死点                       */
#define GRAV_STAT_DEAD_C    (1u << 5)   /* 段C 越过死点                       */
#define GRAV_STAT_FLY       (1u << 6)   /* 抓到过"弹飞"事件（闩锁，不清）      */
#define GRAV_STAT_SAG       (1u << 7)   /* 本拍正处于安全降级（补偿被切掉）     */

/* "弹飞"事件闩锁：角速度超过阈值就记一次，记下当时的关节/角度/速度。
   想看新的就把它清零（g_fly_cnt = 0）。                         */
volatile uint16_t g_fly_cnt   = 0;      /* 累计次数                     */
volatile uint8_t  g_fly_joint = 0xFF;   /* 出事关节 1=J2 2=J3 4=J5      */
volatile float    g_fly_deg   = 0.0f;   /* 出事时该段的仰角（度）        */
volatile float    g_fly_v     = 0.0f;   /* 出事时的角速度（rad/s，带符号）*/

/* 见过的最大角速度，写 0 清零 —— 用它判断"到底甩了多快" */
volatile float g_drag_vmax = 0.0f;

/* 掉线自愈累计次数（清错+重使能发了多少次）。一直在涨说明有电机反复掉线 */
volatile uint32_t g_heal_cnt = 0;

/* 控制循环累计次数（约每 3ms +1）。不涨 = 控制任务没在跑 */
volatile uint32_t g_loop_cnt = 0;

/* 判"弹飞"的角速度阈值：5 rad/s ≈ 286°/s，正常拖动远低于此 */
#define FLY_V_THRESHOLD   5.0f

/* "塌一下"事件计数：重力补偿开着的时候突然掉进安全降级（掉线/过温），
   电机会瞬间失去全部支撑（t_ff=0, kd=0.05），150ms 后才恢复、再花几百 ms
   爬升回补偿力矩 —— 拖动手感上就是"突然一沉然后一顶"。
   如果这个数在涨，难受的根源是通信/安全逻辑，不是模型，先别调参数。 */
volatile uint16_t g_dbg_sag_cnt = 0;

/* ================== 模型误差定量捕捉（配合 g_calib_kp 使用） ==================
 *
 * 原理：g_calib_kp 设 10 左右、g_grav_enable=0 时，电机靠位置环自己"站"在
 * 姿态上。停稳后电机实际输出的力矩 tor 就是该姿态下【真实】的重力力矩，
 * 而 tau_gravity 是【模型】算的。两者相减就是模型误差 —— 不靠手感，纯数字。
 *
 * 用法（一个姿态一行，最多 16 行）：
 *   1. g_grav_enable=0, g_calib_kp=10
 *   2. 手扶臂摆到一个姿态，松手等 1~2 秒（Watch 里 v 都接近 0）
 *   3. g_dbg_pose_cmd 写 1，看到变 2，该姿态已记录
 *   4. 换姿态重复。静摩擦 ~0.5-1 N·m 不可忽略：同一姿态从上方逼近和从
 *      下方逼近各记一行，拟合时成对平均可对消摩擦。
 *   5. 把表抄出来做最小二乘，反推每个段的 lc
 *   重新记就写 g_dbg_pose_cnt = 0 清空。
 * ========================================================================== */
/* 注意：DBG_POSE_N 只在 arm_control.h 里定义，这里不要再 #define 一遍。
   两处各写一个值会导致 .h 的 extern 声明和 .c 的定义尺寸不一致，编不过。 */

/* 记录命令：0=空闲 1=请求 2=已执行（和 g_grav_zero_cmd 同款粘滞写法） */
volatile uint8_t g_dbg_pose_cmd = 0;
/* 已记录行数（写 0 清空整表重记） */
volatile uint8_t g_dbg_pose_cnt = 0;

/* 三段仰角（度）——人眼核对姿态用 */
volatile float g_dbg_pose_th[DBG_POSE_N][3];
/* cos(thA/B/C) ——拟合自变量 */
volatile float g_dbg_pose_c[DBG_POSE_N][3];
/* 模型力矩 [J2, J3, J5] */
volatile float g_dbg_pose_tm[DBG_POSE_N][3];
/* 误差 = 电机实测 tor - 模型 tau_gravity，[J2, J3, J5] */
volatile float g_dbg_pose_te[DBG_POSE_N][3];
/* 该行质量：bit0=三关节都静止(|v|<0.05)  bit1=当时 g_calib_kp>0 */
volatile uint8_t g_dbg_pose_qual[DBG_POSE_N];

/* --------------------------------------------------------------------------
 * 行读取器（只为绕开 Keil 对二维数组的显示问题）
 *
 * Keil Watch 展开二维数组时经常读到旧值 / 不刷新，实测不可信；
 * 但单个表达式 g_dbg_pose_th[0][0] 读出来是对的。
 *
 * 所以这里加一个"翻页"机制：在 Watch 里改 g_dbg_rd_idx（0~15），
 * 下面 9 个标量每拍都会同步成那一行的值，直接读标量就行，不用改表达式。
 * -------------------------------------------------------------------------- */
volatile uint8_t g_dbg_rd_idx = 0;      /* 想看第几行，改这个 */
volatile uint8_t g_dbg_rd_qual = 0;     /* 该行的 qual */
volatile float   g_dbg_rd_thA = 0.0f;   /* 该行仰角（度） */
volatile float   g_dbg_rd_thB = 0.0f;
volatile float   g_dbg_rd_thC = 0.0f;
volatile float   g_dbg_rd_tm2 = 0.0f;   /* 该行模型力矩 */
volatile float   g_dbg_rd_tm3 = 0.0f;
volatile float   g_dbg_rd_tm5 = 0.0f;
volatile float   g_dbg_rd_te2 = 0.0f;   /* 该行误差 = 实测 − 模型 */
volatile float   g_dbg_rd_te3 = 0.0f;
volatile float   g_dbg_rd_te5 = 0.0f;
volatile float   g_dbg_rd_q4   = 0.0f;   /* 该行记录时的 J4 角度，应接近 0 */
volatile float   g_dbg_rd_tor2 = 0.0f;   /* 该行电机实测力矩（直接读，最可靠） */
volatile float   g_dbg_rd_tor3 = 0.0f;
volatile float   g_dbg_rd_tor5 = 0.0f;
volatile float   g_dbg_rd_tc2 = 0.0f;   /* 该行 kp 推算的电机应出力矩（不依赖 tor） */
volatile float   g_dbg_rd_tc3 = 0.0f;
volatile float   g_dbg_rd_tc5 = 0.0f;

/* 实时模型误差（每拍刷新，不用按快照）：标定模式下直接看这三个数。
   拖动模式下它们≈tor-t_ff≈0 没有参考意义，别在那种状态下看。 */
volatile float g_dbg_err_j2 = 0.0f;
volatile float g_dbg_err_j3 = 0.0f;
volatile float g_dbg_err_j5 = 0.0f;

/* "电机应出力矩" = 我们发下去的 MIT 指令在电机侧合成的力矩
       tau_cmd = kp·(p_des − q) + kd·(0 − v) + t_ff     （关节系）
   不依赖 tor 反馈。标定模式(kp>0)下停稳时它就是电机真实出力
   （已被 45°松手实验验证：理论平衡 28°，实测停 29.4°）。
   tor 反馈解码坏掉的期间，用它代替 tor 做重力标定。 */
volatile float g_dbg_tau_cmd_j2 = 0.0f;
volatile float g_dbg_tau_cmd_j3 = 0.0f;
volatile float g_dbg_tau_cmd_j5 = 0.0f;

/* 姿态快照表的第三组力矩：tc = tau_cmd（kp 推算，不依赖 tor）。
   标定拟合优先用这组；te(tor)那组等解码修好后再对照。 */
volatile float g_dbg_pose_tc[DBG_POSE_N][3];

/* 记录时的 J4 角度（q[3]）。
   模型是平面三段，前提是 J4 = 0。上次采集没存这一列，导致事后无法判断
   数据有没有被 J4 的耦合污染 —— 这次必须存。 */
volatile float g_dbg_pose_q4[DBG_POSE_N];

/* 记录时电机直接反馈的力矩 motors[1/2/4].tor。
   和 tm+te 是同一回事，但直接存下来可以省掉"抄错正负"的风险。 */
volatile float g_dbg_pose_tor[DBG_POSE_N][3];

void Arm_Calibrate_Zero(void)
{
    // 刷新反馈
    Joint_UpdateState(g_arm.motors, g_arm.q, g_arm.v);

    for (int i = 0; i < 6; i++) {
        // 存重力零位

        g_gravity_zero[i] += g_arm.q[i];
        g_arm.motors[i].zero_offset = g_arm.motors[i].pos_raw;
    }
}

void Arm_Control_Init(void)
{
    Arm_State_Init();
}

void Arm_Control_Loop(void)
{
    g_loop_cnt++;                       /* 不涨 = 控制任务没跑 */
    Joint_UpdateState(g_arm.motors, g_arm.q, g_arm.v);

    /* 标定模式：g_calib_kp 由 0 变正的瞬间，把当前姿态锁成目标点，
       之后电机就"站"在这个姿态上，靠 kp 的静差顶住重力。
       停稳后 motors[i].tor 就是该姿态的重力力矩，可直接和 tau_gravity 对比。 */
    static float   calib_hold[6] = {0};
    static uint8_t calib_prev    = 0;
    if (g_calib_kp > 0.0f && !calib_prev) {
        for (int i = 0; i < 6; i++) calib_hold[i] = g_arm.q[i];
    }
    calib_prev = (g_calib_kp > 0.0f) ? 1u : 0u;

    // 一次性命令：把当前姿态登记为「全段水平伸直、指向 +x」的重力基准。
    // 只改 g_gravity_zero，不动电机 zero_offset，所以 q 不会跳变。
    //
    // 命令用 1 -> 2 的"粘滞"写法，不要清回 0：
    // 如果清回 0，这条命令在一个控制周期（3ms）内就消失了，Watch 上根本看不到 1，
    // 会被误认为"写不进去"。留成 2 就能确认确实执行过；要再来一次就再写 1。
    if (g_grav_zero_cmd == 1) {
        Arm_GravityZeroCapture(g_arm.q);
        g_grav_zero_cmd = 2;
    }

    // 重力零位
    float q_gravity[6];
    for (int i = 0; i < 6; i++) {
        q_gravity[i] = g_arm.q[i] + g_gravity_zero[i];
    }
    Arm_CalcGravityTorque(q_gravity, g_arm.tau_gravity);

    /* ------------------------- 诊断标志位 ------------------------- */
    static uint8_t fly_above[6] = {0};   /* 角速度超阈值的上升沿锁存 */
    uint16_t stat = 0;

    if (g_grav_enable) stat |= GRAV_STAT_ENABLE;
    if (g_thA_deg >  90.0f || g_thA_deg < -90.0f) stat |= GRAV_STAT_DEAD_A;
    if (g_thB_deg >  90.0f || g_thB_deg < -90.0f) stat |= GRAV_STAT_DEAD_B;
    if (g_thC_deg >  90.0f || g_thC_deg < -90.0f) stat |= GRAV_STAT_DEAD_C;

    /* "弹飞"捕捉：只在上升沿记一次，把出事瞬间的角度和速度存下来，
       省得你盯着一个飞速变化的数字去抓那一帧。
       只监控 J2/J3/J5 —— J4、J6 是滚转关节，模型给它们的前馈恒为 0，
       本来就没补偿，被手带着转是正常的，监控它们只会产生误报。 */
    static const uint8_t MON[3] = {1, 2, 4};
    for (int k = 0; k < 3; k++) {
        int i = MON[k];
        float av = fabsf(g_arm.v[i]);

        if (av > g_drag_vmax) g_drag_vmax = av;   /* 历史峰值，写 0 清零 */

        if (av > FLY_V_THRESHOLD) {
            if (!fly_above[i]) {
                g_fly_cnt++;
                g_fly_joint = (uint8_t)i;
                g_fly_v     = g_arm.v[i];
                g_fly_deg   = (i == 1) ? g_thA_deg
                            : (i == 2) ? g_thB_deg
                            :            g_thC_deg;
            }
            fly_above[i] = 1;
        } else {
            fly_above[i] = 0;
        }
    }
    if (g_fly_cnt) stat |= GRAV_STAT_FLY;

    /* 实时模型误差：标定模式(g_calib_kp>0 且 grav 关)下,
       tor=真实重力力矩, tau_gravity=模型值, 差值就是模型误差。
       正数 = 模型补少了(欠补,松手往下沉), 负数 = 补多了(过补,松手往上顶)。 */
    g_dbg_err_j2 = g_arm.motors[1].tor - g_arm.tau_gravity[1];
    g_dbg_err_j3 = g_arm.motors[2].tor - g_arm.tau_gravity[2];
    g_dbg_err_j5 = g_arm.motors[4].tor - g_arm.tau_gravity[4];

    /* 电机应出力矩：kp/p_des/kd/t_ff 此刻还是上一拍发出去的值，静态下无差别 */
    g_dbg_tau_cmd_j2 = g_arm.motors[1].kp * (g_arm.motors[1].p_des - g_arm.q[1])
                     + g_arm.motors[1].kd * (0.0f - g_arm.v[1])
                     + g_arm.motors[1].t_ff;
    g_dbg_tau_cmd_j3 = g_arm.motors[2].kp * (g_arm.motors[2].p_des - g_arm.q[2])
                     + g_arm.motors[2].kd * (0.0f - g_arm.v[2])
                     + g_arm.motors[2].t_ff;
    g_dbg_tau_cmd_j5 = g_arm.motors[4].kp * (g_arm.motors[4].p_des - g_arm.q[4])
                     + g_arm.motors[4].kd * (0.0f - g_arm.v[4])
                     + g_arm.motors[4].t_ff;

    /* 姿态快照：把当前 (cosA,cosB,cosC, 模型力矩, 实测力矩) 冻一行进表 */
    if (g_dbg_pose_cmd == 1) {
        uint8_t n = g_dbg_pose_cnt;
        if (n < DBG_POSE_N) {
            uint8_t qual = 0;
            if (fabsf(g_arm.v[1]) < 0.05f &&
                fabsf(g_arm.v[2]) < 0.05f &&
                fabsf(g_arm.v[4]) < 0.05f) {
                qual |= 1u;   /* 停稳了，这行数据可信 */
            }
            if (g_calib_kp > 0.0f) qual |= 2u;

            /* bit2: 电机真的在扛活
               实测力矩必须和【期望方向】同号，且不小于期望值的 1/3。
               注意期望方向要乘 g_grav_dir_j* —— J3 是 -1，所以它的实测力矩
               本来就该和模型反号（模型算的是"托住"，电机正方向是"下落"）。
               不加这个修正会把正常数据误判成坏行。 */
            {
                float m1 = g_arm.tau_gravity[1], m2 = g_arm.tau_gravity[2];
                float e1 = g_grav_dir_j2 * m1;
                float e2 = g_grav_dir_j3 * m2;
                float t1 = g_arm.motors[1].tor;
                float t2 = g_arm.motors[2].tor;
                uint8_t ok1 = (fabsf(e1) < 0.05f) ||
                              (t1 * e1 > 0.0f && fabsf(t1) > 0.33f * fabsf(e1));
                uint8_t ok2 = (fabsf(e2) < 0.05f) ||
                              (t2 * e2 > 0.0f && fabsf(t2) > 0.33f * fabsf(e2));
                if (ok1 && ok2) qual |= 4u;
            }

            g_dbg_pose_th[n][0] = g_thA_deg;
            g_dbg_pose_th[n][1] = g_thB_deg;
            g_dbg_pose_th[n][2] = g_thC_deg;
            g_dbg_pose_c[n][0]  = g_cosA;
            g_dbg_pose_c[n][1]  = g_cosB;
            g_dbg_pose_c[n][2]  = g_cosC;
            g_dbg_pose_tm[n][0] = g_arm.tau_gravity[1];
            g_dbg_pose_tm[n][1] = g_arm.tau_gravity[2];
            g_dbg_pose_tm[n][2] = g_arm.tau_gravity[4];
            /* ★ 关键：tor 必须在这里现取一次，并且 te 从同一个值算出来。
               不能在循环前段算 g_dbg_err_j*、在这里再读 motors[].tor ——
               中间隔了 6 帧 CAN 收发，motors[].tor 已被下一帧反馈覆盖，
               两者来自不同时刻，te 和 tor 永远对不上（实测差过 1.2 N·m）。 */
            {
                float t1 = g_arm.motors[1].tor;
                float t2 = g_arm.motors[2].tor;
                float t4 = g_arm.motors[4].tor;
                g_dbg_pose_tor[n][0] = t1;
                g_dbg_pose_tor[n][1] = t2;
                g_dbg_pose_tor[n][2] = t4;
                g_dbg_pose_te[n][0] = t1 - g_arm.tau_gravity[1];
                g_dbg_pose_te[n][1] = t2 - g_arm.tau_gravity[2];
                g_dbg_pose_te[n][2] = t4 - g_arm.tau_gravity[4];
            }
            g_dbg_pose_tc[n][0] = g_dbg_tau_cmd_j2;
            g_dbg_pose_tc[n][1] = g_dbg_tau_cmd_j3;
            g_dbg_pose_tc[n][2] = g_dbg_tau_cmd_j5;
            g_dbg_pose_q4[n]    = g_arm.q[3];
            g_dbg_pose_qual[n]  = qual;
            g_dbg_pose_cnt = n + 1;
        }
        g_dbg_pose_cmd = 2;
    }

    /* 行读取器：把 g_dbg_rd_idx 指定的那一行同步到标量，方便 Watch 里翻页看 */
    {
        uint8_t k = g_dbg_rd_idx;
        if (k >= DBG_POSE_N) k = DBG_POSE_N - 1;
        g_dbg_rd_qual = g_dbg_pose_qual[k];
        g_dbg_rd_thA  = g_dbg_pose_th[k][0];
        g_dbg_rd_thB  = g_dbg_pose_th[k][1];
        g_dbg_rd_thC  = g_dbg_pose_th[k][2];
        g_dbg_rd_tm2  = g_dbg_pose_tm[k][0];
        g_dbg_rd_tm3  = g_dbg_pose_tm[k][1];
        g_dbg_rd_tm5  = g_dbg_pose_tm[k][2];
        g_dbg_rd_te2  = g_dbg_pose_te[k][0];
        g_dbg_rd_te3  = g_dbg_pose_te[k][1];
        g_dbg_rd_te5  = g_dbg_pose_te[k][2];
        g_dbg_rd_q4   = g_dbg_pose_q4[k];
        g_dbg_rd_tor2 = g_dbg_pose_tor[k][0];
        g_dbg_rd_tor3 = g_dbg_pose_tor[k][1];
        g_dbg_rd_tor5 = g_dbg_pose_tor[k][2];
        g_dbg_rd_tc2  = g_dbg_pose_tc[k][0];
        g_dbg_rd_tc3  = g_dbg_pose_tc[k][1];
        g_dbg_rd_tc5  = g_dbg_pose_tc[k][2];
    }

    // 安全检查
    // Safety_CheckArm 返回 false 时仍是“低刚度重力托举”，并非硬急停。
    uint8_t safety_ok = Safety_CheckArm(&g_arm);

    /* ---- 掉线自愈 ----
       达妙电机锁错（欠压/过流/过温/编码器）后错误位一直挂着，不再回复控制帧，
       is_online 就永远是 0，安全逻辑把整条臂按成零力矩 —— 现场表现就是
       "电机使能不了 / 推不动"，而且断电才能恢复。
       这里每 200ms 对离线电机补发一次 清错 + 使能，不用重启。
       上电后 3 秒内不发，避免和 Task_ArmControl 里的使能序列打架。 */
    {
        static uint32_t t_heal = 0;
        uint32_t now = HAL_GetTick();
        if (now > 3000 && (now - t_heal) > 200) {
            t_heal = now;

            /* 先数在线数：
                 0 台  → 总线本身不通（没供电/线断/终端电阻缺），补发毫无意义，
                         而且 Autoretransmission 会把 3 个邮箱全堵死，反而更糟 —— 跳过
                 6 台  → 没得修
                 1~5 台 → 总线是好的，只是个别电机锁死了，这才值得补发 */
            uint8_t online_cnt = 0;
            for (int i = 0; i < 6; i++) online_cnt += g_arm.is_online[i];

            if (online_cnt > 0 && online_cnt < 6) {
                for (int i = 0; i < 6; i++) {
                    if (!g_arm.is_online[i]) {
                        DM_J4310_ClearError(g_arm.motors[i].id);
                        DM_J4310_Enable(g_arm.motors[i].id);
                        g_heal_cnt++;
                    }
                }
            }
        }
    }

    /* 补偿开着时掉进安全降级 → 记一次"塌一下"。只记进入沿，不记持续期 */
    static uint8_t prev_safety_ok = 1;
    if (g_grav_enable && prev_safety_ok && !safety_ok) g_dbg_sag_cnt++;
    prev_safety_ok = safety_ok;
    if (!safety_ok) stat |= GRAV_STAT_SAG;

    for (int i = 0; i < 6; i++) {
        /* 标定模式下锁住标定瞬间的姿态，否则 p_des 跟着 q 走，kp 永远差 0、托不住 */
        g_arm.motors[i].p_des = (g_calib_kp > 0.0f) ? calib_hold[i] : g_arm.q[i];
        g_arm.motors[i].v_des = 0.0f;
        g_arm.motors[i].kp = g_calib_kp;   /* 平时 0 = 零刚度拖动 */

        if (!g_arm.is_online[i] || !safety_ok) {
            g_arm.motors[i].kd = 0.0f;
            g_arm.motors[i].t_ff = 0.0f;
            g_arm.tau_final[i] = 0.0f;
            DM_J4310_SendMIT(&g_arm.motors[i]);
            continue;
        }

        /* J4（i==3）是滚转轴，重力矩不足以抵消摩擦但偏载会自己掉，
           单独给更大的阻尼把它压慢；其余关节仍用 g_drag_kd */
        g_arm.motors[i].kd = safety_ok ? ((i == 3) ? g_j4_kd : g_drag_kd) : 0.05f;

        // 总开关：g_grav_enable = 0 时不向前馈出力，臂只剩 kd 阻尼，可自由拖动。
        // 模型照算，tau_gravity 仍然每拍更新，方便对照姿态。
        float comp_tor = g_grav_enable ? g_arm.tau_gravity[i] : 0.0f;
        if (i == 1) {
            comp_tor *= g_grav_dir_j2 * g_scale_j2;
        } else if (i == 2) {
            comp_tor *= g_grav_dir_j3 * g_scale_j3;
        } else if (i == 3) {
            comp_tor *= g_grav_dir_j4 * g_scale_j4;
        } else if (i == 4) {
            comp_tor = g_j5_comp_enable ? comp_tor * g_grav_dir_j5 * g_scale_j5 : 0.0f;
        } else {
            comp_tor = 0.0f;
        }

        float clamped = Math_Clamp(comp_tor, -TAU_LIMITS[i], TAU_LIMITS[i]);

        if (fabsf(comp_tor) > TAU_LIMITS[i])             stat |= GRAV_STAT_CLAMPED;
        /* 阈值用 0.1 N·m：正常拖动每拍最多变 0.03，只有"模型输出跳变"才会超。
           比如越过死点时 cos 换号，就会看到这一位置 1。 */
        if (fabsf(clamped - g_arm.tau_final[i]) > 0.1f)  stat |= GRAV_STAT_RAMPING;

        g_arm.tau_final[i] = Math_Ramp(clamped, g_arm.tau_final[i], 0.03f);
        g_arm.motors[i].t_ff = g_arm.tau_final[i];

        DM_J4310_SendMIT(&g_arm.motors[i]);
    }

    g_grav_stat = stat;
}
