#include "gravity_model.h"
#include <math.h>

#define G_CONST 9.80665f

/* ==========================================================================
 * 段参数表（实测 2026-10-24，单位：kg / m）
 * 全部声明为 volatile，是为了能在 Keil Watch 里实时改、改完立刻生效，不用重烧。
 * ========================================================================== */

/* 段A：J2 轴 → J3 轴 */
volatile float g_segA_m  = 0.354f;   /* 质量                                  */
volatile float g_segA_L  = 0.150f;   /* 段长（两关节轴心距）                   */
volatile float g_segA_lc = 0.075f;   /* 质心距近端关节  ← TODO 待实测，现按 L/2 */

/* 段B：J3 轴 → J5 轴（含 J4 滚转，轴与连杆共线，合并成一段） */
volatile float g_segB_m  = 0.811f;
volatile float g_segB_L  = 0.150f;
volatile float g_segB_lc = 0.075f;   /* TODO 待实测 */

/* 段C：J5 轴 → 末端（含 J6 滚转，轴与末端共线，合并成一段） */
volatile float g_segC_m  = 0.445f;
volatile float g_segC_L  = 0.082f;   /* 段C 之后没有关节了，此值仅供参考       */
volatile float g_segC_lc = 0.041f;   /* TODO 待实测 */

/* --------------------------------------------------------------------------
 * 每个关节只留一个符号，用一次静态实验定死，之后不要再动。
 * 不要恢复成「方向乘子 + 比例系数」那种双开关：两个符号乘在同一项上会互相
 * 抵消，调参时很容易误以为调对了。
 *
 * 2026-10-24 实测结论：
 *   g_sign_q2 = +1   已确认（J2 抬 45°，thA 正好 +45.4°）
 *   g_sign_q3 = -1   已确认（J3 上抬 45°，thB 正好 +44.9°；取 +1 会得 -45°，方向反）
 *   g_sign_q5 = +1   尚未单独验证，见下方 TODO
 * -------------------------------------------------------------------------- */
volatile float g_sign_q2 = 1.0f;     /* 段A 仰角方向 */
volatile float g_sign_q3 = -1.0f;    /* 段B 相对仰角 */
volatile float g_sign_q5 = 1.0f;     /* 段C 相对仰角  ← TODO 待单独验证 */

/* 整条臂的总闸：补偿力矩整体方向反了（越补越往下掉）就改成 -1.0f */
volatile float g_grav_sign = 1.0f;

/* J4 偏心补偿增益：0 = 关闭（默认，行为与改动前完全一致）
   实机上在 Watch 里试 +1.0 / -1.0，看哪个方向能让 J4 停住。
   若需要 |gain| > 2 才停得住，说明末端实际比 mC 模型重，那要回去改 mC。 */
volatile float g_j4_gain = 0.0f;

/* 重力补偿总开关  0 = 关（t_ff 恒 0，臂只剩 kd 阻尼，可随便拖但会掉）
 *               1 = 开（正常重力补偿）
 *
 * 2026-10-29 改为默认 1：用户要求上电即生效，不用每次在 Watch 里手动打开。
 * 关掉时模型照算（tau_gravity 每拍照常更新，方便对照姿态），只是不下发。
 * 安全性由别处兜底：控制环要等 2.5s 电机自检完才开始跑；掉线/急停时
 * g_arm.mode 不是 ZERO_FORCE，安全逻辑会把 kd 降到 0.05。
 * 想临时关掉（标定、或怀疑补偿顶手），Watch 里写 0 即可。 */
volatile uint8_t g_grav_enable = 1;

/* 模型"认为"的三段仰角（度），仅供 Watch 与实际姿态对照 */
volatile float g_thA_deg = 0.0f;
volatile float g_thB_deg = 0.0f;
volatile float g_thC_deg = 0.0f;

/* 三段仰角的余弦 —— 力矩对它们是线性的：
       τ_J2/g = K1·cosA + K2·cosB + K3·cosC
       τ_J3/g = P2·cosB + P3·cosC
       τ_J5/g = Q3·cosC
   拿标定姿态表(arm_control.c 的 g_dbg_pose_*)做最小二乘就能把 K/P/Q 解出来，
   再反推 lc。导出这三个数就是为了快照时不用重新算三角。 */
volatile float g_cosA = 0.0f;
volatile float g_cosB = 0.0f;
volatile float g_cosC = 0.0f;

/* 重力零位：★ 2026-10-27 起全部为 0
 *
 * 改用「达妙调试助手 → 保存零点」把零点写进电机内部编码器（ID 8/9/17 三台，
 * 在同一「三段共线、水平」姿态下依次保存）。这样电机自己在水平位就报 q=0，
 * 软件侧不再需要任何偏置 —— 断电、换固件、重烧程序都不会丢。
 *
 * 所以这里必须是全 0。如果这里还填着旧值，会变成「电机零点 + 软件偏置」
 * 叠加成双倍偏移，模型直接错乱。
 *
 * 什么时候需要重标：机械拆装 / 换电机 / 在调试助手里重新保存过零点。
 * 其余情况不用动。 */
volatile float g_gravity_zero[6] = {
    0.0f,
    0.0f,             /* J2：零点已在电机内部 */
    0.0f,             /* J3：同上 */
    0.0f,
    0.0f,             /* J5：同上 */
    0.0f
};

/* 标零命令：0=空闲  1=请求  2=已执行（实际处理在 Arm_Control_Loop 里） */
volatile uint8_t g_grav_zero_cmd = 0;

/* ==========================================================================
 * 逐段合并求重力矩
 *
 *   τ_J2 = g·[ mA·xA + mB·xB + mC·xC ]        （力臂都量到 J2 轴）
 *   τ_J3 = g·[ mB·xB' + mC·xC' ]              （力臂都量到 J3 轴）
 *   τ_J5 = g·  mC·lcC·cos(θC)                 （J5 只扛段C）
 *
 * 其中 x 是该段质心的水平坐标。重力矩只看水平力臂，所以仰角 θ 取 cos 就够：
 * 水平(0°)力臂满值，竖直(±90°)力臂为 0，越过 90° 自动变号，不需要额外判符号。
 * ========================================================================== */
void Arm_CalcGravityTorque(float q[6], float tau_g[6])
{
    /* 三段各自的仰角（相对水平 +x）。传进来的 q 已经含 g_gravity_zero。 */
    const float thA = g_sign_q2 * q[1];           /* 段A：J2→J3      */
    const float thB = thA + g_sign_q3 * q[2];     /* 段B：J3→J5      */
    const float thC = thB + g_sign_q5 * q[4];     /* 段C：J5→末端    */

    /* 导出成角度给 Watch 对照：这三个数必须和肉眼看到的实际姿态一致，
       对不上就说明 g_sign_q* 或 g_gravity_zero 错了，别急着调力臂。 */
    g_thA_deg = thA * 57.29578f;
    g_thB_deg = thB * 57.29578f;
    g_thC_deg = thC * 57.29578f;

    const float cA = cosf(thA);
    const float cB = cosf(thB);
    const float cC = cosf(thC);

    g_cosA = cA;
    g_cosB = cB;
    g_cosC = cC;

    /* 各段质心的水平坐标 —— 相对 J2 轴 */
    const float xA_J2 = g_segA_lc * cA;
    const float xB_J2 = g_segA_L * cA + g_segB_lc * cB;
    const float xC_J2 = g_segA_L * cA + g_segB_L * cB + g_segC_lc * cC;

    /* 各段质心的水平坐标 —— 相对 J3 轴（段A 在它近端，不参与） */
    const float xB_J3 = g_segB_lc * cB;
    const float xC_J3 = g_segB_L * cB + g_segC_lc * cC;

    /* J2：扛 段A + 段B + 段C 全部 */
    tau_g[1] = g_grav_sign * G_CONST *
               (g_segA_m * xA_J2 + g_segB_m * xB_J2 + g_segC_m * xC_J2);

    /* J3：扛 段B + 段C */
    tau_g[2] = g_grav_sign * G_CONST *
               (g_segB_m * xB_J3 + g_segC_m * xC_J3);

    /* J5：只扛 段C */
    tau_g[4] = g_grav_sign * G_CONST * g_segC_m * g_segC_lc * cC;

    tau_g[0] = 0.0f;

    /* J4：绕前臂的滚转轴。理想情况段C 质心在轴上、不受重力矩；
       但末端有偏心 —— J5 弯开后质心离轴 lcC·sin(q5)，J4 再滚过 q4，
       这部分偏置就转出臂平面，重力相对前臂轴产生力矩。

       几何推导（J5 转轴垂直于臂平面，用户已确认）：
           r_perp = lcC·sin(q5)·[cos(q4)·n2 + sin(q4)·n1]     n1 面外、n2 面内
           τ_J4 = (r_perp × Fg)·f = mC·g·lcC·sin(q5)·cos(θB)·sin(q4)
       q4 = 0 时恒为 0（偏置在平面内），与"滚到某个角度才掉"的现象吻合。
       该形式与旧工程 commit c20b33a 一致，可交叉印证。

       q[4] 是 J5 的关节角（不是 θC）；q[3] 是 J4 的关节角。
       增益为 0 时整项关闭，与改动前行为完全相同。 */
    tau_g[3] = g_grav_sign * G_CONST * g_segC_m * g_segC_lc * g_j4_gain
             * sinf(q[4]) * cB * sinf(q[3]);

    tau_g[5] = 0.0f;
}

/* ==========================================================================
 * 把当前姿态登记成「全段水平伸直、指向 +x」的基准
 *
 * 注意只改写 g_gravity_zero，不碰电机的 zero_offset：
 *    - zero_offset 是编码器零位（q 的计算基准），改了 q 会立刻归零
 *    - g_gravity_zero 只影响重力模型用的仰角
 * 两者混在一起是最容易把姿态搞乱的地方，所以这里刻意分开。
 *
 * 用法：把臂摆好 → Watch 里 g_grav_zero_cmd 写 1 → 看到变成 2 即已执行
 * ========================================================================== */
void Arm_GravityZeroCapture(float q[6])
{
    for (int i = 0; i < 6; i++)
    {
        g_gravity_zero[i] = -q[i];
    }
}
