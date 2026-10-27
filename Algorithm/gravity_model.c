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

/* 重力补偿总开关：默认 0 = 关。
   关的时候模型照算（tau_gravity 每拍照常更新，方便对照姿态），
   但 t_ff 恒为 0，臂只剩 kd 阻尼，可以随便拖。调好了再置 1。 */
volatile uint8_t g_grav_enable = 0;

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

    /* J1 是竖直偏航轴，J4/J6 是滚转轴且与连杆共线，一阶重力矩都是 0。
       J4 的二阶项（J5 弯开后段C 质心离轴产生力矩）先不建，等 J2/J3/J5 调平
       之后再补，形式大致是  mC·g·lcC·sin(q5)·sin/cos(q4)·cos(θB)，
       具体取 sin 还是 cos 取决于 J5 转轴在前臂零滚转时是朝面内还是面外，
       这一点现有旧代码给不出可信答案（旧版的 sin(q4) 本身就可能是错的，
       否则不会需要 g_scale_j4=0.30 去压），到时候用实验定。 */
    tau_g[0] = 0.0f;
    tau_g[3] = 0.0f;
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
