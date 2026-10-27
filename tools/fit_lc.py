#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从「姿态快照」反解三段质心距 lc —— 把"猜参数"变成"解参数"

纯标准库，不需要 numpy。

## 为什么不用纠结符号对不对

cos 是偶函数：把某个仰角整体取反，cos 不变。所以如果某个 g_sign_q* 反了，
它只会让"角度读数"和"模型算出的力矩"对不上，不会凭空产生别的解。

于是脚本直接把 3 个关节的符号组合（2^3 = 8 种）全试一遍，
看哪一种能把 8 行数据同时拟合上 —— **残差最小的那组就是真实符号**。
你不用事先知道方向，采完数据让它自己挑。

## 用法

  1. 用 arm_control.c 里的 g_dbg_pose_* 表采 6~8 个姿态（差异要大，全程 J4=0）
  2. 把数据填进 POSES（或存成空白分隔的 txt，用命令行参数传路径）
  3. python fit_lc.py

每行 9 个数，顺序和 Watch 三张表一一对应：
    thA  thB  thC   tm2  tm3  tm5   te2  te3  te5
（tm = g_dbg_pose_tm 模型力矩；te = g_dbg_pose_te 实测−模型，脚本内部算实测 = tm+te）
也可以只写 6 个数（角度 + 已算好的实测力矩）。

## 模型

    tau_J2 = K1*cos(thA) + K2*cos(thB) + K3*cos(thC)
    tau_J3 =               K2*cos(thB) + K3*cos(thC)
    tau_J5 =                             K3*cos(thC)

    K1 = g*( mA*lcA + (mB+mC)*LA )    lcA = (K1/g - (mB+mC)*LA) / mA
    K2 = g*( mB*lcB +  mC*LB     )    lcB = (K2/g -  mC*LB     ) / mB
    K3 = g* mC*lcC                    lcC =  K3/(g*mC)
"""
import sys
import math

# ============ 段参数（与 gravity_model.c 保持一致）============
G = 9.80665
M_A, L_A = 0.354, 0.150
M_B, L_B = 0.811, 0.150
M_C, L_C = 0.445, 0.082

# ============ 采到的姿态数据：填这里 ============
POSES = [
    # 示例：
    # (0,   0,    0,    3.539, 1.430, 0.179,  0.10, 0.07, 0.02),
    # (60,  30,   0,    2.5,   1.1,   0.15,   0.10, 0.07, 0.02),
]

SEARCH_SIGNS = True   # 自动搜索符号组合；关掉就按记录值原样解
FIT_J2_ONLY  = True   # 只用 J2 的数据解 K1/K2/K3（J2 信噪比 20:1，J3/J5 信噪比低得多）


def solve_j2_only(poses):
    """只用 tau_J2 = K1·cosA + K2·cosB + K3·cosC 解三个 K。

    故意不带截距：J2 只有 5~8 行时，多一个自由常数项会让 K1 和 b 互相抵消
    （出现过 K1=10.6、b=-10.0 这种荒谬解，残差却只有 0.02）。
    摩擦靠"每个姿态上下各测一次取平均"来抵消，不靠拟合去猜。"""
    rows = [(a, b, c, m2) for (a, b, c, m2, _m3, _m5) in poses if m2 is not None]
    n = len(rows)
    if n < 3:
        return None

    # 正规方程 A^T A x = A^T y，A 的列 = [cosA, cosB, cosC]
    A = []
    y = []
    for a, b, c, m2 in rows:
        A.append([math.cos(math.radians(a)), math.cos(math.radians(b)),
                  math.cos(math.radians(c))])
        y.append(m2)
    m = 3
    AtA = [[sum(A[k][i] * A[k][j] for k in range(n)) for j in range(m)] for i in range(m)]
    Aty = [sum(A[k][i] * y[k] for k in range(n)) for i in range(m)]

    # 高斯消元（带列主元）
    M = [AtA[i][:] + [Aty[i]] for i in range(m)]
    for i in range(m):
        p = max(range(i, m), key=lambda r: abs(M[r][i]))
        if abs(M[p][i]) < 1e-12:
            return None
        M[i], M[p] = M[p], M[i]
        for r in range(i + 1, m):
            f = M[r][i] / M[i][i]
            for cc in range(i, m + 1):
                M[r][cc] -= f * M[i][cc]
    x = [0.0] * m
    for i in range(m - 1, -1, -1):
        x[i] = (M[i][m] - sum(M[i][j] * x[j] for j in range(i + 1, m))) / M[i][i]

    pred = [sum(A[k][j] * x[j] for j in range(m)) for k in range(n)]
    rmse = math.sqrt(sum((pred[k] - y[k]) ** 2 for k in range(n)) / n)
    return {"K": (x[0], x[1], x[2]), "b": (0.0, 0.0, 0.0),
            "rmse": (rmse, 0.0, 0.0), "n": (n, 0, 0),
            "total": rmse, "signs": (1, 1, 1)}


def report_j2(sol, poses):
    K1, K2, K3 = sol["K"]
    bb = sol["b"][0]
    lcA = (K1 / G - (M_B + M_C) * L_A) / M_A
    lcB = (K2 / G - M_C * L_B) / M_B
    lcC = K3 / (G * M_C)
    d2, d3, d5 = sol["signs"]

    print("\n" + "=" * 66)
    print("【只用 J2 拟合】符号组合: sign_q2=%+d sign_q3=%+d sign_q5=%+d" % (d2, d3, d5))
    print("-" * 66)
    print("  n=%d  K1=%+.4f  K2=%+.4f  K3=%+.4f  b(摩擦)=%+.4f" % (sol["n"][0], K1, K2, K3, bb))
    print("  RMSE = %.4f N·m   (J2 满量程 3.5，<0.1 算好)" % sol["rmse"][0])
    print("-" * 66)
    print("  g_segA_lc = %.4f   (L/2=%.4f)" % (lcA, L_A / 2))
    print("  g_segB_lc = %.4f   (L/2=%.4f)" % (lcB, L_B / 2))
    print("  g_segC_lc = %.4f   (L/2=%.4f)" % (lcC, L_C / 2))
    print("=" * 66)

    lim = G * M_C * L_B
    if K2 < lim:
        print("[!] K2 = %.3f 小于物理下限 %.3f (=g·mC·LB)。" % (K2, lim))
        print("    说明 J3 那几行数据互相矛盾，或者有行是在限位/被手扶时记的。")
        print("    先把逐行残差里标'重采'的行换掉再跑。")
    for nm, v, L in (("A", lcA, L_A), ("B", lcB, L_B), ("C", lcC, L_C)):
        pct = v / L * 100
        if not (0.0 < pct < 120.0):
            print("  段%s: lc 占段长 %5.1f%%   <== 不合理" % (nm, pct))

    print("\n逐行残差（J2）：")
    for k, (a, b, c, m2, _m3, _m5) in enumerate(poses):
        if m2 is None:
            continue
        pr = (K1 * math.cos(math.radians(a)) + K2 * math.cos(math.radians(b))
              + K3 * math.cos(math.radians(c)) + bb)
        r = m2 - pr
        q4 = LAST_Q4[k] if k < len(LAST_Q4) else None
        q4s = "  J4=%+6.2f°" % math.degrees(q4) if q4 is not None else ""
        bad = abs(r) > 0.15
        if q4 is not None and abs(math.degrees(q4)) > 10.0:
            bad, q4s = True, q4s + " <J4偏大"
        print("  %2d  %7.1f %7.1f %7.1f | 实测 %+6.3f  模型 %+6.3f  残差 %+6.3f%s%s"
              % (k, a, b, c, m2, pr, r, q4s, "   <== 重采" if bad else ""))



LAST_Q4 = []          # 每行的 J4 角度（有就存，用来诊断数据有没有被 J4 污染）

def normalize(rows):
    """支持多种列数，统一成 (thA,thB,thC, m2,m3,m5)：
         6 列：角度 + 实测力矩
         7 列：角度 + 实测力矩(tor) + q4
         9 列：角度 + 模型力矩 + 误差(te)，实测 = tm+te
        10 列：同上 + q4
        13 列：角度 + tm + te + tor(直接实测) + q4
    """
    global LAST_Q4
    out, q4s = [], []
    for r in rows:
        n = len(r)
        q4 = None
        if n >= 13:                      # 角度 + tm + te + tor + q4（最好用这个）
            th, meas, q4 = r[0:3], list(r[10:13]), r[13]
        elif n >= 10:                    # 角度 + tm + te + q4
            th = r[0:3]
            meas = [None if (r[3+k] is None or r[6+k] is None) else r[3+k] + r[6+k] for k in range(3)]
            q4 = r[9]
        elif n == 9:                     # 角度 + tm + te
            th = r[0:3]
            meas = [None if (r[3+k] is None or r[6+k] is None) else r[3+k] + r[6+k] for k in range(3)]
        elif n == 7:                     # 角度 + 实测力矩 + q4
            th, meas, q4 = r[0:3], list(r[3:6]), r[6]
        else:                            # 角度 + 实测力矩
            th, meas = r[0:3], list(r[3:6])
        out.append(tuple(th) + tuple(meas))
        q4s.append(q4)
    LAST_Q4 = q4s
    return out


def lin_fit(xs, ys):
    """y = a*x + b 的最小二乘 → (a, b, rmse, n)"""
    n = len(xs)
    if n < 2:
        return None
    sx, sy = sum(xs), sum(ys)
    sxx = sum(x * x for x in xs)
    sxy = sum(x * y for x, y in zip(xs, ys))
    den = n * sxx - sx * sx
    if abs(den) < 1e-12:
        return None
    a = (n * sxy - sx * sy) / den
    b = (sy - a * sx) / n
    rmse = math.sqrt(sum((a * x + b - y) ** 2 for x, y in zip(xs, ys)) / n)
    return a, b, rmse, n


def solve_core(poses):
    """解 K1/K2/K3；解不出来返回 None"""
    cs = [(math.cos(math.radians(r[0])),
           math.cos(math.radians(r[1])),
           math.cos(math.radians(r[2]))) for r in poses]

    # ---- J5: tau = K3*cosC + b ----
    xs = [c[2] for c in cs if True]
    ys = [r[5] for r in poses]
    pairs = [(c[2], r[5]) for c, r in zip(cs, poses) if r[5] is not None]
    if len(pairs) < 2:
        return None
    res = lin_fit([p[0] for p in pairs], [p[1] for p in pairs])
    if not res:
        return None
    K3, b5, rmse5, n5 = res

    # ---- J3: tau - K3*cosC = K2*cosB + b ----
    pairs = [(c[1], r[4] - K3 * c[2]) for c, r in zip(cs, poses) if r[4] is not None]
    if len(pairs) < 2:
        return None
    res = lin_fit([p[0] for p in pairs], [p[1] for p in pairs])
    if not res:
        return None
    K2, b3, rmse3, n3 = res

    # ---- J2: tau - K2*cosB - K3*cosC = K1*cosA + b ----
    pairs = [(c[0], r[3] - K2 * c[1] - K3 * c[2])
             for c, r in zip(cs, poses) if r[3] is not None]
    if len(pairs) < 2:
        return None
    res = lin_fit([p[0] for p in pairs], [p[1] for p in pairs])
    if not res:
        return None
    K1, b2, rmse2, n2 = res

    return {
        "K": (K1, K2, K3), "b": (b2, b3, b5),
        "rmse": (rmse2, rmse3, rmse5), "n": (n2, n3, n5),
        "total": rmse2 + rmse3 + rmse5,
        "signs": (1, 1, 1),
    }


def apply_signs(poses, d2, d3, d5):
    """按符号假设重算三段仰角。thB-thA 只含 q3，thC-thB 只含 q5，所以可以精确还原。"""
    out = []
    for a, b, c, m2, m3, m5 in poses:
        ta = d2 * a
        tb = ta + d3 * (b - a)
        tc = tb + d5 * (c - b)
        out.append((ta, tb, tc, m2, m3, m5))
    return out


def report(sol, poses):
    K1, K2, K3 = sol["K"]
    b2, b3, b5 = sol["b"]
    lcA = (K1 / G - (M_B + M_C) * L_A) / M_A
    lcB = (K2 / G - M_C * L_B) / M_B
    lcC = K3 / (G * M_C)

    d2, d3, d5 = sol["signs"]
    print("\n" + "=" * 66)
    print("选中的符号组合:  sign_q2=%+d  sign_q3=%+d  sign_q5=%+d" % (d2, d3, d5))
    print("-" * 66)
    print("  J2: n=%d  K1=%+.4f  b=%+.4f  RMSE=%.4f N·m"
          % (sol["n"][0], K1, sol["b"][0], sol["rmse"][0]))
    print("  J3: n=%d  K2=%+.4f  b=%+.4f  RMSE=%.4f N·m"
          % (sol["n"][1], K2, sol["b"][1], sol["rmse"][1]))
    print("  J5: n=%d  K3=%+.4f  b=%+.4f  RMSE=%.4f N·m"
          % (sol["n"][2], K3, sol["b"][2], sol["rmse"][2]))
    print("-" * 66)
    print("g_segA_lc = %.4f   (段长 %.3f, L/2=%.4f)" % (lcA, L_A, L_A / 2))
    print("g_segB_lc = %.4f   (段长 %.3f, L/2=%.4f)" % (lcB, L_B, L_B / 2))
    print("g_segC_lc = %.4f   (段长 %.3f, L/2=%.4f)" % (lcC, L_C, L_C / 2))
    print("=" * 66)
    for nm, v, L in (("A", lcA, L_A), ("B", lcB, L_B), ("C", lcC, L_C)):
        pct = v / L * 100
        bad = "   <== 不合理(0~120%%)，数据可能有问题" if not (0.0 < pct < 120.0) else ""
        print("  段%s: lc 占段长 %5.1f%%%s" % (nm, pct, bad))

    tot = sol["total"]

    # ---- 逐行残差：把坏行揪出来 ----
    print("\n逐行残差（|r| > 0.15 N·m 的行基本是坏数据）：")
    print("   #     thA     thB     thC   |  r_J2    r_J3    r_J5")
    bad = []
    for k, (a, b, c, m2, m3, m5) in enumerate(poses):
        ca = math.cos(math.radians(a))
        cb = math.cos(math.radians(b))
        cc = math.cos(math.radians(c))
        r2 = None if m2 is None else m2 - (K1 * ca + K2 * cb + K3 * cc + b2)
        r3 = None if m3 is None else m3 - (K2 * cb + K3 * cc + b3)
        r5 = None if m5 is None else m5 - (K3 * cc + b5)

        def fs(x):
            return "  -   " if x is None else "%+6.3f" % x

        hit = any(x is not None and abs(x) > 0.15 for x in (r2, r3, r5))
        if hit:
            bad.append(k)
        print("  %2d  %7.1f %7.1f %7.1f | %s %s %s%s"
              % (k, a, b, c, fs(r2), fs(r3), fs(r5), "   <== 重采" if hit else ""))
    if bad:
        print("\n建议重采第 %s 行（多半是顶到机械限位、或者停稳前被手扶过）。"
              % ", ".join(str(i) for i in bad))
        print("换掉那行数据后重跑本脚本。")

    if tot > 0.35:
        print("\n[注意] 总残差 %.2f N·m 偏大。可能的顺序：" % tot)
        print("  1) J4 没归零      -> 看采集时 q[3] 是否≈0")
        print("  2) 零点偏差       -> th 对应不上真实姿态")
        print("  3) 质量/段长录错  -> 核对 g_segX_m / g_segX_L")
        print("  4) 摩擦很大       -> 该行 qual 是否=3、是否慢慢逼近姿态")
    elif not (0.0 < lcA / L_A < 1.2 and 0.0 < lcB / L_B < 1.2 and 0.0 < lcC / L_C < 1.2):
        print("\n[注意] 残差虽然小，但解出的 lc 超出物理范围，八成是符号组合选错了解。")
    else:
        print("\n残差正常，参数可信。把三个 lc 填回 gravity_model.c 即可。")


def main():
    rows = POSES
    if len(sys.argv) > 1:
        rows = []
        with open(sys.argv[1], encoding="utf-8") as f:
            for ln in f:
                ln = ln.strip()
                if not ln or ln.startswith("#"):
                    continue
                parts = ln.replace(",", " ").split()
                if len(parts) < 6:
                    continue
                vals = []
                for p in parts[:9]:
                    try:
                        vals.append(float(p))
                    except ValueError:
                        vals.append(None)
                rows.append(tuple(vals))
        print("从 %s 读入 %d 行" % (sys.argv[1], len(rows)))

    if not rows:
        print("POSES 是空的。先采数据填进去再跑。")
        print("格式：thA thB thC  tm2 tm3 tm5  te2 te3 te5")
        return

    poses = normalize(rows)
    print("有效行数：%d" % len(poses))

    if not SEARCH_SIGNS:
        sol = solve_core(poses)
        if sol:
            report(sol, poses)
        else:
            print("数据不足，解不出来。")
        return

    # ---- 8 种符号组合全试一遍 ----
    results = []
    for d2 in (1, -1):
        for d3 in (1, -1):
            for d5 in (1, -1):
                p = apply_signs(poses, d2, d3, d5)
                sol = solve_j2_only(p) if FIT_J2_ONLY else solve_core(p)
                if sol:
                    sol["signs"] = (d2, d3, d5)
                    results.append(sol)

    if not results:
        print("8 种符号组合都解不出来，检查数据。")
        return

    results.sort(key=lambda s: s["total"])
    print("\n符号组合残差排名（总残差越小越可信）：")
    for s in results:
        d2, d3, d5 = s["signs"]
        print("  q2=%+d q3=%+d q5=%+d   RMSE总=%.4f  (J2 %.4f / J3 %.4f / J5 %.4f)"
              % (d2, d3, d5, s["total"], *s["rmse"]))

    best_poses = apply_signs(poses, *results[0]["signs"])
    if FIT_J2_ONLY:
        report_j2(results[0], best_poses)
    else:
        report(results[0], best_poses)


if __name__ == "__main__":
    main()
