#!/usr/bin/env python3
"""
J2/J3/J4 重力补偿拟合工具。

用法：
1. STM32 通过 VOFA 串口输出 CSV：q1,q2,q4,tau2_motor,tau3_motor,tau4_motor
2. 正扫 + 反扫各走一遍 J2 角度范围
3. 把 VOFA 保存的 CSV 放到 data/ 目录
4. python fit_gravity.py data/xxx.csv
5. 自动生成 gravity_model_fit.c 替换现有模型
"""
import numpy as np
import csv, math, sys
from pathlib import Path

# 关节索引：0=J1, 1=J2, 2=J3, 3=J4, 4=J5, 5=J6
AXES = [1, 2, 3]  # 只拟合 J2/J3/J4

# 拟合项（和山东大学一致的简化版）
TERMS = {
    1: ["1", "sin(q1)", "cos(q1)", "sin(q1+q2)", "cos(q1+q2)", "sin(q1+q2+q4)", "cos(q1+q2+q4)"],
    2: ["1", "sin(q1+q2)", "cos(q1+q2)", "sin(q1+q2+q4)", "cos(q1+q2+q4)", "sin(q2)", "cos(q2)"],
    3: ["1", "sin(q4)", "cos(q4)", "sin(q3)*sin(q1+q2)", "cos(q3)*sin(q1+q2)"],
}

def term_value(term, q):
    q1, q2, q3, q4 = q[1], q[2], q[3], q[4]
    s1, c1 = math.sin(q1), math.cos(q1)
    s2, c2 = math.sin(q2), math.cos(q2)
    s3, c3 = math.sin(q3), math.cos(q3)
    s4, c4 = math.sin(q4), math.cos(q4)
    s12, c12 = math.sin(q1+q2), math.cos(q1+q2)
    s124, c124 = math.sin(q1+q2+q4), math.cos(q1+q2+q4)
    table = {
        "1": 1.0, "sin(q1)": s1, "cos(q1)": c1, "sin(q2)": s2, "cos(q2)": c2,
        "sin(q3)": s3, "cos(q3)": c3, "sin(q4)": s4, "cos(q4)": c4,
        "sin(q1+q2)": s12, "cos(q1+q2)": c12,
        "sin(q1+q2+q4)": s124, "cos(q1+q2+q4)": c124,
        "sin(q3)*sin(q1+q2)": s3*s12, "cos(q3)*sin(q1+q2)": c3*s12,
    }
    return table[term]

def build_phi(q_data, terms):
    phi = np.zeros((len(q_data), len(terms)))
    for i, q in enumerate(q_data):
        for j, t in enumerate(terms):
            phi[i, j] = term_value(t, q)
    return phi

def fit_ridge(phi, y, ridge=1e-3):
    penalty = np.ones(phi.shape[1])
    penalty[0] = 0.0  # 不正则化常数项
    normal = phi.T @ phi + ridge * np.diag(penalty)
    rhs = phi.T @ y
    return np.linalg.solve(normal, rhs)

def robust_fit(phi, y, ridge=1e-3, iters=6):
    coeff = fit_ridge(phi, y, ridge)
    for _ in range(iters):
        err = phi @ coeff - y
        mad = np.median(np.abs(err - np.median(err)))
        sigma = 1.4826 * mad
        if sigma < 1e-9: break
        delta = 1.5 * sigma
        w = np.ones_like(y)
        mask = np.abs(err) > delta
        w[mask] = delta / np.abs(err[mask])
        w = np.maximum(0.2, w)
        phi_w = phi * np.sqrt(w)[:, None]
        y_w = y * np.sqrt(w)
        penalty = np.ones(phi.shape[1])
        penalty[0] = 0.0
        coeff = np.linalg.solve(phi_w.T @ phi_w + ridge * np.diag(penalty), phi_w.T @ y_w)
    return coeff

def load_csv(path):
    """读 VOFA 导出的 CSV，列：q1,q2,q4,tau2,tau3,tau4"""
    data = []
    with open(path) as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) < 6: continue
            try:
                vals = [float(x) for x in row[:6]]
                data.append(vals)
            except: pass
    return np.array(data)

def main():
    if len(sys.argv) < 2:
        print("用法: python fit_gravity.py data.csv")
        return
    csv_path = Path(sys.argv[1])
    raw = load_csv(csv_path)
    print(f"读入 {len(raw)} 行")

    # raw: [q1, q2, q4, tau2, tau3, tau4]
    q_data = np.zeros((len(raw), 6))
    q_data[:,1] = raw[:,0]  # q1
    q_data[:,2] = raw[:,1]  # q2
    q_data[:,4] = raw[:,2]  # q4

    for axis in AXES:
        col = {1:3, 2:4, 3:5}[axis]
        y = raw[:, col]
        terms = TERMS[axis]
        phi = build_phi(q_data, terms)
        coeff = robust_fit(phi, y)
        y_pred = phi @ coeff
        rmse = np.sqrt(np.mean((y_pred - y)**2))
        print(f"J{axis}: RMSE={rmse:.4f} Nm")
        for t, c in zip(terms, coeff):
            print(f"  {c:+.4f} * {t}")

if __name__ == "__main__":
    main()
