#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
用 URDF 独立算一遍重力矩，和固件里的三段模型对比。

原理（纯刚体力学，不需要 Pinocchio）：
    对每个转动关节 i，绕轴 a_i（世界系）转动，会带动远端所有连杆的质心；
    质心 p_j 对 q_i 的偏导 = a_i × (p_j - o_i)
    所以   tau_i = Σ_j  m_j · [a_i × (p_j - o_i)] · g_vec
    其中 g_vec = (0, 0, -9.81)

如果这个脚本算出来的数和固件一致，就说明 URDF 的关节位置/轴向/质量都没写错，
可以直接拿去喂 Pinocchio 做正式辨识。
"""
import re
import math
import sys


def load_urdf(path):
    """极简 URDF 解析：只取 joint 的 origin/axis 和 link 的 mass/com"""
    s = open(path, encoding="utf-8").read()

    links = {}
    for m in re.finditer(r'<link\s+name="([^"]+)"(.*?)</link>', s, re.S):
        name, body = m.group(1), m.group(2)
        im = re.search(r"<inertial>(.*?)</inertial>", body, re.S)
        if not im:
            links[name] = (0.0, (0.0, 0.0, 0.0))
            continue
        ib = im.group(1)
        mass = float(re.search(r'<mass\s+value="([^"]+)"', ib).group(1))
        om = re.search(r'<origin\s+xyz="([^"]+)"', ib)
        com = tuple(float(v) for v in om.group(1).split()) if om else (0.0, 0.0, 0.0)
        links[name] = (mass, com)

    joints = []
    for m in re.finditer(r'<joint\s+name="([^"]+)"\s+type="([^"]+)"(.*?)</joint>', s, re.S):
        name, jtype, body = m.group(1), m.group(2), m.group(3)
        if jtype != "revolute":
            continue
        om = re.search(r'<origin\s+xyz="([^"]+)"', body)
        am = re.search(r'<axis\s+xyz="([^"]+)"', body)
        pm = re.search(r'<parent\s+link="([^"]+)"', body)
        cm = re.search(r'<child\s+link="([^"]+)"', body)
        joints.append({
            "name": name,
            "xyz": tuple(float(v) for v in om.group(1).split()) if om else (0, 0, 0),
            "axis": tuple(float(v) for v in am.group(1).split()) if am else (0, 0, 1),
            "parent": pm.group(1),
            "child": cm.group(1),
        })

    # 按 parent->child 串成一条链
    chain = []
    cur = "base_link"
    while True:
        nxt = next((j for j in joints if j["parent"] == cur), None)
        if nxt is None:
            break
        chain.append(nxt)
        cur = nxt["child"]
    return links, chain


def mat_mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mat_vec(A, v):
    return tuple(sum(A[i][k] * v[k] for k in range(3)) for i in range(3))


def rot_axis(axis, th):
    """绕单位轴 axis 旋转 th（罗德里格公式）"""
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-9:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    x, y, z = x / n, y / n, z / n
    c, s = math.cos(th), math.sin(th)
    C = 1 - c
    return [
        [c + x * x * C,     x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C,     y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ]


def gravity_torque(links, chain, q, g=9.80665):
    """算出每个关节的重力力矩（世界系 z 向上，重力为 -z）"""
    n = len(chain)
    # 正运动学：每个 link 的位姿
    R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    pos = (0.0, 0.0, 0.0)
    frames = []          # (R, 位置, 关节世界轴, 关节世界位置)
    for i, j in enumerate(chain):
        # 关节原点相对父 link
        p_joint = tuple(pos[k] + sum(R[k][m] * j["xyz"][m] for m in range(3)) for k in range(3))
        a_world = mat_vec(R, j["axis"])
        # 绕关节轴转 q[i]
        Rj = rot_axis(j["axis"], q[i])
        R = mat_mul(R, Rj)
        pos = p_joint
        frames.append((R[:], pos, a_world, p_joint))

    # 收集质心世界坐标（记下它属于第几个 link，只对远端求和）
    coms = []
    for i, j in enumerate(chain):
        m, com = links.get(j["child"], (0.0, (0, 0, 0)))
        if m <= 1e-6:
            continue
        Rw, pw, _, _ = frames[i]
        pc = tuple(pw[k] + sum(Rw[k][t] * com[t] for t in range(3)) for k in range(3))
        coms.append((i, m, pc))

    # tau_i = Σ_{j 在 i 远端} m_j [(a_i × (p_j - o_i)) · g]
    # 近端连杆不随 q_i 转动，偏导恒为 0，必须排除，否则会算出虚高的力矩
    tau = []
    gvec = (0.0, 0.0, -g)
    for i in range(n):
        _, _, a, o = frames[i]
        tot = 0.0
        for idx, m, p in coms:
            if idx < i:
                continue
            r = (p[0] - o[0], p[1] - o[1], p[2] - o[2])
            cr = (a[1] * r[2] - a[2] * r[1],
                  a[2] * r[0] - a[0] * r[2],
                  a[0] * r[1] - a[1] * r[0])
            tot += m * (cr[0] * gvec[0] + cr[1] * gvec[1] + cr[2] * gvec[2])
        tau.append(tot)
    return tau


def firmware_model(thA, thB, thC):
    """固件里 gravity_model.c 的三段模型（scale=1.0，无 scale 修正）"""
    G, mA, LA, mB, LB, mC, LC = 9.80665, 0.354, 0.150, 0.811, 0.150, 0.445, 0.082
    lcA, lcB, lcC = 0.075, 0.075, 0.041
    cA, cB, cC = (math.cos(math.radians(x)) for x in (thA, thB, thC))
    t2 = G * (mA * lcA * cA
              + mB * (LA * cA + lcB * cB)
              + mC * (LA * cA + LB * cB + lcC * cC))
    t3 = G * (mB * lcB * cB + mC * (LB * cB + lcC * cC))
    t5 = G * mC * lcC * cC
    return t2, t3, t5


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "tools/your_arm.urdf"
    links, chain = load_urdf(path)
    print("URDF: %s" % path)
    print("链上转动关节 %d 个: %s" % (len(chain), " -> ".join(j["name"] for j in chain)))
    print()

    cases = [
        ("全部水平（伸直）", 0, 0, 0),
        ("伸直抬 45°", 45, 45, 45),
        ("大臂 50° 前臂 -40° 末端 -100°", 50, -40, -100),
    ]
    print("%-34s %-22s %-22s" % ("姿态", "URDF 算的 [J2,J3,J5]", "固件模型 [J2,J3,J5]"))
    print("-" * 82)
    for name, a, b, c in cases:
        # 关节角：J1=0, J2=a, J3=-(b-a) 因为 θB = θA - q3, J4=0, J5=c-b, J6=0
        q1 = 0.0
        q2 = math.radians(a)
        q3 = math.radians(a - b)
        q4 = 0.0
        q5 = math.radians(c - b)
        q6 = 0.0
        tau = gravity_torque(links, chain, [q1, q2, q3, q4, q5, q6])
        fw = firmware_model(a, b, c)
        print("%-34s %6.3f %7.3f %7.3f   %6.3f %7.3f %7.3f" % (
            name, abs(tau[1]), abs(tau[2]), abs(tau[4]), *fw))
    print()
    print("两列接近 => URDF 的关节位置/轴向/质量与固件模型一致")
    print("差距大      => 改 URDF 里的 axis 符号或 origin，而不是改固件")


if __name__ == "__main__":
    main()
