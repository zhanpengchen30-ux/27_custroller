# 重力补偿采集流程

## 1. STM32 端：VOFA 输出

确认 `vofa_debug.c` 输出这 6 列：
```
q1, q2, q4, tau2_motor, tau3_motor, tau4_motor
```

## 2. 采集步骤

1. 机械臂水平前伸，J5 固定
2. **正扫**：J2 从 -30° 缓慢转到 +90°，J3 固定在一个位置
3. **反扫**：J2 从 +90° 缓慢转回 -30°
4. 换 J3 位置（-45°, 0°, +45°），重复正扫+反扫
5. VOFA 保存 CSV

## 3. 拟合

```
python tools/fit_gravity.py data/xxx.csv
```

自动输出每个关节的拟合系数。

## 4. 替换模型

把拟合结果填回 `gravity_model.c`。
