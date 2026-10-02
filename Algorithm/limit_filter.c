#include "limit_filter.h"

float Math_Clamp(float val, float min, float max) {
    if (val > max) return max;
    if (val < min) return min;
    return val;
}

// 力矩斜坡函数：防止力矩突变打手打坏 3D 打印件
float Math_Ramp(float target, float current, float step_max) {
    float diff = target - current;
    if (diff > step_max)  return current + step_max;
    if (diff < -step_max) return current - step_max;
    return target;
}

float Math_LowPass(float current, float last, float alpha) {
    return (1.0f - alpha) * last + alpha * current;
}
