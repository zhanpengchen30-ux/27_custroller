#ifndef LIMIT_FILTER_H
#define LIMIT_FILTER_H

float Math_Clamp(float val, float min, float max);
float Math_Ramp(float target, float current, float step_max);
float Math_LowPass(float current, float last, float alpha);

#endif
