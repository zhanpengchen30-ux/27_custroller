#include "gravity_model.h"
#include <math.h>

#define G_CONST 9.80665f

volatile float g_grav_m2     = 0.38f;
volatile float g_grav_lc2    = 0.09f;
volatile float g_grav_l2     = 0.16f;

volatile float g_grav_m3     = 0.65f;
volatile float g_grav_lc3    = 0.10f;
volatile float g_grav_l3     = 0.16f;

volatile float g_grav_m_end  = 0.45f;
volatile float g_grav_lc_end = 0.08f;

volatile float g_gravity_zero[6] = {0, 0, 0, 0, 0, 0};
volatile float g_sign_q3 = -1.0f;

void Arm_CalcGravityTorque(float q[6], float tau_g[6])
{
    float th2  = q[1];
    float th23 = q[1] + g_sign_q3 * q[2];

    float c2  = cosf(th2);
    float c23 = cosf(th23);

    tau_g[0] = 0.0f;

    tau_g[1] =
        g_grav_m2 * G_CONST * g_grav_lc2 * c2
      + g_grav_m3 * G_CONST * (g_grav_l2 * c2 + g_grav_lc3 * c23)
      + g_grav_m_end * G_CONST * (g_grav_l2 * c2 + g_grav_l3 * c23);

    tau_g[2] =
        g_grav_m3 * G_CONST * g_grav_lc3 * c23
      + g_grav_m_end * G_CONST * g_grav_l3 * c23;

    tau_g[3] = 0.0f;
    tau_g[4] = 0.0f;
    tau_g[5] = 0.0f;
}