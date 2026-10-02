#include "joint_mapping.h"

void Joint_UpdateState(DM_J4310_t *motors, float q_out[6], float v_out[6]) {
    for (int i = 0; i < 6; i++) {
        q_out[i] = motors[i].pos;
        v_out[i] = motors[i].vel;
    }
}

void Joint_ApplyTorque(DM_J4310_t *motors, float tau_cmd[6]) {
    for (int i = 0; i < 6; i++) {
        motors[i].t_ff = tau_cmd[i];
    }
}
