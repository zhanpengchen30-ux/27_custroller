#ifndef JOINT_MAPPING_H
#define JOINT_MAPPING_H

#include "dm_j4310.h"

void Joint_UpdateState(DM_J4310_t *motors, float q_out[6], float v_out[6]);
void Joint_ApplyTorque(DM_J4310_t *motors, float tau_cmd[6]);

#endif
