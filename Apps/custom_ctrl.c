#include "custom_ctrl.h"

volatile Custom_Ctrl_t custom_ctrl = {0};

void Custom_Ctrl_Init(void)
{
    for (int i = 0; i < CUSTOM_FRAME_ANGLES; i++)
        custom_ctrl.target_joint[i] = 0.0f;
    custom_ctrl.active = 0;
    custom_ctrl.last_update_time = 0;
    custom_ctrl.online = 0;
}

void Custom_Ctrl_Parse(uint8_t *buf, uint16_t len)
{
    /* Only inspect complete frames ending at the newest byte. Previously every
       byte re-parsed the old rolling buffer, repeatedly refreshing the link
       age from stale data. Validate the tail before publishing any fields. */
    if (len != CUSTOM_FRAME_LEN || buf[0] != CUSTOM_FRAME_HEAD1 ||
        buf[CUSTOM_FRAME_LEN - 1] != CUSTOM_FRAME_TAIL) return;

    float parsed_joint[CUSTOM_FRAME_ANGLES];
    for (int j = 0; j < CUSTOM_FRAME_ANGLES; j++)
    {
        int16_t raw = (int16_t)((buf[1 + j * 2]) | (buf[2 + j * 2] << 8));
        parsed_joint[j] = raw * 0.001f;
    }

    for (int j = 0; j < CUSTOM_FRAME_ANGLES; j++)
        custom_ctrl.target_joint[j] = parsed_joint[j];
    custom_ctrl.active = buf[CUSTOM_FRAME_LEN - 2];
    custom_ctrl.last_update_time = HAL_GetTick();
    custom_ctrl.online = 1;
}
