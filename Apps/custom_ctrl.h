#ifndef __CUSTOM_CTRL_H
#define __CUSTOM_CTRL_H

#include "main.h"

// 接收帧格式（与主手发送/从手 h723 接收协议一致，15字节）
// 0xAA + 6个int16角度(小端, 单位0.001rad) + active(1字节, deadman) + 0x55
#define CUSTOM_FRAME_HEAD1   0xAA
#define CUSTOM_FRAME_TAIL    0x55
#define CUSTOM_FRAME_ANGLES  6
#define CUSTOM_FRAME_LEN     15   // 1+12+1+1

// 自定义控制数据
typedef struct {
    float    target_joint[6];    // 收到的6关节目标角度 (rad)
    uint8_t  active;             // deadman使能标志
    uint32_t last_update_time;   // 最后一帧时间 (ms)
    uint8_t  online;             // 在线标志
} Custom_Ctrl_t;

extern volatile Custom_Ctrl_t custom_ctrl;

// 初始化（清状态）
void Custom_Ctrl_Init(void);

// 解析函数：把 USART6 (PG9) 收到的一帧自定义控制器字节流喂进来
void Custom_Ctrl_Parse(uint8_t *buf, uint16_t len);

#endif
