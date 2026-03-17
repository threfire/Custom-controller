#ifndef __SAFEWARNING_H__
#define __SAFEWARNING_H__

#include "main.h"

void ws2812_task(void);
// 提示音类型枚举
typedef enum {
    BEEP_POWER_ON = 0,    // 开机提示音
    BEEP_SHORT_PRESS,      // 短按提示音
    BEEP_DOUBLE_PRESS,       // 连按提示音
    BEEP_ERROR,            // 错误提示音
    BEEP_SUCCESS           // 成功提示音
} BeepType_t;

// 初始化蜂鸣器
void Beep_Init(void);

// 播放提示音
void Beep_Play(BeepType_t type);

// 停止蜂鸣器
void Beep_Stop(void);

// 检查蜂鸣器是否正在播放
uint8_t Beep_IsPlaying(void);

// 蜂鸣器任务（需要在主循环或定时器中调用）
void Beep_Task(void);

#endif
