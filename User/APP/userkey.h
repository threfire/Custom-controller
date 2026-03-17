#ifndef __USERKEY_H__
#define __USERKEY_H__

#include "main.h"

void Key_Beep_Handler(void);
uint8_t Key_GetNum(void);
void Key_Tick(void);
uint8_t Key_GetStableState(void);
uint8_t Key_GetDoubleClick(void);   // 获取双击事件

#endif