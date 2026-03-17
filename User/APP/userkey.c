#include "stm32h7xx.h"
#include "userkey.h"
#include "safewarning.h"
#include "servo_mapping.h"

uint8_t Key_Num = 0;
uint8_t DoubleClickFlag = 0;
static uint8_t StableState = 0;
extern moterMapHeader   MoterMap;
#define DOUBLE_CLICK_TIMEOUT  500  // 双击时间阈值（ms），可调整
#define DEBOUNCE_TIME_MS       4   // 消抖时间（ms）

static uint32_t last_press_time = 0;
static uint32_t tick_counter = 0;   // 1ms递增计数器

// 读取按键原始电平（按下为0，返回1表示按下）
static uint8_t Key_GetRawState(void)
{
    return (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET) ? 1 : 0;
}

// 按键扫描函数，每1ms调用一次
void Key_Tick(void)
{
    static uint8_t Count;
    static uint8_t PrevRaw, CurrRaw;
    static uint8_t DebouncedState;

    tick_counter++;

    // 消抖：连续采样一致才更新状态
    CurrRaw = Key_GetRawState();
    if (CurrRaw != PrevRaw)
    {
        Count = 0;
        PrevRaw = CurrRaw;
    }
    else
    {
        Count++;
        if (Count >= DEBOUNCE_TIME_MS)  // 连续4ms一致
        {
            Count = 0;
            if (DebouncedState != CurrRaw)
            {
                DebouncedState = CurrRaw;
                StableState = DebouncedState;  // 更新稳定状态

                // 检测上升沿（按下）
                if (DebouncedState == 1)
                {
                    // 每次按下都触发蜂鸣器事件
                    Key_Num = 1;

                    // 双击检测：判断与上次按下的时间差
                    uint32_t diff = tick_counter - last_press_time;
                    if (diff > 10 && diff < DOUBLE_CLICK_TIMEOUT)  // 忽略过短（防抖）和超长
                    {
                        DoubleClickFlag = 1;  // 设置双击标志
                    }
                    last_press_time = tick_counter;
                }
            }
        }
    }
}

// 获取单次按下事件（供蜂鸣器）
uint8_t Key_GetNum(void)
{
    uint8_t Temp = 0;
    if (Key_Num)
    {
        Temp = Key_Num;
        Key_Num = 0;
    }
    return Temp;
}

// 获取双击事件（供其他功能）
uint8_t Key_GetDoubleClick(void)
{
    uint8_t flag = 0;
    if (DoubleClickFlag)
    {
        flag = DoubleClickFlag;
        DoubleClickFlag = 0;
    }
    return flag;
}

// 获取当前稳定按键状态（1=按下，0=释放）
uint8_t Key_GetStableState(void)
{
    return StableState;
}

// 按键与蜂鸣器处理接口（在 LedTask 中调用）
void Key_Beep_Handler(void)
{
    uint8_t key = Key_GetNum();
    if (key == 1)
    {
        Beep_Play(BEEP_SHORT_PRESS);  // 播放短按提示音
    }
	if(Key_GetStableState() == 1)
	{
		MoterMap.clamp = 1;
	}
	else if(Key_GetStableState() == 0)
	{
		MoterMap.clamp = 0;
	}
	if (Key_GetDoubleClick() == 1)
    {
        Beep_Play(BEEP_DOUBLE_PRESS);  // 播放短按提示音
		MoterMap.online = !MoterMap.online;
		
    }
}