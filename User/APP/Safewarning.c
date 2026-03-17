#include "safewarning.h"
#include "ws2812.h"
#include "tim.h"
#include "cmsis_os.h"

uint8_t r = 1;
uint8_t g = 1;
uint8_t b = 1;

void ws2812_task(void){
	WS2812_Ctrl(r, g, b);
    r++;
    g += 5;
    b += 10;
    vTaskDelay(1);
    r++;g++;b++;
    vTaskDelay(100);
}

void beep_test(void)
{
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
	TIM12->CCR2 = 1800;
}

// 音符频率定义（单位：Hz）- 调整为接近4kHz的频段
#define NOTE_1K 1000
#define NOTE_2K 2000
#define NOTE_3K 3000
#define NOTE_4K 4000  // 蜂鸣器谐振频率
#define NOTE_5K 5000  // 可选，但注意定时器能力足够

// 蜂鸣器状态结构体
typedef struct {
    BeepType_t current_type;      // 当前播放的提示音类型
    uint16_t step;                 // 当前步骤
    uint16_t counter;              // 计时计数器
    uint8_t is_playing;            // 是否正在播放
    uint16_t frequency;            // 当前频率
    uint16_t duration;             // 当前音符持续时间
} Beep_State_t;

static Beep_State_t beep_state = {0};

// 提示音音效序列定义
typedef struct {
    uint16_t freq;      // 频率（Hz），0表示静音
    uint16_t duration;  // 持续时间（ms）
} Note_t;

// 开机提示音：从1kHz到4kHz的细致音阶
static const Note_t power_on_melody[] = {
    {3000, 200},   // 滴
	{3500, 200},   // 滴
	{4000, 400},   // 滴）
    {0, 0}
};

// 短按提示音：短促的"滴"（4kHz）
static const Note_t short_press_melody[] = {
    {NOTE_4K, 100},
    {0, 50},
    {0, 0}
};

// 连按提示音：低频长音（使用2kHz和4kHz交替，突出长按感）
static const Note_t long_press_melody[] = {
    {NOTE_2K, 300},
    {0, 100},
    {NOTE_4K, 200},
    {0, 100},
    {NOTE_4K, 400},
    {0, 0}
};

// 错误提示音：急促的下降音（从4kHz到1kHz）
static const Note_t error_melody[] = {
    {NOTE_4K, 100},
    {NOTE_3K, 100},
    {NOTE_2K, 100},
    {NOTE_1K, 200},
    {0, 0}
};

// 成功提示音：欢快的双音（4kHz与5kHz交替，5kHz略高于谐振点但仍在可接受范围）
static const Note_t success_melody[] = {
    {NOTE_4K, 100},
    {0, 50},
    {NOTE_4K, 100},
    {NOTE_5K, 150},
    {NOTE_4K, 200},
    {0, 0}
};

// 获取对应提示音的音符序列
static const Note_t* GetMelody(BeepType_t type) {
    switch(type) {
        case BEEP_POWER_ON:     return power_on_melody;
        case BEEP_SHORT_PRESS:  return short_press_melody;
        case BEEP_DOUBLE_PRESS:   return long_press_melody;
        case BEEP_ERROR:        return error_melody;
        case BEEP_SUCCESS:      return success_melody;
        default:                return NULL;
    }
}

// 设置PWM频率
static void SetBeepFrequency(uint16_t freq) {
    uint32_t tim_freq = 1000000;  // 定时器计数时钟为1MHz（PSC=239）
    uint32_t arr, ccr;
    
    if (freq == 0) {
        HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
        return;
    }
    
    arr = tim_freq / freq;
    ccr = arr / 2;  // 50%占空比
    
    __HAL_TIM_SET_AUTORELOAD(&htim12, arr - 1);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, ccr);
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
}

// 初始化蜂鸣器
void Beep_Init(void) {
    beep_state.is_playing = 0;
    beep_state.step = 0;
    beep_state.counter = 0;
    Beep_Stop();
}

// 播放提示音
void Beep_Play(BeepType_t type) {
    if (beep_state.is_playing) {
        Beep_Stop();
    }
    
    beep_state.current_type = type;
    beep_state.step = 0;
    beep_state.counter = 0;
    beep_state.is_playing = 1;
}

// 停止蜂鸣器
void Beep_Stop(void) {
    HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
    beep_state.is_playing = 0;
    beep_state.step = 0;
    beep_state.counter = 0;
}

// 检查蜂鸣器是否正在播放
uint8_t Beep_IsPlaying(void) {
    return beep_state.is_playing;
}

// 蜂鸣器任务（每1ms调用一次，已在TIM6中断中执行）
void Beep_Task(void) {
    if (!beep_state.is_playing) {
        return;
    }
    
    const Note_t* melody = GetMelody(beep_state.current_type);
    if (melody == NULL) {
        beep_state.is_playing = 0;
        return;
    }
    
    Note_t current_note = melody[beep_state.step];
    
    if (current_note.duration == 0 && current_note.freq == 0) {
        Beep_Stop();
        return;
    }
    
    beep_state.counter++;
    
    if (beep_state.counter == 1) {
        SetBeepFrequency(current_note.freq);
    }
    
    if (beep_state.counter >= current_note.duration) {
        beep_state.step++;
        beep_state.counter = 0;
    }
}