#include "safewarning.h"
#include "ws2812.h"
uint8_t r = 1;
uint8_t g = 1;
uint8_t b = 1;

void ws2812_task(void){
	WS2812_Ctrl(r, g, b);
      r++;
      g += 5;
      b += 10;
      HAL_Delay(1);
      r++;g++;b++;
      HAL_Delay(100);
}
