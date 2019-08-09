#ifndef ALCO_LED_MNG_H
#define ALCO_LED_MNG_H

#include "stm32f10x.h"

void Delay(__IO uint32_t nCount);
void blink_green(void);
void blink_blue(void);
void main_delay(void);
void pre_main_delay(void);

#endif
