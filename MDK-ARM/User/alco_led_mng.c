#include <stdio.h>
#include "stm32f10x.h"
#include "STM32vldiscovery.h"
#include "stm32f10x_conf.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_usart.h"
#include "alco_init_periph.h"

/**
  * @brief  Inserts a delay time.
  * @param  nCount: specifies the delay time length.
  * @retval None
  */
void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}

void blink_green(void)
{
	STM32vldiscovery_LEDOff(LED_GREEN);
  Delay(0x1FFFFF);
  STM32vldiscovery_LEDOn(LED_GREEN);
  Delay(0x1FFFFF);
	STM32vldiscovery_LEDOff(LED_GREEN);
  Delay(0x1FFFFF);
  STM32vldiscovery_LEDOn(LED_GREEN);
  Delay(0x1FFFFF);
}

void blink_blue(void)
{
		STM32vldiscovery_LEDOff(LED_BLUE);
    Delay(0x10FFFF);
		STM32vldiscovery_LEDOn(LED_BLUE);
    Delay(0x10FFFF);
}
