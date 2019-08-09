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
	uint8_t i = 0;
	for (i = 0; i < 5; i++)
	{
		STM32vldiscovery_LEDOn(LED_GREEN);
    Delay(0x0FFFFF);
	  STM32vldiscovery_LEDOff(LED_GREEN);
    Delay(0x0FFFFF);
	}
}

void blink_blue(void)
{
	uint8_t i = 0;
	for (i = 0; i < 12; i++)
	{
		STM32vldiscovery_LEDOn(LED_BLUE);
		STM32vldiscovery_LEDOn(MAIN_BLUE);
    Delay(0x0FFFFF);
	  STM32vldiscovery_LEDOff(LED_BLUE);
		STM32vldiscovery_LEDOff(MAIN_BLUE);
    Delay(0x0FFFFF);
	}
}

void main_delay(void)
{
	Delay(0x04FFFF);
	Delay(0x04FFFF);
}

void pre_main_delay(void)
{
	Delay(0xFFFFFFF);
	Delay(0xFFFFFFF);
}
