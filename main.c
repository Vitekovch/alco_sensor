/**
  ******************************************************************************
  * @file    Examples/GPIOToggle/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    09/13/2010
  * @brief   Main program body.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32f10x.h"
#include "STM32vldiscovery.h"
#include "stm32f10x_conf.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_usart.h"
#include "alco_init_periph.h"
#include "alco_led_mng.h"

/** @addtogroup Examples
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;
uint32_t i = 0, helper = 0;
char str_common[50];
char str_alarm[20] = {0};
uint16_t adcValue[2];
uint8_t enter_flag = 0;
uint8_t sms_flag = 0;
uint16_t input_data;
uint8_t rx_buf[70] = {0};
uint8_t rx_buf_ptr = 0;

/* Private function prototypes -----------------------------------------------*/
void USART1_IRQHandler(void);

void USART_Puts(USART_TypeDef *USARTx, volatile char *s)
{
	while(*s)
	{
		while(!(USARTx->SR & USART_FLAG_TC));
		USART_SendData(USARTx, *s);
		*s++;
	}
}

uint16_t read_adc()
{
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
	return ADC_GetConversionValue(ADC1);
}

void read_adc_inj()
{
  //while(ADC_GetSoftwareStartInjectedConvCmdStatus(ADC1) == SET);
  ADC_SoftwareStartInjectedConvCmd(ADC1, ENABLE);

  //+ Wait until ADC Channel 8 end of conversion
  while (ADC_GetFlagStatus(ADC1, ADC_FLAG_JEOC) == RESET);
	
	adcValue[0] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
	adcValue[1] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2);
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
	sdk_Init();
	USART1_Init();
	USART2_Init();
	ADC1_Init();
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	NVIC_EnableIRQ(USART1_IRQn);

  /* Initialize Leds LD3 (C9) and LD4 (C8) mounted on STM32VLDISCOVERY board */
  STM32vldiscovery_LEDInit(LED_GREEN);
  STM32vldiscovery_LEDInit(LED_BLUE);
	
	Delay(0x1FFFFF);
	snprintf(str_common, sizeof(str_common), "Start\r\n");
	USART_Puts(USART2, str_common);
	Delay(0x3FFFFF);
	
	snprintf(str_common, sizeof(str_common), "%08X %08X\r\n", ADC1->CR1, ADC1->CR2);
	USART_Puts(USART2, str_common);
	
	USART_Puts(USART1, "AT\r\n");
	STM32vldiscovery_LEDOff(LED_GREEN);
	
	helper = snprintf(str_alarm, sizeof(str_alarm), "Alconavt%c", 0x1A);
	USART_Puts(USART2, str_alarm);
	helper = snprintf(str_alarm, sizeof(str_alarm), " %d %02X %02X ", helper, str_alarm[8], str_alarm[9]);
	USART_Puts(USART2, str_alarm);
  while (1)
  {
		blink_blue();
		i++;
		// every 100 cycles check GSM module
		if((i%50) == 0)
		{
			USART_Puts(USART1, "AT+CSQ\r\n");
		}
		read_adc_inj();
		snprintf(str_common, sizeof(str_common), "%04d %04d %04d\r\n", i, adcValue[0], adcValue[1]);
		USART_Puts(USART2, str_common);
		if(adcValue[0] > 4000)
		{
			// for avoidance of multiple sms, gisteresis 
			if(0 == enter_flag)
      {
				//USART_Puts(USART1, "AT+CMGS=\"+79992213151\"\r\n");
				USART_Puts(USART1, "AT+CMGS=\"+79531701527\"\r\n");
				enter_flag = 1;
				sms_flag = 1;
      }
		}
		else
		{
			enter_flag = 0;
		}
		
		if((((rx_buf[rx_buf_ptr-2] == '>') && (rx_buf[rx_buf_ptr-1] == ' ')) && (1 == sms_flag)) ||
			  ((rx_buf[rx_buf_ptr-4] == 'O') && (rx_buf[rx_buf_ptr-3] == 'K') && (rx_buf[rx_buf_ptr-2] == 0x0D) && (rx_buf[rx_buf_ptr-1] == 0x0A)))
		{
			rx_buf[rx_buf_ptr] = '\0';
			USART_Puts(USART2, (char *)rx_buf);
			STM32vldiscovery_LEDOn(LED_GREEN);
			Delay(0x1FFFFF);
			if((rx_buf[rx_buf_ptr-2] == '>') && (rx_buf[rx_buf_ptr-1] == ' '))
			{
				char A = 0x1A;
				blink_green();
				snprintf(str_alarm, sizeof(str_alarm), "Alconavt%c", A);
				USART_Puts(USART1, str_alarm);
				sms_flag = 0;
			}
			rx_buf_ptr = 0;
		}
		else
		{
			STM32vldiscovery_LEDOff(LED_GREEN);
		}
  }
}

void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		rx_buf[rx_buf_ptr] = (uint8_t) USART_ReceiveData(USART1);
		rx_buf_ptr++;
	}
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
