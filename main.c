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
#include "SysTick.h"

#define COOLER_TIME     (1500)

/** @addtogroup Examples
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;
uint32_t i = 0, cooler_counter = 0;
char str_common[100];
uint8_t rx_buf[70] = {0};
uint8_t rx_buf_ptr = 0;
uint16_t adcValue[3];
double BAC_1, BAC_2, BAC_3;
uint8_t enter_flag_critical = 0, enter_flag = 0, presense_ok = 0, key_ready = 0;
uint8_t sms_flag = 0, breath_allow = 1, cooler_on = 0;
uint16_t input_data;
uint8_t good_counter = 0, to_neutral_cnt = 0, candidate = 0, candidate_cnt = 0;
char str_alarm[20] = {0};

const int mean_num = 20;
const double alco_critical = 0.225; // mg/L
const double alco_door_open = 0.004; // mg/L

double MQ_1[mean_num];
double MQ_2[mean_num];
double MQ_3[mean_num];
double mean_1, mean_2, mean_3;

uint32_t ext_dallas_1 = 0, ext_dallas_2 = 0, dallas_init = 0, tm_data[3] = {0}, key;
uint8_t ext_dallas_enter_flag = 0, tm_data_pos = 0;

volatile uint32_t nop_cnt = 0, tm_cnt = 0, remember_index = 0;

/* Private function prototypes -----------------------------------------------*/
void USART1_IRQHandler(void);

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
	alco_array_init(MQ_1, MQ_2, MQ_3, mean_num);
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	NVIC_EnableIRQ(USART1_IRQn);

  /* Initialize Leds LD3 (C9) and LD4 (C8) mounted on STM32VLDISCOVERY board */
  led_init();
	GPIO_Camera_Pin_Init();
	GPIO_Door_Pin_Init();
	
	snprintf(str_common, sizeof(str_common), "Start\r\n");
	USART_Puts(USART2, str_common);
	//USART_Puts(USART1, "AT\r\n");
	STM32vldiscovery_LEDOff(LED_GREEN);
	STM32vldiscovery_LEDOff(MAIN_GREEN);
	
	GSM_Pin_Init();
	
	if(!SysTickInit(100000, 0))
        while(1);
	
	STM32vldiscovery_PBInit(BUTTON_USER, BUTTON_MODE_EXTI);
	
	//pre_main_delay();
	
  while (1)
  {
		main_delay();
		
		if ((1 == key_ready) && ((i - remember_index) > 3))
		{
			key_ready = 0;
			key = (tm_data[1] << 16) | (tm_data[0] >> 16); 
			snprintf(str_common, sizeof(str_common), "%08X %08X %08X %d\r\n", tm_data[0], tm_data[1], tm_data[2], key);
		  USART_Puts(USART2, str_common);
			tm_data[0] = 0;
			tm_data[1] = 0;
			tm_data[2] = 0;
			remember_index = i;
		}
		else
		{
			key_ready = 0;
		}
		
		if (i == 40) GSM_power_on();
		
		i++;
		// every 50 cycles check GSM module
		if((i%50) == 0)
		{
			USART_Puts(USART1, "AT+CSQ\r\n");
		}
		read_adc_inj(adcValue);
		// convert to BAC in mg/L
		calc_alcohol(adcValue, &BAC_1, &BAC_2, &BAC_3);
		calc_mean(&mean_1, &mean_2, &mean_3, mean_num, MQ_1, MQ_2, MQ_3);
		snprintf(str_common, sizeof(str_common), "%04d %04d %04d %04d %f %f %f %f %f %f\r\n", i, adcValue[0], adcValue[1], adcValue[2], BAC_1, BAC_2, BAC_3, mean_1, mean_2, mean_3);
		USART_Puts(USART2, str_common);
		if((BAC_1 > alco_critical) || (BAC_2 > alco_critical) || (BAC_3 > alco_critical))
		{
			STM32vldiscovery_LEDOn(COOLER);
			cooler_on = 1;
			cooler_counter = 0;
			
			to_neutral_cnt = 0;
			candidate = 0;
			breath_allow = 0;
			// for avoidance of multiple sms, gisteresis 
			if(0 == enter_flag_critical)
      {
				snapshot();
				snprintf(str_common, sizeof(str_common), "Critical\r\n");
	      USART_Puts(USART2, str_common);
				USART_Puts(USART1, "AT+CMGS=\"+79992213151\"\r\n");
				//USART_Puts(USART1, "AT+CMGS=\"+79531701527\"\r\n");
				enter_flag_critical = 1;
				sms_flag = 1;
				STM32vldiscovery_LEDOff(LED_GREEN);
				STM32vldiscovery_LEDOff(MAIN_GREEN);
				blink_blue();
      }
		}
		else if (1 == candidate)
		{
			//snprintf(str_common, sizeof(str_common), "Candidate\r\n");
	    //USART_Puts(USART2, str_common);
			candidate_cnt++;
			breath_allow = 0;
			if (5 == candidate_cnt)
			{
				STM32vldiscovery_LEDOn(COOLER);
				cooler_on = 1;
				cooler_counter = 0;
				
				candidate = 0;
				candidate_cnt = 0;
				door_open();
				blink_green();
				alco_array_init(MQ_1, MQ_2, MQ_3, mean_num);
				snprintf(str_common, sizeof(str_common), "Door open\r\n");
	      USART_Puts(USART2, str_common);
			}
		}
		else if((((BAC_1 - mean_1) > alco_door_open) && ((BAC_2 - mean_2) > alco_door_open)) ||
			      (((BAC_2 - mean_2) > alco_door_open) && ((BAC_3 - mean_3) > alco_door_open)) ||
		        (((BAC_1 - mean_1) > alco_door_open) && ((BAC_3 - mean_3) > alco_door_open)))
		{
			//snprintf(str_common, sizeof(str_common), "State 3\r\n");
	    //USART_Puts(USART2, str_common);
			to_neutral_cnt = 0;
			
			if (0 == enter_flag_critical)
			{
				good_counter++;
				//snprintf(str_common, sizeof(str_common), "good_counter = %d\r\n", good_counter);
	      //USART_Puts(USART2, str_common);
			  if (5 == good_counter)
			  {
					breath_allow = 0;
				  good_counter = 0;
					candidate = 1;
			  }
			  enter_flag = 1;
			}
		}
		else
		{
			//snprintf(str_common, sizeof(str_common), "Neutral\r\n");
	    //USART_Puts(USART2, str_common);
			to_neutral_cnt++;
			if (15 == to_neutral_cnt)
			{
				enter_flag_critical = 0;
			  enter_flag = 0;
			  to_neutral_cnt = 0;
				good_counter = 0;
				candidate = 0;
				breath_allow = 1;
			}
		}
		
		green_light_for_next_breath(breath_allow);
		
		if((0 == enter_flag) && (0 == enter_flag_critical))
    {
	    math(MQ_1, MQ_2, MQ_3, mean_num, BAC_1, BAC_2, BAC_3);
	  }
	
    if((((rx_buf[rx_buf_ptr-2] == '>') && (rx_buf[rx_buf_ptr-1] == ' ')) && (1 == sms_flag)) ||
			  ((rx_buf[rx_buf_ptr-4] == 'O') && (rx_buf[rx_buf_ptr-3] == 'K') && (rx_buf[rx_buf_ptr-2] == 0x0D) && (rx_buf[rx_buf_ptr-1] == 0x0A)))
	  {
		  rx_buf[rx_buf_ptr] = '\0';
		  USART_Puts(USART2, (char *)rx_buf);
		  if((rx_buf[rx_buf_ptr-2] == '>') && (rx_buf[rx_buf_ptr-1] == ' '))
		  {
			  char A = 0x1A;
			  //blink_green();
			  snprintf(str_alarm, sizeof(str_alarm), "Alconavt%c", A);
			  USART_Puts(USART1, str_alarm);
			  sms_flag = 0;
		  }
		  rx_buf_ptr = 0;
	  }
		
		if (cooler_on == 1)
		{
			cooler_counter++;
			if (cooler_counter == COOLER_TIME)
			{
				cooler_on = 0;
				cooler_counter = 0;
				STM32vldiscovery_LEDOff(COOLER);
			}
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

/**
  * @brief  This function handles External line0 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI4_IRQHandler(void)
{
  if(EXTI_GetITStatus(USER_BUTTON_EXTI_LINE) != RESET)
  {
		if (0 == presense_ok)
		{
			if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4) == 0)
		  {
			  /* Toggle LED3 */
		    if (0 == ext_dallas_enter_flag)
		    {
			    ext_dallas_1 = SysTickGet();
			    ext_dallas_enter_flag = 1;
		    }
		    else
		    {
			    ext_dallas_2 = SysTickGet();
			    if ((ext_dallas_2 - ext_dallas_1) < 85)
			    {
					  ext_dallas_enter_flag = 0;
					  presense_ok = 1;
						set_pseudo_mutex(1);
			    }
			    else
				  {
					  ext_dallas_1 = ext_dallas_2;
					  ext_dallas_enter_flag = 1;
				  }
		    }
    /* Clear the User Button EXTI line pending bit */
		  }
		}
		else if (1 == presense_ok)
		{
      Delay(0x0000000F);
			if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4) != 0)
			{
				tm_data[tm_data_pos] |= 0x00000001 << tm_cnt;
			}
			
			if ((tm_data_pos == 2) && (tm_cnt == 7))
			{
				presense_ok = 0;
				key_ready = 1;
				tm_data_pos = 0;
				tm_cnt = 0;
				set_pseudo_mutex(0);
				/*if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0)
			  {
				  STM32vldiscovery_LEDOn(LED_BLUE);
			  }
			  else
			  { 
				  STM32vldiscovery_LEDOff(LED_BLUE);
			  }*/
			}
			else
			{
				tm_cnt++;
				if (32 == tm_cnt)
			  {
				  tm_cnt = 0;
				  tm_data_pos++;
			  }
			}
		}
		
    
    EXTI_ClearITPendingBit(USER_BUTTON_EXTI_LINE);
    
  }
}
