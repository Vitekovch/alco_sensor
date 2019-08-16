// Triple alcosensor

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32f10x.h"
#include "STM32vldiscovery.h"
#include "stm32f10x_conf.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_usart.h"
#include "alco_init_periph.h"
#include "alco_led_mng.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define PRINT_HARD_DEBUG    0 

#define COOLER_TIME         (1500)
#define TM_UID_OFFSET       (1 + 16)  // byte related to reset/presense (1) + 0x33 command (8) + 0x01 byte (8)
#define TM_UID_LEN          (32)
#define ONE_WIRE_PACKET_LEN (72)  // 1 byte command + 8 byte ID
/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;
uint32_t i = 0;  // main loop increment
// cooler switch on/off variables
uint32_t cooler_counter = 0;
uint8_t cooler_on = 0;

char str_common[100];      // debug print buf
uint8_t rx_buf[70] = {0};  // buf for UART stream from GSM shield
uint8_t rx_buf_ptr = 0;
uint8_t tm_buf[200] = {0}; // buf for 1-wire sniffer
uint8_t tm_buf_ptr = 0;
// ADC and alcohol calc variables
uint16_t adcValue[3];
double BAC_1, BAC_2, BAC_3;
// 1-wire (TM Dallas) listening variables
uint8_t j = 0, catch_byte = 0, presence_ok = 0, key_ready = 0;
uint32_t key, key_for_gsm;
// alcohol breath detection logic variables
uint8_t enter_flag_critical = 0, enter_flag = 0;
uint8_t sms_flag = 0, breath_allow = 1;
uint8_t good_counter = 0, to_neutral_cnt = 0, candidate = 0, candidate_cnt = 0;
// SMS send buf
char str_alarm[20] = {0};

const int mean_num = 20;
const double alco_critical = 0.225; // mg/L
const double alco_door_open = 0.004; // mg/L

double MQ_1[mean_num];
double MQ_2[mean_num];
double MQ_3[mean_num];
double mean_1, mean_2, mean_3;

/* Private function prototypes -----------------------------------------------*/
void USART1_IRQHandler(void);
void USART3_IRQHandler(void);

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
	  USART3_Init(9600);
	  ADC1_Init();
	  alco_array_init(MQ_1, MQ_2, MQ_3, mean_num);
	
	  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	  NVIC_EnableIRQ(USART1_IRQn);
	  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	  NVIC_EnableIRQ(USART3_IRQn);

    /* Initialize Leds LD3 (C9) and LD4 (C8) mounted on STM32VLDISCOVERY board */
    led_init();
	  GPIO_Camera_Pin_Init();
	  GPIO_Door_Pin_Init();
	
	  snprintf(str_common, sizeof(str_common), "Start\r\n");
	  USART_Puts(USART2, str_common);
	  USART_Puts(USART1, "AT\r\n");
	  STM32vldiscovery_LEDOff(LED_GREEN);
	  STM32vldiscovery_LEDOff(MAIN_GREEN);
	
	  GSM_Pin_Init();
	  //pre_main_delay();
	
    while (1)
    {
		    main_delay();
			  i++;
		
		    if (1 == key_ready)
		    {
			      for (j = TM_UID_OFFSET; j < (TM_UID_OFFSET + TM_UID_LEN); j++)
			      {
							  // take bit in the middle 0x08 = 0b00001000
				        key |= ((tm_buf[j] & 0x08) >> 3) << (j-TM_UID_OFFSET);
			      }
			      if (0 != key)
			      {
				        snprintf(str_common, sizeof(str_common), "--------\r\n%d\r\n--------\r\n", key);
		            USART_Puts(USART2, str_common);
							  #if PRINT_HARD_DEBUG
			              snprintf(str_common, sizeof(str_common), "%02X\r\n", catch_byte);
	                  USART_Puts(USART2, str_common);
					      #endif
				        key_for_gsm = key;
			      }
			
			      USART3_Init(9600);
			      USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
			
			      key_ready = 0;
			      tm_buf[0] = 0;
			      key = 0;
		    }
				
				if (i == 40) GSM_power_on();
		
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
					  #if PRINT_HARD_DEBUG
			          snprintf(str_common, sizeof(str_common), "Candidate\r\n");
	              USART_Puts(USART2, str_common);
					  #endif
					  
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
				        snprintf(str_common, sizeof(str_common), "Door open %d\r\n", key_for_gsm);
	              USART_Puts(USART2, str_common);
			      }
		    }
		    else if((((BAC_1 - mean_1) > alco_door_open) && ((BAC_2 - mean_2) > alco_door_open)) ||
			          (((BAC_2 - mean_2) > alco_door_open) && ((BAC_3 - mean_3) > alco_door_open)) ||
		            (((BAC_1 - mean_1) > alco_door_open) && ((BAC_3 - mean_3) > alco_door_open)))
		    {
					  #if PRINT_HARD_DEBUG
			          snprintf(str_common, sizeof(str_common), "State 3\r\n");
	              USART_Puts(USART2, str_common);
					  #endif
					  
			      to_neutral_cnt = 0;
			
			      if (0 == enter_flag_critical)
			      {
				        good_counter++;
							  #if PRINT_HARD_DEBUG
				            snprintf(str_common, sizeof(str_common), "good_counter = %d\r\n", good_counter);
	                  USART_Puts(USART2, str_common);
							  #endif
							  
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
					  #if PRINT_HARD_DEBUG
		            snprintf(str_common, sizeof(str_common), "Neutral\r\n");
	              USART_Puts(USART2, str_common);
					  #endif
					  
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
			      ((rx_buf[rx_buf_ptr-4] == 'O') && (rx_buf[rx_buf_ptr-3] == 'K') &&
		         (rx_buf[rx_buf_ptr-2] == 0x0D) && (rx_buf[rx_buf_ptr-1] == 0x0A)))
	      {
		        rx_buf[rx_buf_ptr] = '\0';
		        USART_Puts(USART2, (char *)rx_buf);
		        if((rx_buf[rx_buf_ptr-2] == '>') && (rx_buf[rx_buf_ptr-1] == ' '))
		        {
			          char A = 0x1A;
							  #if PRINT_HARD_DEBUG
			              blink_green();
							  #endif
			          snprintf(str_alarm, sizeof(str_alarm), "%d alconavt%c", key_for_gsm, A);
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

void USART3_IRQHandler(void)
{
	  if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	  {
		    tm_buf[tm_buf_ptr] = (uint8_t) USART_ReceiveData(USART3);
		    if ((0xE0 != tm_buf[tm_buf_ptr]) && (0xFF != tm_buf[tm_buf_ptr]) && (0x00 != tm_buf[tm_buf_ptr]) && (0 == presence_ok))
		    {
			      catch_byte = tm_buf[tm_buf_ptr];
			      tm_buf_ptr++;
			      USART3_Init(230400);
			      presence_ok = 1;
		    }
		    else if (1 == presence_ok)
		    {
			      tm_buf_ptr++;
		    }
		    if ((1 == presence_ok) && (ONE_WIRE_PACKET_LEN == tm_buf_ptr))
		    {
			      presence_ok = 0;
			      tm_buf_ptr = 0;
			      key_ready = 1;
			      USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
			     //USART3_Init(9600);
		    }
		    USART_ClearITPendingBit(USART3, USART_IT_RXNE);
	  }
}
