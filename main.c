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
#define GSM_USART            USART1
#define TM_USART             USART2
#define CONSOLE_USART        USART3

#define ASCII_CODE_26        0x1A

//#define SMS_AT_STRING       "AT+CMGS=\"+79992213151\"\r\n"
#define SMS_AT_STRING       "AT+CMGS=\"+79964978596\"\r\n"

#define PRINT_HARD_DEBUG    0
#define PRODUCTION          1

#define COOLER_TIME         (1500)
#define GSM_REPLY_TIME      (30)
#define TM_UID_OFFSET       (1 + 16)  // byte related to reset/presense (1) + 0x33 command (8) + 0x01 byte (8)
#define TM_UID_LEN          (48)
#define ONE_WIRE_PACKET_LEN (72)  // 1 byte command + 8 byte ID
/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;
uint32_t i = 0;  // main loop increment
// cooler switch on/off variables
uint32_t cooler_counter = 0;
uint8_t cooler_on = 0;
// gsm stability variables
uint32_t gsm_reply_counter = 0;
uint8_t reply_from_gsm_ok_it_enable = 1;

char str_common[100], str_sms[30];      // debug print buf
uint8_t rx_buf[70] = {0};  // buf for UART stream from GSM shield
uint8_t rx_buf_ptr = 0;
uint8_t tm_buf[200] = {0}; // buf for 1-wire sniffer
uint8_t tm_buf_ptr = 0;
// ADC and alcohol calc variables
uint16_t adcValue[3];
double BAC_1, BAC_2, BAC_3;
// 1-wire (TM Dallas) listening variables
uint8_t j = 0, catch_byte = 0, presence_ok = 0, key_ready = 0;
uint32_t key[2], key_for_gsm[2];
uint8_t index = 0;
uint8_t bit_pos = 0;
uint8_t entering = 0;  // note the moment of sending sms with key ID
uint8_t breath_ok = 0;  // type of sms - good or alco
uint8_t card_touch = 0;
// alcohol breath detection logic variables
uint8_t alco_detected = 0, freeze_meas_human_here = 0;
uint8_t sms_flag = 0, breath_allow = 1;
uint8_t good_counter = 0, to_neutral_cnt = 0, candidate = 0, candidate_cnt = 0;
// SMS send buf
char str_alarm[50] = {0};

const int mean_num = 20;
const double alco_critical = 0.225; // mg/L
const double alco_door_open = 0.004; // mg/L

double MQ_1[mean_num];
double MQ_2[mean_num];
double MQ_3[mean_num];
double mean_1, mean_2, mean_3;

/* Private function prototypes -----------------------------------------------*/
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);

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
    USART2_Init(9600);
    USART3_Init(9600);
    ADC1_Init();
    alco_array_init(MQ_1, MQ_2, MQ_3, mean_num);
	
    USART_ITConfig(GSM_USART, USART_IT_RXNE, ENABLE);
    NVIC_EnableIRQ(USART1_IRQn);
    USART_ITConfig(TM_USART, USART_IT_RXNE, ENABLE);
    NVIC_EnableIRQ(USART2_IRQn);
    reply_from_gsm_ok_it_enable = 1;

    /* Initialize Leds LD3 (C9) and LD4 (C8) mounted on STM32VLDISCOVERY board */
    led_init();
    GPIO_Camera_Pin_Init();
	
    snprintf(str_common, sizeof(str_common), "Start\r\n");
    USART_Puts(CONSOLE_USART, str_common);
    USART_Puts(GSM_USART, "AT\r\n");
    STM32vldiscovery_LEDOff(LED_GREEN);
    STM32vldiscovery_LEDOff(MAIN_GREEN);
	
    GSM_Pin_Init();
    #if PRODUCTION
        pre_main_delay();
    #endif
	
    while (1)
    {
        main_delay();
        i++;
        if (i > 4000000000UL)
        {
            i = 100;
        }
		
        if (1 == key_ready)
        {
            for (j = TM_UID_OFFSET; j < (TM_UID_OFFSET + TM_UID_LEN); j++)
            {
                // take bit in the middle 0x08 = 0b00001000
                index = (j - TM_UID_OFFSET) / 32;
                bit_pos = j-TM_UID_OFFSET - (((j - TM_UID_OFFSET) / 32) * 32);
                key[index] |= ((tm_buf[j] & 0x08) >> 3) << bit_pos;
            }
            if (0 != key[0])
            {
                snprintf(str_sms, sizeof(str_sms), "%02X %02X %02X %02X %02X %02X\r\n", (key[1] >> 8) & 0xFF,
                                                                                                  key[1] & 0xFF,
                                                                                                  (key[0] >> 24) & 0xFF,
                                                                                                  (key[0] >> 16) & 0xFF,
                                                                                                  (key[0] >> 8) & 0xFF,
                                                                                                  key[0] & 0xFF);
                USART_Puts(CONSOLE_USART, str_sms);
                #if PRINT_HARD_DEBUG
                    snprintf(str_common, sizeof(str_common), "%02X\r\n", catch_byte);
                    USART_Puts(CONSOLE_USART, str_common);
                #endif
   
                sms_flag = 1;
                entering = 1;
                card_touch = 1;
                USART_Puts(GSM_USART, SMS_AT_STRING);
            }
			
            key_ready = 0;
            tm_buf[0] = 0;
            key[0] = 0;
            key[1] = 0;
        }

        #if PRODUCTION
            if (i == 40) GSM_power_on();
        #endif

        // every 50 cycles check GSM module
        #if PRINT_HARD_DEBUG
            if((i%5000) == 0)
            {
                USART_ITConfig(TM_USART, USART_IT_RXNE, DISABLE);
                reply_from_gsm_ok_it_enable = 0;
                #if PRINT_HARD_DEBUG
                    snprintf(str_common, sizeof(str_common), "SET IT 0 -----\r\n");
                    USART_Puts(CONSOLE_USART, str_common);
                #endif
                USART_Puts(GSM_USART, "AT+CSQ\r\n");
            }
        #endif
        read_adc_inj(adcValue);
        // convert to BAC in mg/L
        calc_alcohol(adcValue, &BAC_1, &BAC_2, &BAC_3);
        calc_mean(&mean_1, &mean_2, &mean_3, mean_num, MQ_1, MQ_2, MQ_3);
        snprintf(str_common, sizeof(str_common), "%04d %04d %04d %04d %f %f %f %f %f %f\r\n", i, adcValue[0], adcValue[1], adcValue[2], BAC_1, BAC_2, BAC_3, mean_1, mean_2, mean_3);
        USART_Puts(CONSOLE_USART, str_common);
        if(((BAC_1 > alco_critical) || (BAC_2 > alco_critical) || (BAC_3 > alco_critical)) && (1 == card_touch))
        {
            STM32vldiscovery_LEDOn(COOLER);
            cooler_on = 1;
            cooler_counter = 0;
			
            to_neutral_cnt = 0;
            candidate = 0;
            breath_allow = 0;
            // for avoidance of multiple sms, gisteresis 
            if(0 == alco_detected)
            {
                snapshot();
                USART_ITConfig(TM_USART, USART_IT_RXNE, DISABLE);
                reply_from_gsm_ok_it_enable = 0;
                #if PRINT_HARD_DEBUG
                    snprintf(str_common, sizeof(str_common), "SET IT 0   +++++\r\n");
                    USART_Puts(CONSOLE_USART, str_common);
                #endif
                snprintf(str_common, sizeof(str_common), "Critical\r\n");
                USART_Puts(CONSOLE_USART, str_common);
                USART_Puts(GSM_USART, SMS_AT_STRING);
                alco_detected = 1;
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
                USART_Puts(CONSOLE_USART, str_common);
            #endif

            candidate_cnt++;
            breath_allow = 0;
            if (5 == candidate_cnt)
            {
                STM32vldiscovery_LEDOn(COOLER);
                USART_ITConfig(TM_USART, USART_IT_RXNE, DISABLE);
                reply_from_gsm_ok_it_enable = 0;
                #if PRINT_HARD_DEBUG
                    snprintf(str_common, sizeof(str_common), "SET IT 0 )))))\r\n");
                    USART_Puts(CONSOLE_USART, str_common);
                #endif
                breath_ok = 1;
                USART_Puts(GSM_USART, SMS_AT_STRING);
                sms_flag = 1;
                cooler_on = 1;
                cooler_counter = 0;
				
                candidate = 0;
                candidate_cnt = 0;
                card_touch = 0;
                door_open();
                blink_green();
                alco_array_init(MQ_1, MQ_2, MQ_3, mean_num);
                snprintf(str_common, sizeof(str_common), "Door open %s\r\n", str_sms);
                USART_Puts(CONSOLE_USART, str_common);
            }
        }
        else if(((((BAC_1 - mean_1) > alco_door_open) && ((BAC_2 - mean_2) > alco_door_open)) ||
                (((BAC_2 - mean_2) > alco_door_open) && ((BAC_3 - mean_3) > alco_door_open)) ||
                (((BAC_1 - mean_1) > alco_door_open) && ((BAC_3 - mean_3) > alco_door_open))) && (1 == card_touch))
        {
            #if PRINT_HARD_DEBUG
                snprintf(str_common, sizeof(str_common), "State 3\r\n");
                USART_Puts(CONSOLE_USART, str_common);
            #endif

            to_neutral_cnt = 0;

            if (0 == alco_detected)
            {
                good_counter++;
                #if PRINT_HARD_DEBUG
                    snprintf(str_common, sizeof(str_common), "good_counter = %d\r\n", good_counter);
                    USART_Puts(CONSOLE_USART, str_common);
                #endif

                if (5 == good_counter)
                {
                    breath_allow = 0;
                    good_counter = 0;
                    candidate = 1;
                }
                freeze_meas_human_here = 1;
            }
        }
        else
        {   
            #if PRINT_HARD_DEBUG
                snprintf(str_common, sizeof(str_common), "Neutral\r\n");
                USART_Puts(CONSOLE_USART, str_common);
            #endif

            to_neutral_cnt++;
            if (15 == to_neutral_cnt)
            {
                alco_detected = 0;
                freeze_meas_human_here = 0;
                to_neutral_cnt = 0;
                good_counter = 0;
                candidate = 0;
                breath_allow = 1;
            }
        }

        green_light_for_next_breath(breath_allow);
        // freeze measurements during breath
        if((0 == freeze_meas_human_here) && (0 == alco_detected))
        {
            push_to_alco_value_buf(MQ_1, MQ_2, MQ_3, mean_num, BAC_1, BAC_2, BAC_3);
        }

        if((((rx_buf[rx_buf_ptr-2] == '>') && (rx_buf[rx_buf_ptr-1] == ' ')) && (1 == sms_flag)) ||
            ((rx_buf[rx_buf_ptr-4] == 'O') && (rx_buf[rx_buf_ptr-3] == 'K') &&
             (rx_buf[rx_buf_ptr-2] == 0x0D) && (rx_buf[rx_buf_ptr-1] == 0x0A)))
        {
            rx_buf[rx_buf_ptr] = '\0';
            USART_Puts(CONSOLE_USART, (char *)rx_buf);
            if((rx_buf[rx_buf_ptr-2] == '>') && (rx_buf[rx_buf_ptr-1] == ' '))
            {
                #if PRINT_HARD_DEBUG
                    blink_green();
                #endif
                if (1 == entering)
                {
                    snprintf(str_alarm, sizeof(str_alarm), "%s%c", str_sms, ASCII_CODE_26);
                    entering = 0;
                }
                else if (1 == breath_ok)
                {
                    snprintf(str_alarm, sizeof(str_alarm), "%s OK%c", str_sms, ASCII_CODE_26);
                    breath_ok = 0;
                }
                else if (1 == alco_detected)
                {
                  snprintf(str_alarm, sizeof(str_alarm), "%s alconavt%c", str_sms, ASCII_CODE_26);
                }
                else
                {
                    snprintf(str_common, sizeof(str_common), "Strange combination\r\n");
                    USART_Puts(CONSOLE_USART, str_common);
                }
                
                USART_Puts(GSM_USART, str_alarm);
                sms_flag = 0;
            }
            rx_buf_ptr = 0;
            USART2_Init(9600);
            USART_ITConfig(TM_USART, USART_IT_RXNE, ENABLE);
            reply_from_gsm_ok_it_enable = 1;
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
        if (reply_from_gsm_ok_it_enable == 0)
        {
            //#if PRINT_HARD_DEBUG
                snprintf(str_common, sizeof(str_common), "IT 0 %d\r\n", gsm_reply_counter);
                USART_Puts(CONSOLE_USART, str_common);
            //#endif
            gsm_reply_counter++;
            if (gsm_reply_counter >= GSM_REPLY_TIME)
            {
                USART2_Init(9600);
                USART_ITConfig(TM_USART, USART_IT_RXNE, ENABLE);
                reply_from_gsm_ok_it_enable = 1;
                gsm_reply_counter = 0; 
            }
        }
        else
        {
            //#if PRINT_HARD_DEBUG
                snprintf(str_common, sizeof(str_common), "IT 1\r\n");
                USART_Puts(CONSOLE_USART, str_common);
            //#endif
            gsm_reply_counter = 0; 
        }
    }
}

void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(GSM_USART, USART_IT_RXNE) != RESET)
    {
        USART_ClearITPendingBit(GSM_USART, USART_IT_RXNE);
        rx_buf[rx_buf_ptr] = (uint8_t) USART_ReceiveData(GSM_USART);
        rx_buf_ptr++;
    }
}

void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(TM_USART, USART_IT_RXNE) != RESET)
    {
        tm_buf[tm_buf_ptr] = (uint8_t) USART_ReceiveData(TM_USART);
        if ((0xE0 != tm_buf[tm_buf_ptr]) &&
            (0xFF != tm_buf[tm_buf_ptr]) &&
            (0x00 != tm_buf[tm_buf_ptr]) &&
            (0x00 == (tm_buf[tm_buf_ptr] & 0x0F)) &&
               (0 == presence_ok))
        {
            catch_byte = tm_buf[tm_buf_ptr];
            tm_buf_ptr++;
            USART2_Init(230400);
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
            USART_ITConfig(TM_USART, USART_IT_RXNE, DISABLE);
            reply_from_gsm_ok_it_enable = 0;
        }
        USART_ClearITPendingBit(TM_USART, USART_IT_RXNE);
    }
}
