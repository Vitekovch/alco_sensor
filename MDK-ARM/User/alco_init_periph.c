#include <stdio.h>
#include "stm32f10x.h"
#include "STM32vldiscovery.h"
#include "stm32f10x_conf.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_usart.h"
#include "alco_init_periph.h"
#include "alco_led_mng.h"
#include "math.h"

static GPIO_InitTypeDef GPIO_InitStructure;
static USART_InitTypeDef USART_InitStruct;

static const double ADC_up_level = 3.04;
static const double R2 = 1000;
static const double R0_1 = 196.7;
static const double R0_2 = 552;
static const double R0_3 = 350;
static const int    ADC_resolution = 4096;
static uint8_t wrong_key_avoid_flag = 0;

void sdk_Init(void)
{
	/*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */     
       
  /* Configure all unused GPIO port pins in Analog Input mode (floating input
     trigger OFF), this will reduce the power consumption and increase the device
     immunity against EMI/EMC *************************************************/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                         RCC_APB2Periph_GPIOE, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                         RCC_APB2Periph_GPIOE, DISABLE);
}

void ADC1_Init(void)
{
	ADC_InitTypeDef ADC_InitStruct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

  ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_ExternalTrigConv = DISABLE;
	ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStruct.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStruct.ADC_NbrOfChannel = 3;
	ADC_InitStruct.ADC_ScanConvMode = ENABLE;
	ADC_Init(ADC1, &ADC_InitStruct);
	ADC_Cmd(ADC1, ENABLE);
	
	 ADC_InjectedSequencerLengthConfig(ADC1, 3);
 //+ ADC1 Injected Channel Config
   ADC_InjectedChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);
   ADC_InjectedChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_239Cycles5);
	 ADC_InjectedChannelConfig(ADC1, ADC_Channel_4, 3, ADC_SampleTime_239Cycles5);
	//ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
	
	ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_None);
	
	//ADC_AutoInjectedConvCmd( ADC1, ENABLE );
  //ADC_SoftwareStartInjectedConvCmd ( ADC1 , ENABLE ) ;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void USART1_Init(void)
{
	// Init USART
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	USART_InitStruct.USART_BaudRate = 9600;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1, &USART_InitStruct);
	USART_Cmd(USART1, ENABLE);
}

void USART2_Init(void)
{
	// Init USART 2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	USART_InitStruct.USART_BaudRate = 9600;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2, &USART_InitStruct);
	USART_Cmd(USART2, ENABLE);
}

// TODO change for common usage
void GPIO_Camera_Pin_Init()
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void GPIO_Door_Pin_Init()
{
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void GSM_Pin_Init()
{
	RCC_APB2PeriphClockCmd(GSM_GPIO_CLK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GSM_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GSM_GPIO_PORT, &GPIO_InitStructure);
}

void snapshot()
{
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	STM32vldiscovery_LEDOff(CAMERA);
	Delay(0xAAFFFF);
	STM32vldiscovery_LEDOn(CAMERA);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void door_open()
{
	GPIO_InitStructure.GPIO_Pin = DOOR_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	STM32vldiscovery_LEDOff(DOOR);
	Delay(0xAAFFFF);
	STM32vldiscovery_LEDOn(DOOR);
	
	GPIO_InitStructure.GPIO_Pin = DOOR_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void GSM_power_on()
{
	GPIO_InitStructure.GPIO_Pin = GSM_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GSM_GPIO_PORT, &GPIO_InitStructure);
	
	STM32vldiscovery_LEDOff(GSM);
	Delay(0xFFFFF0);
	STM32vldiscovery_LEDOn(GSM);
	
	GPIO_InitStructure.GPIO_Pin = GSM_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GSM_GPIO_PORT, &GPIO_InitStructure);
}

void read_adc_inj(uint16_t *adcValue)
{
  //while(ADC_GetSoftwareStartInjectedConvCmdStatus(ADC1) == SET);
  ADC_SoftwareStartInjectedConvCmd(ADC1, ENABLE);

  //+ Wait until ADC Channel 8 end of conversion
  while (ADC_GetFlagStatus(ADC1, ADC_FLAG_JEOC) == RESET);
	
	adcValue[0] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
	adcValue[1] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2);
	adcValue[2] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_3);
}

uint16_t read_adc()
{
	ADC_SoftwareStartConvCmd(ADC1,ENABLE);
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));
	return ADC_GetConversionValue(ADC1);
}

void led_init()
{
	STM32vldiscovery_LEDInit(LED_GREEN);
  STM32vldiscovery_LEDInit(LED_BLUE);
	STM32vldiscovery_LEDInit(MAIN_GREEN);
	STM32vldiscovery_LEDInit(MAIN_BLUE);
	STM32vldiscovery_LEDInit(COOLER);
}

void calc_alcohol(uint16_t *adcValue, double *BAC_1, double *BAC_2, double *BAC_3)
{
	double RS_gas, ratio, x, sensor_volt;
	sensor_volt = (double)adcValue[0]/ADC_resolution * ADC_up_level;
  RS_gas = ((ADC_up_level * R2)/sensor_volt) - R2;
  ratio = RS_gas / R0_1;// ratio = RS/R0
  x = 0.4*ratio;   
  *BAC_1 = pow(x,-1.431);  //BAC in mg/L
	
	sensor_volt = (double)adcValue[1]/ADC_resolution * ADC_up_level;
  RS_gas = ((ADC_up_level * R2)/sensor_volt) - R2;
  ratio = RS_gas / R0_2;// ratio = RS/R0
  x = 0.4*ratio;   
  *BAC_2 = pow(x,-1.431);  //BAC in mg/L
	
	sensor_volt = (double)adcValue[2]/ADC_resolution * ADC_up_level;
  RS_gas = ((ADC_up_level * R2)/sensor_volt) - R2;
  ratio = RS_gas / R0_3;// ratio = RS/R0
  x = 0.4*ratio;   
  *BAC_3 = pow(x,-1.431);  //BAC in mg/L
}

void calc_mean(double *in_mean_1, double *in_mean_2, double *in_mean_3, int mean_num, double *MQ_1, double *MQ_2, double *MQ_3)
{
	uint8_t i = 0;
	double mean_1 = 0, mean_2 = 0, mean_3 = 0;
	for(i = 0; i < mean_num; i++)
	{
		mean_1 += MQ_1[i];
		mean_2 += MQ_2[i];
		mean_3 += MQ_3[i];
	}
	*in_mean_1 = mean_1 / mean_num;
	*in_mean_2 = mean_2 / mean_num;
	*in_mean_3 = mean_3 / mean_num;
}

void math(double *MQ_1, double *MQ_2, double *MQ_3, int mean_num, double BAC_1, double BAC_2, double BAC_3)
{
	static uint8_t iterator = 0;
	MQ_1[iterator] = BAC_1;
	MQ_2[iterator] = BAC_2;
	MQ_3[iterator] = BAC_3;
	
	iterator++;
	if(iterator == mean_num)
	{
		iterator = 0;
	}
}

void alco_array_init(double *MQ_1, double *MQ_2, double *MQ_3, int mean_num)
{
	uint8_t i = 0;
	for(i = 0; i < mean_num; i++)
	{
		MQ_1[i] = 0.2;
		MQ_2[i] = 0.2;
		MQ_3[i] = 0.2;
	}
}

void USART_Puts(USART_TypeDef *USARTx, volatile char *s)
{
	while(*s)
	{
		if (wrong_key_avoid_flag == 0)
		{
			while(!(USARTx->SR & USART_FLAG_TC));
		  USART_SendData(USARTx, *s);
		  *s++;
		}
		else
		{
			Delay(0x1FFFFF);
		}
	}
}

void green_light_for_next_breath(uint8_t breath_allow)
{
	if (breath_allow == 1)
	{
		STM32vldiscovery_LEDOn(LED_GREEN);
		STM32vldiscovery_LEDOn(MAIN_GREEN);
	}
	else
	{
		STM32vldiscovery_LEDOff(LED_GREEN);
		STM32vldiscovery_LEDOff(MAIN_GREEN);
	}
}

void send_sms_or_check_gsm_link(uint8_t *sms_flag, uint8_t *rx_buf, uint8_t *rx_buf_ptr)
{
	static char str_alarm[20] = {0};
	
	if((((rx_buf[*rx_buf_ptr-2] == '>') && (rx_buf[*rx_buf_ptr-1] == ' ')) && (1 == *sms_flag)) ||
			  ((rx_buf[*rx_buf_ptr-4] == 'O') && (rx_buf[*rx_buf_ptr-3] == 'K') && (rx_buf[*rx_buf_ptr-2] == 0x0D) && (rx_buf[*rx_buf_ptr-1] == 0x0A)))
		{
			rx_buf[*rx_buf_ptr] = '\0';
			USART_Puts(USART2, (char *)rx_buf);
			if((rx_buf[*rx_buf_ptr-2] == '>') && (rx_buf[*rx_buf_ptr-1] == ' '))
			{
				char A = 0x1A;
				//blink_green();
				snprintf(str_alarm, sizeof(str_alarm), "Alconavt%c", A);
				USART_Puts(USART1, str_alarm);
				*sms_flag = 0;
			}
			rx_buf_ptr = 0;
		}
}

void set_pseudo_mutex(uint8_t flag)
{
	wrong_key_avoid_flag = flag;
}
