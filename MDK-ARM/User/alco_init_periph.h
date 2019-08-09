#ifndef ALCO_INIT_PERIPH_H
#define ALCO_INIT_PERIPH_H

#include <stdio.h>
#include "stm32f10x.h"
#include "STM32vldiscovery.h"

void sdk_Init(void);
void ADC1_Init(void);
void USART1_Init(void);
void USART2_Init(void);
void GPIO_Camera_Pin_Init(void);
void GPIO_Door_Pin_Init(void);
void GSM_Pin_Init(void);
void snapshot(void);
void door_open(void);
void GSM_power_on(void);
void read_adc_inj(uint16_t *adcValue);
void led_init(void);
void calc_alcohol   (uint16_t *adcValue, double *BAC_1, double *BAC_2, double *BAC_3);
void calc_mean      (double *in_mean_1, double *in_mean_2, double *in_mean_3, int mean_num, double *MQ_1, double *MQ_2, double *MQ_3);
void math           (double *MQ_1, double *MQ_2, double *MQ_3, int mean_num, double BAC_1, double BAC_2, double BAC_3);
void alco_array_init(double *MQ_1, double *MQ_2, double *MQ_3, int mean_num);
void USART_Puts     (USART_TypeDef *USARTx, volatile char *s);
void green_light_for_next_breath(uint8_t breath_allow);
void send_sms_or_check_gsm_link(uint8_t *sms_flag, uint8_t *rx_buf, uint8_t *rx_buf_ptr);

#endif
