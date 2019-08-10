//******************************************************************************
// Имя файла    :  'SysTick.h'
// заголовок    :  Постоянный глобальный таймер
// Автор        :  Маньянов Р.Р.
// Контакты     :  
// Дата         :  13.05.2014 - 15.05.2014 
//******************************************************************************

#ifndef __SYSTICK_H
#define __SYSTICK_H

#include "stm32f10x.h"

//==============================================================================
//                      Прототипы функций
//==============================================================================
    
    // Инициализация глобального таймера
    uint8_t SysTickInit (uint32_t frequence, uint8_t uSTimer, uint32_t *SysCore);

    // Денициализация глобального таймера
    void SysTickDeinit(void);

    // Пауза в отсчётах глобального таймера
    void SysTickDelay (uint16_t delay);
    
    // Микросекундная пауза
    void SysTickDelayuS (uint16_t delay);
        
    // Получение текущего системного времени
    uint32_t SysTickGet (void);
    
#endif // __SYSTICK_H