#ifndef PERIPHERAL_CONFIG_H
#define PERIPHERAL_CONFIG_H

#include "main.h"

void SystemClock_Config( void );
void GPIO_Init( void );
void ADC1_Init( void );
void UART2_Init( void );
void TIM2_Init( void );
void HAL_TIM_MspPostInit( TIM_HandleTypeDef *timer );

#endif