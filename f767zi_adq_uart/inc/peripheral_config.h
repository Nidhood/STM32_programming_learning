#pragma once

#include "main.h"

void SystemClock_Config( void );
void GPIO_Init( void );
void ADC1_Init( void );
void UART3_Init( void );
void TIM2_Init( void );
void HAL_TIM_MspPostInit( TIM_HandleTypeDef *timer );
