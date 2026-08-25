#include "main.h"

void HAL_MspInit( void ) {
	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_RCC_PWR_CLK_ENABLE();

	HAL_NVIC_SetPriorityGrouping( NVIC_PRIORITYGROUP_4 );
}

void HAL_ADC_MspInit( ADC_HandleTypeDef *adc ) {
	GPIO_InitTypeDef gpio = { 0 };

	if ( adc->Instance == ADC1 ) {
		__HAL_RCC_ADC1_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();

		/* PA0: ADC1_IN0. */
		gpio.Pin = GPIO_PIN_0;
		gpio.Mode = GPIO_MODE_ANALOG;
		gpio.Pull = GPIO_NOPULL;
		HAL_GPIO_Init( GPIOA, &gpio );

		HAL_NVIC_SetPriority( ADC_IRQn, 1U, 0U );
		HAL_NVIC_EnableIRQ( ADC_IRQn );
	}
}

void HAL_ADC_MspDeInit( ADC_HandleTypeDef *adc ) {
	if ( adc->Instance == ADC1 ) {
		__HAL_RCC_ADC1_CLK_DISABLE();
		HAL_GPIO_DeInit( GPIOA, GPIO_PIN_0 );
		HAL_NVIC_DisableIRQ( ADC_IRQn );
	}
}

void HAL_TIM_PWM_MspInit( TIM_HandleTypeDef *timer ) {
	if ( timer->Instance == TIM2 ) {
		__HAL_RCC_TIM2_CLK_ENABLE();

		HAL_NVIC_SetPriority( TIM2_IRQn, 2U, 0U );
		HAL_NVIC_EnableIRQ( TIM2_IRQn );
	}
}

void HAL_TIM_MspPostInit( TIM_HandleTypeDef *timer ) {
	GPIO_InitTypeDef gpio = { 0 };

	if ( timer->Instance == TIM2 ) {
		__HAL_RCC_GPIOA_CLK_ENABLE();

		/* PA5/D13: TIM2_CH1, pulso hardware de referencia. */
		gpio.Pin = DEBUG_TIMER_REF_PIN;
		gpio.Mode = GPIO_MODE_AF_PP;
		gpio.Pull = GPIO_NOPULL;
		gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		gpio.Alternate = GPIO_AF1_TIM2;
		HAL_GPIO_Init( DEBUG_TIMER_REF_PORT, &gpio );
	}
}

void HAL_TIM_PWM_MspDeInit( TIM_HandleTypeDef *timer ) {
	if ( timer->Instance == TIM2 ) {
		__HAL_RCC_TIM2_CLK_DISABLE();
		HAL_GPIO_DeInit( DEBUG_TIMER_REF_PORT, DEBUG_TIMER_REF_PIN );
		HAL_NVIC_DisableIRQ( TIM2_IRQn );
	}
}

void HAL_UART_MspInit( UART_HandleTypeDef *uart ) {
	GPIO_InitTypeDef gpio = { 0 };

	if ( uart->Instance == USART2 ) {
		__HAL_RCC_USART2_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();

		/* PA2: USART2_TX. PA3: USART2_RX. */
		gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3;
		gpio.Mode = GPIO_MODE_AF_PP;
		gpio.Pull = GPIO_PULLUP;
		gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		gpio.Alternate = GPIO_AF7_USART2;
		HAL_GPIO_Init( GPIOA, &gpio );

		HAL_NVIC_SetPriority( USART2_IRQn, 3U, 0U );
		HAL_NVIC_EnableIRQ( USART2_IRQn );
	}
}

void HAL_UART_MspDeInit( UART_HandleTypeDef *uart ) {
	if ( uart->Instance == USART2 ) {
		__HAL_RCC_USART2_CLK_DISABLE();
		HAL_GPIO_DeInit( GPIOA, GPIO_PIN_2 | GPIO_PIN_3 );
		HAL_NVIC_DisableIRQ( USART2_IRQn );
	}
}
