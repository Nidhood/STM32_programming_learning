#include "main.h"
#include "test_mode.h"

// Inicializa los recursos globales de bajo nivel: SYSCFG, PWR y prioridades del NVIC.
void HAL_MspInit( void ) {
	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_RCC_PWR_CLK_ENABLE();
	HAL_NVIC_SetPriorityGrouping( NVIC_PRIORITYGROUP_4 );
}

// Inicializa el hardware asociado al ADC1: reloj, entrada analógica e interrupción.
void HAL_ADC_MspInit( ADC_HandleTypeDef *adc ) {
	GPIO_InitTypeDef gpio = { 0 };
	if ( adc->Instance != ADC1 )
		return;
	__HAL_RCC_ADC1_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	gpio.Pin = GPIO_PIN_3;
	gpio.Mode = GPIO_MODE_ANALOG;
	gpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init( GPIOA, &gpio );
#if ( APP_EXECUTION_MODE != APP_MODE_POLLING )
	HAL_NVIC_SetPriority( ADC_IRQn, 1U, 0U );
	HAL_NVIC_EnableIRQ( ADC_IRQn );
#endif
}

// Libera los recursos de hardware asociados al ADC1.
void HAL_ADC_MspDeInit( ADC_HandleTypeDef *adc ) {
	if ( adc->Instance != ADC1 )
		return;
	__HAL_RCC_ADC1_CLK_DISABLE();
	HAL_GPIO_DeInit( GPIOA, GPIO_PIN_3 );
#if ( APP_EXECUTION_MODE != APP_MODE_POLLING )
	HAL_NVIC_DisableIRQ( ADC_IRQn );
#endif
}

// Inicializa los recursos de bajo nivel utilizados por TIM2.
void HAL_TIM_PWM_MspInit( TIM_HandleTypeDef *timer ) {
	if ( timer->Instance != TIM2 )
		return;
	__HAL_RCC_TIM2_CLK_ENABLE();
#if ( APP_EXECUTION_MODE != APP_MODE_POLLING )
	HAL_NVIC_SetPriority( TIM2_IRQn, 2U, 0U );
	HAL_NVIC_EnableIRQ( TIM2_IRQn );
#endif
}

// Configura PA5 como salida TIM2_CH1 para obtener una referencia hardware de muestreo.
void HAL_TIM_MspPostInit( TIM_HandleTypeDef *timer ) {
	GPIO_InitTypeDef gpio = { 0 };
	if ( timer->Instance != TIM2 )
		return;
	__HAL_RCC_GPIOA_CLK_ENABLE();
	gpio.Pin = DEBUG_TIMER_REF_PIN;
	gpio.Mode = GPIO_MODE_AF_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	gpio.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init( DEBUG_TIMER_REF_PORT, &gpio );
}

// Libera los recursos de hardware asociados a TIM2.
void HAL_TIM_PWM_MspDeInit( TIM_HandleTypeDef *timer ) {
	if ( timer->Instance != TIM2 )
		return;
	__HAL_RCC_TIM2_CLK_DISABLE();
	HAL_GPIO_DeInit( DEBUG_TIMER_REF_PORT, DEBUG_TIMER_REF_PIN );
#if ( APP_EXECUTION_MODE != APP_MODE_POLLING )
	HAL_NVIC_DisableIRQ( TIM2_IRQn );
#endif
}

// Inicializa el hardware asociado a USART3: reloj, pines e interrupción.
void HAL_UART_MspInit( UART_HandleTypeDef *uart ) {
	GPIO_InitTypeDef gpio = { 0 };

	if ( uart->Instance != USART3 )
		return;

	// 1. Habilita el reloj de USART3 y GPIOD.
	__HAL_RCC_USART3_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	// 2. Configura:
	//    PD8 -> USART3_TX
	//    PD9 -> USART3_RX
	gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	gpio.Mode = GPIO_MODE_AF_PP;
	gpio.Pull = GPIO_PULLUP;
	gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	gpio.Alternate = GPIO_AF7_USART3;

	HAL_GPIO_Init( GPIOD, &gpio );

#if ( APP_EXECUTION_MODE != APP_MODE_POLLING )

	// 3. Habilita la interrupción de USART3.
	HAL_NVIC_SetPriority( USART3_IRQn, 3U, 0U );
	HAL_NVIC_EnableIRQ( USART3_IRQn );

#endif
}

// Libera los recursos de hardware asociados a USART3.
void HAL_UART_MspDeInit( UART_HandleTypeDef *uart ) {

	if ( uart->Instance != USART3 )
		return;

	__HAL_RCC_USART3_CLK_DISABLE();

	HAL_GPIO_DeInit( GPIOD, GPIO_PIN_8 | GPIO_PIN_9 );

#if ( APP_EXECUTION_MODE != APP_MODE_POLLING )

	HAL_NVIC_DisableIRQ( USART3_IRQn );

#endif
}
