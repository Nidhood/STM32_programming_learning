#include "main.h"
#include "test_mode.h"

// Atiende la interrupción periódica utilizada por la HAL como base de tiempo.
void SysTick_Handler( void ) {
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

#if ( APP_EXECUTION_MODE != APP_MODE_POLLING )

// Atiende los eventos periódicos generados por TIM2.
void TIM2_IRQHandler( void ) {
	HAL_TIM_IRQHandler( &htim2 );
}

// Atiende el final de conversión y los errores generados por ADC1.
void ADC_IRQHandler( void ) {
	HAL_ADC_IRQHandler( &hadc1 );
}

// Atiende la transmisión no bloqueante y los errores generados por USART2.
void USART3_IRQHandler( void ) {
	HAL_UART_IRQHandler( &huart3 );
}

#endif
