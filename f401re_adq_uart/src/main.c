#include "main.h"
#include "app_mode.h"
#include "peripheral_config.h"

ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

int main( void ) {
	HAL_Init();
	SystemClock_Config();
	GPIO_Init();
	ADC1_Init();
	UART2_Init();
	TIM2_Init();

	AppMode_Init();

	while ( 1 ) {
#if ( APP_EXECUTION_MODE == APP_MODE_POLLING )
		Timer_PollTask();
		ADC_PollTask();
		UART_PollTask();
#else
		Samples_Process();
		UART_InterruptTask();

#if ( APP_EXECUTION_MODE == APP_MODE_LOW_POWER )
		LowPower_SleepIfIdle();
#endif
#endif
	}
}
