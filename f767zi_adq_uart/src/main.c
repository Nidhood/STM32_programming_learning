#include "main.h"
#include "peripheral_config.h"
#include "test_mode.h"

int main( void ) {

	// 1. Inicializa la HAL y configura el reloj principal del microcontrolador.
	HAL_Init();
	SystemClock_Config();

	// 2. Inicializa los GPIO y los periféricos utilizados durante la prueba.
	GPIO_Init();
	ADC1_Init();
	UART3_Init();
	TIM2_Init();

	// 3. Inicializa las variables y activa la estrategia de ejecución seleccionada.
	AppMode_Init();

	// 4. Ejecuta permanentemente la estrategia seleccionada durante compilación.
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
