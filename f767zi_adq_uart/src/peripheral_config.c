#include "peripheral_config.h"
#include "performance_monitor.h"

// Define una única instancia global para cada handle declarado como extern en main.h.
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart3;

// Configura el STM32F767 para trabajar a 216 MHz utilizando HSI y el PLL principal.
void SystemClock_Config( void ) {
	RCC_OscInitTypeDef oscillator = { 0 };
	RCC_ClkInitTypeDef clock = { 0 };

	// 1. Habilita PWR y configura el regulador en Scale 1 para alta frecuencia.
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );

	// 2. Configura HSI de 16 MHz como entrada del PLL para obtener 216 MHz.
	oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	oscillator.HSIState = RCC_HSI_ON;
	oscillator.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	oscillator.PLL.PLLState = RCC_PLL_ON;
	oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	oscillator.PLL.PLLM = 8U;
	oscillator.PLL.PLLN = 216U;
	oscillator.PLL.PLLP = RCC_PLLP_DIV2;
	oscillator.PLL.PLLQ = 9U;
	oscillator.PLL.PLLR = 7U;

	if ( HAL_RCC_OscConfig( &oscillator ) != HAL_OK ) {
		Error_Handler();
	}

	// 3. Activa OverDrive para permitir el funcionamiento del núcleo a 216 MHz.
	if ( HAL_PWREx_EnableOverDrive() != HAL_OK ) {
		Error_Handler();
	}

	// 4. Distribuye SYSCLK: HCLK = 216 MHz, PCLK1 = 54 MHz y PCLK2 = 108 MHz.
	clock.ClockType =
		RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	clock.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	clock.AHBCLKDivider = RCC_SYSCLK_DIV1;
	clock.APB1CLKDivider = RCC_HCLK_DIV4;
	clock.APB2CLKDivider = RCC_HCLK_DIV2;

	if ( HAL_RCC_ClockConfig( &clock, FLASH_LATENCY_7 ) != HAL_OK ) {
		Error_Handler();
	}
}

// Inicializa los GPIO utilizados como señales de instrumentación del ensayo.
void GPIO_Init( void ) {
	GPIO_InitTypeDef gpio = { 0 };

	// 1. Habilita los relojes de los puertos utilizados por las trazas.
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	// 2. Establece todas las señales de instrumentación inicialmente en nivel bajo.
	HAL_GPIO_WritePin( GPIOA, DEBUG_PROCESS_PIN | DEBUG_FRAME_PIN, GPIO_PIN_RESET );

	HAL_GPIO_WritePin( GPIOB, DEBUG_SAMPLE_PIN | DEBUG_ADC_PIN, GPIO_PIN_RESET );

	HAL_GPIO_WritePin( DEBUG_UART_PORT, DEBUG_UART_PIN, GPIO_PIN_RESET );

	HAL_GPIO_WritePin( DEBUG_SLEEP_PORT, DEBUG_SLEEP_PIN, GPIO_PIN_RESET );

	// 3. Configura procesamiento y trama.
	gpio.Pin = DEBUG_PROCESS_PIN | DEBUG_FRAME_PIN;
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init( GPIOA, &gpio );

	// 4. Configura muestreo y ADC.
	gpio.Pin = DEBUG_SAMPLE_PIN | DEBUG_ADC_PIN;
	HAL_GPIO_Init( GPIOB, &gpio );

	// 5. Configura D9 = PD15 como traza UART.
	gpio.Pin = DEBUG_UART_PIN;
	HAL_GPIO_Init( DEBUG_UART_PORT, &gpio );

	// 6. Configura actividad/reposo de CPU.
	gpio.Pin = DEBUG_SLEEP_PIN;
	HAL_GPIO_Init( DEBUG_SLEEP_PORT, &gpio );
}

// Configura ADC1 para adquirir una muestra de 12 bits desde ADC1_IN3 en PA3.
void ADC1_Init( void ) {
	ADC_ChannelConfTypeDef channel = { 0 };

	// 1. Configura ADC1 para conversiones individuales iniciadas por software.
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.NbrOfDiscConversion = 0U;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1U;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

	if ( HAL_ADC_Init( &hadc1 ) != HAL_OK ) {
		Error_Handler();
	}

	// 2. Selecciona ADC_CHANNEL_3 y define el tiempo de muestreo.
	channel.Channel = ADC_CHANNEL_3;
	channel.Rank = 1U;
	channel.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	channel.Offset = 0U;

	if ( HAL_ADC_ConfigChannel( &hadc1, &channel ) != HAL_OK ) {
		Error_Handler();
	}
}

// Configura USART3 para transmitir y recibir tramas a la velocidad definida por UART_BAUD_RATE.
void UART3_Init( void ) {

	// 1. Configura los parámetros básicos de USART2.
	huart3.Instance = USART3;
	huart3.Init.BaudRate = UART_BAUD_RATE;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;

	// 2. Inicializa USART2 mediante la HAL.
	if ( HAL_UART_Init( &huart3 ) != HAL_OK ) {
		Error_Handler();
	}
}

// Configura TIM2 para producir eventos a SAMPLE_RATE_HZ y una referencia hardware opcional.
void TIM2_Init( void ) {
	TIM_OC_InitTypeDef output_compare = { 0 };

	// 1. Verifica que las frecuencias seleccionadas puedan generarse exactamente.
	if ( ( SAMPLE_RATE_HZ == 0U ) || ( SAMPLE_RATE_HZ >= TIMER_TICK_HZ ) ||
		 ( TIMER_INPUT_CLOCK_HZ % TIMER_TICK_HZ != 0U ) ||
		 ( TIMER_TICK_HZ % SAMPLE_RATE_HZ != 0U ) ) {
		Error_Handler();
	}

	// 2. Reduce el reloj de TIM2 a 1 MHz y genera un periodo de muestreo exacto.
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = ( TIMER_INPUT_CLOCK_HZ / TIMER_TICK_HZ ) - 1U;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = ( TIMER_TICK_HZ / SAMPLE_RATE_HZ ) - 1U;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	if ( HAL_TIM_PWM_Init( &htim2 ) != HAL_OK ) {
		Error_Handler();
	}

#if ( PERFORMANCE_TRACES_ENABLED == 1U )
	// 3. Genera un pulso hardware de 1 us al inicio de cada periodo de muestreo.
	output_compare.OCMode = TIM_OCMODE_PWM1;
	output_compare.Pulse = 1U;
	output_compare.OCPolarity = TIM_OCPOLARITY_HIGH;
	output_compare.OCFastMode = TIM_OCFAST_DISABLE;

	if ( HAL_TIM_PWM_ConfigChannel( &htim2, &output_compare, TIM_CHANNEL_1 ) != HAL_OK ) {
		Error_Handler();
	}

	HAL_TIM_MspPostInit( &htim2 );
#else
	(void)output_compare;
#endif
}

// Detiene el programa ante un error y genera una señal visible en el pin de traza UART.
void Error_Handler( void ) {
	__disable_irq();

	while ( 1 ) {
		HAL_GPIO_TogglePin( DEBUG_UART_PORT, DEBUG_UART_PIN );

		for ( volatile uint32_t delay = 0U; delay < 1000000U; ++delay ) {
		}
	}
}
