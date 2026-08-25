#include "peripheral_config.h"
#include "performance_monitor.h"

void SystemClock_Config( void ) {
	RCC_OscInitTypeDef oscillator = { 0 };
	RCC_ClkInitTypeDef clock = { 0 };

	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE2 );

	oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	oscillator.HSIState = RCC_HSI_ON;
	oscillator.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	oscillator.PLL.PLLState = RCC_PLL_ON;
	oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	oscillator.PLL.PLLM = 16U;
	oscillator.PLL.PLLN = 336U;
	oscillator.PLL.PLLP = RCC_PLLP_DIV4;
	oscillator.PLL.PLLQ = 7U;

	if ( HAL_RCC_OscConfig( &oscillator ) != HAL_OK ) {
		Error_Handler();
	}

	clock.ClockType =
		RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	clock.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	clock.AHBCLKDivider = RCC_SYSCLK_DIV1;
	clock.APB1CLKDivider = RCC_HCLK_DIV2;
	clock.APB2CLKDivider = RCC_HCLK_DIV1;

	if ( HAL_RCC_ClockConfig( &clock, FLASH_LATENCY_2 ) != HAL_OK ) {
		Error_Handler();
	}
}

void GPIO_Init( void ) {
	GPIO_InitTypeDef gpio = { 0 };

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/* Procesamiento y trama. */
	HAL_GPIO_WritePin( GPIOA, DEBUG_PROCESS_PIN | DEBUG_FRAME_PIN, GPIO_PIN_RESET );

	/* Atención de muestra, ADC y actividad de CPU. */
	HAL_GPIO_WritePin( GPIOB, DEBUG_SAMPLE_PIN | DEBUG_ADC_PIN | DEBUG_SLEEP_PIN, GPIO_PIN_RESET );

	/* D9: actividad UART. */
	HAL_GPIO_WritePin( DEBUG_UART_PORT, DEBUG_UART_PIN, GPIO_PIN_RESET );

	/* PA8 y PA9. */
	gpio.Pin = DEBUG_PROCESS_PIN | DEBUG_FRAME_PIN;
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init( GPIOA, &gpio );

	/* PB5, PB4 y PB10. */
	gpio.Pin = DEBUG_SAMPLE_PIN | DEBUG_ADC_PIN | DEBUG_SLEEP_PIN;
	HAL_GPIO_Init( GPIOB, &gpio );

	/* PC7 = D9. */
	gpio.Pin = DEBUG_UART_PIN;
	HAL_GPIO_Init( DEBUG_UART_PORT, &gpio );
}

void ADC1_Init( void ) {
	ADC_ChannelConfTypeDef channel = { 0 };

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

	channel.Channel = ADC_CHANNEL_0;
	channel.Rank = 1U;
	channel.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	channel.Offset = 0U;

	if ( HAL_ADC_ConfigChannel( &hadc1, &channel ) != HAL_OK ) {
		Error_Handler();
	}
}

void UART2_Init( void ) {
	huart2.Instance = USART2;
	huart2.Init.BaudRate = UART_BAUD_RATE;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;

	if ( HAL_UART_Init( &huart2 ) != HAL_OK ) {
		Error_Handler();
	}
}

void TIM2_Init( void ) {
	TIM_OC_InitTypeDef output_compare = { 0 };

	if ( ( SAMPLE_RATE_HZ == 0U ) || ( SAMPLE_RATE_HZ >= TIMER_TICK_HZ ) ||
		 ( TIMER_INPUT_CLOCK_HZ % TIMER_TICK_HZ != 0U ) ||
		 ( TIMER_TICK_HZ % SAMPLE_RATE_HZ != 0U ) ) {
		Error_Handler();
	}

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
	/* Pulso hardware de 1 us al comienzo de cada periodo de muestreo. */
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

void Error_Handler( void ) {
	__disable_irq();

	while ( 1 ) {
		HAL_GPIO_TogglePin( DEBUG_UART_PORT, DEBUG_UART_PIN );

		for ( volatile uint32_t delay = 0U; delay < 1000000U; ++delay ) {
		}
	}
}
