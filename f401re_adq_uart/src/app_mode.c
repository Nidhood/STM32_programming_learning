#include "app_mode.h"
#include "performance_monitor.h"

#include <stdbool.h>

#define TX_RING_SIZE 512U

#if ( FRAME_PAYLOAD_BYTES >= TX_RING_SIZE )
#error "TX_RING_SIZE debe ser mayor que FRAME_PAYLOAD_BYTES"
#endif

#if ( FRAME_SAMPLES > 255U )
#error "FRAME_SAMPLES no cabe en sample_block_count"
#endif

static uint8_t tx_ring[TX_RING_SIZE];
static volatile uint16_t tx_head;
static volatile uint16_t tx_tail;

static uint8_t sample_block[FRAME_PAYLOAD_BYTES];
static uint8_t sample_block_count;

static volatile bool adc_busy;
static volatile bool sample_ready;
static volatile uint16_t pending_sample;

static volatile bool uart_tx_busy;
static volatile uint16_t uart_tx_chunk_length;
static volatile bool uart_trace_active;

volatile uint32_t adc_overruns;
volatile uint32_t dropped_samples;
volatile uint32_t dropped_frames;
volatile uint32_t timer_events;
volatile uint32_t processed_samples;
volatile uint32_t generated_frames;
volatile uint32_t queued_frames;
volatile uint32_t transmitted_frames;
volatile uint32_t transmitted_bytes;
volatile uint32_t sleep_entries;

static void SerialPlot_AddSample( uint16_t sample );
static bool TX_PushBlock( const uint8_t *data, uint16_t length );
static uint16_t TX_FreeBytes( uint16_t head, uint16_t tail );
static bool AppMode_HasPendingMainWork( void );

void AppMode_Init( void ) {
	tx_head = 0U;
	tx_tail = 0U;
	sample_block_count = 0U;
	adc_busy = false;
	sample_ready = false;
	pending_sample = 0U;
	uart_tx_busy = false;
	uart_tx_chunk_length = 0U;
	uart_trace_active = false;

	adc_overruns = 0U;
	dropped_samples = 0U;
	dropped_frames = 0U;
	timer_events = 0U;
	processed_samples = 0U;
	generated_frames = 0U;
	queued_frames = 0U;
	transmitted_frames = 0U;
	transmitted_bytes = 0U;
	sleep_entries = 0U;

	PerformanceMonitor_Init();

	__HAL_TIM_DISABLE( &htim2 );
	__HAL_TIM_SET_COUNTER( &htim2, 0U );
	__HAL_TIM_CLEAR_FLAG( &htim2, TIM_FLAG_UPDATE );

#if ( APP_EXECUTION_MODE == APP_MODE_POLLING )
	if ( HAL_TIM_Base_Start( &htim2 ) != HAL_OK ) {
		Error_Handler();
	}
#else
	if ( HAL_TIM_Base_Start_IT( &htim2 ) != HAL_OK ) {
		Error_Handler();
	}

	Performance_ActiveBegin();
#endif

#if ( PERFORMANCE_TRACES_ENABLED == 1U )
	if ( HAL_TIM_PWM_Start( &htim2, TIM_CHANNEL_1 ) != HAL_OK ) {
		Error_Handler();
	}
#endif

#if ( APP_DISABLE_SYSTICK_DURING_TEST == 1U )
	HAL_SuspendTick();
#endif
}

void Timer_PollTask( void ) {
	Performance_ActiveBegin();

	if ( ( __HAL_TIM_GET_FLAG( &htim2, TIM_FLAG_UPDATE ) != RESET ) &&
		 ( __HAL_TIM_GET_IT_SOURCE( &htim2, TIM_IT_UPDATE ) == RESET ) ) {
		__HAL_TIM_CLEAR_FLAG( &htim2, TIM_FLAG_UPDATE );
		timer_events++;
		Performance_SampleResponseBegin();

		if ( !adc_busy ) {
			adc_busy = true;
			Performance_ADCBegin();

			if ( HAL_ADC_Start( &hadc1 ) != HAL_OK ) {
				adc_busy = false;
				Performance_ADCEnd();
				Performance_SampleResponseEnd();
				Error_Handler();
			}
		} else {
			adc_overruns++;
		}

		Performance_SampleResponseEnd();
	}
}

void ADC_PollTask( void ) {
	if ( adc_busy && ( __HAL_ADC_GET_FLAG( &hadc1, ADC_FLAG_EOC ) != RESET ) ) {
		uint16_t sample = (uint16_t)HAL_ADC_GetValue( &hadc1 );

		if ( HAL_ADC_Stop( &hadc1 ) != HAL_OK ) {
			Error_Handler();
		}

		adc_busy = false;
		Performance_ADCEnd();
		Performance_ProcessBegin();
		SerialPlot_AddSample( sample );
		processed_samples++;
		Performance_ProcessEnd();
	}
}

void UART_PollTask( void ) {
	if ( ( tx_head != tx_tail ) && ( __HAL_UART_GET_FLAG( &huart2, UART_FLAG_TXE ) != RESET ) ) {
		if ( !uart_trace_active ) {
			uart_trace_active = true;
			Performance_UARTBegin();
		}

		huart2.Instance->DR = tx_ring[tx_tail];
		tx_tail = (uint16_t)( ( tx_tail + 1U ) % TX_RING_SIZE );
	}

	if ( uart_trace_active && ( tx_head == tx_tail ) &&
		 ( __HAL_UART_GET_FLAG( &huart2, UART_FLAG_TC ) != RESET ) ) {
		uart_trace_active = false;
		transmitted_frames = queued_frames;
		transmitted_bytes = queued_frames * FRAME_PAYLOAD_BYTES;
		Performance_UARTEnd();
	}

	Performance_ActiveEnd();
}

void Samples_Process( void ) {
	uint32_t interrupt_state;
	uint16_t sample = 0U;
	bool has_sample = false;

	interrupt_state = __get_PRIMASK();
	__disable_irq();

	if ( sample_ready ) {
		sample = pending_sample;
		sample_ready = false;
		has_sample = true;
	}

	if ( interrupt_state == 0U ) {
		__enable_irq();
	}

	if ( has_sample ) {
		Performance_ProcessBegin();
		SerialPlot_AddSample( sample );
		processed_samples++;
		Performance_ProcessEnd();
	}
}

void UART_InterruptTask( void ) {
	uint16_t head;
	uint16_t tail;
	uint16_t length;

	if ( uart_tx_busy || ( tx_head == tx_tail ) ) {
		return;
	}

	head = tx_head;
	tail = tx_tail;

	if ( head > tail ) {
		length = (uint16_t)( head - tail );
	} else {
		length = (uint16_t)( TX_RING_SIZE - tail );
	}

	if ( !uart_trace_active ) {
		uart_trace_active = true;
		Performance_UARTBegin();
	}

	uart_tx_chunk_length = length;
	uart_tx_busy = true;

	if ( HAL_UART_Transmit_IT( &huart2, &tx_ring[tail], length ) != HAL_OK ) {
		uart_tx_busy = false;
		Performance_UARTEnd();
		uart_trace_active = false;
		Error_Handler();
	}
}

void LowPower_SleepIfIdle( void ) {
	uint32_t interrupt_state = __get_PRIMASK();

	__disable_irq();

	if ( !AppMode_HasPendingMainWork() ) {
		sleep_entries++;
		Performance_SleepEnter();
		__DSB();
		__WFI();
		__ISB();
		Performance_SleepExit();
	}

	if ( interrupt_state == 0U ) {
		__enable_irq();
	}
}

static void SerialPlot_AddSample( uint16_t sample ) {
	uint16_t index = (uint16_t)( 2U * sample_block_count );

	sample_block[index] = (uint8_t)( sample & 0xFFU );
	sample_block[index + 1U] = (uint8_t)( ( sample >> 8U ) & 0x0FU );
	sample_block_count++;

	if ( sample_block_count == FRAME_SAMPLES ) {
		generated_frames++;
		Performance_FrameReadyEvent();

		if ( TX_PushBlock( sample_block, FRAME_PAYLOAD_BYTES ) ) {
			queued_frames++;
		} else {
			dropped_frames++;
		}

		sample_block_count = 0U;
	}
}

static bool TX_PushBlock( const uint8_t *data, uint16_t length ) {
	uint16_t head = tx_head;
	uint16_t tail = tx_tail;
	uint16_t i;

	if ( TX_FreeBytes( head, tail ) < length ) {
		return false;
	}

	for ( i = 0U; i < length; ++i ) {
		tx_ring[head] = data[i];
		head = (uint16_t)( ( head + 1U ) % TX_RING_SIZE );
	}

	__DMB();
	tx_head = head;
	return true;
}

static uint16_t TX_FreeBytes( uint16_t head, uint16_t tail ) {
	uint16_t used;

	if ( head >= tail ) {
		used = (uint16_t)( head - tail );
	} else {
		used = (uint16_t)( TX_RING_SIZE - tail + head );
	}

	return (uint16_t)( TX_RING_SIZE - 1U - used );
}

static bool AppMode_HasPendingMainWork( void ) {
	if ( sample_ready ) {
		return true;
	}

	if ( !uart_tx_busy && ( tx_head != tx_tail ) ) {
		return true;
	}

	return false;
}

void HAL_TIM_PeriodElapsedCallback( TIM_HandleTypeDef *timer ) {
#if ( APP_EXECUTION_MODE != APP_MODE_POLLING )
	if ( timer->Instance == TIM2 ) {
		timer_events++;
		Performance_SampleResponseBegin();

		if ( !adc_busy ) {
			adc_busy = true;
			Performance_ADCBegin();

			if ( HAL_ADC_Start_IT( &hadc1 ) != HAL_OK ) {
				adc_busy = false;
				Performance_ADCEnd();
				Performance_SampleResponseEnd();
				Error_Handler();
			}
		} else {
			adc_overruns++;
		}

		Performance_SampleResponseEnd();
	}
#else
	(void)timer;
#endif
}

void HAL_ADC_ConvCpltCallback( ADC_HandleTypeDef *adc ) {
#if ( APP_EXECUTION_MODE != APP_MODE_POLLING )
	if ( adc->Instance == ADC1 ) {
		uint16_t sample = (uint16_t)HAL_ADC_GetValue( adc );

		if ( HAL_ADC_Stop_IT( adc ) != HAL_OK ) {
			Error_Handler();
		}

		adc_busy = false;
		Performance_ADCEnd();

		if ( sample_ready ) {
			dropped_samples++;
		} else {
			pending_sample = sample;
			__DMB();
			sample_ready = true;
		}
	}
#else
	(void)adc;
#endif
}

void HAL_UART_TxCpltCallback( UART_HandleTypeDef *uart ) {
#if ( APP_EXECUTION_MODE != APP_MODE_POLLING )
	if ( uart->Instance == USART2 ) {
		tx_tail = (uint16_t)( ( tx_tail + uart_tx_chunk_length ) % TX_RING_SIZE );
		uart_tx_chunk_length = 0U;
		uart_tx_busy = false;

		if ( tx_tail == tx_head ) {
			transmitted_frames = queued_frames;
			transmitted_bytes = queued_frames * FRAME_PAYLOAD_BYTES;
			uart_trace_active = false;
			Performance_UARTEnd();
		}
	}
#else
	(void)uart;
#endif
}

void HAL_ADC_ErrorCallback( ADC_HandleTypeDef *adc ) {
	if ( adc->Instance == ADC1 ) {
		adc_busy = false;
		Performance_ADCEnd();
		Error_Handler();
	}
}

void HAL_UART_ErrorCallback( UART_HandleTypeDef *uart ) {
	if ( uart->Instance == USART2 ) {
		uart_tx_busy = false;
		uart_trace_active = false;
		Performance_UARTEnd();
		Error_Handler();
	}
}
