#include "test_mode.h"
#include "performance_monitor.h"

#include <stdbool.h>

#define TX_RING_SIZE 512U

// Verifica que una trama completa pueda almacenarse dentro del buffer circular.
#if ( FRAME_PAYLOAD_BYTES >= TX_RING_SIZE )
#error "TX_RING_SIZE debe ser mayor que FRAME_PAYLOAD_BYTES"
#endif

// Verifica que el contador de muestras pueda representarse con uint8_t.
#if ( FRAME_SAMPLES > 255U )
#error "FRAME_SAMPLES no cabe en sample_block_count"
#endif

// Buffer circular utilizado para desacoplar el procesamiento de la transmisión UART.
static uint8_t tx_ring[TX_RING_SIZE];
static volatile uint16_t tx_head;
static volatile uint16_t tx_tail;

// Buffer temporal utilizado para construir una trama de FRAME_SAMPLES muestras.
static uint8_t sample_block[FRAME_PAYLOAD_BYTES];
static uint8_t sample_block_count;

// Variables compartidas entre la adquisición y el procesamiento principal.
static volatile bool adc_busy;
static volatile bool sample_ready;
static volatile uint16_t pending_sample;

// Variables utilizadas por la transmisión UART no bloqueante.
static volatile bool uart_tx_busy;
static volatile uint16_t uart_tx_chunk_length;
static volatile bool uart_trace_active;

// Contadores utilizados para evaluar el desempeño de cada estrategia.
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

// Inicializa el estado del ensayo y activa TIM2 según la estrategia seleccionada.
void AppMode_Init( void ) {

	// 1. Reinicia los índices y estados internos de adquisición y transmisión.
	tx_head = 0U;
	tx_tail = 0U;
	sample_block_count = 0U;
	adc_busy = false;
	sample_ready = false;
	pending_sample = 0U;
	uart_tx_busy = false;
	uart_tx_chunk_length = 0U;
	uart_trace_active = false;

	// 2. Reinicia todos los contadores utilizados durante la prueba.
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

	// 3. Inicializa las señales digitales utilizadas para medir tiempos y latencias.
	PerformanceMonitor_Init();

	// 4. Reinicia TIM2 antes de comenzar el ensayo.
	__HAL_TIM_DISABLE( &htim2 );
	__HAL_TIM_SET_COUNTER( &htim2, 0U );
	__HAL_TIM_CLEAR_FLAG( &htim2, TIM_FLAG_UPDATE );

#if ( APP_EXECUTION_MODE == APP_MODE_POLLING )
	// 5. En Polling, inicia TIM2 sin habilitar su interrupción.
	if ( HAL_TIM_Base_Start( &htim2 ) != HAL_OK ) {
		Error_Handler();
	}
#else
	// 5. En Interrupts y Low Power, inicia TIM2 con interrupción periódica.
	if ( HAL_TIM_Base_Start_IT( &htim2 ) != HAL_OK ) {
		Error_Handler();
	}

	Performance_ActiveBegin();
#endif

#if ( PERFORMANCE_TRACES_ENABLED == 1U )
	// 6. Activa el canal PWM usado como referencia hardware del instante de muestreo.
	if ( HAL_TIM_PWM_Start( &htim2, TIM_CHANNEL_1 ) != HAL_OK ) {
		Error_Handler();
	}
#endif

#if ( APP_DISABLE_SYSTICK_DURING_TEST == 1U )
	// 7. Suspende la interrupción periódica de SysTick para no alterar las mediciones.
	HAL_SuspendTick();
#endif
}

// Atiende por Polling cada evento periódico de TIM2 e inicia una conversión ADC.
void Timer_PollTask( void ) {
	Performance_ActiveBegin();

	// 1. Verifica que ocurrió un Update Event y que la interrupción de TIM2 está deshabilitada.
	if ( ( __HAL_TIM_GET_FLAG( &htim2, TIM_FLAG_UPDATE ) != RESET ) &&
		 ( __HAL_TIM_GET_IT_SOURCE( &htim2, TIM_IT_UPDATE ) == RESET ) ) {

		// 2. Limpia el evento y registra el instante en que el software lo atiende.
		__HAL_TIM_CLEAR_FLAG( &htim2, TIM_FLAG_UPDATE );
		timer_events++;
		Performance_SampleResponseBegin();

		// 3. Inicia una nueva conversión solamente si el ADC está disponible.
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

		// 4. Finaliza la medición del tiempo utilizado para responder al evento del timer.
		Performance_SampleResponseEnd();
	}
}

// Supervisa por Polling el final de conversión del ADC y procesa la muestra obtenida.
void ADC_PollTask( void ) {

	// 1. Verifica que exista una conversión activa y que el ADC terminó la conversión.
	if ( adc_busy && ( __HAL_ADC_GET_FLAG( &hadc1, ADC_FLAG_EOC ) != RESET ) ) {

		uint16_t sample = (uint16_t)HAL_ADC_GetValue( &hadc1 );

		// Resultado adquirido y almacenado.
		Performance_ADCEnd();

		if ( HAL_ADC_Stop( &hadc1 ) != HAL_OK ) {
			Error_Handler();
		}

		adc_busy = false;

		Performance_ProcessBegin();
		SerialPlot_AddSample( sample );
		processed_samples++;
		Performance_ProcessEnd();
	}
}

// Transmite por Polling los bytes pendientes utilizando directamente el registro TDR de USART3.
void UART_PollTask( void ) {

	// 1. Si existen datos y TDR está disponible, transmite el siguiente byte.
	if ( ( tx_head != tx_tail ) && ( __HAL_UART_GET_FLAG( &huart3, UART_FLAG_TXE ) != RESET ) ) {

		if ( !uart_trace_active ) {
			uart_trace_active = true;
			Performance_UARTBegin();
		}

		huart3.Instance->TDR = tx_ring[tx_tail];
		tx_tail = (uint16_t)( ( tx_tail + 1U ) % TX_RING_SIZE );
	}

	// 2. Finaliza la transmisión cuando el buffer está vacío y el último byte salió físicamente.
	if ( uart_trace_active && ( tx_head == tx_tail ) &&
		 ( __HAL_UART_GET_FLAG( &huart3, UART_FLAG_TC ) != RESET ) ) {

		uart_trace_active = false;
		transmitted_frames = queued_frames;
		transmitted_bytes = queued_frames * FRAME_PAYLOAD_BYTES;
		Performance_UARTEnd();
	}

	// 3. Marca el final del ciclo de trabajo activo del Polling.
	Performance_ActiveEnd();
}

// Transfiere de forma segura al main la muestra producida por la interrupción del ADC.
void Samples_Process( void ) {
	uint32_t interrupt_state;
	uint16_t sample = 0U;
	bool has_sample = false;

	// 1. Guarda el estado de las interrupciones y protege la variable compartida.
	interrupt_state = __get_PRIMASK();
	__disable_irq();

	// 2. Copia la muestra pendiente y libera el espacio para la siguiente conversión.
	if ( sample_ready ) {
		sample = pending_sample;
		sample_ready = false;
		has_sample = true;
	}

	// 3. Restaura las interrupciones solamente si estaban habilitadas previamente.
	if ( interrupt_state == 0U ) {
		__enable_irq();
	}

	// 4. Procesa la muestra fuera de la sección crítica.
	if ( has_sample ) {
		Performance_ProcessBegin();
		SerialPlot_AddSample( sample );
		processed_samples++;
		Performance_ProcessEnd();
	}
}

// Inicia una transmisión UART por interrupciones cuando existen bytes pendientes.
void UART_InterruptTask( void ) {
	uint16_t head;
	uint16_t tail;
	uint16_t length;

	// 1. Sale si USART3 ya está transmitiendo o si el buffer circular está vacío.
	if ( uart_tx_busy || ( tx_head == tx_tail ) ) {
		return;
	}

	// 2. Obtiene una región continua del buffer circular para transmitirla de una sola vez.
	head = tx_head;
	tail = tx_tail;

	if ( head > tail ) {
		length = (uint16_t)( head - tail );
	} else {
		length = (uint16_t)( TX_RING_SIZE - tail );
	}

	// 3. Inicia la medición de tiempo de UART si comienza una nueva ráfaga de transmisión.
	if ( !uart_trace_active ) {
		uart_trace_active = true;
		Performance_UARTBegin();
	}

	// 4. Guarda el tamaño del bloque e inicia la transmisión no bloqueante.
	uart_tx_chunk_length = length;
	uart_tx_busy = true;

	if ( HAL_UART_Transmit_IT( &huart3, &tx_ring[tail], length ) != HAL_OK ) {
		uart_tx_busy = false;
		uart_trace_active = false;
		Performance_UARTEnd();
		Error_Handler();
	}
}

// Coloca el Cortex-M7 en espera mediante WFI cuando el modo Low Power no tiene trabajo pendiente.
void LowPower_SleepIfIdle( void ) {
	uint32_t interrupt_state = __get_PRIMASK();

	// 1. Protege la comprobación de trabajo pendiente contra condiciones de carrera.
	__disable_irq();

	// 2. Si no existe trabajo pendiente, registra la entrada y coloca la CPU en espera.
	if ( !AppMode_HasPendingMainWork() ) {
		sleep_entries++;
		Performance_SleepEnter();
		__DSB();
		__WFI();
		__ISB();
		Performance_SleepExit();
	}

	// 3. Restaura las interrupciones si estaban habilitadas antes de entrar a la función.
	if ( interrupt_state == 0U ) {
		__enable_irq();
	}
}

// Empaqueta cada muestra de 12 bits en dos bytes y genera una trama al completar el bloque.
static void SerialPlot_AddSample( uint16_t sample ) {
	uint16_t index = (uint16_t)( 2U * sample_block_count );

	// 1. Guarda la muestra en formato little-endian dentro del bloque actual.
	sample_block[index] = (uint8_t)( sample & 0xFFU );
	sample_block[index + 1U] = (uint8_t)( ( sample >> 8U ) & 0x0FU );
	sample_block_count++;

	// 2. Cuando se completa la trama, intenta almacenarla en el buffer circular de UART.
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

// Inserta un bloque completo dentro del buffer circular de transmisión.
static bool TX_PushBlock( const uint8_t *data, uint16_t length ) {
	uint16_t head = tx_head;
	uint16_t tail = tx_tail;
	uint16_t i;

	// 1. Verifica que exista espacio suficiente antes de modificar el buffer.
	if ( TX_FreeBytes( head, tail ) < length ) {
		return false;
	}

	// 2. Copia los bytes y actualiza localmente la posición de escritura.
	for ( i = 0U; i < length; ++i ) {
		tx_ring[head] = data[i];
		head = (uint16_t)( ( head + 1U ) % TX_RING_SIZE );
	}

	// 3. Garantiza que los datos sean visibles antes de publicar el nuevo índice de escritura.
	__DMB();
	tx_head = head;
	return true;
}

// Calcula el espacio libre disponible dentro del buffer circular de transmisión.
static uint16_t TX_FreeBytes( uint16_t head, uint16_t tail ) {
	uint16_t used;

	// 1. Calcula los bytes ocupados considerando el posible retorno al inicio del buffer.
	if ( head >= tail ) {
		used = (uint16_t)( head - tail );
	} else {
		used = (uint16_t)( TX_RING_SIZE - tail + head );
	}

	// 2. Reserva una posición para diferenciar el estado lleno del estado vacío.
	return (uint16_t)( TX_RING_SIZE - 1U - used );
}

// Determina si el main tiene trabajo pendiente antes de permitir la entrada a Low Power.
static bool AppMode_HasPendingMainWork( void ) {

	// 1. Existe trabajo si hay una muestra pendiente de procesamiento.
	if ( sample_ready ) {
		return true;
	}

	// 2. Existe trabajo si UART está libre y todavía hay bytes esperando transmisión.
	if ( !uart_tx_busy && ( tx_head != tx_tail ) ) {
		return true;
	}

	return false;
}

// Callback de TIM2: inicia una conversión ADC en los modos basados en interrupciones.
void HAL_TIM_PeriodElapsedCallback( TIM_HandleTypeDef *timer ) {
#if ( APP_EXECUTION_MODE != APP_MODE_POLLING )

	// 1. Verifica que la interrupción provenga de TIM2.
	if ( timer->Instance != TIM2 )
		return;

	// 2. Registra el evento y mide el tiempo de respuesta del software.
	timer_events++;
	Performance_SampleResponseBegin();

	// 3. Inicia una conversión ADC por interrupción si el ADC está disponible.
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

	// 4. Finaliza la medición de atención del evento de TIM2.
	Performance_SampleResponseEnd();
#else
	(void)timer;
#endif
}

// Callback del ADC: recupera la muestra y la entrega al procesamiento principal.
void HAL_ADC_ConvCpltCallback( ADC_HandleTypeDef *adc ) {
#if ( APP_EXECUTION_MODE != APP_MODE_POLLING )

	// 1. Verifica que la conversión corresponda al ADC1.
	if ( adc->Instance != ADC1 )
		return;

	// 2. Recupera el resultado de la conversión.
	uint16_t sample = (uint16_t)HAL_ADC_GetValue( adc );

	// 3. El resultado ya fue almacenado: termina la medición ADC.
	Performance_ADCEnd();

	// 4. Libera el ADC para permitir la siguiente adquisición.
	adc_busy = false;

	// 5. Publica la muestra si el main alcanzó a consumir la anterior.
	if ( sample_ready ) {
		dropped_samples++;
	} else {
		pending_sample = sample;
		__DMB();
		sample_ready = true;
	}

#else
	(void)adc;
#endif
}

// Callback de UART: libera el bloque transmitido y actualiza el estado del buffer circular.
void HAL_UART_TxCpltCallback( UART_HandleTypeDef *uart ) {
#if ( APP_EXECUTION_MODE != APP_MODE_POLLING )

	// 1. Verifica que la transmisión corresponda a USART3.
	if ( uart->Instance != USART3 )
		return;

	// 2. Avanza la posición de lectura según la cantidad de bytes transmitidos.
	tx_tail = (uint16_t)( ( tx_tail + uart_tx_chunk_length ) % TX_RING_SIZE );
	uart_tx_chunk_length = 0U;
	uart_tx_busy = false;

	// 3. Finaliza la medición cuando ya no existen datos pendientes en el buffer.
	if ( tx_tail == tx_head ) {
		transmitted_frames = queued_frames;
		transmitted_bytes = queued_frames * FRAME_PAYLOAD_BYTES;
		uart_trace_active = false;
		Performance_UARTEnd();
	}
#else
	(void)uart;
#endif
}

// Atiende un error del ADC1 y lleva el sistema al manejador general de errores.
void HAL_ADC_ErrorCallback( ADC_HandleTypeDef *adc ) {
	if ( adc->Instance != ADC1 )
		return;

	adc_busy = false;
	Performance_ADCEnd();
	Error_Handler();
}

// Atiende un error de USART3 y lleva el sistema al manejador general de errores.
void HAL_UART_ErrorCallback( UART_HandleTypeDef *uart ) {
	if ( uart->Instance != USART3 )
		return;

	uart_tx_busy = false;
	uart_trace_active = false;
	Performance_UARTEnd();
	Error_Handler();
}
