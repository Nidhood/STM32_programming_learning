#ifndef PERFORMANCE_MONITOR_H
#define PERFORMANCE_MONITOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#ifndef PERFORMANCE_TRACES_ENABLED
#define PERFORMANCE_TRACES_ENABLED 1U
#endif

#if ( PERFORMANCE_TRACES_ENABLED == 1U )

static inline void Performance_PinSet( GPIO_TypeDef *port, uint16_t pin ) {
	port->BSRR = (uint32_t)pin;
}

static inline void Performance_PinReset( GPIO_TypeDef *port, uint16_t pin ) {
	port->BSRR = ( (uint32_t)pin << 16U );
}

static inline void PerformanceMonitor_Init( void ) {
	Performance_PinReset( DEBUG_SAMPLE_PORT, DEBUG_SAMPLE_PIN );
	Performance_PinReset( DEBUG_ADC_PORT, DEBUG_ADC_PIN );
	Performance_PinReset( DEBUG_PROCESS_PORT, DEBUG_PROCESS_PIN );
	Performance_PinReset( DEBUG_FRAME_PORT, DEBUG_FRAME_PIN );
	Performance_PinReset( DEBUG_UART_PORT, DEBUG_UART_PIN );
	Performance_PinReset( DEBUG_SLEEP_PORT, DEBUG_SLEEP_PIN );
}

static inline void Performance_ActiveBegin( void ) {
	Performance_PinSet( DEBUG_SLEEP_PORT, DEBUG_SLEEP_PIN );
}

static inline void Performance_ActiveEnd( void ) {
	Performance_PinReset( DEBUG_SLEEP_PORT, DEBUG_SLEEP_PIN );
}

static inline void Performance_SampleResponseBegin( void ) {
	Performance_PinSet( DEBUG_SAMPLE_PORT, DEBUG_SAMPLE_PIN );
}

static inline void Performance_SampleResponseEnd( void ) {
	Performance_PinReset( DEBUG_SAMPLE_PORT, DEBUG_SAMPLE_PIN );
}

static inline void Performance_ADCBegin( void ) {
	Performance_PinSet( DEBUG_ADC_PORT, DEBUG_ADC_PIN );
}

static inline void Performance_ADCEnd( void ) {
	Performance_PinReset( DEBUG_ADC_PORT, DEBUG_ADC_PIN );
}

static inline void Performance_ProcessBegin( void ) {
	Performance_PinSet( DEBUG_PROCESS_PORT, DEBUG_PROCESS_PIN );
}

static inline void Performance_ProcessEnd( void ) {
	Performance_PinReset( DEBUG_PROCESS_PORT, DEBUG_PROCESS_PIN );
}

static inline void Performance_FrameReadyEvent( void ) {
	if ( ( DEBUG_FRAME_PORT->ODR & DEBUG_FRAME_PIN ) == 0U ) {
		Performance_PinSet( DEBUG_FRAME_PORT, DEBUG_FRAME_PIN );
	} else {
		Performance_PinReset( DEBUG_FRAME_PORT, DEBUG_FRAME_PIN );
	}
}

static inline void Performance_UARTBegin( void ) {
	Performance_PinSet( DEBUG_UART_PORT, DEBUG_UART_PIN );
}

static inline void Performance_UARTEnd( void ) {
	Performance_PinReset( DEBUG_UART_PORT, DEBUG_UART_PIN );
}

static inline void Performance_SleepEnter( void ) {
	Performance_ActiveEnd();
}

static inline void Performance_SleepExit( void ) {
	Performance_ActiveBegin();
}

#else

#define PerformanceMonitor_Init() ( (void)0 )
#define Performance_ActiveBegin() ( (void)0 )
#define Performance_ActiveEnd() ( (void)0 )
#define Performance_SampleResponseBegin() ( (void)0 )
#define Performance_SampleResponseEnd() ( (void)0 )
#define Performance_ADCBegin() ( (void)0 )
#define Performance_ADCEnd() ( (void)0 )
#define Performance_ProcessBegin() ( (void)0 )
#define Performance_ProcessEnd() ( (void)0 )
#define Performance_FrameReadyEvent() ( (void)0 )
#define Performance_UARTBegin() ( (void)0 )
#define Performance_UARTEnd() ( (void)0 )
#define Performance_SleepEnter() ( (void)0 )
#define Performance_SleepExit() ( (void)0 )

#endif

#ifdef __cplusplus
}
#endif

#endif
