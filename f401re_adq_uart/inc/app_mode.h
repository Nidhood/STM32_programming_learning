#ifndef APP_MODE_H
#define APP_MODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define APP_MODE_POLLING 0U
#define APP_MODE_INTERRUPTS 1U
#define APP_MODE_LOW_POWER 2U

/* Cambiar solamente esta definición para seleccionar la prueba. */
#ifndef APP_EXECUTION_MODE
#define APP_EXECUTION_MODE APP_MODE_POLLING
#endif

/* Elimina la interrupción periódica de SysTick durante el ensayo. */
#define APP_DISABLE_SYSTICK_DURING_TEST 1U

#if ( APP_EXECUTION_MODE != APP_MODE_POLLING ) && ( APP_EXECUTION_MODE != APP_MODE_INTERRUPTS ) && \
	( APP_EXECUTION_MODE != APP_MODE_LOW_POWER )
#error "APP_EXECUTION_MODE no es válido"
#endif

extern volatile uint32_t adc_overruns;
extern volatile uint32_t dropped_samples;
extern volatile uint32_t dropped_frames;
extern volatile uint32_t timer_events;
extern volatile uint32_t processed_samples;
extern volatile uint32_t generated_frames;
extern volatile uint32_t queued_frames;
extern volatile uint32_t transmitted_frames;
extern volatile uint32_t transmitted_bytes;
extern volatile uint32_t sleep_entries;

void AppMode_Init( void );

void Timer_PollTask( void );
void ADC_PollTask( void );
void UART_PollTask( void );

void Samples_Process( void );
void UART_InterruptTask( void );
void LowPower_SleepIfIdle( void );

#ifdef __cplusplus
}
#endif

#endif
