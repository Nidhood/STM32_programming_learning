#pragma once

#include "main.h"

// Modos disponibles de ejecución.
#define APP_MODE_POLLING 0U
#define APP_MODE_INTERRUPTS 1U
#define APP_MODE_LOW_POWER 2U

// Selecciona el modo que se compilará.
#ifndef APP_EXECUTION_MODE
#define APP_EXECUTION_MODE APP_MODE_INTERRUPTS
#endif

// Elimina la interrupción periódica de SysTick durante el ensayo.
#define APP_DISABLE_SYSTICK_DURING_TEST 1U

// Verifica durante compilación que el modo seleccionado sea válido.
#if ( APP_EXECUTION_MODE != APP_MODE_POLLING ) && ( APP_EXECUTION_MODE != APP_MODE_INTERRUPTS ) && \
	( APP_EXECUTION_MODE != APP_MODE_LOW_POWER )
#error "APP_EXECUTION_MODE no es valido"
#endif

// Contadores utilizados para evaluar el desempeño de cada estrategia.
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
