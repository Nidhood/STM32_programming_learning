#pragma once

#include "stm32f7xx_hal.h"

// Frecuencia de adquisición y temporización.
#define SAMPLE_RATE_HZ 2000U
#define TIMER_INPUT_CLOCK_HZ 108000000U
#define TIMER_TICK_HZ 1000000U

// Configuración de comunicación y trama.
#define UART_BAUD_RATE 115200U
#define FRAME_SAMPLES 1U
#define FRAME_PAYLOAD_BYTES ( 2U * FRAME_SAMPLES )

// TIM2_CH1: referencia hardware exacta del periodo de muestreo.
#define DEBUG_TIMER_REF_PORT GPIOA
#define DEBUG_TIMER_REF_PIN GPIO_PIN_5

// Pulso generado cuando el software atiende un evento de muestreo.
#define DEBUG_SAMPLE_PORT GPIOB
#define DEBUG_SAMPLE_PIN GPIO_PIN_5

// Nivel alto durante la adquisición ADC.
#define DEBUG_ADC_PORT GPIOB
#define DEBUG_ADC_PIN GPIO_PIN_4

// Nivel alto durante el procesamiento y empaquetado de una muestra.
#define DEBUG_PROCESS_PORT GPIOA
#define DEBUG_PROCESS_PIN GPIO_PIN_8

// Cada flanco representa una trama de muestras terminada.
#define DEBUG_FRAME_PORT GPIOA
#define DEBUG_FRAME_PIN GPIO_PIN_9

// Nivel alto mientras existe una transmisión UART activa.
#define DEBUG_UART_PORT GPIOD
#define DEBUG_UART_PIN GPIO_PIN_15

// Nivel alto: CPU activa. Nivel bajo: ciclo finalizado o CPU dormida.
#define DEBUG_SLEEP_PORT GPIOC
#define DEBUG_SLEEP_PIN GPIO_PIN_7

// Handles globales de los periféricos utilizados por la aplicación.
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart3;

void Error_Handler( void );
