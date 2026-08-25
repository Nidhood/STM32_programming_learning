#ifndef MAIN_H
#define MAIN_H

#include "stm32f4xx_hal.h"

#define SAMPLE_RATE_HZ 2000U
#define TIMER_INPUT_CLOCK_HZ 84000000U
#define TIMER_TICK_HZ 1000000U

#define UART_BAUD_RATE 115200U
#define FRAME_SAMPLES 32U
#define FRAME_PAYLOAD_BYTES ( 2U * FRAME_SAMPLES )

/* TIM2_CH1: referencia hardware exacta del instante de muestreo. */
#define DEBUG_TIMER_REF_PORT GPIOA
#define DEBUG_TIMER_REF_PIN GPIO_PIN_5

/* Pulso generado cuando el software atiende el evento de TIM2. */
#define DEBUG_SAMPLE_PORT GPIOB
#define DEBUG_SAMPLE_PIN GPIO_PIN_5

/* Nivel alto desde el inicio hasta el final de una adquisición ADC. */
#define DEBUG_ADC_PORT GPIOB
#define DEBUG_ADC_PIN GPIO_PIN_4

/* Nivel alto mientras se procesa y empaqueta una muestra. */
#define DEBUG_PROCESS_PORT GPIOA
#define DEBUG_PROCESS_PIN GPIO_PIN_8

/* Cada flanco representa un bloque de 32 muestras terminado. */
#define DEBUG_FRAME_PORT GPIOA
#define DEBUG_FRAME_PIN GPIO_PIN_9

/* Nivel alto mientras existe una transmisión UART activa. */
#define DEBUG_UART_PORT GPIOB
#define DEBUG_UART_PIN GPIO_PIN_7

/* Nivel alto: CPU activa. Nivel bajo: ciclo finalizado o CPU dormida. */
#define DEBUG_SLEEP_PORT GPIOC
#define DEBUG_SLEEP_PIN GPIO_PIN_10

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;

void Error_Handler( void );

#endif
