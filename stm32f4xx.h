#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#if defined( STM32F401xE )
#include "stm32f401xe.h"
#else
#error "Unsupported STM32 device. Define STM32F401xE in the project preprocessor symbols."
#endif

#define STM32F_DEVICE_F401RE 1
#define STM32F_DEVICE_F401XE 1
#define STM32F_CORTEX_M4 1
#define STM32F_CORTEX_M7 0

typedef void ( *FunInt )( void );
typedef void ( *FunInt2 )( uint8_t data );

typedef uint8_t PinName_t;

typedef enum {
	STM32F_PORT_A = 0U,
	STM32F_PORT_B = 1U,
	STM32F_PORT_C = 2U,
	STM32F_PORT_D = 3U,
	STM32F_PORT_E = 4U,
	STM32F_PORT_F = 5U,
	STM32F_PORT_G = 6U,
	STM32F_PORT_H = 7U,
	STM32F_PORT_I = 8U,
	STM32F_PORT_J = 9U,
	STM32F_PORT_K = 10U
} STM32F_PortCode_t;

#define STM32F_PIN_INVALID ( (PinName_t)0xFFU )

#define STM32F_PIN_MAKE( port, number ) \
	( (PinName_t)( ( ( (uint8_t)( port ) ) << 4U ) | ( ( (uint8_t)( number ) ) & 0x0FU ) ) )

#define STM32F_PIN_PORT( pin ) ( (uint8_t)( ( pin ) >> 4U ) )
#define STM32F_PIN_NUMBER( pin ) ( (uint8_t)( ( pin ) & 0x0FU ) )
#define STM32F_PIN_MASK( pin ) ( (uint16_t)( 1UL << STM32F_PIN_NUMBER( pin ) ) )

#define PA( n ) STM32F_PIN_MAKE( STM32F_PORT_A, ( n ) )
#define PB( n ) STM32F_PIN_MAKE( STM32F_PORT_B, ( n ) )
#define PC( n ) STM32F_PIN_MAKE( STM32F_PORT_C, ( n ) )
#define PD( n ) STM32F_PIN_MAKE( STM32F_PORT_D, ( n ) )
#define PE( n ) STM32F_PIN_MAKE( STM32F_PORT_E, ( n ) )
#define PF( n ) STM32F_PIN_MAKE( STM32F_PORT_F, ( n ) )
#define PG( n ) STM32F_PIN_MAKE( STM32F_PORT_G, ( n ) )
#define PH( n ) STM32F_PIN_MAKE( STM32F_PORT_H, ( n ) )
#define PI( n ) STM32F_PIN_MAKE( STM32F_PORT_I, ( n ) )
#define PJ( n ) STM32F_PIN_MAKE( STM32F_PORT_J, ( n ) )
#define PK( n ) STM32F_PIN_MAKE( STM32F_PORT_K, ( n ) )

#ifndef GPIO_AF0_SYSTEM
#define GPIO_AF0_SYSTEM ( (uint8_t)0x00U )
#endif

#ifndef GPIO_AF1_TIM1
#define GPIO_AF1_TIM1 ( (uint8_t)0x01U )
#endif

#ifndef GPIO_AF1_TIM2
#define GPIO_AF1_TIM2 ( (uint8_t)0x01U )
#endif

#ifndef GPIO_AF2_TIM3
#define GPIO_AF2_TIM3 ( (uint8_t)0x02U )
#endif

#ifndef GPIO_AF2_TIM4
#define GPIO_AF2_TIM4 ( (uint8_t)0x02U )
#endif

#ifndef GPIO_AF2_TIM5
#define GPIO_AF2_TIM5 ( (uint8_t)0x02U )
#endif

#ifndef GPIO_AF3_TIM9
#define GPIO_AF3_TIM9 ( (uint8_t)0x03U )
#endif

#ifndef GPIO_AF3_TIM10
#define GPIO_AF3_TIM10 ( (uint8_t)0x03U )
#endif

#ifndef GPIO_AF3_TIM11
#define GPIO_AF3_TIM11 ( (uint8_t)0x03U )
#endif

#ifndef GPIO_AF4_I2C1
#define GPIO_AF4_I2C1 ( (uint8_t)0x04U )
#endif

#ifndef GPIO_AF4_I2C2
#define GPIO_AF4_I2C2 ( (uint8_t)0x04U )
#endif

#ifndef GPIO_AF4_I2C3
#define GPIO_AF4_I2C3 ( (uint8_t)0x04U )
#endif

#ifndef GPIO_AF5_SPI1
#define GPIO_AF5_SPI1 ( (uint8_t)0x05U )
#endif

#ifndef GPIO_AF5_SPI2
#define GPIO_AF5_SPI2 ( (uint8_t)0x05U )
#endif

#ifndef GPIO_AF5_SPI3
#define GPIO_AF5_SPI3 ( (uint8_t)0x05U )
#endif

#ifndef GPIO_AF6_SPI2
#define GPIO_AF6_SPI2 ( (uint8_t)0x06U )
#endif

#ifndef GPIO_AF6_SPI3
#define GPIO_AF6_SPI3 ( (uint8_t)0x06U )
#endif

#ifndef GPIO_AF7_USART1
#define GPIO_AF7_USART1 ( (uint8_t)0x07U )
#endif

#ifndef GPIO_AF7_USART2
#define GPIO_AF7_USART2 ( (uint8_t)0x07U )
#endif

#ifndef GPIO_AF8_USART6
#define GPIO_AF8_USART6 ( (uint8_t)0x08U )
#endif

#ifndef GPIO_AF9_I2C2
#define GPIO_AF9_I2C2 ( (uint8_t)0x09U )
#endif

#ifndef GPIO_AF9_I2C3
#define GPIO_AF9_I2C3 ( (uint8_t)0x09U )
#endif

#ifndef GPIO_AF10_OTG_FS
#define GPIO_AF10_OTG_FS ( (uint8_t)0x0AU )
#endif

#ifndef GPIO_AF12_SDIO
#define GPIO_AF12_SDIO ( (uint8_t)0x0CU )
#endif

#ifndef GPIO_AF15_EVENTOUT
#define GPIO_AF15_EVENTOUT ( (uint8_t)0x0FU )
#endif

typedef struct {
		PinName_t pin;
		uint8_t af;
} STM32F_PinAF_t;

static inline GPIO_TypeDef *STM32F_GPIOFromPin( PinName_t pin ) {
	switch( STM32F_PIN_PORT( pin ) ) {
#if defined( GPIOA )
		case STM32F_PORT_A:
			return GPIOA;
#endif
#if defined( GPIOB )
		case STM32F_PORT_B:
			return GPIOB;
#endif
#if defined( GPIOC )
		case STM32F_PORT_C:
			return GPIOC;
#endif
#if defined( GPIOD )
		case STM32F_PORT_D:
			return GPIOD;
#endif
#if defined( GPIOH )
		case STM32F_PORT_H:
			return GPIOH;
#endif
		default:
			return NULL;
	}
}

static inline bool STM32F401RE_PinIsPackagePin( PinName_t pin ) {
	const uint8_t port = STM32F_PIN_PORT( pin );
	const uint8_t number = STM32F_PIN_NUMBER( pin );

	switch( port ) {
		case STM32F_PORT_A:
		case STM32F_PORT_B:
		case STM32F_PORT_C:
			return number <= 15U;

		case STM32F_PORT_D:
			return number == 2U;

		case STM32F_PORT_H:
			return number <= 1U;

		default:
			return false;
	}
}

static inline bool STM32F_PinIsAvailable( PinName_t pin ) {
	return ( STM32F_GPIOFromPin( pin ) != NULL ) && STM32F401RE_PinIsPackagePin( pin );
}

static inline void STM32F_GPIOClockEnable( GPIO_TypeDef *gpio ) {
	if( gpio == NULL ) {
		return;
	}

#if defined( GPIOA ) && defined( RCC_AHB1ENR_GPIOAEN )
	if( gpio == GPIOA ) {
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
		(void)RCC->AHB1ENR;
		return;
	}
#endif

#if defined( GPIOB ) && defined( RCC_AHB1ENR_GPIOBEN )
	if( gpio == GPIOB ) {
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
		(void)RCC->AHB1ENR;
		return;
	}
#endif

#if defined( GPIOC ) && defined( RCC_AHB1ENR_GPIOCEN )
	if( gpio == GPIOC ) {
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
		(void)RCC->AHB1ENR;
		return;
	}
#endif

#if defined( GPIOD ) && defined( RCC_AHB1ENR_GPIODEN )
	if( gpio == GPIOD ) {
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
		(void)RCC->AHB1ENR;
		return;
	}
#endif

#if defined( GPIOH ) && defined( RCC_AHB1ENR_GPIOHEN )
	if( gpio == GPIOH ) {
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
		(void)RCC->AHB1ENR;
		return;
	}
#endif
}

static inline void STM32F_GPIOClockEnableFromPin( PinName_t pin ) {
	STM32F_GPIOClockEnable( STM32F_GPIOFromPin( pin ) );
}