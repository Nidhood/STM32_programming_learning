#pragma once

#include "stm32f4xx.h"

#define PIN_PA0 PA( 0 )
#define PIN_PA1 PA( 1 )
#define PIN_PA2 PA( 2 )
#define PIN_PA3 PA( 3 )
#define PIN_PA4 PA( 4 )
#define PIN_PA5 PA( 5 )
#define PIN_PA6 PA( 6 )
#define PIN_PA7 PA( 7 )
#define PIN_PA8 PA( 8 )
#define PIN_PA9 PA( 9 )
#define PIN_PA10 PA( 10 )
#define PIN_PA11 PA( 11 )
#define PIN_PA12 PA( 12 )
#define PIN_PA13 PA( 13 )
#define PIN_PA14 PA( 14 )
#define PIN_PA15 PA( 15 )

#define PIN_PB0 PB( 0 )
#define PIN_PB1 PB( 1 )
#define PIN_PB2 PB( 2 )
#define PIN_PB3 PB( 3 )
#define PIN_PB4 PB( 4 )
#define PIN_PB5 PB( 5 )
#define PIN_PB6 PB( 6 )
#define PIN_PB7 PB( 7 )
#define PIN_PB8 PB( 8 )
#define PIN_PB9 PB( 9 )
#define PIN_PB10 PB( 10 )
#define PIN_PB11 PB( 11 )
#define PIN_PB12 PB( 12 )
#define PIN_PB13 PB( 13 )
#define PIN_PB14 PB( 14 )
#define PIN_PB15 PB( 15 )

#define PIN_PC0 PC( 0 )
#define PIN_PC1 PC( 1 )
#define PIN_PC2 PC( 2 )
#define PIN_PC3 PC( 3 )
#define PIN_PC4 PC( 4 )
#define PIN_PC5 PC( 5 )
#define PIN_PC6 PC( 6 )
#define PIN_PC7 PC( 7 )
#define PIN_PC8 PC( 8 )
#define PIN_PC9 PC( 9 )
#define PIN_PC10 PC( 10 )
#define PIN_PC11 PC( 11 )
#define PIN_PC12 PC( 12 )
#define PIN_PC13 PC( 13 )
#define PIN_PC14 PC( 14 )
#define PIN_PC15 PC( 15 )

#define PIN_PD2 PD( 2 )

#define PIN_PH0 PH( 0 )
#define PIN_PH1 PH( 1 )

#define NUCLEO_LD2_PIN PA( 5 )
#define NUCLEO_USER_BUTTON_PIN PC( 13 )

#define NUCLEO_VCP_USART USART2
#define NUCLEO_VCP_TX_PIN PA( 2 )
#define NUCLEO_VCP_RX_PIN PA( 3 )
#define NUCLEO_VCP_TX_AF GPIO_AF7_USART2
#define NUCLEO_VCP_RX_AF GPIO_AF7_USART2

static const STM32F_PinAF_t NUCLEO_VCP_TX = {
	NUCLEO_VCP_TX_PIN,
	NUCLEO_VCP_TX_AF };

static const STM32F_PinAF_t NUCLEO_VCP_RX = {
	NUCLEO_VCP_RX_PIN,
	NUCLEO_VCP_RX_AF };

/*
 * Common I2C1 mapping on Nucleo-F401RE.
 */
#define NUCLEO_I2C1 I2C1
#define NUCLEO_I2C1_SCL_PIN PB( 8 )
#define NUCLEO_I2C1_SDA_PIN PB( 9 )
#define NUCLEO_I2C1_SCL_AF GPIO_AF4_I2C1
#define NUCLEO_I2C1_SDA_AF GPIO_AF4_I2C1

static const STM32F_PinAF_t NUCLEO_I2C1_SCL = {
	NUCLEO_I2C1_SCL_PIN,
	NUCLEO_I2C1_SCL_AF };

static const STM32F_PinAF_t NUCLEO_I2C1_SDA = {
	NUCLEO_I2C1_SDA_PIN,
	NUCLEO_I2C1_SDA_AF };

/*
 * Common SPI1 mapping.
 * Warning: PA5 is also connected to LD2 on the Nucleo board.
 */
#define NUCLEO_SPI1 SPI1
#define NUCLEO_SPI1_NSS_PIN PA( 4 )
#define NUCLEO_SPI1_SCK_PIN PA( 5 )
#define NUCLEO_SPI1_MISO_PIN PA( 6 )
#define NUCLEO_SPI1_MOSI_PIN PA( 7 )

#define NUCLEO_SPI1_NSS_AF GPIO_AF5_SPI1
#define NUCLEO_SPI1_SCK_AF GPIO_AF5_SPI1
#define NUCLEO_SPI1_MISO_AF GPIO_AF5_SPI1
#define NUCLEO_SPI1_MOSI_AF GPIO_AF5_SPI1

static const STM32F_PinAF_t NUCLEO_SPI1_NSS = {
	NUCLEO_SPI1_NSS_PIN,
	NUCLEO_SPI1_NSS_AF };

static const STM32F_PinAF_t NUCLEO_SPI1_SCK = {
	NUCLEO_SPI1_SCK_PIN,
	NUCLEO_SPI1_SCK_AF };

static const STM32F_PinAF_t NUCLEO_SPI1_MISO = {
	NUCLEO_SPI1_MISO_PIN,
	NUCLEO_SPI1_MISO_AF };

static const STM32F_PinAF_t NUCLEO_SPI1_MOSI = {
	NUCLEO_SPI1_MOSI_PIN,
	NUCLEO_SPI1_MOSI_AF };

/*
 * Common SPI2 mapping.
 */
#define NUCLEO_SPI2 SPI2
#define NUCLEO_SPI2_NSS_PIN PB( 12 )
#define NUCLEO_SPI2_SCK_PIN PB( 13 )
#define NUCLEO_SPI2_MISO_PIN PB( 14 )
#define NUCLEO_SPI2_MOSI_PIN PB( 15 )

#define NUCLEO_SPI2_NSS_AF GPIO_AF5_SPI2
#define NUCLEO_SPI2_SCK_AF GPIO_AF5_SPI2
#define NUCLEO_SPI2_MISO_AF GPIO_AF5_SPI2
#define NUCLEO_SPI2_MOSI_AF GPIO_AF5_SPI2

static const STM32F_PinAF_t NUCLEO_SPI2_NSS = {
	NUCLEO_SPI2_NSS_PIN,
	NUCLEO_SPI2_NSS_AF };

static const STM32F_PinAF_t NUCLEO_SPI2_SCK = {
	NUCLEO_SPI2_SCK_PIN,
	NUCLEO_SPI2_SCK_AF };

static const STM32F_PinAF_t NUCLEO_SPI2_MISO = {
	NUCLEO_SPI2_MISO_PIN,
	NUCLEO_SPI2_MISO_AF };

static const STM32F_PinAF_t NUCLEO_SPI2_MOSI = {
	NUCLEO_SPI2_MOSI_PIN,
	NUCLEO_SPI2_MOSI_AF };

/*
 * SWD debug pins.
 * Do not reuse these pins unless you intentionally disable or avoid debugging.
 */
#define NUCLEO_SWDIO_PIN PA( 13 )
#define NUCLEO_SWCLK_PIN PA( 14 )
#define NUCLEO_SWO_PIN PB( 3 )

static inline bool NUCLEO_F401RE_PinIsDebugPin( PinName_t pin ) {
	return ( pin == NUCLEO_SWDIO_PIN ) ||
		   ( pin == NUCLEO_SWCLK_PIN ) ||
		   ( pin == NUCLEO_SWO_PIN );
}

static inline bool NUCLEO_F401RE_PinIsBoardUsedPin( PinName_t pin ) {
	return ( pin == NUCLEO_LD2_PIN ) ||
		   ( pin == NUCLEO_USER_BUTTON_PIN ) ||
		   ( pin == NUCLEO_VCP_TX_PIN ) ||
		   ( pin == NUCLEO_VCP_RX_PIN ) ||
		   NUCLEO_F401RE_PinIsDebugPin( pin );
}