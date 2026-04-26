#include "pin.h"

bool Pin_Init( const Pin_Config_t *config ) {
	if( config == NULL ) {
		return false;
	}

	GPIO_TypeDef *gpio = STM32F_GPIOFromPin( config->pin );

	if( gpio == NULL ) {
		return false;
	}

	uint32_t pin_number = STM32F_PIN_NUMBER( config->pin );
	uint32_t pin_position_2bit = pin_number * 2U;
	uint32_t pin_position_4bit = ( pin_number % 8U ) * 4U;
	uint32_t afr_index = pin_number / 8U;

	STM32F_GPIOClockEnable( gpio );

	gpio->MODER &= ~( 0x3UL << pin_position_2bit );
	gpio->MODER |= ( ( (uint32_t)config->mode & 0x3UL ) << pin_position_2bit );

	gpio->OTYPER &= ~( 1UL << pin_number );
	gpio->OTYPER |= ( ( (uint32_t)config->output_type & 0x1UL ) << pin_number );

	gpio->OSPEEDR &= ~( 0x3UL << pin_position_2bit );
	gpio->OSPEEDR |= ( ( (uint32_t)config->speed & 0x3UL ) << pin_position_2bit );

	gpio->PUPDR &= ~( 0x3UL << pin_position_2bit );
	gpio->PUPDR |= ( ( (uint32_t)config->pull & 0x3UL ) << pin_position_2bit );

	if( config->mode == PIN_MODE_AF ) {
		gpio->AFR[afr_index] &= ~( 0xFUL << pin_position_4bit );
		gpio->AFR[afr_index] |= ( ( (uint32_t)config->alternate_function & 0xFUL ) << pin_position_4bit );
	}

	return true;
}

void Pin_Write( PinName_t pin, bool state ) {
	GPIO_TypeDef *gpio = STM32F_GPIOFromPin( pin );

	if( gpio == NULL ) {
		return;
	}

	uint32_t mask = STM32F_PIN_MASK( pin );

	if( state ) {
		gpio->BSRR = mask;
	} else {
		gpio->BSRR = mask << 16U;
	}
}

bool Pin_Read( PinName_t pin ) {
	GPIO_TypeDef *gpio = STM32F_GPIOFromPin( pin );

	if( gpio == NULL ) {
		return false;
	}

	return ( gpio->IDR & STM32F_PIN_MASK( pin ) ) != 0U;
}

void Pin_Toggle( PinName_t pin ) {
	GPIO_TypeDef *gpio = STM32F_GPIOFromPin( pin );

	if( gpio == NULL ) {
		return;
	}

	uint32_t mask = STM32F_PIN_MASK( pin );

	if( ( gpio->ODR & mask ) != 0U ) {
		gpio->BSRR = mask << 16U;
	} else {
		gpio->BSRR = mask;
	}
}