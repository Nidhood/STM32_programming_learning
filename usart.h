#pragma once

#include <cstddef>
#include <cstdint>

#include "stm32f4xx.h"

namespace first_project {

class USART {
	private:
		static constexpr std::uint32_t kDefaultBaudrate = 115200U;
		static constexpr std::uint32_t kTimeoutIterations = 100000U;

		USART_TypeDef *port_ = nullptr;
		PinName_t tx_pin_ = PA( 2 );
		PinName_t rx_pin_ = PA( 3 );
		std::uint32_t baudrate_ = kDefaultBaudrate;

		static std::uint32_t decodeAHBPrescaler( std::uint32_t hpre_bits );
		static std::uint32_t decodeAPBPrescaler( std::uint32_t ppre_bits );

		static std::uint32_t getHCLK();
		static std::uint32_t getPCLK1();
		static std::uint32_t getPCLK2();

		bool selectPortFromPins( PinName_t tx, PinName_t rx );
		bool configurePins( STM32F_PinAF_t tx, STM32F_PinAF_t rx );
		bool enablePeripheralClock();
		void resetPeripheral();

		std::uint32_t getPeripheralClock() const;

		bool waitFlagSet( volatile std::uint32_t &reg, std::uint32_t flag ) const;
		bool isConfigured() const;

		void configurePeripheral( std::uint32_t baudrate );

	public:
		USART() = default;
		explicit USART( std::uint32_t baudrate );

		void start();
		void start( std::uint32_t baudrate );
		void start( STM32F_PinAF_t tx, STM32F_PinAF_t rx, std::uint32_t baudrate = kDefaultBaudrate );

		bool write( std::uint8_t data );
		std::size_t write( const std::uint8_t *data, std::size_t length );
		std::size_t write( const char *text );

		bool available() const;
		bool read( std::uint8_t &data );
		std::uint8_t read();

		void flush();
};

} // namespace first_project