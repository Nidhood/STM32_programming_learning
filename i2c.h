#pragma once

#include <cstddef>
#include <cstdint>

#include "i2cbus.h"
#include "stm32f4xx.h"

namespace first_project {

class I2C : public I2cbus {
	private:
		static constexpr std::uint32_t kDefaultFrequencyHz = 100000U;
		static constexpr std::uint32_t kMaxStandardModeHz = 100000U;
		static constexpr std::uint32_t kMaxFastModeHz = 400000U;
		static constexpr std::uint32_t kTimeoutIterations = 100000U;
		static constexpr std::uint8_t kI2cAlternateFunction = 4U;

		I2C_TypeDef *port_ = nullptr;

		PinName_t sda_pin_ = PB( 9 );
		PinName_t scl_pin_ = PB( 8 );

		std::uint8_t sda_af_ = kI2cAlternateFunction;
		std::uint8_t scl_af_ = kI2cAlternateFunction;

		std::uint32_t frequency_hz_ = kDefaultFrequencyHz;

		FunInt2 rx_callback_ = nullptr;
		FunInt init_callback_ = nullptr;
		FunInt finish_callback_ = nullptr;

		static std::uint32_t getHCLK();
		static std::uint32_t decodeAHBPrescaler( std::uint32_t hpre_bits );
		static std::uint32_t decodeAPBPrescaler( std::uint32_t ppre_bits );
		static std::uint32_t getPCLK1();

		static std::uint8_t normalizeAddress( std::uint8_t address );
		static std::uint8_t makeAddressByte( std::uint8_t address, bool read );

		bool selectPortFromPins( PinName_t sda, PinName_t scl );
		bool configurePins();
		bool enablePeripheralClock();
		void resetPeripheral();

		void configurePeripheral( std::uint32_t frequency_hz );

		bool isConfigured() const;

		bool waitFlagSet( volatile std::uint32_t &reg, std::uint32_t flag ) const;
		bool waitFlagCleared( volatile std::uint32_t &reg, std::uint32_t flag ) const;
		bool waitAddressSent() const;
		bool waitBusFree() const;

		void clearAddressFlag() const;
		void clearAcknowledgeFailure() const;
		void generateStart() const;
		void generateStop() const;

		bool sendAddress( std::uint8_t address, bool read );
		bool readPayload( std::uint8_t *data, std::size_t length );

	public:
		I2C() = default;

		void start() override;
		void start( unsigned int frequency_hz ) override;
		void start( unsigned char sda, unsigned char scl, unsigned int frequency_hz ) override;

		void start(
			unsigned char sda,
			unsigned char scl,
			unsigned int frequency_hz,
			unsigned char own_address );

		void start(
			STM32F_PinAF_t sda,
			STM32F_PinAF_t scl,
			unsigned int frequency_hz,
			unsigned char own_address = 0U );

		bool testid( unsigned char address ) override;

		int rxi2c( unsigned char *data );

		void dmatx( bool enable );
		void dmarx( bool enable );

		void txi2c( unsigned char address, const unsigned char *data, int length ) override;
		void txi2c( unsigned char address, unsigned char data, int length ) override;
		void txi2c( unsigned char address, unsigned char data ) override;

		unsigned char rxi2c( unsigned char address ) override;
		void rxi2c( unsigned char address, unsigned char *data, int length ) override;

		bool write(
			std::uint8_t address,
			const std::uint8_t *data,
			std::size_t length ) override;

		bool read(
			std::uint8_t address,
			std::uint8_t *data,
			std::size_t length ) override;

		bool writeRead(
			std::uint8_t address,
			const std::uint8_t *tx_data,
			std::size_t tx_length,
			std::uint8_t *rx_data,
			std::size_t rx_length ) override;

		void setfuncrx( FunInt2 callback );
		void setfunini( FunInt callback );
		void setfunfin( FunInt callback );
};

} // namespace first_project