#pragma once

#include <cstddef>
#include <cstdint>

namespace first_project {

class I2cbus {
	public:
		I2cbus() = default;
		virtual ~I2cbus() = default;

		virtual void start();
		virtual void start( unsigned int frequency_hz );
		virtual void start( unsigned char sda, unsigned char scl, unsigned int frequency_hz );

		virtual bool testid( unsigned char address );

		virtual void txi2c( unsigned char address, const unsigned char *data, int length );
		virtual void txi2c( unsigned char address, unsigned char data, int length );
		virtual void txi2c( unsigned char address, unsigned char data );

		virtual unsigned char rxi2c( unsigned char address );
		virtual void rxi2c( unsigned char address, unsigned char *data, int length );

		virtual bool write( std::uint8_t address, const std::uint8_t *data, std::size_t length );
		virtual bool read( std::uint8_t address, std::uint8_t *data, std::size_t length );
		virtual bool writeRead(
			std::uint8_t address,
			const std::uint8_t *tx_data,
			std::size_t tx_length,
			std::uint8_t *rx_data,
			std::size_t rx_length );
};

} // namespace first_project