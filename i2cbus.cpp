#include "i2cbus.h"

void first_project::I2cbus::start() {
	start( 100000U );
}

void first_project::I2cbus::start( unsigned int frequency_hz ) {
	(void)frequency_hz;
}

void first_project::I2cbus::start( unsigned char sda, unsigned char scl, unsigned int frequency_hz ) {
	(void)sda;
	(void)scl;
	(void)frequency_hz;
}

bool first_project::I2cbus::testid( unsigned char address ) {
	(void)address;
	return false;
}

void first_project::I2cbus::txi2c( unsigned char address, const unsigned char *data, int length ) {
	(void)address;
	(void)data;
	(void)length;
}

void first_project::I2cbus::txi2c( unsigned char address, unsigned char data, int length ) {
	(void)address;
	(void)data;
	(void)length;
}

void first_project::I2cbus::txi2c( unsigned char address, unsigned char data ) {
	(void)address;
	(void)data;
}

unsigned char first_project::I2cbus::rxi2c( unsigned char address ) {
	(void)address;
	return 0U;
}

void first_project::I2cbus::rxi2c( unsigned char address, unsigned char *data, int length ) {
	(void)address;
	(void)data;
	(void)length;
}

bool first_project::I2cbus::write( std::uint8_t address, const std::uint8_t *data, std::size_t length ) {
	(void)address;
	(void)data;
	(void)length;
	return false;
}

bool first_project::I2cbus::read( std::uint8_t address, std::uint8_t *data, std::size_t length ) {
	(void)address;
	(void)data;
	(void)length;
	return false;
}

bool first_project::I2cbus::writeRead(
	std::uint8_t address,
	const std::uint8_t *tx_data,
	std::size_t tx_length,
	std::uint8_t *rx_data,
	std::size_t rx_length ) {
	(void)address;
	(void)tx_data;
	(void)tx_length;
	(void)rx_data;
	(void)rx_length;
	return false;
}