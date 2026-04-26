#include <cstdint>

#include "delay.h"
#include "i2c.h"
#include "stm32f4xx.h"

namespace {

constexpr std::uint8_t MT6701_ADDRESS_DEFAULT = 0x06U;
constexpr std::uint8_t MT6701_ADDRESS_ALT = 0x46U;

constexpr std::uint8_t MT6701_REG_ANGLE_MSB = 0x03U;
constexpr std::uint8_t MT6701_REG_ANGLE_LSB = 0x04U;

constexpr STM32F_PinAF_t I2C1_SDA_PIN = { PB( 9 ), 4U };
constexpr STM32F_PinAF_t I2C1_SCL_PIN = { PB( 8 ), 4U };

volatile bool g_mt6701_detected = false;
volatile std::uint8_t g_mt6701_address = MT6701_ADDRESS_DEFAULT;

volatile std::uint8_t g_mt6701_angle_msb = 0U;
volatile std::uint8_t g_mt6701_angle_lsb = 0U;

volatile std::uint16_t g_mt6701_raw_angle = 0U;
volatile double g_mt6701_angle_deg = 0.0;

bool MT6701_ReadRegister(
	first_project::I2C &i2c,
	std::uint8_t address,
	std::uint8_t reg,
	std::uint8_t &data ) {

	return i2c.writeRead( address, &reg, 1U, &data, 1U );
}

bool MT6701_ReadAngle( first_project::I2C &i2c, std::uint8_t address ) {
	std::uint8_t msb = 0U;
	std::uint8_t lsb = 0U;

	if( !MT6701_ReadRegister( i2c, address, MT6701_REG_ANGLE_MSB, msb ) ) {
		return false;
	}

	if( !MT6701_ReadRegister( i2c, address, MT6701_REG_ANGLE_LSB, lsb ) ) {
		return false;
	}

	g_mt6701_angle_msb = msb;
	g_mt6701_angle_lsb = lsb;

	g_mt6701_raw_angle = static_cast<std::uint16_t>(
		( static_cast<std::uint16_t>( msb ) << 6U ) |
		( static_cast<std::uint16_t>( lsb ) >> 2U ) );

	g_mt6701_angle_deg =
		static_cast<double>( g_mt6701_raw_angle ) * 360.0 / 16384.0;

	return true;
}

bool MT6701_Detect( first_project::I2C &i2c ) {
	if( i2c.testid( MT6701_ADDRESS_DEFAULT ) ) {
		g_mt6701_address = MT6701_ADDRESS_DEFAULT;
		return true;
	}

	if( i2c.testid( MT6701_ADDRESS_ALT ) ) {
		g_mt6701_address = MT6701_ADDRESS_ALT;
		return true;
	}

	return false;
}

} // namespace

int main( void ) {
	SystemCoreClockUpdate();

	first_project::initializeDelay();

	first_project::I2C i2c;
	i2c.start( I2C1_SDA_PIN, I2C1_SCL_PIN, 400000U );

	g_mt6701_detected = MT6701_Detect( i2c );

	std::uint32_t previous_sample_ms = 0U;

	while( 1 ) {
		if( first_project::every_ms( previous_sample_ms, 20U ) ) {
			if( g_mt6701_detected ) {
				g_mt6701_detected = MT6701_ReadAngle( i2c, g_mt6701_address );
			} else {
				g_mt6701_detected = MT6701_Detect( i2c );
			}
		}
	}
}