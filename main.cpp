#include <cstdint>

#include "delay.h"
#include "i2c.h"
#include "stm32f4xx.h"
#include "usart.h"

namespace {

constexpr std::uint8_t MT6701_ADDRESS_DEFAULT = 0x06U;
constexpr std::uint8_t MT6701_ADDRESS_ALT = 0x46U;

constexpr std::uint8_t MT6701_REG_ANGLE_MSB = 0x03U;

constexpr STM32F_PinAF_t I2C1_SDA_PIN = { PB( 9 ), 4U };
constexpr STM32F_PinAF_t I2C1_SCL_PIN = { PB( 8 ), 4U };

constexpr STM32F_PinAF_t USART2_TX_PIN = { PA( 2 ), 7U };
constexpr STM32F_PinAF_t USART2_RX_PIN = { PA( 3 ), 7U };

volatile bool g_mt6701_detected = false;
volatile std::uint8_t g_mt6701_address = MT6701_ADDRESS_DEFAULT;

volatile std::uint8_t g_mt6701_angle_msb = 0U;
volatile std::uint8_t g_mt6701_angle_lsb = 0U;

volatile std::uint16_t g_mt6701_raw_angle = 0U;
volatile std::uint32_t g_mt6701_angle_cdeg = 0U;

void Serial_PrintUnsigned( first_project::USART &serial, std::uint32_t value ) {
	char buffer[10];
	std::uint8_t index = 0U;

	if( value == 0U ) {
		serial.write( "0" );
		return;
	}

	while( value > 0U && index < sizeof( buffer ) ) {
		buffer[index] = static_cast<char>( '0' + ( value % 10U ) );
		value /= 10U;
		++index;
	}

	while( index > 0U ) {
		--index;
		serial.write( static_cast<std::uint8_t>( buffer[index] ) );
	}
}

void Serial_PrintHex8( first_project::USART &serial, std::uint8_t value ) {
	constexpr char hex[] = "0123456789ABCDEF";

	serial.write( "0x" );
	serial.write( static_cast<std::uint8_t>( hex[( value >> 4U ) & 0x0FU] ) );
	serial.write( static_cast<std::uint8_t>( hex[value & 0x0FU] ) );
}

void Serial_PrintAngle( first_project::USART &serial, std::uint32_t angle_cdeg ) {
	const std::uint32_t integer_part = angle_cdeg / 100U;
	const std::uint32_t decimal_part = angle_cdeg % 100U;

	Serial_PrintUnsigned( serial, integer_part );
	serial.write( "." );

	if( decimal_part < 10U ) {
		serial.write( "0" );
	}

	Serial_PrintUnsigned( serial, decimal_part );
}

bool MT6701_ReadAngle( first_project::I2C &i2c, std::uint8_t address ) {
	std::uint8_t reg = MT6701_REG_ANGLE_MSB;
	std::uint8_t data[2] = { 0U, 0U };

	if( !i2c.writeRead( address, &reg, 1U, data, 2U ) ) {
		return false;
	}

	g_mt6701_angle_msb = data[0];
	g_mt6701_angle_lsb = data[1];

	g_mt6701_raw_angle = static_cast<std::uint16_t>(
		( static_cast<std::uint16_t>( data[0] ) << 6U ) |
		( static_cast<std::uint16_t>( data[1] ) >> 2U ) );

	g_mt6701_angle_cdeg =
		( static_cast<std::uint32_t>( g_mt6701_raw_angle ) * 36000U ) / 16384U;

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

void MT6701_PrintStatus( first_project::USART &serial ) {
	if( g_mt6701_detected ) {
		serial.write( "MT6701 detected at address " );
		Serial_PrintHex8( serial, g_mt6701_address );
		serial.write( "\r\n" );
	} else {
		serial.write( "MT6701 not detected\r\n" );
	}
}

void MT6701_PrintData( first_project::USART &serial ) {
	//serial.write( "raw=" );
	//Serial_PrintUnsigned( serial, g_mt6701_raw_angle );

	//serial.write( " angle=" );
	Serial_PrintAngle( serial, g_mt6701_angle_cdeg / 360.0 );
	//serial.write( " deg" );

	//serial.write( " msb=" );
	//Serial_PrintHex8( serial, g_mt6701_angle_msb );

	//serial.write( " lsb=" );
	//Serial_PrintHex8( serial, g_mt6701_angle_lsb );

	serial.write( "\r\n" );
}

} // namespace

int main( void ) {
	SystemCoreClockUpdate();

	first_project::initializeDelay();

	first_project::USART serial;
	serial.start( USART2_TX_PIN, USART2_RX_PIN, 115200U );

	serial.write( "\r\n" );
	serial.write( "================================\r\n" );
	serial.write( "MT6701 Magnetic Encoder Monitor\r\n" );
	serial.write( "USART2: PA2 TX / PA3 RX\r\n" );
	serial.write( "Baudrate: 115200\r\n" );
	serial.write( "================================\r\n" );

	first_project::I2C i2c;
	i2c.start( I2C1_SDA_PIN, I2C1_SCL_PIN, 400000U );

	g_mt6701_detected = MT6701_Detect( i2c );
	MT6701_PrintStatus( serial );

	std::uint32_t previous_sample_ms = 0U;
	std::uint32_t previous_detection_ms = 0U;

	while( 1 ) {
		if( g_mt6701_detected ) {
			if( first_project::every_ms( previous_sample_ms, 100U ) ) {
				g_mt6701_detected = MT6701_ReadAngle( i2c, g_mt6701_address );

				if( g_mt6701_detected ) {
					MT6701_PrintData( serial );
				} else {
					serial.write( "MT6701 communication lost\r\n" );
				}
			}
		} else {
			if( first_project::every_ms( previous_detection_ms, 1000U ) ) {
				g_mt6701_detected = MT6701_Detect( i2c );
				MT6701_PrintStatus( serial );
			}
		}
	}
}