#include "servo_control_app.h"

#include "delay.h"

namespace first_project {

void ServoControlApp::setup() {
	initializePeripherals();

	printBootBanner();

	initializeServoController();

	previous_update_ms_ = millis();
}

void ServoControlApp::loop() {
	runDemoTask();
}

void ServoControlApp::initializePeripherals() {
	initializeDelay();

	serial_.start(
		kUsart2TxPin,
		kUsart2RxPin,
		kSerialBaudrate );

	i2c_.start(
		kI2C1SdaPin,
		kI2C1SclPin,
		kI2CFrequencyHz );
}

void ServoControlApp::initializeServoController() {
	status_ = servo_->begin( kServoFrequencyHz );

	serial_.write( "[SERVO] begin status: " );
	printStatus( status_ );
	serial_.write( "\r\n" );

	if( status_ != ServoStatus::Ok ) {
		serial_.write( "[SERVO] Check I2C address, SDA, SCL, VCC, GND and servo VIN.\r\n" );
		return;
	}

	status_ = servo_->setNeutral( kServoTestChannel );

	serial_.write( "[SERVO] neutral status: " );
	printStatus( status_ );
	serial_.write( "\r\n" );
}

void ServoControlApp::runDemoTask() {
	if( status_ != ServoStatus::Ok ) {
		return;
	}

	if( !every_ms( previous_update_ms_, kServoUpdatePeriodMs ) ) {
		return;
	}

	float angle_deg = 90.0F;

	if( demo_step_ == 0U ) {
		angle_deg = 0.0F;
		demo_step_ = 1U;
	} else if( demo_step_ == 1U ) {
		angle_deg = 90.0F;
		demo_step_ = 2U;
	} else {
		angle_deg = 180.0F;
		demo_step_ = 0U;
	}

	status_ =
		servo_->setAngleDeg(
			kServoTestChannel,
			angle_deg,
			test_servo_range_ );

	serial_.write( "[SERVO] channel=" );
	printUnsigned( kServoTestChannel );

	serial_.write( " angle_deg=" );
	printUnsigned( static_cast<std::uint32_t>( angle_deg ) );

	serial_.write( " status=" );
	printStatus( status_ );

	serial_.write( "\r\n" );
}

void ServoControlApp::printBootBanner() {
	serial_.write( "\r\n" );
	serial_.write( "========================================\r\n" );
	serial_.write( "STM32 Servo Control App\r\n" );
	serial_.write( "Driver: PCA9685 / WVS-17035\r\n" );
	serial_.write( "I2C1: PB9 SDA / PB8 SCL\r\n" );
	serial_.write( "USART2: PA2 TX / PA3 RX\r\n" );
	serial_.write( "========================================\r\n" );

	serial_.write( "[SERVO] address=0x40\r\n" );
	serial_.write( "[SERVO] frequency_hz=" );
	printUnsigned( kServoFrequencyHz );
	serial_.write( "\r\n" );
}

void ServoControlApp::printStatus(
	ServoStatus status ) {

	switch( status ) {
		case ServoStatus::Ok:
			serial_.write( "OK" );
			break;

		case ServoStatus::I2cError:
			serial_.write( "I2C_ERROR" );
			break;

		case ServoStatus::InvalidChannel:
			serial_.write( "INVALID_CHANNEL" );
			break;

		case ServoStatus::InvalidArgument:
			serial_.write( "INVALID_ARGUMENT" );
			break;

		case ServoStatus::NotDetected:
			serial_.write( "NOT_DETECTED" );
			break;

		case ServoStatus::NotConfigured:
			serial_.write( "NOT_CONFIGURED" );
			break;

		default:
			serial_.write( "UNKNOWN" );
			break;
	}
}

void ServoControlApp::printUnsigned(
	std::uint32_t value ) {

	char buffer[10];
	std::uint8_t index = 0U;

	if( value == 0U ) {
		serial_.write( "0" );
		return;
	}

	while( value > 0U && index < sizeof( buffer ) ) {
		buffer[index] = static_cast<char>( '0' + ( value % 10U ) );
		value /= 10U;
		++index;
	}

	while( index > 0U ) {
		--index;
		serial_.write( static_cast<std::uint8_t>( buffer[index] ) );
	}
}

} // namespace first_project