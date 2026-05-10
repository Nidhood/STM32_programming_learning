#include <cstdint>

#include "delay.h"
#include "dfrobot_i2c_multiplexer.h"
#include "flash.h"
#include "i2c.h"
#include "magnetic_encoder_manager.h"
#include "mt6701_encoder.h"
#include "pca9685_servo_controller.h"
#include "servo_controller.h"
#include "stm32f4xx.h"
#include "usart.h"

namespace {

constexpr STM32F_PinAF_t kI2C1SdaPin = { PB( 9 ), 4U };
constexpr STM32F_PinAF_t kI2C1SclPin = { PB( 8 ), 4U };

constexpr STM32F_PinAF_t kUsart2TxPin = { PA( 2 ), 7U };
constexpr STM32F_PinAF_t kUsart2RxPin = { PA( 3 ), 7U };

constexpr std::uint32_t kSerialBaudrate = 115200U;
constexpr std::uint32_t kI2CFrequencyHz = 100000U;

constexpr std::uint8_t kServoDriverAddress = 0x40U;
constexpr std::uint16_t kServoFrequencyHz = 50U;

constexpr std::uint8_t kEncoderMuxAddress = 0x71U;
constexpr std::uint8_t kEncoderMuxPort = 0U;

// Select here the servo channels you want to control with the encoder.
constexpr std::uint8_t kSelectedServoChannels[] = {
	0U,
	1U,
	2U,
	3U,
	4U,
	5U };

constexpr std::uint8_t kSelectedServoCount =
	static_cast<std::uint8_t>(
		sizeof( kSelectedServoChannels ) / sizeof( kSelectedServoChannels[0] ) );

// Safe servo range.
constexpr float kServoMinAngleDeg = 0.0F;
constexpr float kServoInitialAngleDeg = 80.0F;
constexpr float kServoMaxAngleDeg = 160.0F;

constexpr std::uint16_t kServoMinPulseUs = 700U;
constexpr std::uint16_t kServoNeutralPulseUs = 1500U;
constexpr std::uint16_t kServoMaxPulseUs = 2300U;

// Encoder to servo mapping.
constexpr float kEncoderToServoGain = 1.0F;

// Change this to -1.0F if the servos move in the opposite direction.
constexpr float kEncoderToServoDirection = 1.0F;

constexpr std::uint32_t kControlPeriodMs = 10U;
constexpr std::uint32_t kPrintPeriodMs = 200U;

constexpr float kRadToDeg = 57.2957795131F;

const first_project::ServoRange kServoRange{
	kServoMinAngleDeg,
	kServoMaxAngleDeg,
	kServoMinPulseUs,
	kServoNeutralPulseUs,
	kServoMaxPulseUs };

static_assert( kSelectedServoCount > 0U, "At least one servo channel must be selected" );

void printUnsigned(
	first_project::USART &serial,
	std::uint32_t value ) {

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

void printSigned(
	first_project::USART &serial,
	std::int32_t value ) {

	if( value < 0 ) {
		serial.write( "-" );
		value = -value;
	}

	printUnsigned( serial, static_cast<std::uint32_t>( value ) );
}

void printFloat3(
	first_project::USART &serial,
	float value ) {

	std::int32_t scaled = 0;

	if( value >= 0.0F ) {
		scaled = static_cast<std::int32_t>( value * 1000.0F + 0.5F );
	} else {
		scaled = static_cast<std::int32_t>( value * 1000.0F - 0.5F );
	}

	if( scaled < 0 ) {
		serial.write( "-" );
		scaled = -scaled;
	}

	const std::uint32_t integer_part =
		static_cast<std::uint32_t>( scaled / 1000 );

	const std::uint32_t decimal_part =
		static_cast<std::uint32_t>( scaled % 1000 );

	printUnsigned( serial, integer_part );
	serial.write( "." );

	if( decimal_part < 100U ) {
		serial.write( "0" );
	}

	if( decimal_part < 10U ) {
		serial.write( "0" );
	}

	printUnsigned( serial, decimal_part );
}

void printHex8(
	first_project::USART &serial,
	std::uint8_t value ) {

	constexpr char hex[] = "0123456789ABCDEF";

	serial.write( "0x" );
	serial.write( static_cast<std::uint8_t>( hex[( value >> 4U ) & 0x0FU] ) );
	serial.write( static_cast<std::uint8_t>( hex[value & 0x0FU] ) );
}

float clampFloat(
	float value,
	float min_value,
	float max_value ) {

	if( value < min_value ) {
		return min_value;
	}

	if( value > max_value ) {
		return max_value;
	}

	return value;
}

void printServoStatus(
	first_project::USART &serial,
	first_project::ServoStatus status ) {

	using Status = first_project::ServoStatus;

	switch( status ) {
		case Status::Ok:
			serial.write( "OK" );
			break;

		case Status::I2cError:
			serial.write( "I2C_ERROR" );
			break;

		case Status::InvalidChannel:
			serial.write( "INVALID_CHANNEL" );
			break;

		case Status::InvalidArgument:
			serial.write( "INVALID_ARGUMENT" );
			break;

		case Status::NotDetected:
			serial.write( "NOT_DETECTED" );
			break;

		case Status::NotConfigured:
			serial.write( "NOT_CONFIGURED" );
			break;

		default:
			serial.write( "UNKNOWN" );
			break;
	}
}

void printEncoderStatus(
	first_project::USART &serial,
	first_project::MagneticEncoderManager::Status status ) {

	using Status = first_project::MagneticEncoderManager::Status;

	switch( status ) {
		case Status::Ok:
			serial.write( "OK" );
			break;

		case Status::SensorError:
			serial.write( "SENSOR_ERROR" );
			break;

		case Status::SensorInvalid:
			serial.write( "SENSOR_INVALID" );
			break;

		case Status::FlashError:
			serial.write( "FLASH_ERROR" );
			break;

		case Status::StorageInvalid:
			serial.write( "STORAGE_INVALID" );
			break;

		case Status::NotCalibrated:
			serial.write( "NOT_CALIBRATED" );
			break;

		case Status::InvalidArgument:
			serial.write( "INVALID_ARGUMENT" );
			break;

		case Status::OutOfLimits:
			serial.write( "OUT_OF_LIMITS" );
			break;

		default:
			serial.write( "UNKNOWN" );
			break;
	}
}

void printBootBanner(
	first_project::USART &serial ) {

	serial.write( "\r\n" );
	serial.write( "========================================\r\n" );
	serial.write( "STM32 Encoder-Master Servo Control\r\n" );
	serial.write( "Servo driver: PCA9685 / WVS-17035 at " );
	printHex8( serial, kServoDriverAddress );
	serial.write( "\r\n" );

	serial.write( "Encoder mux: " );
	printHex8( serial, kEncoderMuxAddress );
	serial.write( " port " );
	printUnsigned( serial, kEncoderMuxPort );
	serial.write( "\r\n" );

	serial.write( "Selected servo channels: " );

	for( std::uint8_t i = 0U; i < kSelectedServoCount; ++i ) {
		printUnsigned( serial, kSelectedServoChannels[i] );

		if( i + 1U < kSelectedServoCount ) {
			serial.write( ", " );
		}
	}

	serial.write( "\r\n" );
	serial.write( "Initial servo angle: " );
	printFloat3( serial, kServoInitialAngleDeg );
	serial.write( " deg\r\n" );

	serial.write( "Servo limits: " );
	printFloat3( serial, kServoMinAngleDeg );
	serial.write( " to " );
	printFloat3( serial, kServoMaxAngleDeg );
	serial.write( " deg\r\n" );

	serial.write( "I2C1: PB9 SDA / PB8 SCL\r\n" );
	serial.write( "USART2: PA2 TX / PA3 RX\r\n" );
	serial.write( "========================================\r\n" );
}

bool setSelectedServosAngle(
	first_project::USART &serial,
	first_project::PCA9685ServoController &servo_driver,
	float angle_deg ) {

	bool all_ok = true;

	for( std::uint8_t i = 0U; i < kSelectedServoCount; ++i ) {
		const std::uint8_t channel = kSelectedServoChannels[i];

		const first_project::ServoStatus status =
			servo_driver.setAngleDeg(
				channel,
				angle_deg,
				kServoRange );

		if( status != first_project::ServoStatus::Ok ) {
			all_ok = false;

			serial.write( "[SERVO ERROR] channel=" );
			printUnsigned( serial, channel );
			serial.write( " status=" );
			printServoStatus( serial, status );
			serial.write( "\r\n" );
		}
	}

	return all_ok;
}

bool waitForValidEncoder(
	first_project::USART &serial,
	first_project::MagneticEncoderManager &encoder,
	float &initial_unwrapped_rad ) {

	for( std::uint8_t attempt = 0U; attempt < 50U; ++attempt ) {
		const first_project::MagneticEncoderManager::Status status =
			encoder.update();

		const first_project::MagneticEncoderManager::Sample &sample =
			encoder.sample();

		if( status == first_project::MagneticEncoderManager::Status::Ok &&
			sample.valid ) {

			initial_unwrapped_rad = sample.unwrapped_angle_rad;

			serial.write( "[ENCODER] Initial reference captured: " );
			printFloat3( serial, initial_unwrapped_rad * kRadToDeg );
			serial.write( " deg\r\n" );

			return true;
		}

		first_project::Delay wait = first_project::delay_ms( 20U );

		while( !wait.expired() ) {
		}
	}

	serial.write( "[ERROR] Encoder did not provide a valid sample\r\n" );
	return false;
}

} // namespace

int main( void ) {
	SystemCoreClockUpdate();

	first_project::initializeDelay();

	first_project::USART serial;
	serial.start(
		kUsart2TxPin,
		kUsart2RxPin,
		kSerialBaudrate );

	printBootBanner( serial );

	first_project::I2C i2c;
	i2c.start(
		kI2C1SdaPin,
		kI2C1SclPin,
		kI2CFrequencyHz );

	serial.write( "[I2C] Started\r\n" );

	first_project::PCA9685ServoController servo_driver(
		i2c,
		kServoDriverAddress );

	first_project::ServoStatus servo_status =
		servo_driver.begin( kServoFrequencyHz );

	serial.write( "[SERVO DRIVER] begin status: " );
	printServoStatus( serial, servo_status );
	serial.write( "\r\n" );

	if( servo_status != first_project::ServoStatus::Ok ) {
		serial.write( "[ERROR] PCA9685 not ready\r\n" );

		while( 1 ) {
		}
	}

	serial.write( "[SERVO] Moving selected servos to initial angle\r\n" );

	(void)setSelectedServosAngle(
		serial,
		servo_driver,
		kServoInitialAngleDeg );

	first_project::DFRobotI2CMultiplexer encoder_mux(
		i2c,
		kEncoderMuxAddress );

	if( !encoder_mux.begin() ) {
		serial.write( "[ERROR] Encoder mux not detected at " );
		printHex8( serial, kEncoderMuxAddress );
		serial.write( "\r\n" );

		while( 1 ) {
		}
	}

	serial.write( "[MUX] Encoder mux detected at " );
	printHex8( serial, kEncoderMuxAddress );
	serial.write( "\r\n" );

	first_project::MT6701Encoder encoder_sensor(
		encoder_mux,
		kEncoderMuxPort );

	first_project::Flash flash;

	first_project::MagneticEncoderManager encoder(
		encoder_sensor,
		flash,
		first_project::MagneticSensorType::MT6701 );

	first_project::MagneticEncoderManager::Status encoder_status =
		encoder.begin( true );

	serial.write( "[ENCODER] begin status: " );
	printEncoderStatus( serial, encoder_status );
	serial.write( "\r\n" );

	(void)encoder.setDirection( 1.0F );
	(void)encoder.setLimits(
		-first_project::MagneticEncoderManager::kPi,
		first_project::MagneticEncoderManager::kPi );
	(void)encoder.setVelocityFilterAlpha( 0.15F );

	if( !encoder.isStorageValid() ) {
		serial.write( "[CAL] No valid calibration. Current encoder position becomes 0 deg.\r\n" );

		(void)encoder.update();

		encoder_status = encoder.calibrateZero( 0.0F );

		serial.write( "[CAL] zero status: " );
		printEncoderStatus( serial, encoder_status );
		serial.write( "\r\n" );

		encoder_status = encoder.saveToFlash();

		serial.write( "[FLASH] save status: " );
		printEncoderStatus( serial, encoder_status );
		serial.write( "\r\n" );
	} else {
		serial.write( "[FLASH] Encoder calibration loaded\r\n" );
	}

	float initial_encoder_unwrapped_rad = 0.0F;

	if( !waitForValidEncoder(
			serial,
			encoder,
			initial_encoder_unwrapped_rad ) ) {

		while( 1 ) {
		}
	}

	std::uint32_t previous_control_ms = first_project::millis();
	std::uint32_t previous_print_ms = first_project::millis();

	float last_commanded_angle_deg = kServoInitialAngleDeg;

	serial.write( "\r\n[READY] Move the encoder. Selected servos will follow.\r\n" );

	while( 1 ) {
		if( first_project::every_ms( previous_control_ms, kControlPeriodMs ) ) {
			encoder_status = encoder.update();

			const first_project::MagneticEncoderManager::Sample &sample =
				encoder.sample();

			if( encoder_status != first_project::MagneticEncoderManager::Status::Ok ||
				!sample.valid ) {

				continue;
			}

			const float encoder_delta_deg =
				( sample.unwrapped_angle_rad - initial_encoder_unwrapped_rad ) *
				kRadToDeg;

			const float commanded_angle_deg =
				clampFloat(
					kServoInitialAngleDeg +
						( kEncoderToServoDirection *
						  kEncoderToServoGain *
						  encoder_delta_deg ),
					kServoMinAngleDeg,
					kServoMaxAngleDeg );

			last_commanded_angle_deg = commanded_angle_deg;

			(void)setSelectedServosAngle(
				serial,
				servo_driver,
				commanded_angle_deg );
		}

		if( first_project::every_ms( previous_print_ms, kPrintPeriodMs ) ) {
			const first_project::MagneticEncoderManager::Sample &sample =
				encoder.sample();

			const float encoder_delta_deg =
				( sample.unwrapped_angle_rad - initial_encoder_unwrapped_rad ) *
				kRadToDeg;

			serial.write( "[CTRL] encoder_delta_deg=" );
			printFloat3( serial, encoder_delta_deg );

			serial.write( " command_deg=" );
			printFloat3( serial, last_commanded_angle_deg );

			serial.write( " raw_deg=" );
			printFloat3( serial, sample.raw_angle_rad * kRadToDeg );

			serial.write( " corrected_deg=" );
			printFloat3( serial, sample.corrected_angle_rad * kRadToDeg );

			serial.write( " unwrapped_deg=" );
			printFloat3( serial, sample.unwrapped_angle_rad * kRadToDeg );

			serial.write( " encoder_status=" );
			printEncoderStatus( serial, encoder_status );

			serial.write( "\r\n" );
		}
	}
}