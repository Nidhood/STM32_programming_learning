#include "magnetic_encoder_app.h"

#include "delay.h"

namespace first_project {

void MagneticEncoderApp::setup() {
	initializePeripherals();
	selectActiveEncoder();

	printBootBanner();

	initializeMuxIfNeeded();
	initializeEncoder();
	configureEncoderIfNeeded();

	previous_sample_ms_ = millis();
	previous_print_ms_ = millis();
	previous_recovery_ms_ = millis();

	serial_.write( "\r\n[RUN] Magnetic encoder application started\r\n" );
}

void MagneticEncoderApp::loop() {
	updateEncoderTask();
	printTask();
	recoveryTask();
}

void MagneticEncoderApp::initializePeripherals() {
	initializeDelay();

	serial_.start(
		first_project::kUsart2TxPin,
		first_project::kUsart2RxPin,
		first_project::kSerialBaudrate );

	i2c_.start(
		first_project::kI2C1SdaPin,
		first_project::kI2C1SclPin,
		first_project::kI2CFrequencyHz );
}

void MagneticEncoderApp::selectActiveEncoder() {
	if constexpr( first_project::kSelectedSensor == MagneticSensorType::AS5600 ) {
		if constexpr( first_project::kUseI2CMux ) {
			encoder_ = &as5600_mux_manager_;
		} else {
			encoder_ = &as5600_direct_manager_;
		}
	} else if constexpr( first_project::kSelectedSensor == MagneticSensorType::MT6701 ) {
		if constexpr( first_project::kUseI2CMux ) {
			encoder_ = &mt6701_mux_manager_;
		} else {
			encoder_ = &mt6701_direct_manager_;
		}
	} else {
		encoder_ = nullptr;
	}
}

void MagneticEncoderApp::initializeMuxIfNeeded() {
	if constexpr( first_project::kUseI2CMux ) {
		serial_.write( "[MUX] Checking multiplexer at " );
		printHex8( first_project::kMuxAddress );
		serial_.write( "\r\n" );

		if( mux_.begin() ) {
			serial_.write( "[MUX] OK\r\n" );
		} else {
			serial_.write( "[MUX] ERROR\r\n" );
		}
	}
}

void MagneticEncoderApp::initializeEncoder() {
	if( encoder_ == nullptr ) {
		serial_.write( "[ENC] ERROR: no active encoder selected\r\n" );
		return;
	}

	status_ = encoder_->begin( true );

	serial_.write( "[ENC] begin status: " );
	printManagerStatus( status_ );
	serial_.write( "\r\n" );

	serial_.write( "[ENC] active sensor: " );
	serial_.write( encoder_->sensorName() );
	serial_.write( "\r\n" );

	serial_.write( "[FLASH] storage valid: " );
	printYesNo( encoder_->isStorageValid() );
	serial_.write( "\r\n" );
}

void MagneticEncoderApp::configureEncoderIfNeeded() {
	if( encoder_ == nullptr ) {
		return;
	}

	if( encoder_->isStorageValid() ) {
		serial_.write( "[FLASH] Calibration loaded from Flash\r\n" );
		return;
	}

	serial_.write( "[FLASH] No valid calibration found\r\n" );
	serial_.write( "[CAL] Keep the joint in mechanical zero position\r\n" );

	status_ = encoder_->setDirection( 1.0F );

	serial_.write( "[CAL] direction status: " );
	printManagerStatus( status_ );
	serial_.write( "\r\n" );

	status_ = encoder_->setLimits(
		-MagneticEncoderManager::kPi,
		MagneticEncoderManager::kPi );

	serial_.write( "[CAL] limits status: " );
	printManagerStatus( status_ );
	serial_.write( "\r\n" );

	status_ = encoder_->setVelocityFilterAlpha(
		first_project::kVelocityFilterAlpha );

	serial_.write( "[FILTER] velocity alpha status: " );
	printManagerStatus( status_ );
	serial_.write( "\r\n" );

	status_ = encoder_->calibrateZero( 0.0F );

	serial_.write( "[CAL] zero calibration status: " );
	printManagerStatus( status_ );
	serial_.write( "\r\n" );

	if( status_ != MagneticEncoderManager::Status::Ok ) {
		serial_.write( "[FLASH] Calibration not saved because calibration failed\r\n" );
		return;
	}

	status_ = encoder_->saveToFlash();

	serial_.write( "[FLASH] save status: " );
	printManagerStatus( status_ );
	serial_.write( "\r\n" );
}

void MagneticEncoderApp::updateEncoderTask() {
	if( encoder_ == nullptr ) {
		return;
	}

	if( every_ms( previous_sample_ms_, first_project::kSensorUpdatePeriodMs ) ) {
		status_ = encoder_->update();
	}
}

void MagneticEncoderApp::printTask() {
	if( encoder_ == nullptr ) {
		return;
	}

	if( every_ms( previous_print_ms_, first_project::kSerialPrintPeriodMs ) ) {
		serial_.write( "\r\n[UPDATE] status: " );
		printManagerStatus( status_ );

		serial_.write( " time_ms=" );
		printUnsigned( millis() );
		serial_.write( "\r\n" );

		printEncoderData();
	}
}

void MagneticEncoderApp::recoveryTask() {
	if( encoder_ == nullptr ) {
		return;
	}

	const bool needs_recovery =
		status_ == MagneticEncoderManager::Status::SensorError ||
		status_ == MagneticEncoderManager::Status::SensorInvalid;

	if( !needs_recovery ) {
		return;
	}

	if( every_ms( previous_recovery_ms_, first_project::kRecoveryPeriodMs ) ) {
		serial_.write( "\r\n[RECOVERY] Trying encoder begin again\r\n" );

		status_ = encoder_->begin( false );

		serial_.write( "[RECOVERY] begin status: " );
		printManagerStatus( status_ );
		serial_.write( "\r\n" );
	}
}

void MagneticEncoderApp::printBootBanner() {
	serial_.write( "\r\n" );
	serial_.write( "========================================\r\n" );
	serial_.write( "Generic Magnetic Encoder Application\r\n" );
	serial_.write( "NUCLEO-F401RE ST-LINK Virtual COM\r\n" );
	serial_.write( "USART2: PA2 TX / PA3 RX\r\n" );
	serial_.write( "I2C1: PB9 SDA / PB8 SCL\r\n" );
	serial_.write( "Baudrate: 115200\r\n" );
	serial_.write( "========================================\r\n" );

	serial_.write( "Selected sensor: " );
	printSelectedSensor();
	serial_.write( "\r\n" );

	serial_.write( "Use I2C mux: " );
	printYesNo( first_project::kUseI2CMux );
	serial_.write( "\r\n" );
}

void MagneticEncoderApp::printEncoderData() {
	const MagneticEncoderManager::Sample &sample = encoder_->sample();
	const MagneticEncoderManager::Calibration &cal = encoder_->calibration();

	serial_.write( "\r\n[ENCODER DATA]\r\n" );

	serial_.write( "sensor=" );
	serial_.write( encoder_->sensorName() );
	serial_.write( "\r\n" );

	serial_.write( "valid=" );
	printYesNo( sample.valid );

	serial_.write( " calibrated=" );
	printYesNo( sample.calibrated );

	serial_.write( " timestamp_us=" );
	printUnsigned( sample.timestamp_us );
	serial_.write( "\r\n" );

	serial_.write( "sensor_status=" );
	printSensorStatus( sample.sensor_status );
	serial_.write( "\r\n" );

	serial_.write( "raw_count=" );
	printUnsigned( sample.raw_count );
	serial_.write( "\r\n" );

	serial_.write( "raw_angle=" );
	printMilliDegrees( sample.raw_angle_rad );
	serial_.write( " deg\r\n" );

	serial_.write( "calibrated_angle=" );
	printMilliDegrees( sample.calibrated_angle_rad );
	serial_.write( " deg\r\n" );

	serial_.write( "corrected_angle=" );
	printMilliDegrees( sample.corrected_angle_rad );
	serial_.write( " deg\r\n" );

	serial_.write( "unwrapped_angle=" );
	printMilliDegrees( sample.unwrapped_angle_rad );
	serial_.write( " deg\r\n" );

	serial_.write( "raw_velocity=" );
	printMilliRadPerSecond( sample.raw_velocity_rad_s );
	serial_.write( " mrad/s\r\n" );

	serial_.write( "filtered_velocity=" );
	printMilliRadPerSecond( sample.filtered_velocity_rad_s );
	serial_.write( " mrad/s\r\n" );

	serial_.write( "zero_offset=" );
	printMilliDegrees( cal.zero_offset_rad );
	serial_.write( " deg\r\n" );

	serial_.write( "direction=" );
	serial_.write( cal.direction < 0.0F ? "-1" : "+1" );
	serial_.write( "\r\n" );

	serial_.write( "limit_min=" );
	printMilliDegrees( cal.min_angle_rad );

	serial_.write( " deg limit_max=" );
	printMilliDegrees( cal.max_angle_rad );
	serial_.write( " deg\r\n" );

	printDiagnostic( sample.diagnostic );
}

void MagneticEncoderApp::printUnsigned( std::uint32_t value ) {
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

void MagneticEncoderApp::printSigned( std::int32_t value ) {
	if( value < 0 ) {
		serial_.write( "-" );
		value = -value;
	}

	printUnsigned( static_cast<std::uint32_t>( value ) );
}

void MagneticEncoderApp::printHex8( std::uint8_t value ) {
	constexpr char hex[] = "0123456789ABCDEF";

	serial_.write( "0x" );
	serial_.write( static_cast<std::uint8_t>( hex[( value >> 4U ) & 0x0FU] ) );
	serial_.write( static_cast<std::uint8_t>( hex[value & 0x0FU] ) );
}

void MagneticEncoderApp::printYesNo( bool value ) {
	serial_.write( value ? "YES" : "NO" );
}

void MagneticEncoderApp::printMilliDegrees( float angle_rad ) {
	const std::int32_t value_mdeg = radToMilliDegrees( angle_rad );

	std::int32_t abs_value = value_mdeg;

	if( abs_value < 0 ) {
		serial_.write( "-" );
		abs_value = -abs_value;
	}

	const std::uint32_t integer_part =
		static_cast<std::uint32_t>( abs_value / 1000 );

	const std::uint32_t decimal_part =
		static_cast<std::uint32_t>( abs_value % 1000 );

	printUnsigned( integer_part );
	serial_.write( "." );

	if( decimal_part < 100U ) {
		serial_.write( "0" );
	}

	if( decimal_part < 10U ) {
		serial_.write( "0" );
	}

	printUnsigned( decimal_part );
}

void MagneticEncoderApp::printMilliRadPerSecond( float velocity_rad_s ) {
	printSigned( radPerSecondToMilliRadPerSecond( velocity_rad_s ) );
}

void MagneticEncoderApp::printSelectedSensor() {
	switch( first_project::kSelectedSensor ) {
		case MagneticSensorType::AS5600:
			serial_.write( "AS5600" );
			break;

		case MagneticSensorType::MT6701:
			serial_.write( "MT6701" );
			break;

		default:
			serial_.write( "UNKNOWN" );
			break;
	}
}

void MagneticEncoderApp::printManagerStatus(
	MagneticEncoderManager::Status status ) {

	using Status = MagneticEncoderManager::Status;

	switch( status ) {
		case Status::Ok:
			serial_.write( "OK" );
			break;

		case Status::SensorError:
			serial_.write( "SENSOR_ERROR" );
			break;

		case Status::SensorInvalid:
			serial_.write( "SENSOR_INVALID" );
			break;

		case Status::FlashError:
			serial_.write( "FLASH_ERROR" );
			break;

		case Status::StorageInvalid:
			serial_.write( "STORAGE_INVALID" );
			break;

		case Status::NotCalibrated:
			serial_.write( "NOT_CALIBRATED" );
			break;

		case Status::InvalidArgument:
			serial_.write( "INVALID_ARGUMENT" );
			break;

		case Status::OutOfLimits:
			serial_.write( "OUT_OF_LIMITS" );
			break;

		default:
			serial_.write( "UNKNOWN" );
			break;
	}
}

void MagneticEncoderApp::printSensorStatus( SensorStatus status ) {
	using Status = SensorStatus;

	switch( status ) {
		case Status::Ok:
			serial_.write( "OK" );
			break;

		case Status::I2cError:
			serial_.write( "I2C_ERROR" );
			break;

		case Status::InvalidArgument:
			serial_.write( "INVALID_ARGUMENT" );
			break;

		case Status::NotDetected:
			serial_.write( "NOT_DETECTED" );
			break;

		case Status::InvalidData:
			serial_.write( "INVALID_DATA" );
			break;

		case Status::NotSupported:
			serial_.write( "NOT_SUPPORTED" );
			break;

		default:
			serial_.write( "UNKNOWN" );
			break;
	}
}

void MagneticEncoderApp::printDiagnostic(
	const MagneticAngleDiagnostic &diagnostic ) {

	serial_.write( "diag: comm=" );
	printYesNo( diagnostic.communication_ok );

	serial_.write( " supported=" );
	printYesNo( diagnostic.diagnostic_supported );

	serial_.write( " magnet=" );
	printYesNo( diagnostic.magnet_detected );

	serial_.write( " weak=" );
	printYesNo( diagnostic.magnet_too_weak );

	serial_.write( " strong=" );
	printYesNo( diagnostic.magnet_too_strong );

	serial_.write( " AGC=" );
	printUnsigned( diagnostic.agc );

	serial_.write( " MAG=" );
	printUnsigned( diagnostic.magnitude );

	serial_.write( " STATUS=" );
	printHex8( diagnostic.status_register );

	serial_.write( "\r\n" );
}

std::int32_t MagneticEncoderApp::radToMilliDegrees( float angle_rad ) {
	constexpr float kRadToMilliDeg = 57295.7795131F;

	if( angle_rad >= 0.0F ) {
		return static_cast<std::int32_t>( angle_rad * kRadToMilliDeg + 0.5F );
	}

	return static_cast<std::int32_t>( angle_rad * kRadToMilliDeg - 0.5F );
}

std::int32_t MagneticEncoderApp::radPerSecondToMilliRadPerSecond(
	float velocity_rad_s ) {

	if( velocity_rad_s >= 0.0F ) {
		return static_cast<std::int32_t>( velocity_rad_s * 1000.0F + 0.5F );
	}

	return static_cast<std::int32_t>( velocity_rad_s * 1000.0F - 0.5F );
}

} // namespace first_project