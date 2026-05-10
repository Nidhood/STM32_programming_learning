#include "pca9685_servo_controller.h"

#include "delay.h"

namespace first_project {

PCA9685ServoController::PCA9685ServoController(
	I2C &i2c,
	std::uint8_t address )
	: i2c_( i2c ),
	  address_( address ) {}

ServoStatus PCA9685ServoController::begin(
	std::uint16_t frequency_hz ) {

	if( !isDetected() ) {
		configured_ = false;
		return ServoStatus::NotDetected;
	}

	if( !writeRegister( kRegMode1, kMode1AutoIncrement | kMode1AllCall ) ) {
		configured_ = false;
		return ServoStatus::I2cError;
	}

	if( !writeRegister( kRegMode2, kMode2OutDrv ) ) {
		configured_ = false;
		return ServoStatus::I2cError;
	}

	waitUs( 500U );

	const ServoStatus freq_status = setPWMFrequency( frequency_hz );

	if( freq_status != ServoStatus::Ok ) {
		configured_ = false;
		return freq_status;
	}

	configured_ = true;

	return disableAll();
}

ServoStatus PCA9685ServoController::setAngleDeg(
	std::uint8_t channel,
	float angle_deg ) {

	ServoRange range{};
	return setAngleDeg( channel, angle_deg, range );
}

ServoStatus PCA9685ServoController::setAngleDeg(
	std::uint8_t channel,
	float angle_deg,
	const ServoRange &range ) {

	if( !configured_ ) {
		return ServoStatus::NotConfigured;
	}

	if( !isValidChannel( channel ) ) {
		return ServoStatus::InvalidChannel;
	}

	if( range.max_angle_deg <= range.min_angle_deg ||
		range.max_pulse_us <= range.min_pulse_us ) {
		return ServoStatus::InvalidArgument;
	}

	const float safe_angle =
		clampFloat(
			angle_deg,
			range.min_angle_deg,
			range.max_angle_deg );

	const float angle_span =
		range.max_angle_deg - range.min_angle_deg;

	const float pulse_span =
		static_cast<float>( range.max_pulse_us - range.min_pulse_us );

	const float normalized =
		( safe_angle - range.min_angle_deg ) / angle_span;

	const float pulse_us_f =
		static_cast<float>( range.min_pulse_us ) +
		normalized * pulse_span;

	const std::uint16_t pulse_us =
		static_cast<std::uint16_t>( pulse_us_f + 0.5F );

	return setPulseUs( channel, pulse_us );
}

ServoStatus PCA9685ServoController::setPulseUs(
	std::uint8_t channel,
	std::uint16_t pulse_us ) {

	if( !configured_ ) {
		return ServoStatus::NotConfigured;
	}

	if( !isValidChannel( channel ) ) {
		return ServoStatus::InvalidChannel;
	}

	const std::uint16_t safe_pulse_us =
		clampU16( pulse_us, 500U, 2500U );

	const std::uint16_t off_tick =
		pulseUsToTicks( safe_pulse_us, frequency_hz_ );

	return setRawPWM( channel, 0U, off_tick );
}

ServoStatus PCA9685ServoController::setNeutral(
	std::uint8_t channel ) {

	ServoRange range{};
	return setPulseUs( channel, range.neutral_pulse_us );
}

ServoStatus PCA9685ServoController::disableChannel(
	std::uint8_t channel ) {

	if( !configured_ ) {
		return ServoStatus::NotConfigured;
	}

	if( !isValidChannel( channel ) ) {
		return ServoStatus::InvalidChannel;
	}

	const std::uint8_t reg =
		static_cast<std::uint8_t>(
			kRegLed0OnLow + static_cast<std::uint8_t>( channel * 4U ) );

	std::uint8_t data[5] = {
		reg,
		0x00U,
		0x00U,
		0x00U,
		0x10U };

	if( !writeBytes( data, sizeof( data ) ) ) {
		return ServoStatus::I2cError;
	}

	return ServoStatus::Ok;
}

ServoStatus PCA9685ServoController::disableAll() {
	if( !configured_ ) {
		return ServoStatus::NotConfigured;
	}

	std::uint8_t data[5] = {
		kRegAllLedOnLow,
		0x00U,
		0x00U,
		0x00U,
		0x10U };

	if( !writeBytes( data, sizeof( data ) ) ) {
		return ServoStatus::I2cError;
	}

	return ServoStatus::Ok;
}

std::uint8_t PCA9685ServoController::channelCount() const {
	return kChannelCount;
}

std::uint16_t PCA9685ServoController::frequencyHz() const {
	return frequency_hz_;
}

const char *PCA9685ServoController::name() const {
	return "PCA9685 Servo Controller";
}

ServoStatus PCA9685ServoController::setRawPWM(
	std::uint8_t channel,
	std::uint16_t on_tick,
	std::uint16_t off_tick ) {

	if( !configured_ ) {
		return ServoStatus::NotConfigured;
	}

	if( !isValidChannel( channel ) ) {
		return ServoStatus::InvalidChannel;
	}

	if( on_tick > 4095U || off_tick > 4095U ) {
		return ServoStatus::InvalidArgument;
	}

	const std::uint8_t reg =
		static_cast<std::uint8_t>(
			kRegLed0OnLow + static_cast<std::uint8_t>( channel * 4U ) );

	std::uint8_t data[5] = {
		reg,
		static_cast<std::uint8_t>( on_tick & 0xFFU ),
		static_cast<std::uint8_t>( ( on_tick >> 8U ) & 0x0FU ),
		static_cast<std::uint8_t>( off_tick & 0xFFU ),
		static_cast<std::uint8_t>( ( off_tick >> 8U ) & 0x0FU ) };

	if( !writeBytes( data, sizeof( data ) ) ) {
		return ServoStatus::I2cError;
	}

	return ServoStatus::Ok;
}

bool PCA9685ServoController::isDetected() {
	return i2c_.testid( address_ );
}

std::uint8_t PCA9685ServoController::address() const {
	return address_;
}

bool PCA9685ServoController::writeRegister(
	std::uint8_t reg,
	std::uint8_t value ) {

	std::uint8_t data[2] = { reg, value };
	return writeBytes( data, sizeof( data ) );
}

bool PCA9685ServoController::readRegister(
	std::uint8_t reg,
	std::uint8_t &value ) {

	return i2c_.writeRead( address_, &reg, 1U, &value, 1U );
}

bool PCA9685ServoController::writeBytes(
	const std::uint8_t *data,
	std::uint8_t length ) {

	if( data == nullptr || length == 0U ) {
		return false;
	}

	return i2c_.write( address_, data, length );
}

ServoStatus PCA9685ServoController::setPWMFrequency(
	std::uint16_t frequency_hz ) {

	if( frequency_hz < 24U || frequency_hz > 1526U ) {
		return ServoStatus::InvalidArgument;
	}

	std::uint8_t old_mode = 0U;

	if( !readRegister( kRegMode1, old_mode ) ) {
		return ServoStatus::I2cError;
	}

	const std::uint8_t sleep_mode =
		static_cast<std::uint8_t>(
			( old_mode & static_cast<std::uint8_t>( ~kMode1Restart ) ) |
			kMode1Sleep );

	if( !writeRegister( kRegMode1, sleep_mode ) ) {
		return ServoStatus::I2cError;
	}

	const std::uint8_t prescale =
		calculatePrescale( frequency_hz );

	if( !writeRegister( kRegPrescale, prescale ) ) {
		return ServoStatus::I2cError;
	}

	if( !writeRegister( kRegMode1, old_mode ) ) {
		return ServoStatus::I2cError;
	}

	waitUs( 500U );

	const std::uint8_t restart_mode =
		static_cast<std::uint8_t>(
			old_mode |
			kMode1Restart |
			kMode1AutoIncrement );

	if( !writeRegister( kRegMode1, restart_mode ) ) {
		return ServoStatus::I2cError;
	}

	frequency_hz_ = frequency_hz;

	return ServoStatus::Ok;
}

bool PCA9685ServoController::isValidChannel(
	std::uint8_t channel ) {

	return channel < kChannelCount;
}

std::uint8_t PCA9685ServoController::calculatePrescale(
	std::uint16_t frequency_hz ) {

	const std::uint32_t denominator =
		static_cast<std::uint32_t>( kResolutionTicks ) *
		static_cast<std::uint32_t>( frequency_hz );

	const std::uint32_t rounded =
		( kOscillatorHz + ( denominator / 2UL ) ) / denominator;

	if( rounded == 0UL ) {
		return 0U;
	}

	const std::uint32_t prescale = rounded - 1UL;

	if( prescale > 255UL ) {
		return 255U;
	}

	return static_cast<std::uint8_t>( prescale );
}

std::uint16_t PCA9685ServoController::pulseUsToTicks(
	std::uint16_t pulse_us,
	std::uint16_t frequency_hz ) {

	const std::uint64_t numerator =
		static_cast<std::uint64_t>( pulse_us ) *
		static_cast<std::uint64_t>( frequency_hz ) *
		static_cast<std::uint64_t>( kResolutionTicks );

	const std::uint64_t ticks =
		( numerator + 500000ULL ) / 1000000ULL;

	if( ticks > 4095ULL ) {
		return 4095U;
	}

	return static_cast<std::uint16_t>( ticks );
}

std::uint16_t PCA9685ServoController::clampU16(
	std::uint16_t value,
	std::uint16_t min_value,
	std::uint16_t max_value ) {

	if( value < min_value ) {
		return min_value;
	}

	if( value > max_value ) {
		return max_value;
	}

	return value;
}

float PCA9685ServoController::clampFloat(
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

void PCA9685ServoController::waitUs(
	std::uint32_t time_us ) {

	const std::uint32_t start_time = micros();

	while( !hasElapsed_us( start_time, time_us ) ) {
	}
}

} // namespace first_project