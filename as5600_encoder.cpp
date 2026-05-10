#include "as5600_encoder.h"

namespace first_project {

namespace {

constexpr std::uint8_t AS5600_REG_STATUS = 0x0BU;
constexpr std::uint8_t AS5600_REG_RAW_ANGLE_HIGH = 0x0CU;
constexpr std::uint8_t AS5600_REG_AGC = 0x1AU;
constexpr std::uint8_t AS5600_REG_MAGNITUDE_HIGH = 0x1BU;

constexpr std::uint8_t AS5600_STATUS_MAGNET_HIGH = 1U << 3U;
constexpr std::uint8_t AS5600_STATUS_MAGNET_LOW = 1U << 4U;
constexpr std::uint8_t AS5600_STATUS_MAGNET_DETECTED = 1U << 5U;

} // namespace

AS5600Encoder::AS5600Encoder(
	I2C &i2c,
	std::uint8_t address )
	: i2c_( &i2c ),
	  mux_( nullptr ),
	  mux_port_( 0U ),
	  address_( address ),
	  use_mux_( false ) {}

AS5600Encoder::AS5600Encoder(
	DFRobotI2CMultiplexer &mux,
	std::uint8_t mux_port,
	std::uint8_t address )
	: i2c_( nullptr ),
	  mux_( &mux ),
	  mux_port_( mux_port ),
	  address_( address ),
	  use_mux_( true ) {}

SensorStatus AS5600Encoder::begin() {
	MagneticAngleDiagnostic diagnostic{};

	const SensorStatus status = readDiagnostic( diagnostic );

	if( status != SensorStatus::Ok ) {
		return SensorStatus::NotDetected;
	}

	return SensorStatus::Ok;
}

SensorStatus AS5600Encoder::readRawAngle( std::uint16_t &raw_angle ) {
	return readRegister12( AS5600_REG_RAW_ANGLE_HIGH, raw_angle );
}

SensorStatus AS5600Encoder::readAngleRad( float &angle_rad ) {
	std::uint16_t raw_angle = 0U;

	const SensorStatus status = readRawAngle( raw_angle );

	if( status != SensorStatus::Ok ) {
		return status;
	}

	angle_rad =
		static_cast<float>( raw_angle & kRawMask ) *
		( kTwoPi / static_cast<float>( kCountsPerRevolution ) );

	return SensorStatus::Ok;
}

SensorStatus AS5600Encoder::readDiagnostic(
	MagneticAngleDiagnostic &diagnostic ) {

	std::uint8_t status_register = 0U;
	std::uint8_t agc = 0U;
	std::uint16_t magnitude = 0U;

	SensorStatus status = readRegister8( AS5600_REG_STATUS, status_register );

	if( status != SensorStatus::Ok ) {
		diagnostic.communication_ok = false;
		return status;
	}

	status = readRegister8( AS5600_REG_AGC, agc );

	if( status != SensorStatus::Ok ) {
		diagnostic.communication_ok = false;
		return status;
	}

	status = readRegister12( AS5600_REG_MAGNITUDE_HIGH, magnitude );

	if( status != SensorStatus::Ok ) {
		diagnostic.communication_ok = false;
		return status;
	}

	diagnostic.communication_ok = true;
	diagnostic.diagnostic_supported = true;

	diagnostic.status_register = status_register;
	diagnostic.agc = agc;
	diagnostic.magnitude = magnitude;

	diagnostic.magnet_detected =
		( status_register & AS5600_STATUS_MAGNET_DETECTED ) != 0U;

	diagnostic.magnet_too_weak =
		( status_register & AS5600_STATUS_MAGNET_LOW ) != 0U;

	diagnostic.magnet_too_strong =
		( status_register & AS5600_STATUS_MAGNET_HIGH ) != 0U;

	return SensorStatus::Ok;
}

std::uint16_t AS5600Encoder::countsPerRevolution() const {
	return kCountsPerRevolution;
}

const char *AS5600Encoder::name() const {
	return "AS5600";
}

bool AS5600Encoder::writeRead(
	const std::uint8_t *tx_data,
	std::size_t tx_length,
	std::uint8_t *rx_data,
	std::size_t rx_length ) {

	if( tx_data == nullptr || rx_data == nullptr ) {
		return false;
	}

	if( use_mux_ ) {
		if( mux_ == nullptr ) {
			return false;
		}

		return mux_->writeReadOnPort(
			mux_port_,
			address_,
			tx_data,
			tx_length,
			rx_data,
			rx_length );
	}

	if( i2c_ == nullptr ) {
		return false;
	}

	return i2c_->writeRead(
		address_,
		tx_data,
		tx_length,
		rx_data,
		rx_length );
}

SensorStatus AS5600Encoder::readRegister8(
	std::uint8_t reg,
	std::uint8_t &value ) {

	if( !writeRead( &reg, 1U, &value, 1U ) ) {
		return SensorStatus::I2cError;
	}

	return SensorStatus::Ok;
}

SensorStatus AS5600Encoder::readRegister12(
	std::uint8_t high_reg,
	std::uint16_t &value ) {

	std::uint8_t data[2] = { 0U, 0U };

	if( !writeRead( &high_reg, 1U, data, sizeof( data ) ) ) {
		return SensorStatus::I2cError;
	}

	value =
		static_cast<std::uint16_t>(
			( static_cast<std::uint16_t>( data[0] & 0x0FU ) << 8U ) |
			static_cast<std::uint16_t>( data[1] ) );

	value &= kRawMask;

	return SensorStatus::Ok;
}

} // namespace first_project