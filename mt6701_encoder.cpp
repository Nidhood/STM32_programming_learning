#include "mt6701_encoder.h"

namespace first_project {

MT6701Encoder::MT6701Encoder(
	I2C &i2c,
	std::uint8_t address )
	: i2c_( &i2c ),
	  mux_( nullptr ),
	  mux_port_( 0U ),
	  address_( address ),
	  use_mux_( false ) {}

MT6701Encoder::MT6701Encoder(
	DFRobotI2CMultiplexer &mux,
	std::uint8_t mux_port,
	std::uint8_t address )
	: i2c_( nullptr ),
	  mux_( &mux ),
	  mux_port_( mux_port ),
	  address_( address ),
	  use_mux_( true ) {}

SensorStatus MT6701Encoder::begin() {
	std::uint16_t raw_angle = 0U;

	if( readRawAngle( raw_angle ) == SensorStatus::Ok ) {
		return SensorStatus::Ok;
	}

	const std::uint8_t original_address = address_;

	address_ = kAlternativeAddress;

	if( readRawAngle( raw_angle ) == SensorStatus::Ok ) {
		return SensorStatus::Ok;
	}

	address_ = original_address;

	return SensorStatus::NotDetected;
}

SensorStatus MT6701Encoder::readRawAngle( std::uint16_t &raw_angle ) {
	std::uint8_t reg = kAngleMsbRegister;
	std::uint8_t data[2] = { 0U, 0U };

	if( !writeRead( &reg, 1U, data, sizeof( data ) ) ) {
		return SensorStatus::I2cError;
	}

	raw_angle =
		static_cast<std::uint16_t>(
			( static_cast<std::uint16_t>( data[0] ) << 6U ) |
			( static_cast<std::uint16_t>( data[1] ) >> 2U ) );

	raw_angle &= kRawMask;

	return SensorStatus::Ok;
}

SensorStatus MT6701Encoder::readAngleRad( float &angle_rad ) {
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

SensorStatus MT6701Encoder::readDiagnostic(
	MagneticAngleDiagnostic &diagnostic ) {

	std::uint16_t raw_angle = 0U;

	const SensorStatus status = readRawAngle( raw_angle );

	if( status != SensorStatus::Ok ) {
		diagnostic.communication_ok = false;
		diagnostic.diagnostic_supported = false;
		return status;
	}

	diagnostic.communication_ok = true;
	diagnostic.diagnostic_supported = false;

	diagnostic.magnet_detected = true;
	diagnostic.magnet_too_weak = false;
	diagnostic.magnet_too_strong = false;

	diagnostic.status_register = 0U;
	diagnostic.agc = 0U;
	diagnostic.magnitude = 0U;

	return SensorStatus::Ok;
}

std::uint16_t MT6701Encoder::countsPerRevolution() const {
	return kCountsPerRevolution;
}

const char *MT6701Encoder::name() const {
	return "MT6701";
}

std::uint8_t MT6701Encoder::address() const {
	return address_;
}

bool MT6701Encoder::writeRead(
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

} // namespace first_project