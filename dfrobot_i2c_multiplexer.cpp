#include "dfrobot_i2c_multiplexer.h"

namespace first_project {

DFRobotI2CMultiplexer::DFRobotI2CMultiplexer(
	I2cbus &bus,
	std::uint8_t address )
	: bus_( bus ),
	  address_( address ) {}

bool DFRobotI2CMultiplexer::begin() {
	return disableAll();
}

bool DFRobotI2CMultiplexer::selectPort( std::uint8_t port ) {
	if( !isValidPort( port ) ) {
		return false;
	}

	const std::uint8_t mask = portToMask( port );

	if( !writeControlByte( mask ) ) {
		return false;
	}

	selected_port_ = port;
	channel_mask_ = mask;

	return true;
}

bool DFRobotI2CMultiplexer::disableAll() {
	return selectPort( kDisableAllPorts );
}

bool DFRobotI2CMultiplexer::setChannelMask( std::uint8_t mask ) {
	if( !writeControlByte( mask ) ) {
		return false;
	}

	channel_mask_ = mask;
	selected_port_ = maskToSinglePort( mask );

	return true;
}

bool DFRobotI2CMultiplexer::devicePresentOnPort(
	std::uint8_t port,
	std::uint8_t device_address ) {

	if( !selectPort( port ) ) {
		return false;
	}

	return bus_.testid( device_address );
}

bool DFRobotI2CMultiplexer::writeToDeviceOnPort(
	std::uint8_t port,
	std::uint8_t device_address,
	const std::uint8_t *data,
	std::size_t length ) {

	if( data == nullptr || length == 0U ) {
		return false;
	}

	if( !selectPort( port ) ) {
		return false;
	}

	return bus_.write( device_address, data, length );
}

bool DFRobotI2CMultiplexer::readFromDeviceOnPort(
	std::uint8_t port,
	std::uint8_t device_address,
	std::uint8_t *data,
	std::size_t length ) {

	if( data == nullptr || length == 0U ) {
		return false;
	}

	if( !selectPort( port ) ) {
		return false;
	}

	return bus_.read( device_address, data, length );
}

bool DFRobotI2CMultiplexer::writeReadOnPort(
	std::uint8_t port,
	std::uint8_t device_address,
	const std::uint8_t *tx_data,
	std::size_t tx_length,
	std::uint8_t *rx_data,
	std::size_t rx_length ) {

	if( tx_length > 0U && tx_data == nullptr ) {
		return false;
	}

	if( rx_length > 0U && rx_data == nullptr ) {
		return false;
	}

	if( tx_length == 0U && rx_length == 0U ) {
		return false;
	}

	if( !selectPort( port ) ) {
		return false;
	}

	if( tx_length == 0U ) {
		return bus_.read( device_address, rx_data, rx_length );
	}

	if( rx_length == 0U ) {
		return bus_.write( device_address, tx_data, tx_length );
	}

	return bus_.writeRead(
		device_address,
		tx_data,
		tx_length,
		rx_data,
		rx_length );
}

bool DFRobotI2CMultiplexer::writeRegister8OnPort(
	std::uint8_t port,
	std::uint8_t device_address,
	std::uint8_t reg,
	std::uint8_t value ) {

	const std::uint8_t buffer[2] = { reg, value };

	return writeToDeviceOnPort(
		port,
		device_address,
		buffer,
		sizeof( buffer ) );
}

bool DFRobotI2CMultiplexer::readRegisterOnPort(
	std::uint8_t port,
	std::uint8_t device_address,
	std::uint8_t reg,
	std::uint8_t *data,
	std::size_t length ) {

	return writeReadOnPort(
		port,
		device_address,
		&reg,
		1U,
		data,
		length );
}

std::uint8_t DFRobotI2CMultiplexer::address() const {
	return address_;
}

std::uint8_t DFRobotI2CMultiplexer::selectedPort() const {
	return selected_port_;
}

std::uint8_t DFRobotI2CMultiplexer::channelMask() const {
	return channel_mask_;
}

bool DFRobotI2CMultiplexer::isValidPort( std::uint8_t port ) {
	return port <= kDisableAllPorts;
}

std::uint8_t DFRobotI2CMultiplexer::portToMask( std::uint8_t port ) {
	if( port >= kDisableAllPorts ) {
		return 0U;
	}

	return static_cast<std::uint8_t>( 1U << port );
}

std::uint8_t DFRobotI2CMultiplexer::maskToSinglePort( std::uint8_t mask ) {
	if( mask == 0U ) {
		return kDisableAllPorts;
	}

	for( std::uint8_t port = 0U; port < kPortCount; ++port ) {
		if( mask == static_cast<std::uint8_t>( 1U << port ) ) {
			return port;
		}
	}

	return kMultiplePortsSelected;
}

bool DFRobotI2CMultiplexer::writeControlByte( std::uint8_t value ) {
	return bus_.write( address_, &value, 1U );
}

} // namespace first_project