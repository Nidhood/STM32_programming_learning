#pragma once

#include "i2cbus.h"
#include <cstdint>

namespace first_project {

class DFRobotI2CMultiplexer {
	private:
		I2cbus &bus_;
		std::uint8_t address_ = kDefaultAddress;
		std::uint8_t selected_port_ = kDisableAllPorts;
		std::uint8_t channel_mask_ = 0U;

		static bool isValidPort( std::uint8_t port );
		static std::uint8_t portToMask( std::uint8_t port );
		static std::uint8_t maskToSinglePort( std::uint8_t mask );

		bool writeControlByte( std::uint8_t value );

	public:
		static constexpr std::uint8_t kDefaultAddress = 0x70U;
		static constexpr std::uint8_t kPortCount = 8U;
		static constexpr std::uint8_t kDisableAllPorts = 8U;
		static constexpr std::uint8_t kMultiplePortsSelected = 0xFFU;

		explicit DFRobotI2CMultiplexer(
			I2cbus &bus,
			std::uint8_t address = kDefaultAddress );

		bool begin();

		bool selectPort( std::uint8_t port );
		bool disableAll();
		bool setChannelMask( std::uint8_t mask );

		bool devicePresentOnPort(
			std::uint8_t port,
			std::uint8_t device_address );

		bool writeToDeviceOnPort(
			std::uint8_t port,
			std::uint8_t device_address,
			const std::uint8_t *data,
			std::size_t length );

		bool readFromDeviceOnPort(
			std::uint8_t port,
			std::uint8_t device_address,
			std::uint8_t *data,
			std::size_t length );

		bool writeReadOnPort(
			std::uint8_t port,
			std::uint8_t device_address,
			const std::uint8_t *tx_data,
			std::size_t tx_length,
			std::uint8_t *rx_data,
			std::size_t rx_length );

		bool writeRegister8OnPort(
			std::uint8_t port,
			std::uint8_t device_address,
			std::uint8_t reg,
			std::uint8_t value );

		bool readRegisterOnPort(
			std::uint8_t port,
			std::uint8_t device_address,
			std::uint8_t reg,
			std::uint8_t *data,
			std::size_t length );

		std::uint8_t address() const;
		std::uint8_t selectedPort() const;
		std::uint8_t channelMask() const;
};

} // namespace first_project
