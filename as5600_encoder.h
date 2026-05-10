#pragma once

#include <cstdint>

#include "dfrobot_i2c_multiplexer.h"
#include "i2c.h"
#include "magnetic_angle_sensor.h"

namespace first_project {

class AS5600Encoder : public MagneticAngleSensor {
	public:
		static constexpr std::uint8_t kDefaultAddress = 0x36U;
		static constexpr std::uint16_t kCountsPerRevolution = 4096U;

		explicit AS5600Encoder(
			I2C &i2c,
			std::uint8_t address = kDefaultAddress );

		AS5600Encoder(
			DFRobotI2CMultiplexer &mux,
			std::uint8_t mux_port,
			std::uint8_t address = kDefaultAddress );

		SensorStatus begin() override;

		SensorStatus readRawAngle( std::uint16_t &raw_angle ) override;

		SensorStatus readAngleRad( float &angle_rad ) override;

		SensorStatus readDiagnostic(
			MagneticAngleDiagnostic &diagnostic ) override;

		std::uint16_t countsPerRevolution() const override;

		const char *name() const override;

	private:
		static constexpr float kTwoPi = 6.28318530717958647692F;
		static constexpr std::uint16_t kRawMask = 0x0FFFU;

		I2C *i2c_ = nullptr;
		DFRobotI2CMultiplexer *mux_ = nullptr;

		std::uint8_t mux_port_ = 0U;
		std::uint8_t address_ = kDefaultAddress;

		bool use_mux_ = false;

		bool writeRead(
			const std::uint8_t *tx_data,
			std::size_t tx_length,
			std::uint8_t *rx_data,
			std::size_t rx_length );

		SensorStatus readRegister8(
			std::uint8_t reg,
			std::uint8_t &value );

		SensorStatus readRegister12(
			std::uint8_t high_reg,
			std::uint16_t &value );
};

} // namespace first_project