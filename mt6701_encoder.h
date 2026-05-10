#pragma once

#include <cstdint>

#include "dfrobot_i2c_multiplexer.h"
#include "i2c.h"
#include "magnetic_angle_sensor.h"

namespace first_project {

class MT6701Encoder : public MagneticAngleSensor {
	public:
		static constexpr std::uint8_t kDefaultAddress = 0x06U;
		static constexpr std::uint8_t kAlternativeAddress = 0x46U;
		static constexpr std::uint16_t kCountsPerRevolution = 16384U;

		explicit MT6701Encoder(
			I2C &i2c,
			std::uint8_t address = kDefaultAddress );

		MT6701Encoder(
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

		std::uint8_t address() const;

	private:
		static constexpr float kTwoPi = 6.28318530717958647692F;
		static constexpr std::uint8_t kAngleMsbRegister = 0x03U;
		static constexpr std::uint16_t kRawMask = 0x3FFFU;

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
};

} // namespace first_project