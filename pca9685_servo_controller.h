#pragma once

#include <cstdint>

#include "i2c.h"
#include "servo_controller.h"

namespace first_project {

class PCA9685ServoController : public ServoController {
	public:
		static constexpr std::uint8_t kDefaultAddress = 0x40U;
		static constexpr std::uint8_t kChannelCount = 16U;
		static constexpr std::uint16_t kDefaultFrequencyHz = 50U;

		explicit PCA9685ServoController(
			I2C &i2c,
			std::uint8_t address = kDefaultAddress );

		ServoStatus begin( std::uint16_t frequency_hz ) override;

		ServoStatus setAngleDeg(
			std::uint8_t channel,
			float angle_deg ) override;

		ServoStatus setAngleDeg(
			std::uint8_t channel,
			float angle_deg,
			const ServoRange &range ) override;

		ServoStatus setPulseUs(
			std::uint8_t channel,
			std::uint16_t pulse_us ) override;

		ServoStatus setNeutral(
			std::uint8_t channel ) override;

		ServoStatus disableChannel(
			std::uint8_t channel ) override;

		ServoStatus disableAll() override;

		std::uint8_t channelCount() const override;

		std::uint16_t frequencyHz() const override;

		const char *name() const override;

		ServoStatus setRawPWM(
			std::uint8_t channel,
			std::uint16_t on_tick,
			std::uint16_t off_tick );

		bool isDetected();

		std::uint8_t address() const;

	private:
		static constexpr std::uint32_t kOscillatorHz = 25000000UL;
		static constexpr std::uint16_t kResolutionTicks = 4096U;

		static constexpr std::uint8_t kRegMode1 = 0x00U;
		static constexpr std::uint8_t kRegMode2 = 0x01U;
		static constexpr std::uint8_t kRegLed0OnLow = 0x06U;
		static constexpr std::uint8_t kRegAllLedOnLow = 0xFAU;
		static constexpr std::uint8_t kRegPrescale = 0xFEU;

		static constexpr std::uint8_t kMode1Restart = 0x80U;
		static constexpr std::uint8_t kMode1AutoIncrement = 0x20U;
		static constexpr std::uint8_t kMode1Sleep = 0x10U;
		static constexpr std::uint8_t kMode1AllCall = 0x01U;

		static constexpr std::uint8_t kMode2OutDrv = 0x04U;

		I2C &i2c_;
		std::uint8_t address_ = kDefaultAddress;
		std::uint16_t frequency_hz_ = kDefaultFrequencyHz;
		bool configured_ = false;

		bool writeRegister(
			std::uint8_t reg,
			std::uint8_t value );

		bool readRegister(
			std::uint8_t reg,
			std::uint8_t &value );

		bool writeBytes(
			const std::uint8_t *data,
			std::uint8_t length );

		ServoStatus setPWMFrequency(
			std::uint16_t frequency_hz );

		static bool isValidChannel(
			std::uint8_t channel );

		static std::uint8_t calculatePrescale(
			std::uint16_t frequency_hz );

		static std::uint16_t pulseUsToTicks(
			std::uint16_t pulse_us,
			std::uint16_t frequency_hz );

		static std::uint16_t clampU16(
			std::uint16_t value,
			std::uint16_t min_value,
			std::uint16_t max_value );

		static float clampFloat(
			float value,
			float min_value,
			float max_value );

		static void waitUs(
			std::uint32_t time_us );
};

} // namespace first_project