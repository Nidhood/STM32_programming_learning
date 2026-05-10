#pragma once

#include <cstdint>

namespace first_project {

enum class ServoStatus : std::uint8_t {
	Ok,
	I2cError,
	InvalidChannel,
	InvalidArgument,
	NotDetected,
	NotConfigured
};

struct ServoRange {
		float min_angle_deg = 0.0F;
		float max_angle_deg = 180.0F;

		std::uint16_t min_pulse_us = 600U;
		std::uint16_t neutral_pulse_us = 1500U;
		std::uint16_t max_pulse_us = 2400U;

		constexpr ServoRange() = default;

		constexpr ServoRange(
			float min_angle_deg_value,
			float max_angle_deg_value,
			std::uint16_t min_pulse_us_value,
			std::uint16_t neutral_pulse_us_value,
			std::uint16_t max_pulse_us_value )
			: min_angle_deg( min_angle_deg_value ),
			  max_angle_deg( max_angle_deg_value ),
			  min_pulse_us( min_pulse_us_value ),
			  neutral_pulse_us( neutral_pulse_us_value ),
			  max_pulse_us( max_pulse_us_value ) {}
};

class ServoController {
	public:
		virtual ~ServoController() = default;

		virtual ServoStatus begin( std::uint16_t frequency_hz ) = 0;

		virtual ServoStatus setAngleDeg(
			std::uint8_t channel,
			float angle_deg ) = 0;

		virtual ServoStatus setAngleDeg(
			std::uint8_t channel,
			float angle_deg,
			const ServoRange &range ) = 0;

		virtual ServoStatus setPulseUs(
			std::uint8_t channel,
			std::uint16_t pulse_us ) = 0;

		virtual ServoStatus setNeutral(
			std::uint8_t channel ) = 0;

		virtual ServoStatus disableChannel(
			std::uint8_t channel ) = 0;

		virtual ServoStatus disableAll() = 0;

		virtual std::uint8_t channelCount() const = 0;

		virtual std::uint16_t frequencyHz() const = 0;

		virtual const char *name() const = 0;
};

} // namespace first_project