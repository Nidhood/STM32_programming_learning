#pragma once

#include <cstdint>

namespace first_project {

enum class MagneticSensorType : std::uint32_t {
	Unknown = 0U,
	AS5600 = 1U,
	MT6701 = 2U
};

enum class SensorStatus : std::uint8_t {
	Ok,
	I2cError,
	InvalidArgument,
	NotDetected,
	InvalidData,
	NotSupported
};

struct MagneticAngleDiagnostic {
		bool communication_ok = false;
		bool diagnostic_supported = false;

		bool magnet_detected = false;
		bool magnet_too_weak = false;
		bool magnet_too_strong = false;

		std::uint8_t status_register = 0U;
		std::uint8_t agc = 0U;
		std::uint16_t magnitude = 0U;
};

class MagneticAngleSensor {
	public:
		virtual ~MagneticAngleSensor() = default;

		virtual SensorStatus begin() = 0;

		virtual SensorStatus readRawAngle( std::uint16_t &raw_angle ) = 0;

		virtual SensorStatus readAngleRad( float &angle_rad ) = 0;

		virtual SensorStatus readDiagnostic(
			MagneticAngleDiagnostic &diagnostic ) = 0;

		virtual std::uint16_t countsPerRevolution() const = 0;

		virtual const char *name() const = 0;
};

} // namespace first_project