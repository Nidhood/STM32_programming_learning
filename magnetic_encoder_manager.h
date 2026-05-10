#pragma once

#include <cstddef>
#include <cstdint>

#include "flash.h"
#include "magnetic_angle_sensor.h"

namespace first_project {

class MagneticEncoderManager {
	public:
		static constexpr std::uint8_t kMaxCorrectionPoints = 16U;

		static constexpr float kPi = 3.14159265358979323846F;
		static constexpr float kTwoPi = 2.0F * kPi;

		static constexpr std::uint32_t kStorageMagic = 0x4D454E43UL;
		static constexpr std::uint32_t kStorageVersion = 1UL;

		enum class Status : std::uint8_t {
			Ok,
			SensorError,
			SensorInvalid,
			FlashError,
			StorageInvalid,
			NotCalibrated,
			InvalidArgument,
			OutOfLimits
		};

		struct CorrectionPoint {
				float measured_angle_rad = 0.0F;
				float correction_rad = 0.0F;
		};

		struct Calibration {
				float zero_offset_rad = 0.0F;
				float direction = 1.0F;
				float min_angle_rad = -kPi;
				float max_angle_rad = kPi;

				std::uint8_t calibrated = 0U;
				std::uint8_t correction_count = 0U;
				std::uint8_t reserved0 = 0U;
				std::uint8_t reserved1 = 0U;

				CorrectionPoint correction_points[kMaxCorrectionPoints]{};
		};

		struct Sample {
				bool valid = false;
				bool calibrated = false;

				std::uint16_t raw_count = 0U;

				float raw_angle_rad = 0.0F;
				float calibrated_angle_rad = 0.0F;
				float corrected_angle_rad = 0.0F;
				float unwrapped_angle_rad = 0.0F;

				float raw_velocity_rad_s = 0.0F;
				float filtered_velocity_rad_s = 0.0F;

				std::uint32_t timestamp_us = 0U;

				SensorStatus sensor_status = SensorStatus::NotDetected;
				MagneticAngleDiagnostic diagnostic{};
		};

		struct StorageBlock {
				std::uint32_t magic = kStorageMagic;
				std::uint32_t version = kStorageVersion;
				std::uint32_t sensor_type = 0U;
				std::uint32_t counts_per_revolution = 0U;
				std::uint32_t sequence = 0U;

				Calibration calibration{};

				std::uint32_t crc32 = 0U;
		};

		MagneticEncoderManager(
			MagneticAngleSensor &sensor,
			Flash &flash,
			MagneticSensorType sensor_type,
			std::uint32_t flash_offset = 0U );

		Status begin( bool load_from_flash = true );

		Status update();
		Status update( std::uint32_t timestamp_us );

		Status calibrateZero( float known_joint_angle_rad = 0.0F );

		Status setDirection( float direction );

		Status setLimits(
			float min_angle_rad,
			float max_angle_rad );

		Status setVelocityFilterAlpha( float alpha );

		Status setLinearityCorrectionTable(
			const CorrectionPoint *points,
			std::uint8_t count );

		Status clearLinearityCorrectionTable();

		Status loadFromFlash();
		Status saveToFlash();

		void clearCalibration();
		void resetRuntimeState();

		bool isStorageValid() const;
		bool isDirty() const;
		bool isCalibrated() const;
		bool isValid() const;

		const Sample &sample() const;
		const Calibration &calibration() const;
		const MagneticAngleDiagnostic &diagnostic() const;

		float angleRad() const;
		float unwrappedAngleRad() const;
		float velocityRadS() const;
		float rawVelocityRadS() const;

		const char *sensorName() const;
		MagneticSensorType sensorType() const;

	private:
		MagneticAngleSensor &sensor_;
		Flash &flash_;

		MagneticSensorType sensor_type_ = MagneticSensorType::Unknown;
		std::uint32_t flash_offset_ = 0U;

		Calibration calibration_{};
		Sample sample_{};

		float velocity_alpha_ = 0.15F;
		float velocity_filter_state_ = 0.0F;
		float last_angle_rad_ = 0.0F;
		std::uint32_t last_timestamp_us_ = 0U;
		bool runtime_initialized_ = false;

		bool storage_valid_ = false;
		bool dirty_ = false;
		std::uint32_t storage_sequence_ = 0U;

		static float clamp(
			float value,
			float min_value,
			float max_value );

		static float wrapTwoPi( float angle_rad );
		static float wrapPi( float angle_rad );
		static float normalizeDirection( float direction );

		static std::uint32_t calculateCrc32(
			const std::uint8_t *data,
			std::size_t length );

		static std::uint32_t calculateStorageCrc(
			const StorageBlock &block );

		static bool diagnosticIsValid(
			const MagneticAngleDiagnostic &diagnostic );

		float rawCountToRad( std::uint16_t raw_count ) const;

		float applyCalibration( float raw_angle_rad ) const;

		float applyLinearityCorrection( float calibrated_angle_rad ) const;

		bool isInsideLimits( float angle_rad ) const;

		StorageBlock buildStorageBlock() const;

		Status applyStorageBlock( const StorageBlock &block );
};

} // namespace first_project