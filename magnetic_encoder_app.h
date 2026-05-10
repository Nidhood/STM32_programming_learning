#pragma once

#include <cstdint>

#include "as5600_encoder.h"
#include "dfrobot_i2c_multiplexer.h"
#include "flash.h"
#include "i2c.h"
#include "magnetic_encoder_manager.h"
#include "mt6701_encoder.h"
#include "usart.h"
#include "app_config.h"

namespace first_project {

class MagneticEncoderApp {
	public:
		MagneticEncoderApp() = default;

		void setup();
		void loop();

	private:
		USART serial_{};
		I2C i2c_{};
		DFRobotI2CMultiplexer mux_{ i2c_, first_project::kMuxAddress };
		Flash flash_{};

		AS5600Encoder as5600_direct_{ i2c_ };
		MT6701Encoder mt6701_direct_{ i2c_ };

		AS5600Encoder as5600_mux_{ mux_, first_project::kMuxPort };
		MT6701Encoder mt6701_mux_{ mux_, first_project::kMuxPort };

		MagneticEncoderManager as5600_direct_manager_{
			as5600_direct_,
			flash_,
			MagneticSensorType::AS5600 };

		MagneticEncoderManager mt6701_direct_manager_{
			mt6701_direct_,
			flash_,
			MagneticSensorType::MT6701 };

		MagneticEncoderManager as5600_mux_manager_{
			as5600_mux_,
			flash_,
			MagneticSensorType::AS5600 };

		MagneticEncoderManager mt6701_mux_manager_{
			mt6701_mux_,
			flash_,
			MagneticSensorType::MT6701 };

		MagneticEncoderManager *encoder_ = nullptr;

		MagneticEncoderManager::Status status_ =
			MagneticEncoderManager::Status::SensorError;

		std::uint32_t previous_sample_ms_ = 0U;
		std::uint32_t previous_print_ms_ = 0U;
		std::uint32_t previous_recovery_ms_ = 0U;

		void selectActiveEncoder();

		void initializePeripherals();
		void initializeMuxIfNeeded();
		void initializeEncoder();
		void configureEncoderIfNeeded();

		void updateEncoderTask();
		void printTask();
		void recoveryTask();

		void printBootBanner();
		void printEncoderData();

		void printUnsigned( std::uint32_t value );
		void printSigned( std::int32_t value );
		void printHex8( std::uint8_t value );
		void printYesNo( bool value );
		void printMilliDegrees( float angle_rad );
		void printMilliRadPerSecond( float velocity_rad_s );

		void printSelectedSensor();
		void printManagerStatus( MagneticEncoderManager::Status status );
		void printSensorStatus( SensorStatus status );
		void printDiagnostic( const MagneticAngleDiagnostic &diagnostic );

		static std::int32_t radToMilliDegrees( float angle_rad );
		static std::int32_t radPerSecondToMilliRadPerSecond( float velocity_rad_s );
};

} // namespace first_project