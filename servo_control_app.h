#pragma once

#include <cstdint>

#include "app_config.h"
#include "i2c.h"
#include "pca9685_servo_controller.h"
#include "servo_controller.h"
#include "usart.h"

namespace first_project {

class ServoControlApp {
	public:
		ServoControlApp() = default;

		void setup();
		void loop();

	private:
		USART serial_{};
		I2C i2c_{};

		PCA9685ServoController servo_driver_{
			i2c_,
			kServoDriverAddress };

		ServoController *servo_ = &servo_driver_;

		ServoRange test_servo_range_{
			kServoMinAngleDeg,
			kServoMaxAngleDeg,
			kServoMinPulseUs,
			kServoNeutralPulseUs,
			kServoMaxPulseUs };

		ServoStatus status_ = ServoStatus::NotConfigured;

		std::uint32_t previous_update_ms_ = 0U;
		std::uint8_t demo_step_ = 0U;

		void initializePeripherals();
		void initializeServoController();
		void runDemoTask();

		void printBootBanner();
		void printStatus( ServoStatus status );
		void printUnsigned( std::uint32_t value );
};

} // namespace first_project