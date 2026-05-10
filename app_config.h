#pragma once

#include <cstdint>

#include "magnetic_angle_sensor.h"
#include "stm32f4xx.h"

namespace first_project {

constexpr MagneticSensorType kSelectedSensor =
	MagneticSensorType::MT6701;

// true  -> sensor through DFRobot I2C multiplexer
// false -> sensor directly on I2C1
constexpr bool kUseI2CMux = true;

constexpr STM32F_PinAF_t kI2C1SdaPin = { PB( 9 ), 4U };
constexpr STM32F_PinAF_t kI2C1SclPin = { PB( 8 ), 4U };

constexpr STM32F_PinAF_t kUsart2TxPin = { PA( 2 ), 7U };
constexpr STM32F_PinAF_t kUsart2RxPin = { PA( 3 ), 7U };

constexpr std::uint32_t kSerialBaudrate = 115200U;
constexpr std::uint32_t kI2CFrequencyHz = 100000U;

constexpr std::uint8_t kMuxAddress = 0x71U;
constexpr std::uint8_t kMuxPort = 0U;

constexpr std::uint32_t kSensorUpdatePeriodMs = 10U;
constexpr std::uint32_t kSerialPrintPeriodMs = 200U;
constexpr std::uint32_t kRecoveryPeriodMs = 1000U;

constexpr float kVelocityFilterAlpha = 0.15F;

// Servo driver configuration.
constexpr std::uint8_t kServoDriverAddress = 0x40U;
constexpr std::uint16_t kServoFrequencyHz = 50U;
constexpr std::uint8_t kServoTestChannel = 0U;

constexpr std::uint16_t kServoMinPulseUs = 600U;
constexpr std::uint16_t kServoNeutralPulseUs = 1500U;
constexpr std::uint16_t kServoMaxPulseUs = 2400U;

constexpr float kServoMinAngleDeg = 0.0F;
constexpr float kServoMaxAngleDeg = 180.0F;

constexpr std::uint32_t kServoUpdatePeriodMs = 1500U;

} // namespace first_project