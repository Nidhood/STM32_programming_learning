#pragma once

#include <cstdint>
#include <cstddef>

#include "stm32f4xx.h"
#include "timer.h"

namespace first_project {

class PWM : public Timer {
	private:
		std::uint8_t active_channel_ = 0U;

		static constexpr std::uint32_t kPwmMode1 = 6U;

		static std::uint8_t getAlternateFunction( std::uint8_t timer_id );

		std::uint8_t getMaxChannel() const;
		bool isValidChannel( std::uint8_t channel ) const;

		void configurePwmMode( std::uint8_t channel );
		void enableChannelOutput( std::uint8_t channel );
		void disableChannelOutput( std::uint8_t channel );

		volatile std::uint32_t *compareRegister( std::uint8_t channel ) const;

	public:
		PWM() = default;
		explicit PWM( std::uint8_t timer_id );

		void pwmInit( std::uint8_t timer_id, double frequency_hz );

		void pinChannel( PinName_t pin );

		void activateChannel( std::uint8_t channel );

		void setPolarity( std::uint8_t channel, bool inverted );
		void invertPolarity();

		void setPwm( std::uint8_t channel, double duty_percent );

		std::uintptr_t getPtr( std::uint8_t channel ) const;
};

} // namespace first_project