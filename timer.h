#pragma once

#include <cstdint>

#include "stm32f401xe.h"

namespace first_project {

using TimerCallback = void ( * )( void );

class Timer {
	protected:
		TIM_TypeDef *timer_ = nullptr;
		std::uint8_t timer_id_ = 0U;
		std::uint32_t timer_clock_hz_ = 0U;

		static constexpr std::uint32_t kMaxPrescaler = 0xFFFFU;

		static std::uint32_t getHCLK();
		static std::uint32_t decodeAPBPrescaler( std::uint32_t ppre_bits );
		static std::uint32_t getAPB1TimerClock();
		static std::uint32_t getAPB2TimerClock();
		static std::uint32_t getMaxAutoReload( std::uint8_t timer_id );

		bool isConfigured() const;
		void refreshTimerClock();

		void applyBaseConfiguration( std::uint32_t prescaler, std::uint32_t auto_reload );
		void setPeriodWithAutoReload( double period_s );
		void setPeriodWithFixedAutoReload( double period_s, std::uint32_t auto_reload );

	public:
		Timer() = default;
		explicit Timer( std::uint8_t timer_id );

		void setTimer( std::uint8_t timer_id );

		void dma( bool enable );

		void setPeriod( double period_s );
		void setPeriod( double period_s, unsigned int auto_reload );

		void setFreq( double frequency_hz );
		void setFreq( double frequency_hz, unsigned int auto_reload );

		double getPeriod() const;
		double getFreq() const;

		void intr( TimerCallback callback );

		double operator=( double period_s );
		void operator=( TimerCallback callback );
};

} // namespace first_project