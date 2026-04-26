#pragma once

#include <cstdint>

namespace first_project {

class Delay {
	private:
		std::uint32_t start_ticks_ = 0U;
		std::uint32_t duration_ticks_ = 0U;
		bool active_ = false;

	public:
		Delay() = default;

		explicit Delay( std::uint32_t duration_us );

		void start_us( std::uint32_t duration_us );
		void start_ms( std::uint32_t duration_ms );
		void start_s( std::uint32_t duration_s );

		void restart();
		void stop();

		bool expired() const;
		bool running() const;
		bool active() const;
};

bool initializeDelay( std::uint8_t timer_id = 5U );

std::uint32_t micros();
std::uint32_t millis();

Delay delay_us( std::uint32_t time_us );
Delay delay_ms( std::uint32_t time_ms );
Delay delay_s( std::uint32_t time_s );

bool hasElapsed_us( std::uint32_t start_us, std::uint32_t interval_us );
bool hasElapsed_ms( std::uint32_t start_ms, std::uint32_t interval_ms );

bool every_us( std::uint32_t &previous_us, std::uint32_t period_us );
bool every_ms( std::uint32_t &previous_ms, std::uint32_t period_ms );

} // namespace first_project