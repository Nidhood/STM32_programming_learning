#include "delay.h"
#include "stm32f401xe.h"

extern "C" {
extern std::uint32_t SystemCoreClock;
void SystemCoreClockUpdate( void );
}

namespace {

constexpr std::uint32_t DELAY_COUNTER_HZ = 1000000UL;
constexpr std::uint32_t MAX_SAFE_DELAY_US = 0x7FFFFFFFUL;

TIM_TypeDef *g_delay_timer = nullptr;
std::uint8_t g_delay_timer_id = 0U;
std::uint32_t g_delay_timer_clock_hz = 0U;
std::uint32_t g_delay_counter_hz = DELAY_COUNTER_HZ;
bool g_delay_initialized = false;

std::uint32_t decodeAPBPrescaler( std::uint32_t ppre_bits ) {
	if( ( ppre_bits & 0x4U ) == 0U ) {
		return 1U;
	}

	return 1UL << ( ( ppre_bits & 0x3U ) + 1U );
}

std::uint32_t getAPB1TimerClock() {
	SystemCoreClockUpdate();

	const std::uint32_t hclk = SystemCoreClock;
	const std::uint32_t ppre1_bits = ( RCC->CFGR >> RCC_CFGR_PPRE1_Pos ) & 0x7U;
	const std::uint32_t apb_divider = decodeAPBPrescaler( ppre1_bits );
	const std::uint32_t pclk = hclk / apb_divider;

	return ( apb_divider == 1U ) ? pclk : ( 2U * pclk );
}

bool configureDelayTimer( std::uint8_t timer_id ) {
	switch( timer_id ) {
		case 2:
			g_delay_timer = TIM2;
			g_delay_timer_id = 2U;
			RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
			(void)RCC->APB1ENR;
			break;

		case 5:
			g_delay_timer = TIM5;
			g_delay_timer_id = 5U;
			RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
			(void)RCC->APB1ENR;
			break;

		default:
			g_delay_timer = nullptr;
			g_delay_timer_id = 0U;
			return false;
	}

	g_delay_timer_clock_hz = getAPB1TimerClock();

	if( g_delay_timer_clock_hz == 0U ) {
		return false;
	}

	std::uint32_t prescaler_counts =
		( g_delay_timer_clock_hz + ( DELAY_COUNTER_HZ / 2U ) ) / DELAY_COUNTER_HZ;

	if( prescaler_counts == 0U ) {
		prescaler_counts = 1U;
	}

	if( prescaler_counts > 0x10000U ) {
		prescaler_counts = 0x10000U;
	}

	g_delay_counter_hz = g_delay_timer_clock_hz / prescaler_counts;

	g_delay_timer->CR1 &= ~TIM_CR1_CEN;

	g_delay_timer->PSC = prescaler_counts - 1U;
	g_delay_timer->ARR = 0xFFFFFFFFUL;
	g_delay_timer->CNT = 0U;

	g_delay_timer->DIER = 0U;
	g_delay_timer->SR = 0U;

	g_delay_timer->EGR = TIM_EGR_UG;
	g_delay_timer->SR = 0U;

	g_delay_timer->CR1 |= TIM_CR1_CEN;

	return true;
}

std::uint32_t readDelayTicks() {
	if( !g_delay_initialized || g_delay_timer == nullptr ) {
		return 0U;
	}

	return g_delay_timer->CNT;
}

std::uint32_t saturatingMultiply( std::uint32_t value, std::uint32_t multiplier ) {
	if( value == 0U ) {
		return 0U;
	}

	if( value > ( MAX_SAFE_DELAY_US / multiplier ) ) {
		return MAX_SAFE_DELAY_US;
	}

	return value * multiplier;
}

std::uint32_t usToTicks( std::uint32_t time_us ) {
	if( time_us == 0U ) {
		return 0U;
	}

	const std::uint64_t ticks =
		( static_cast<std::uint64_t>( time_us ) * g_delay_counter_hz + 999999ULL ) /
		1000000ULL;

	if( ticks == 0ULL ) {
		return 1U;
	}

	if( ticks > MAX_SAFE_DELAY_US ) {
		return MAX_SAFE_DELAY_US;
	}

	return static_cast<std::uint32_t>( ticks );
}

std::uint32_t ticksToUs( std::uint32_t ticks ) {
	const std::uint64_t us =
		( static_cast<std::uint64_t>( ticks ) * 1000000ULL ) /
		static_cast<std::uint64_t>( g_delay_counter_hz );

	return static_cast<std::uint32_t>( us );
}

bool elapsedTicks( std::uint32_t start_ticks, std::uint32_t duration_ticks ) {
	const std::uint32_t now = readDelayTicks();
	return static_cast<std::uint32_t>( now - start_ticks ) >= duration_ticks;
}

} // namespace

namespace first_project {

Delay::Delay( std::uint32_t duration_us ) {
	start_us( duration_us );
}

void Delay::start_us( std::uint32_t duration_us ) {
	duration_ticks_ = usToTicks( duration_us );
	start_ticks_ = readDelayTicks();
	active_ = true;
}

void Delay::start_ms( std::uint32_t duration_ms ) {
	start_us( saturatingMultiply( duration_ms, 1000U ) );
}

void Delay::start_s( std::uint32_t duration_s ) {
	start_us( saturatingMultiply( duration_s, 1000000U ) );
}

void Delay::restart() {
	start_ticks_ = readDelayTicks();
	active_ = true;
}

void Delay::stop() {
	active_ = false;
}

bool Delay::expired() const {
	if( !active_ ) {
		return false;
	}

	return elapsedTicks( start_ticks_, duration_ticks_ );
}

bool Delay::running() const {
	return active_ && !expired();
}

bool Delay::active() const {
	return active_;
}

bool initializeDelay( std::uint8_t timer_id ) {
	g_delay_initialized = false;
	g_delay_initialized = configureDelayTimer( timer_id );
	return g_delay_initialized;
}

std::uint32_t micros() {
	return ticksToUs( readDelayTicks() );
}

std::uint32_t millis() {
	return micros() / 1000U;
}

Delay delay_us( std::uint32_t time_us ) {
	return Delay( time_us );
}

Delay delay_ms( std::uint32_t time_ms ) {
	Delay delay;
	delay.start_ms( time_ms );
	return delay;
}

Delay delay_s( std::uint32_t time_s ) {
	Delay delay;
	delay.start_s( time_s );
	return delay;
}

bool hasElapsed_us( std::uint32_t start_us, std::uint32_t interval_us ) {
	const std::uint32_t now = micros();
	return static_cast<std::uint32_t>( now - start_us ) >= interval_us;
}

bool hasElapsed_ms( std::uint32_t start_ms, std::uint32_t interval_ms ) {
	const std::uint32_t now = millis();
	return static_cast<std::uint32_t>( now - start_ms ) >= interval_ms;
}

bool every_us( std::uint32_t &previous_us, std::uint32_t period_us ) {
	const std::uint32_t now = micros();

	if( static_cast<std::uint32_t>( now - previous_us ) >= period_us ) {
		previous_us += period_us;
		return true;
	}

	return false;
}

bool every_ms( std::uint32_t &previous_ms, std::uint32_t period_ms ) {
	const std::uint32_t now = millis();

	if( static_cast<std::uint32_t>( now - previous_ms ) >= period_ms ) {
		previous_ms += period_ms;
		return true;
	}

	return false;
}

} // namespace first_project