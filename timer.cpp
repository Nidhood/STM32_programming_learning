#include "timer.h"

#include <cmath>

namespace {

first_project::TimerCallback tim2_callback = nullptr;
first_project::TimerCallback tim3_callback = nullptr;
first_project::TimerCallback tim4_callback = nullptr;
first_project::TimerCallback tim5_callback = nullptr;
first_project::TimerCallback tim9_callback = nullptr;
first_project::TimerCallback tim10_callback = nullptr;
first_project::TimerCallback tim11_callback = nullptr;

void handleUpdateInterrupt( TIM_TypeDef *timer, first_project::TimerCallback callback ) {
	if( timer == nullptr ) {
		return;
	}

	if( ( timer->SR & TIM_SR_UIF ) == 0U ) {
		return;
	}

	timer->SR &= ~TIM_SR_UIF;

	if( callback != nullptr ) {
		callback();
	}
}

} // namespace

extern "C" {

void TIM2_IRQHandler( void ) {
	handleUpdateInterrupt( TIM2, tim2_callback );
}

void TIM3_IRQHandler( void ) {
	handleUpdateInterrupt( TIM3, tim3_callback );
}

void TIM4_IRQHandler( void ) {
	handleUpdateInterrupt( TIM4, tim4_callback );
}

void TIM5_IRQHandler( void ) {
	handleUpdateInterrupt( TIM5, tim5_callback );
}

void TIM1_BRK_TIM9_IRQHandler( void ) {
	handleUpdateInterrupt( TIM9, tim9_callback );
}

void TIM1_UP_TIM10_IRQHandler( void ) {
	handleUpdateInterrupt( TIM10, tim10_callback );
}

void TIM1_TRG_COM_TIM11_IRQHandler( void ) {
	handleUpdateInterrupt( TIM11, tim11_callback );
}
}

namespace first_project {

Timer::Timer( std::uint8_t timer_id ) {
	setTimer( timer_id );
}

std::uint32_t Timer::getHCLK() {
	SystemCoreClockUpdate();
	return SystemCoreClock;
}

std::uint32_t Timer::decodeAPBPrescaler( std::uint32_t ppre_bits ) {
	switch( ppre_bits & 0x7UL ) {
		case 4U:
			return 2U;

		case 5U:
			return 4U;

		case 6U:
			return 8U;

		case 7U:
			return 16U;

		default:
			return 1U;
	}
}

std::uint32_t Timer::getAPB1TimerClock() {
	const std::uint32_t hclk = getHCLK();
	const std::uint32_t ppre1_bits = ( RCC->CFGR >> RCC_CFGR_PPRE1_Pos ) & 0x7UL;
	const std::uint32_t apb1_divider = decodeAPBPrescaler( ppre1_bits );
	const std::uint32_t pclk1 = hclk / apb1_divider;

	if( apb1_divider == 1U ) {
		return pclk1;
	}

	return pclk1 * 2U;
}

std::uint32_t Timer::getAPB2TimerClock() {
	const std::uint32_t hclk = getHCLK();
	const std::uint32_t ppre2_bits = ( RCC->CFGR >> RCC_CFGR_PPRE2_Pos ) & 0x7UL;
	const std::uint32_t apb2_divider = decodeAPBPrescaler( ppre2_bits );
	const std::uint32_t pclk2 = hclk / apb2_divider;

	if( apb2_divider == 1U ) {
		return pclk2;
	}

	return pclk2 * 2U;
}

std::uint32_t Timer::getMaxAutoReload( std::uint8_t timer_id ) {
	switch( timer_id ) {
		case 2U:
		case 5U:
			return 0xFFFFFFFFUL;

		default:
			return 0xFFFFUL;
	}
}

bool Timer::isConfigured() const {
	return ( timer_ != nullptr ) && ( timer_id_ != 0U ) && ( timer_clock_hz_ != 0U );
}

void Timer::refreshTimerClock() {
	switch( timer_id_ ) {
		case 1U:
		case 9U:
		case 10U:
		case 11U:
			timer_clock_hz_ = getAPB2TimerClock();
			break;

		case 2U:
		case 3U:
		case 4U:
		case 5U:
			timer_clock_hz_ = getAPB1TimerClock();
			break;

		default:
			timer_clock_hz_ = 0U;
			break;
	}
}

void Timer::setTimer( std::uint8_t timer_id ) {
	timer_ = nullptr;
	timer_id_ = 0U;
	timer_clock_hz_ = 0U;

	switch( timer_id ) {
		case 1U:
			timer_ = TIM1;
			timer_id_ = 1U;
			RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
			(void)RCC->APB2ENR;
			break;

		case 2U:
			timer_ = TIM2;
			timer_id_ = 2U;
			RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
			(void)RCC->APB1ENR;
			break;

		case 3U:
			timer_ = TIM3;
			timer_id_ = 3U;
			RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
			(void)RCC->APB1ENR;
			break;

		case 4U:
			timer_ = TIM4;
			timer_id_ = 4U;
			RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
			(void)RCC->APB1ENR;
			break;

		case 5U:
			timer_ = TIM5;
			timer_id_ = 5U;
			RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
			(void)RCC->APB1ENR;
			break;

		case 9U:
			timer_ = TIM9;
			timer_id_ = 9U;
			RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
			(void)RCC->APB2ENR;
			break;

		case 10U:
			timer_ = TIM10;
			timer_id_ = 10U;
			RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
			(void)RCC->APB2ENR;
			break;

		case 11U:
			timer_ = TIM11;
			timer_id_ = 11U;
			RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
			(void)RCC->APB2ENR;
			break;

		default:
			return;
	}

	refreshTimerClock();

	timer_->CR1 &= ~TIM_CR1_CEN;
	timer_->CNT = 0U;
	timer_->SR = 0U;
}

void Timer::applyBaseConfiguration( std::uint32_t prescaler, std::uint32_t auto_reload ) {
	if( !isConfigured() ) {
		return;
	}

	const std::uint32_t max_auto_reload = getMaxAutoReload( timer_id_ );

	if( prescaler > kMaxPrescaler ) {
		prescaler = kMaxPrescaler;
	}

	if( auto_reload > max_auto_reload ) {
		auto_reload = max_auto_reload;
	}

	timer_->CR1 &= ~TIM_CR1_CEN;

	timer_->PSC = prescaler;
	timer_->ARR = auto_reload;
	timer_->CNT = 0U;

	timer_->CR1 |= TIM_CR1_ARPE;
	timer_->EGR = TIM_EGR_UG;
	timer_->SR &= ~TIM_SR_UIF;

	timer_->CR1 |= TIM_CR1_CEN;
}

void Timer::setPeriodWithAutoReload( double period_s ) {
	if( !isConfigured() || period_s <= 0.0 ) {
		return;
	}

	const std::uint32_t max_auto_reload = getMaxAutoReload( timer_id_ );

	double total_ticks = period_s * static_cast<double>( timer_clock_hz_ );

	if( total_ticks < 1.0 ) {
		total_ticks = 1.0;
	}

	double prescaler_value = std::ceil( total_ticks / ( static_cast<double>( max_auto_reload ) + 1.0 ) ) - 1.0;

	if( prescaler_value < 0.0 ) {
		prescaler_value = 0.0;
	}

	if( prescaler_value > static_cast<double>( kMaxPrescaler ) ) {
		prescaler_value = static_cast<double>( kMaxPrescaler );
	}

	const std::uint32_t prescaler = static_cast<std::uint32_t>( prescaler_value );

	double auto_reload_value = ( total_ticks / static_cast<double>( prescaler + 1U ) ) - 1.0;

	if( auto_reload_value < 0.0 ) {
		auto_reload_value = 0.0;
	}

	if( auto_reload_value > static_cast<double>( max_auto_reload ) ) {
		auto_reload_value = static_cast<double>( max_auto_reload );
	}

	const std::uint32_t auto_reload = static_cast<std::uint32_t>( auto_reload_value + 0.5 );

	applyBaseConfiguration( prescaler, auto_reload );
}

void Timer::setPeriodWithFixedAutoReload( double period_s, std::uint32_t auto_reload ) {
	if( !isConfigured() || period_s <= 0.0 ) {
		return;
	}

	const std::uint32_t max_auto_reload = getMaxAutoReload( timer_id_ );

	if( auto_reload > max_auto_reload ) {
		auto_reload = max_auto_reload;
	}

	double total_ticks = period_s * static_cast<double>( timer_clock_hz_ );

	if( total_ticks < 1.0 ) {
		total_ticks = 1.0;
	}

	double prescaler_value = ( total_ticks / static_cast<double>( auto_reload + 1U ) ) - 1.0;

	if( prescaler_value < 0.0 ) {
		prescaler_value = 0.0;
	}

	if( prescaler_value > static_cast<double>( kMaxPrescaler ) ) {
		prescaler_value = static_cast<double>( kMaxPrescaler );
	}

	const std::uint32_t prescaler = static_cast<std::uint32_t>( prescaler_value + 0.5 );

	applyBaseConfiguration( prescaler, auto_reload );
}

void Timer::dma( bool enable ) {
	if( !isConfigured() ) {
		return;
	}

	if( enable ) {
		timer_->DIER |= TIM_DIER_UDE;
	} else {
		timer_->DIER &= ~TIM_DIER_UDE;
	}
}

void Timer::setPeriod( double period_s ) {
	setPeriodWithAutoReload( period_s );
}

void Timer::setPeriod( double period_s, unsigned int auto_reload ) {
	setPeriodWithFixedAutoReload( period_s, static_cast<std::uint32_t>( auto_reload ) );
}

void Timer::setFreq( double frequency_hz ) {
	if( frequency_hz <= 0.0 ) {
		return;
	}

	setPeriod( 1.0 / frequency_hz );
}

void Timer::setFreq( double frequency_hz, unsigned int auto_reload ) {
	if( frequency_hz <= 0.0 ) {
		return;
	}

	setPeriod( 1.0 / frequency_hz, auto_reload );
}

double Timer::getPeriod() const {
	if( !isConfigured() ) {
		return 0.0;
	}

	const double prescaler = static_cast<double>( timer_->PSC ) + 1.0;
	const double auto_reload = static_cast<double>( timer_->ARR ) + 1.0;
	const double timer_clock = static_cast<double>( timer_clock_hz_ );

	return ( prescaler * auto_reload ) / timer_clock;
}

double Timer::getFreq() const {
	const double period = getPeriod();

	if( period <= 0.0 ) {
		return 0.0;
	}

	return 1.0 / period;
}

void Timer::intr( TimerCallback callback ) {
	if( !isConfigured() ) {
		return;
	}

	IRQn_Type irq_number;

	switch( timer_id_ ) {
		case 2U:
			tim2_callback = callback;
			irq_number = TIM2_IRQn;
			break;

		case 3U:
			tim3_callback = callback;
			irq_number = TIM3_IRQn;
			break;

		case 4U:
			tim4_callback = callback;
			irq_number = TIM4_IRQn;
			break;

		case 5U:
			tim5_callback = callback;
			irq_number = TIM5_IRQn;
			break;

		case 9U:
			tim9_callback = callback;
			irq_number = TIM1_BRK_TIM9_IRQn;
			break;

		case 10U:
			tim10_callback = callback;
			irq_number = TIM1_UP_TIM10_IRQn;
			break;

		case 11U:
			tim11_callback = callback;
			irq_number = TIM1_TRG_COM_TIM11_IRQn;
			break;

		default:
			return;
	}

	timer_->SR &= ~TIM_SR_UIF;

	if( callback == nullptr ) {
		timer_->DIER &= ~TIM_DIER_UIE;
		return;
	}

	timer_->DIER |= TIM_DIER_UIE;

	NVIC_ClearPendingIRQ( irq_number );
	NVIC_EnableIRQ( irq_number );
}

double Timer::operator=( double period_s ) {
	setPeriod( period_s );
	return period_s;
}

void Timer::operator=( TimerCallback callback ) {
	intr( callback );
}

} // namespace first_project