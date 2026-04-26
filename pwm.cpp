#include "pwm.h"

#include "pin.h"
#include "stm32f401xe.h"

namespace first_project {

PWM::PWM( std::uint8_t timer_id ) {
	setTimer( timer_id );
}

std::uint8_t PWM::getAlternateFunction( std::uint8_t timer_id ) {
	switch( timer_id ) {
		case 1U:
		case 2U:
			return 1U; // AF1: TIM1 / TIM2.

		case 3U:
		case 4U:
		case 5U:
			return 2U; // AF2: TIM3 / TIM4 / TIM5.

		case 9U:
		case 10U:
		case 11U:
			return 3U; // AF3: TIM9 / TIM10 / TIM11.

		default:
			return 0U;
	}
}

std::uint8_t PWM::getMaxChannel() const {
	switch( timer_id_ ) {
		case 1U:
		case 2U:
		case 3U:
		case 4U:
		case 5U:
			return 4U;

		case 9U:
			return 2U;

		case 10U:
		case 11U:
			return 1U;

		default:
			return 0U;
	}
}

bool PWM::isValidChannel( std::uint8_t channel ) const {
	return ( channel >= 1U ) && ( channel <= getMaxChannel() );
}

volatile std::uint32_t *PWM::compareRegister( std::uint8_t channel ) const {
	if( timer_ == nullptr ) {
		return nullptr;
	}

	switch( channel ) {
		case 1U:
			return &timer_->CCR1;

		case 2U:
			return &timer_->CCR2;

		case 3U:
			return &timer_->CCR3;

		case 4U:
			return &timer_->CCR4;

		default:
			return nullptr;
	}
}

void PWM::pwmInit( std::uint8_t timer_id, double frequency_hz ) {
	setTimer( timer_id );

	if( !isConfigured() || frequency_hz <= 0.0 ) {
		return;
	}

	setFreq( frequency_hz );

	active_channel_ = 0U;

	timer_->CR1 &= ~TIM_CR1_CEN;

	timer_->CCER = 0U;
	timer_->CCMR1 = 0U;
	timer_->CCMR2 = 0U;

	timer_->CCR1 = 0U;
	timer_->CCR2 = 0U;
	timer_->CCR3 = 0U;
	timer_->CCR4 = 0U;

	timer_->CR1 |= TIM_CR1_ARPE;
	timer_->EGR = TIM_EGR_UG;
	timer_->SR &= ~TIM_SR_UIF;

	timer_->CR1 |= TIM_CR1_CEN;
}

void PWM::pinChannel( PinName_t pin ) {
	if( !isConfigured() ) {
		return;
	}

	const std::uint8_t alternate_function = getAlternateFunction( timer_id_ );

	if( alternate_function == 0U ) {
		return;
	}

	Pin_Config_t config = {};

	config.pin = pin;
	config.mode = PIN_MODE_AF;
	config.output_type = PIN_OUTPUT_PUSH_PULL;
	config.speed = PIN_SPEED_VERY_HIGH;
	config.pull = PIN_PULL_NONE;
	config.alternate_function = alternate_function;

	Pin_Init( &config );
}

void PWM::disableChannelOutput( std::uint8_t channel ) {
	if( timer_ == nullptr ) {
		return;
	}

	switch( channel ) {
		case 1U:
			timer_->CCER &= ~TIM_CCER_CC1E;
			break;

		case 2U:
			timer_->CCER &= ~TIM_CCER_CC2E;
			break;

		case 3U:
			timer_->CCER &= ~TIM_CCER_CC3E;
			break;

		case 4U:
			timer_->CCER &= ~TIM_CCER_CC4E;
			break;

		default:
			break;
	}
}

void PWM::configurePwmMode( std::uint8_t channel ) {
	if( timer_ == nullptr ) {
		return;
	}

	switch( channel ) {
		case 1U:
			timer_->CCMR1 &= ~( TIM_CCMR1_CC1S | TIM_CCMR1_OC1M );
			timer_->CCMR1 |= ( kPwmMode1 << TIM_CCMR1_OC1M_Pos );
			timer_->CCMR1 |= TIM_CCMR1_OC1PE;
			break;

		case 2U:
			timer_->CCMR1 &= ~( TIM_CCMR1_CC2S | TIM_CCMR1_OC2M );
			timer_->CCMR1 |= ( kPwmMode1 << TIM_CCMR1_OC2M_Pos );
			timer_->CCMR1 |= TIM_CCMR1_OC2PE;
			break;

		case 3U:
			timer_->CCMR2 &= ~( TIM_CCMR2_CC3S | TIM_CCMR2_OC3M );
			timer_->CCMR2 |= ( kPwmMode1 << TIM_CCMR2_OC3M_Pos );
			timer_->CCMR2 |= TIM_CCMR2_OC3PE;
			break;

		case 4U:
			timer_->CCMR2 &= ~( TIM_CCMR2_CC4S | TIM_CCMR2_OC4M );
			timer_->CCMR2 |= ( kPwmMode1 << TIM_CCMR2_OC4M_Pos );
			timer_->CCMR2 |= TIM_CCMR2_OC4PE;
			break;

		default:
			break;
	}
}

void PWM::enableChannelOutput( std::uint8_t channel ) {
	if( timer_ == nullptr ) {
		return;
	}

	switch( channel ) {
		case 1U:
			timer_->CCER |= TIM_CCER_CC1E;
			break;

		case 2U:
			timer_->CCER |= TIM_CCER_CC2E;
			break;

		case 3U:
			timer_->CCER |= TIM_CCER_CC3E;
			break;

		case 4U:
			timer_->CCER |= TIM_CCER_CC4E;
			break;

		default:
			break;
	}
}

void PWM::activateChannel( std::uint8_t channel ) {
	if( !isConfigured() || !isValidChannel( channel ) ) {
		return;
	}

	active_channel_ = channel;

	disableChannelOutput( channel );
	configurePwmMode( channel );
	setPolarity( channel, false );
	setPwm( channel, 0.0 );
	enableChannelOutput( channel );

	timer_->CR1 |= TIM_CR1_ARPE;

	if( timer_id_ == 1U ) {
		timer_->BDTR |= TIM_BDTR_MOE;
	}

	timer_->EGR = TIM_EGR_UG;
	timer_->SR &= ~TIM_SR_UIF;
	timer_->CR1 |= TIM_CR1_CEN;
}

void PWM::setPolarity( std::uint8_t channel, bool inverted ) {
	if( !isConfigured() || !isValidChannel( channel ) ) {
		return;
	}

	std::uint32_t polarity_bit = 0U;

	switch( channel ) {
		case 1U:
			polarity_bit = TIM_CCER_CC1P;
			break;

		case 2U:
			polarity_bit = TIM_CCER_CC2P;
			break;

		case 3U:
			polarity_bit = TIM_CCER_CC3P;
			break;

		case 4U:
			polarity_bit = TIM_CCER_CC4P;
			break;

		default:
			return;
	}

	if( inverted ) {
		timer_->CCER |= polarity_bit;
	} else {
		timer_->CCER &= ~polarity_bit;
	}
}

void PWM::invertPolarity() {
	if( !isConfigured() || !isValidChannel( active_channel_ ) ) {
		return;
	}

	switch( active_channel_ ) {
		case 1U:
			timer_->CCER ^= TIM_CCER_CC1P;
			break;

		case 2U:
			timer_->CCER ^= TIM_CCER_CC2P;
			break;

		case 3U:
			timer_->CCER ^= TIM_CCER_CC3P;
			break;

		case 4U:
			timer_->CCER ^= TIM_CCER_CC4P;
			break;

		default:
			break;
	}
}

void PWM::setPwm( std::uint8_t channel, double duty_percent ) {
	if( !isConfigured() || !isValidChannel( channel ) ) {
		return;
	}

	volatile std::uint32_t *ccr = compareRegister( channel );

	if( ccr == nullptr ) {
		return;
	}

	if( duty_percent < 0.0 ) {
		duty_percent = 0.0;
	}

	if( duty_percent > 100.0 ) {
		duty_percent = 100.0;
	}

	const std::uint64_t period_counts = static_cast<std::uint64_t>( timer_->ARR ) + 1ULL;

	std::uint64_t compare_value =
		static_cast<std::uint64_t>(
			( static_cast<double>( period_counts ) * duty_percent / 100.0 ) + 0.5 );

	if( compare_value > static_cast<std::uint64_t>( timer_->ARR ) ) {
		compare_value = static_cast<std::uint64_t>( timer_->ARR );
	}

	*ccr = static_cast<std::uint32_t>( compare_value );
}

std::uintptr_t PWM::getPtr( std::uint8_t channel ) const {
	volatile std::uint32_t *ccr = compareRegister( channel );

	if( ccr == nullptr ) {
		return 0U;
	}

	return reinterpret_cast<std::uintptr_t>( ccr );
}

} // namespace first_project