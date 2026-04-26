#include "usart.h"
#include "pin.h"

namespace {

const STM32F_PinAF_t kDefaultUSART2Tx = { PA( 2 ), 7U };
const STM32F_PinAF_t kDefaultUSART2Rx = { PA( 3 ), 7U };

bool pinEquals( PinName_t a, PinName_t b ) {
	return a == b;
}

} // namespace

first_project::USART::USART( std::uint32_t baudrate ) {
	start( baudrate );
}

std::uint32_t first_project::USART::decodeAHBPrescaler( std::uint32_t hpre_bits ) {
	switch( hpre_bits & 0x0FU ) {
		case 0x08U:
			return 2U;

		case 0x09U:
			return 4U;

		case 0x0AU:
			return 8U;

		case 0x0BU:
			return 16U;

		case 0x0CU:
			return 64U;

		case 0x0DU:
			return 128U;

		case 0x0EU:
			return 256U;

		case 0x0FU:
			return 512U;

		default:
			return 1U;
	}
}

std::uint32_t first_project::USART::decodeAPBPrescaler( std::uint32_t ppre_bits ) {
	switch( ppre_bits & 0x07U ) {
		case 0x04U:
			return 2U;

		case 0x05U:
			return 4U;

		case 0x06U:
			return 8U;

		case 0x07U:
			return 16U;

		default:
			return 1U;
	}
}

std::uint32_t first_project::USART::getHCLK() {
	SystemCoreClockUpdate();

	const std::uint32_t hpre_bits =
		( RCC->CFGR & RCC_CFGR_HPRE ) >> RCC_CFGR_HPRE_Pos;

	return SystemCoreClock / decodeAHBPrescaler( hpre_bits );
}

std::uint32_t first_project::USART::getPCLK1() {
	const std::uint32_t ppre1_bits =
		( RCC->CFGR & RCC_CFGR_PPRE1 ) >> RCC_CFGR_PPRE1_Pos;

	return getHCLK() / decodeAPBPrescaler( ppre1_bits );
}

std::uint32_t first_project::USART::getPCLK2() {
	const std::uint32_t ppre2_bits =
		( RCC->CFGR & RCC_CFGR_PPRE2 ) >> RCC_CFGR_PPRE2_Pos;

	return getHCLK() / decodeAPBPrescaler( ppre2_bits );
}

bool first_project::USART::selectPortFromPins( PinName_t tx, PinName_t rx ) {
#if defined( USART1 )
	if( ( pinEquals( tx, PA( 9 ) ) && pinEquals( rx, PA( 10 ) ) ) ||
		( pinEquals( tx, PB( 6 ) ) && pinEquals( rx, PB( 7 ) ) ) ) {
		port_ = USART1;
		return true;
	}
#endif

#if defined( USART2 )
	if( pinEquals( tx, PA( 2 ) ) && pinEquals( rx, PA( 3 ) ) ) {
		port_ = USART2;
		return true;
	}
#endif

#if defined( USART6 )
	if( ( pinEquals( tx, PC( 6 ) ) && pinEquals( rx, PC( 7 ) ) ) ||
		( pinEquals( tx, PA( 11 ) ) && pinEquals( rx, PA( 12 ) ) ) ) {
		port_ = USART6;
		return true;
	}
#endif

	port_ = nullptr;
	return false;
}

bool first_project::USART::configurePins( STM32F_PinAF_t tx, STM32F_PinAF_t rx ) {
	Pin_Config_t tx_config = {};
	tx_config.pin = tx.pin;
	tx_config.mode = PIN_MODE_AF;
	tx_config.output_type = PIN_OUTPUT_PUSH_PULL;
	tx_config.speed = PIN_SPEED_VERY_HIGH;
	tx_config.pull = PIN_PULL_UP;
	tx_config.alternate_function = tx.af;

	Pin_Config_t rx_config = {};
	rx_config.pin = rx.pin;
	rx_config.mode = PIN_MODE_AF;
	rx_config.output_type = PIN_OUTPUT_PUSH_PULL;
	rx_config.speed = PIN_SPEED_VERY_HIGH;
	rx_config.pull = PIN_PULL_UP;
	rx_config.alternate_function = rx.af;

	return Pin_Init( &tx_config ) && Pin_Init( &rx_config );
}

bool first_project::USART::enablePeripheralClock() {
	if( port_ == nullptr ) {
		return false;
	}

#if defined( USART1 ) && defined( RCC_APB2ENR_USART1EN )
	if( port_ == USART1 ) {
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
		(void)RCC->APB2ENR;
		return true;
	}
#endif

#if defined( USART2 ) && defined( RCC_APB1ENR_USART2EN )
	if( port_ == USART2 ) {
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
		(void)RCC->APB1ENR;
		return true;
	}
#endif

#if defined( USART6 ) && defined( RCC_APB2ENR_USART6EN )
	if( port_ == USART6 ) {
		RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
		(void)RCC->APB2ENR;
		return true;
	}
#endif

	return false;
}

void first_project::USART::resetPeripheral() {
	if( port_ == nullptr ) {
		return;
	}

#if defined( USART1 ) && defined( RCC_APB2RSTR_USART1RST )
	if( port_ == USART1 ) {
		RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST;
		RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;
		return;
	}
#endif

#if defined( USART2 ) && defined( RCC_APB1RSTR_USART2RST )
	if( port_ == USART2 ) {
		RCC->APB1RSTR |= RCC_APB1RSTR_USART2RST;
		RCC->APB1RSTR &= ~RCC_APB1RSTR_USART2RST;
		return;
	}
#endif

#if defined( USART6 ) && defined( RCC_APB2RSTR_USART6RST )
	if( port_ == USART6 ) {
		RCC->APB2RSTR |= RCC_APB2RSTR_USART6RST;
		RCC->APB2RSTR &= ~RCC_APB2RSTR_USART6RST;
		return;
	}
#endif
}

std::uint32_t first_project::USART::getPeripheralClock() const {
	if( port_ == nullptr ) {
		return 0U;
	}

#if defined( USART1 )
	if( port_ == USART1 ) {
		return getPCLK2();
	}
#endif

#if defined( USART2 )
	if( port_ == USART2 ) {
		return getPCLK1();
	}
#endif

#if defined( USART6 )
	if( port_ == USART6 ) {
		return getPCLK2();
	}
#endif

	return 0U;
}

bool first_project::USART::waitFlagSet( volatile std::uint32_t &reg, std::uint32_t flag ) const {
	std::uint32_t timeout = kTimeoutIterations;

	while( ( reg & flag ) == 0U ) {
		if( timeout == 0U ) {
			return false;
		}

		--timeout;
	}

	return true;
}

bool first_project::USART::isConfigured() const {
	return port_ != nullptr;
}

void first_project::USART::configurePeripheral( std::uint32_t baudrate ) {
	if( port_ == nullptr || baudrate == 0U ) {
		return;
	}

	const std::uint32_t peripheral_clock = getPeripheralClock();

	if( peripheral_clock == 0U ) {
		port_ = nullptr;
		return;
	}

	baudrate_ = baudrate;

	port_->CR1 &= ~USART_CR1_UE;

	port_->CR1 = 0U;
	port_->CR2 = 0U;
	port_->CR3 = 0U;

	port_->BRR = ( peripheral_clock + ( baudrate / 2U ) ) / baudrate;

	port_->CR1 |= USART_CR1_TE;
	port_->CR1 |= USART_CR1_RE;
	port_->CR1 |= USART_CR1_UE;
}

void first_project::USART::start() {
	start( kDefaultUSART2Tx, kDefaultUSART2Rx, kDefaultBaudrate );
}

void first_project::USART::start( std::uint32_t baudrate ) {
	start( kDefaultUSART2Tx, kDefaultUSART2Rx, baudrate );
}

void first_project::USART::start( STM32F_PinAF_t tx, STM32F_PinAF_t rx, std::uint32_t baudrate ) {
	tx_pin_ = tx.pin;
	rx_pin_ = rx.pin;

	if( !selectPortFromPins( tx_pin_, rx_pin_ ) ) {
		return;
	}

	if( !enablePeripheralClock() ) {
		port_ = nullptr;
		return;
	}

	if( !configurePins( tx, rx ) ) {
		port_ = nullptr;
		return;
	}

	resetPeripheral();
	configurePeripheral( baudrate );
}

bool first_project::USART::write( std::uint8_t data ) {
	if( !isConfigured() ) {
		return false;
	}

	if( !waitFlagSet( port_->SR, USART_SR_TXE ) ) {
		return false;
	}

	port_->DR = data;
	return true;
}

std::size_t first_project::USART::write( const std::uint8_t *data, std::size_t length ) {
	if( data == nullptr ) {
		return 0U;
	}

	std::size_t written = 0U;

	while( written < length ) {
		if( !write( data[written] ) ) {
			break;
		}

		++written;
	}

	return written;
}

std::size_t first_project::USART::write( const char *text ) {
	if( text == nullptr ) {
		return 0U;
	}

	std::size_t written = 0U;

	while( text[written] != '\0' ) {
		if( !write( static_cast<std::uint8_t>( text[written] ) ) ) {
			break;
		}

		++written;
	}

	return written;
}

bool first_project::USART::available() const {
	if( !isConfigured() ) {
		return false;
	}

	return ( port_->SR & USART_SR_RXNE ) != 0U;
}

bool first_project::USART::read( std::uint8_t &data ) {
	if( !available() ) {
		return false;
	}

	data = static_cast<std::uint8_t>( port_->DR & 0xFFU );
	return true;
}

std::uint8_t first_project::USART::read() {
	std::uint8_t data = 0U;
	(void)read( data );
	return data;
}

void first_project::USART::flush() {
	if( !isConfigured() ) {
		return;
	}

	(void)waitFlagSet( port_->SR, USART_SR_TC );
}