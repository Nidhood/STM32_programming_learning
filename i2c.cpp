#include "i2c.h"

#include "pin.h"

namespace {

constexpr std::uint32_t kOwnAddressEnableBit = 0x4000U;
constexpr std::uint32_t kMinI2cPeripheralClockMHz = 2U;
constexpr std::uint32_t kMaxI2cPeripheralClockMHz = 42U;

std::uint32_t clamp_u32( std::uint32_t value, std::uint32_t min_value, std::uint32_t max_value ) {
	if( value < min_value ) {
		return min_value;
	}

	if( value > max_value ) {
		return max_value;
	}

	return value;
}

} // namespace

std::uint32_t first_project::I2C::getHCLK() {
	SystemCoreClockUpdate();
	return SystemCoreClock;
}

std::uint32_t first_project::I2C::decodeAHBPrescaler( std::uint32_t hpre_bits ) {
	switch( hpre_bits & 0x0FU ) {
		case 8U:
			return 2U;
		case 9U:
			return 4U;
		case 10U:
			return 8U;
		case 11U:
			return 16U;
		case 12U:
			return 64U;
		case 13U:
			return 128U;
		case 14U:
			return 256U;
		case 15U:
			return 512U;
		default:
			return 1U;
	}
}

std::uint32_t first_project::I2C::decodeAPBPrescaler( std::uint32_t ppre_bits ) {
	switch( ppre_bits & 0x07U ) {
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

std::uint32_t first_project::I2C::getPCLK1() {
	const std::uint32_t hclk = getHCLK();

	const std::uint32_t ppre1_bits =
		( RCC->CFGR >> RCC_CFGR_PPRE1_Pos ) & 0x07U;

	const std::uint32_t apb1_divider = decodeAPBPrescaler( ppre1_bits );

	return hclk / apb1_divider;
}

std::uint8_t first_project::I2C::normalizeAddress( std::uint8_t address ) {
	if( address > 0x7FU ) {
		return static_cast<std::uint8_t>( address >> 1U );
	}

	return address;
}

std::uint8_t first_project::I2C::makeAddressByte( std::uint8_t address, bool read ) {
	const std::uint8_t normalized_address = normalizeAddress( address );

	return static_cast<std::uint8_t>(
		( normalized_address << 1U ) | ( read ? 1U : 0U ) );
}

bool first_project::I2C::selectPortFromPins( PinName_t sda, PinName_t scl ) {
	if( ( sda == PB( 9 ) && scl == PB( 8 ) ) ||
		( sda == PB( 7 ) && scl == PB( 6 ) ) ) {
		port_ = I2C1;
		return true;
	}

	port_ = nullptr;
	return false;
}

bool first_project::I2C::configurePins() {
	Pin_Config_t sda_config = {
		sda_pin_,
		PIN_MODE_AF,
		PIN_OUTPUT_OPEN_DRAIN,
		PIN_SPEED_VERY_HIGH,
		PIN_PULL_UP,
		sda_af_ };

	Pin_Config_t scl_config = {
		scl_pin_,
		PIN_MODE_AF,
		PIN_OUTPUT_OPEN_DRAIN,
		PIN_SPEED_VERY_HIGH,
		PIN_PULL_UP,
		scl_af_ };

	const bool sda_ok = Pin_Init( &sda_config );
	const bool scl_ok = Pin_Init( &scl_config );

	return sda_ok && scl_ok;
}

bool first_project::I2C::enablePeripheralClock() {
	if( port_ == I2C1 ) {
		RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
		(void)RCC->APB1ENR;
		return true;
	}

#ifdef I2C2
	if( port_ == I2C2 ) {
		RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
		(void)RCC->APB1ENR;
		return true;
	}
#endif

#ifdef I2C3
	if( port_ == I2C3 ) {
		RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;
		(void)RCC->APB1ENR;
		return true;
	}
#endif

	return false;
}

void first_project::I2C::resetPeripheral() {
	if( port_ == I2C1 ) {
		RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
		RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
		return;
	}

#ifdef I2C2
	if( port_ == I2C2 ) {
		RCC->APB1RSTR |= RCC_APB1RSTR_I2C2RST;
		RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C2RST;
		return;
	}
#endif

#ifdef I2C3
	if( port_ == I2C3 ) {
		RCC->APB1RSTR |= RCC_APB1RSTR_I2C3RST;
		RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C3RST;
		return;
	}
#endif
}

void first_project::I2C::configurePeripheral( std::uint32_t frequency_hz ) {
	if( frequency_hz == 0U ) {
		frequency_hz = kDefaultFrequencyHz;
	}

	if( frequency_hz > kMaxFastModeHz ) {
		frequency_hz = kMaxFastModeHz;
	}

	frequency_hz_ = frequency_hz;

	const std::uint32_t pclk1_hz = getPCLK1();

	std::uint32_t pclk1_mhz = pclk1_hz / 1000000U;
	pclk1_mhz = clamp_u32(
		pclk1_mhz,
		kMinI2cPeripheralClockMHz,
		kMaxI2cPeripheralClockMHz );

	port_->CR1 &= ~I2C_CR1_PE;

	port_->CR2 &= ~I2C_CR2_FREQ;
	port_->CR2 |= pclk1_mhz & I2C_CR2_FREQ;

	if( frequency_hz <= kMaxStandardModeHz ) {
		std::uint32_t ccr = pclk1_hz / ( 2U * frequency_hz );

		if( ccr < 4U ) {
			ccr = 4U;
		}

		port_->CCR = ccr & I2C_CCR_CCR;
		port_->TRISE = pclk1_mhz + 1U;
	} else {
		std::uint32_t ccr = pclk1_hz / ( 3U * frequency_hz );

		if( ccr < 1U ) {
			ccr = 1U;
		}

		port_->CCR = I2C_CCR_FS | ( ccr & I2C_CCR_CCR );
		port_->TRISE = ( ( pclk1_mhz * 300U ) / 1000U ) + 1U;
	}

	port_->CR1 |= I2C_CR1_ACK;
	port_->CR1 |= I2C_CR1_PE;
}

bool first_project::I2C::isConfigured() const {
	return port_ != nullptr;
}

bool first_project::I2C::waitFlagSet( volatile std::uint32_t &reg, std::uint32_t flag ) const {
	for( std::uint32_t timeout = kTimeoutIterations; timeout > 0U; --timeout ) {
		if( ( reg & flag ) != 0U ) {
			return true;
		}
	}

	return false;
}

bool first_project::I2C::waitFlagCleared( volatile std::uint32_t &reg, std::uint32_t flag ) const {
	for( std::uint32_t timeout = kTimeoutIterations; timeout > 0U; --timeout ) {
		if( ( reg & flag ) == 0U ) {
			return true;
		}
	}

	return false;
}

bool first_project::I2C::waitAddressSent() const {
	return waitFlagSet( port_->SR1, I2C_SR1_ADDR );
}

bool first_project::I2C::waitBusFree() const {
	if( !isConfigured() ) {
		return false;
	}

	return waitFlagCleared( port_->SR2, I2C_SR2_BUSY );
}

void first_project::I2C::clearAddressFlag() const {
	volatile std::uint32_t sr1 = port_->SR1;
	volatile std::uint32_t sr2 = port_->SR2;

	(void)sr1;
	(void)sr2;
}

void first_project::I2C::clearAcknowledgeFailure() const {
	port_->SR1 &= ~I2C_SR1_AF;
}

void first_project::I2C::generateStart() const {
	port_->CR1 |= I2C_CR1_START;
}

void first_project::I2C::generateStop() const {
	port_->CR1 |= I2C_CR1_STOP;
}

bool first_project::I2C::sendAddress( std::uint8_t address, bool read ) {
	if( !isConfigured() ) {
		return false;
	}

	clearAcknowledgeFailure();

	port_->DR = makeAddressByte( address, read );

	for( std::uint32_t timeout = kTimeoutIterations; timeout > 0U; --timeout ) {
		if( ( port_->SR1 & I2C_SR1_ADDR ) != 0U ) {
			return true;
		}

		if( ( port_->SR1 & I2C_SR1_AF ) != 0U ) {
			clearAcknowledgeFailure();
			return false;
		}
	}

	return false;
}

bool first_project::I2C::readPayload( std::uint8_t *data, std::size_t length ) {
	if( data == nullptr || length == 0U ) {
		return false;
	}

	if( length == 1U ) {
		port_->CR1 &= ~I2C_CR1_ACK;

		clearAddressFlag();
		generateStop();

		if( !waitFlagSet( port_->SR1, I2C_SR1_RXNE ) ) {
			port_->CR1 |= I2C_CR1_ACK;
			return false;
		}

		data[0] = static_cast<std::uint8_t>( port_->DR );

		port_->CR1 |= I2C_CR1_ACK;
		return true;
	}

	if( length == 2U ) {
		port_->CR1 |= I2C_CR1_POS;
		port_->CR1 &= ~I2C_CR1_ACK;

		clearAddressFlag();

		if( !waitFlagSet( port_->SR1, I2C_SR1_BTF ) ) {
			port_->CR1 &= ~I2C_CR1_POS;
			port_->CR1 |= I2C_CR1_ACK;
			return false;
		}

		generateStop();

		data[0] = static_cast<std::uint8_t>( port_->DR );
		data[1] = static_cast<std::uint8_t>( port_->DR );

		port_->CR1 &= ~I2C_CR1_POS;
		port_->CR1 |= I2C_CR1_ACK;

		return true;
	}

	port_->CR1 |= I2C_CR1_ACK;
	clearAddressFlag();

	std::size_t index = 0U;
	std::size_t remaining = length;

	while( remaining > 3U ) {
		if( !waitFlagSet( port_->SR1, I2C_SR1_RXNE ) ) {
			generateStop();
			port_->CR1 |= I2C_CR1_ACK;
			return false;
		}

		data[index++] = static_cast<std::uint8_t>( port_->DR );
		--remaining;
	}

	if( !waitFlagSet( port_->SR1, I2C_SR1_BTF ) ) {
		generateStop();
		port_->CR1 |= I2C_CR1_ACK;
		return false;
	}

	port_->CR1 &= ~I2C_CR1_ACK;

	data[index++] = static_cast<std::uint8_t>( port_->DR );
	--remaining;

	if( !waitFlagSet( port_->SR1, I2C_SR1_BTF ) ) {
		generateStop();
		port_->CR1 |= I2C_CR1_ACK;
		return false;
	}

	generateStop();

	data[index++] = static_cast<std::uint8_t>( port_->DR );
	--remaining;

	data[index] = static_cast<std::uint8_t>( port_->DR );

	port_->CR1 |= I2C_CR1_ACK;

	return true;
}

void first_project::I2C::start() {
	start( kDefaultFrequencyHz );
}

void first_project::I2C::start( unsigned int frequency_hz ) {
	start( PB( 9 ), PB( 8 ), frequency_hz, 0U );
}

void first_project::I2C::start(
	unsigned char sda,
	unsigned char scl,
	unsigned int frequency_hz ) {
	start( sda, scl, frequency_hz, 0U );
}

void first_project::I2C::start(
	unsigned char sda,
	unsigned char scl,
	unsigned int frequency_hz,
	unsigned char own_address ) {
	sda_pin_ = static_cast<PinName_t>( sda );
	scl_pin_ = static_cast<PinName_t>( scl );

	sda_af_ = kI2cAlternateFunction;
	scl_af_ = kI2cAlternateFunction;

	if( !selectPortFromPins( sda_pin_, scl_pin_ ) ) {
		return;
	}

	if( !configurePins() ) {
		port_ = nullptr;
		return;
	}

	if( !enablePeripheralClock() ) {
		port_ = nullptr;
		return;
	}

	resetPeripheral();
	configurePeripheral( frequency_hz );

	if( own_address != 0U ) {
		port_->OAR1 =
			kOwnAddressEnableBit |
			( static_cast<std::uint32_t>( normalizeAddress( own_address ) ) << 1U );
	}
}

void first_project::I2C::start(
	STM32F_PinAF_t sda,
	STM32F_PinAF_t scl,
	unsigned int frequency_hz,
	unsigned char own_address ) {
	sda_pin_ = sda.pin;
	scl_pin_ = scl.pin;

	sda_af_ = sda.af;
	scl_af_ = scl.af;

	if( sda_af_ == 0U ) {
		sda_af_ = kI2cAlternateFunction;
	}

	if( scl_af_ == 0U ) {
		scl_af_ = kI2cAlternateFunction;
	}

	if( !selectPortFromPins( sda_pin_, scl_pin_ ) ) {
		return;
	}

	if( !configurePins() ) {
		port_ = nullptr;
		return;
	}

	if( !enablePeripheralClock() ) {
		port_ = nullptr;
		return;
	}

	resetPeripheral();
	configurePeripheral( frequency_hz );

	if( own_address != 0U ) {
		port_->OAR1 =
			kOwnAddressEnableBit |
			( static_cast<std::uint32_t>( normalizeAddress( own_address ) ) << 1U );
	}
}

bool first_project::I2C::testid( unsigned char address ) {
	if( !isConfigured() ) {
		return false;
	}

	if( !waitBusFree() ) {
		return false;
	}

	generateStart();

	if( !waitFlagSet( port_->SR1, I2C_SR1_SB ) ) {
		generateStop();
		return false;
	}

	const bool address_ok = sendAddress( address, false );

	if( address_ok ) {
		clearAddressFlag();
	}

	generateStop();

	return address_ok;
}

bool first_project::I2C::write(
	std::uint8_t address,
	const std::uint8_t *data,
	std::size_t length ) {
	if( !isConfigured() ) {
		return false;
	}

	if( data == nullptr && length > 0U ) {
		return false;
	}

	if( init_callback_ != nullptr ) {
		init_callback_();
	}

	if( !waitBusFree() ) {
		return false;
	}

	generateStart();

	if( !waitFlagSet( port_->SR1, I2C_SR1_SB ) ) {
		generateStop();
		return false;
	}

	if( !sendAddress( address, false ) ) {
		generateStop();
		return false;
	}

	clearAddressFlag();

	for( std::size_t i = 0U; i < length; ++i ) {
		if( !waitFlagSet( port_->SR1, I2C_SR1_TXE ) ) {
			generateStop();
			return false;
		}

		port_->DR = data[i];
	}

	if( length > 0U ) {
		if( !waitFlagSet( port_->SR1, I2C_SR1_BTF ) ) {
			generateStop();
			return false;
		}
	}

	generateStop();

	if( finish_callback_ != nullptr ) {
		finish_callback_();
	}

	return true;
}

bool first_project::I2C::read(
	std::uint8_t address,
	std::uint8_t *data,
	std::size_t length ) {
	if( !isConfigured() ) {
		return false;
	}

	if( data == nullptr || length == 0U ) {
		return false;
	}

	if( init_callback_ != nullptr ) {
		init_callback_();
	}

	if( !waitBusFree() ) {
		return false;
	}

	generateStart();

	if( !waitFlagSet( port_->SR1, I2C_SR1_SB ) ) {
		generateStop();
		return false;
	}

	if( !sendAddress( address, true ) ) {
		generateStop();
		return false;
	}

	const bool read_ok = readPayload( data, length );

	if( finish_callback_ != nullptr ) {
		finish_callback_();
	}

	return read_ok;
}

bool first_project::I2C::writeRead(
	std::uint8_t address,
	const std::uint8_t *tx_data,
	std::size_t tx_length,
	std::uint8_t *rx_data,
	std::size_t rx_length ) {
	if( tx_length == 0U ) {
		return read( address, rx_data, rx_length );
	}

	if( rx_length == 0U ) {
		return write( address, tx_data, tx_length );
	}

	if( !isConfigured() ) {
		return false;
	}

	if( tx_data == nullptr || rx_data == nullptr ) {
		return false;
	}

	if( init_callback_ != nullptr ) {
		init_callback_();
	}

	if( !waitBusFree() ) {
		return false;
	}

	generateStart();

	if( !waitFlagSet( port_->SR1, I2C_SR1_SB ) ) {
		generateStop();
		return false;
	}

	if( !sendAddress( address, false ) ) {
		generateStop();
		return false;
	}

	clearAddressFlag();

	for( std::size_t i = 0U; i < tx_length; ++i ) {
		if( !waitFlagSet( port_->SR1, I2C_SR1_TXE ) ) {
			generateStop();
			return false;
		}

		port_->DR = tx_data[i];
	}

	if( !waitFlagSet( port_->SR1, I2C_SR1_BTF ) ) {
		generateStop();
		return false;
	}

	generateStart();

	if( !waitFlagSet( port_->SR1, I2C_SR1_SB ) ) {
		generateStop();
		return false;
	}

	if( !sendAddress( address, true ) ) {
		generateStop();
		return false;
	}

	const bool read_ok = readPayload( rx_data, rx_length );

	if( finish_callback_ != nullptr ) {
		finish_callback_();
	}

	return read_ok;
}

int first_project::I2C::rxi2c( unsigned char *data ) {
	if( !isConfigured() || data == nullptr ) {
		return 0;
	}

	if( ( port_->SR1 & I2C_SR1_RXNE ) == 0U ) {
		return 0;
	}

	*data = static_cast<unsigned char>( port_->DR );

	return 1;
}

void first_project::I2C::dmatx( bool enable ) {
	if( !isConfigured() ) {
		return;
	}

	if( enable ) {
		port_->CR2 |= I2C_CR2_DMAEN;
	} else {
		port_->CR2 &= ~I2C_CR2_DMAEN;
	}
}

void first_project::I2C::dmarx( bool enable ) {
	if( !isConfigured() ) {
		return;
	}

	if( enable ) {
		port_->CR2 |= I2C_CR2_DMAEN | I2C_CR2_LAST;
	} else {
		port_->CR2 &= ~( I2C_CR2_DMAEN | I2C_CR2_LAST );
	}
}

void first_project::I2C::txi2c(
	unsigned char address,
	const unsigned char *data,
	int length ) {
	if( length <= 0 ) {
		return;
	}

	write(
		static_cast<std::uint8_t>( address ),
		reinterpret_cast<const std::uint8_t *>( data ),
		static_cast<std::size_t>( length ) );
}

void first_project::I2C::txi2c(
	unsigned char address,
	unsigned char data,
	int length ) {
	if( length <= 0 ) {
		return;
	}

	for( int i = 0; i < length; ++i ) {
		write(
			static_cast<std::uint8_t>( address ),
			reinterpret_cast<const std::uint8_t *>( &data ),
			1U );
	}
}

void first_project::I2C::txi2c( unsigned char address, unsigned char data ) {
	write(
		static_cast<std::uint8_t>( address ),
		reinterpret_cast<const std::uint8_t *>( &data ),
		1U );
}

unsigned char first_project::I2C::rxi2c( unsigned char address ) {
	std::uint8_t data = 0U;

	read(
		static_cast<std::uint8_t>( address ),
		&data,
		1U );

	return static_cast<unsigned char>( data );
}

void first_project::I2C::rxi2c(
	unsigned char address,
	unsigned char *data,
	int length ) {
	if( length <= 0 ) {
		return;
	}

	read(
		static_cast<std::uint8_t>( address ),
		reinterpret_cast<std::uint8_t *>( data ),
		static_cast<std::size_t>( length ) );
}

void first_project::I2C::setfuncrx( FunInt2 callback ) {
	rx_callback_ = callback;
}

void first_project::I2C::setfunini( FunInt callback ) {
	init_callback_ = callback;
}

void first_project::I2C::setfunfin( FunInt callback ) {
	finish_callback_ = callback;
}