#include "dma.h"

namespace first_project {

Dma::Dma( DmaId id, std::uint8_t stream ) {
	setController( id );
	setStream( stream );
}

bool Dma::setController( DmaId id ) {
	id_ = id;
	controller_ = controllerFromId( id_ );
	stream_ = streamFromIndex( id_, stream_index_ );

	if( controller_ == nullptr || stream_ == nullptr ) {
		return false;
	}

	enableClock();
	return true;
}

bool Dma::setStream( std::uint8_t stream ) {
	if( !isValidStream( stream ) ) {
		return false;
	}

	stream_index_ = stream;
	stream_ = streamFromIndex( id_, stream_index_ );

	return stream_ != nullptr;
}

DmaStatus Dma::configure( const DmaConfig &config ) {
	if( stream_ == nullptr ) {
		return DmaStatus::InvalidStream;
	}

	if( !isValidChannel( config.channel ) ) {
		return DmaStatus::InvalidArgument;
	}

	if( config.length == 0U ) {
		return DmaStatus::InvalidArgument;
	}

	if( disable() == false ) {
		return DmaStatus::Timeout;
	}

	clearFlags();

	stream_->CR = 0U;
	stream_->FCR = 0U;

	stream_->PAR = config.peripheral_address;
	stream_->M0AR = config.memory0_address;
	stream_->M1AR = config.memory1_address;
	stream_->NDTR = config.length;

	std::uint32_t cr = 0U;

	cr |= static_cast<std::uint32_t>( config.channel & 0x7U ) << 25U;
	cr |= encodeDirection( config.direction );
	cr |= encodePriority( config.priority );

	if( config.circular ) {
		cr |= DMA_SxCR_CIRC;
	}

	if( config.peripheral_increment ) {
		cr |= DMA_SxCR_PINC;
	}

	if( config.memory_increment ) {
		cr |= DMA_SxCR_MINC;
	}

	if( config.peripheral_flow_controller ) {
		cr |= DMA_SxCR_PFCTRL;
	}

	if( config.double_buffer ) {
		cr |= DMA_SxCR_DBM;
	}

	cr |= encodeDataSize( config.peripheral_size ) << 11U;
	cr |= encodeDataSize( config.memory_size ) << 13U;

	std::uint32_t fcr = 0U;

	if( config.fifo_enabled ) {
		fcr |= DMA_SxFCR_DMDIS;
	}

	stream_->FCR = fcr;
	stream_->CR = cr;

	clearFlags();

	return DmaStatus::Ok;
}

bool Dma::enable() {
	if( stream_ == nullptr ) {
		return false;
	}

	clearFlags();
	stream_->CR |= DMA_SxCR_EN;

	return true;
}

bool Dma::disable() {
	if( stream_ == nullptr ) {
		return false;
	}

	stream_->CR &= ~DMA_SxCR_EN;

	std::uint32_t timeout = kTimeoutIterations;

	while( ( stream_->CR & DMA_SxCR_EN ) != 0U ) {
		if( timeout-- == 0U ) {
			return false;
		}
	}

	return true;
}

bool Dma::isEnabled() const {
	if( stream_ == nullptr ) {
		return false;
	}

	return ( stream_->CR & DMA_SxCR_EN ) != 0U;
}

void Dma::clearFlags() {
	if( controller_ == nullptr ) {
		return;
	}

	interruptFlagClearRegister() = allFlagsMask( stream_index_ );
}

bool Dma::getFlag( DmaFlag flag ) const {
	const std::uint32_t mask = singleFlagMask( stream_index_, flag );
	return ( readInterruptStatusRegister() & mask ) != 0U;
}

bool Dma::transferComplete() const {
	return getFlag( DmaFlag::TransferComplete );
}

bool Dma::halfTransfer() const {
	return getFlag( DmaFlag::HalfTransfer );
}

bool Dma::transferError() const {
	return getFlag( DmaFlag::TransferError );
}

bool Dma::directModeError() const {
	return getFlag( DmaFlag::DirectModeError );
}

bool Dma::fifoError() const {
	return getFlag( DmaFlag::FifoError );
}

bool Dma::setChannel( std::uint8_t channel ) {
	if( stream_ == nullptr || isEnabled() || !isValidChannel( channel ) ) {
		return false;
	}

	stream_->CR &= ~( 0x7UL << 25U );
	stream_->CR |= static_cast<std::uint32_t>( channel & 0x7U ) << 25U;

	return true;
}

bool Dma::setDirection( DmaDirection direction ) {
	if( stream_ == nullptr || isEnabled() ) {
		return false;
	}

	stream_->CR &= ~DMA_SxCR_DIR;
	stream_->CR |= encodeDirection( direction );

	return true;
}

bool Dma::setCircularMode( bool enable ) {
	if( stream_ == nullptr || isEnabled() ) {
		return false;
	}

	if( enable ) {
		stream_->CR |= DMA_SxCR_CIRC;
	} else {
		stream_->CR &= ~DMA_SxCR_CIRC;
	}

	return true;
}

bool Dma::setPeripheralIncrement( bool enable ) {
	if( stream_ == nullptr || isEnabled() ) {
		return false;
	}

	if( enable ) {
		stream_->CR |= DMA_SxCR_PINC;
	} else {
		stream_->CR &= ~DMA_SxCR_PINC;
	}

	return true;
}

bool Dma::setMemoryIncrement( bool enable ) {
	if( stream_ == nullptr || isEnabled() ) {
		return false;
	}

	if( enable ) {
		stream_->CR |= DMA_SxCR_MINC;
	} else {
		stream_->CR &= ~DMA_SxCR_MINC;
	}

	return true;
}

bool Dma::setPeripheralFlowController( bool enable ) {
	if( stream_ == nullptr || isEnabled() ) {
		return false;
	}

	if( enable ) {
		stream_->CR |= DMA_SxCR_PFCTRL;
	} else {
		stream_->CR &= ~DMA_SxCR_PFCTRL;
	}

	return true;
}

bool Dma::setPeripheralDataSize( DmaDataSize size ) {
	if( stream_ == nullptr || isEnabled() ) {
		return false;
	}

	stream_->CR &= ~DMA_SxCR_PSIZE;
	stream_->CR |= encodeDataSize( size ) << 11U;

	return true;
}

bool Dma::setMemoryDataSize( DmaDataSize size ) {
	if( stream_ == nullptr || isEnabled() ) {
		return false;
	}

	stream_->CR &= ~DMA_SxCR_MSIZE;
	stream_->CR |= encodeDataSize( size ) << 13U;

	return true;
}

bool Dma::setPriority( DmaPriority priority ) {
	if( stream_ == nullptr || isEnabled() ) {
		return false;
	}

	stream_->CR &= ~DMA_SxCR_PL;
	stream_->CR |= encodePriority( priority );

	return true;
}

bool Dma::setLength( std::uint16_t length ) {
	if( stream_ == nullptr || isEnabled() || length == 0U ) {
		return false;
	}

	stream_->NDTR = length;
	return true;
}

bool Dma::setPeripheralAddress( const volatile void *address ) {
	if( stream_ == nullptr || isEnabled() || address == nullptr ) {
		return false;
	}

	stream_->PAR = toAddress( address );
	return true;
}

bool Dma::setMemoryAddress( const void *address ) {
	if( stream_ == nullptr || isEnabled() || address == nullptr ) {
		return false;
	}

	stream_->M0AR = toAddress( address );
	return true;
}

bool Dma::setMemoryAddress( const void *address0, const void *address1 ) {
	if( stream_ == nullptr || isEnabled() || address0 == nullptr || address1 == nullptr ) {
		return false;
	}

	stream_->M0AR = toAddress( address0 );
	stream_->M1AR = toAddress( address1 );
	stream_->CR |= DMA_SxCR_DBM;

	return true;
}

DMA_Stream_TypeDef *Dma::stream() const {
	return stream_;
}

DMA_TypeDef *Dma::controller() const {
	return controller_;
}

void Dma::SetDma( int d ) {
	if( d == 2 ) {
		(void)setController( DmaId::Dma2 );
	} else {
		(void)setController( DmaId::Dma1 );
	}
}

void Dma::SetStream( int s ) {
	if( s < 0 ) {
		return;
	}

	(void)setStream( static_cast<std::uint8_t>( s ) );
}

void Dma::Enabled( bool c ) {
	if( c ) {
		(void)enable();
	} else {
		(void)disable();
	}
}

void Dma::TransDirection( unsigned char d ) {
	if( d == PerToMem ) {
		(void)setDirection( DmaDirection::PeripheralToMemory );
	} else if( d == MemToPer ) {
		(void)setDirection( DmaDirection::MemoryToPeripheral );
	} else {
		(void)setDirection( DmaDirection::MemoryToMemory );
	}
}

void Dma::Circular( bool c ) {
	(void)setCircularMode( c );
}

void Dma::PerInc( bool i ) {
	(void)setPeripheralIncrement( i );
}

void Dma::MemInc( bool i ) {
	(void)setMemoryIncrement( i );
}

void Dma::PerCont( bool c ) {
	(void)setPeripheralFlowController( c );
}

void Dma::PerSize( int s ) {
	if( s == 16 ) {
		(void)setPeripheralDataSize( DmaDataSize::HalfWord );
	} else if( s == 32 ) {
		(void)setPeripheralDataSize( DmaDataSize::Word );
	} else {
		(void)setPeripheralDataSize( DmaDataSize::Byte );
	}
}

void Dma::MemSize( int s ) {
	if( s == 16 ) {
		(void)setMemoryDataSize( DmaDataSize::HalfWord );
	} else if( s == 32 ) {
		(void)setMemoryDataSize( DmaDataSize::Word );
	} else {
		(void)setMemoryDataSize( DmaDataSize::Byte );
	}
}

void Dma::SetChannel( int c ) {
	if( c < 0 ) {
		return;
	}

	(void)setChannel( static_cast<std::uint8_t>( c ) );
}

void Dma::setLon( unsigned short l ) {
	(void)setLength( static_cast<std::uint16_t>( l ) );
}

void Dma::SetPerDir( unsigned int dir ) {
	(void)setPeripheralAddress( reinterpret_cast<const volatile void *>( dir ) );
}

void Dma::SetMemDir( unsigned int dir ) {
	(void)setMemoryAddress( reinterpret_cast<const void *>( dir ) );
}

void Dma::SetMemDir( unsigned int dir0, unsigned int dir1 ) {
	(void)setMemoryAddress(
		reinterpret_cast<const void *>( dir0 ),
		reinterpret_cast<const void *>( dir1 ) );
}

DMA_TypeDef *Dma::controllerFromId( DmaId id ) {
	if( id == DmaId::Dma2 ) {
		return DMA2;
	}

	return DMA1;
}

DMA_Stream_TypeDef *Dma::streamFromIndex( DmaId id, std::uint8_t stream ) {
	if( !isValidStream( stream ) ) {
		return nullptr;
	}

	if( id == DmaId::Dma1 ) {
		switch( stream ) {
			case 0U:
				return DMA1_Stream0;
			case 1U:
				return DMA1_Stream1;
			case 2U:
				return DMA1_Stream2;
			case 3U:
				return DMA1_Stream3;
			case 4U:
				return DMA1_Stream4;
			case 5U:
				return DMA1_Stream5;
			case 6U:
				return DMA1_Stream6;
			case 7U:
				return DMA1_Stream7;
			default:
				return nullptr;
		}
	}

	switch( stream ) {
		case 0U:
			return DMA2_Stream0;
		case 1U:
			return DMA2_Stream1;
		case 2U:
			return DMA2_Stream2;
		case 3U:
			return DMA2_Stream3;
		case 4U:
			return DMA2_Stream4;
		case 5U:
			return DMA2_Stream5;
		case 6U:
			return DMA2_Stream6;
		case 7U:
			return DMA2_Stream7;
		default:
			return nullptr;
	}
}

std::uint32_t Dma::encodeDirection( DmaDirection direction ) {
	switch( direction ) {
		case DmaDirection::MemoryToPeripheral:
			return DMA_SxCR_DIR_0;

		case DmaDirection::MemoryToMemory:
			return DMA_SxCR_DIR_1;

		case DmaDirection::PeripheralToMemory:
		default:
			return 0U;
	}
}

std::uint32_t Dma::encodeDataSize( DmaDataSize size ) {
	return static_cast<std::uint32_t>( size ) & 0x3U;
}

std::uint32_t Dma::encodePriority( DmaPriority priority ) {
	return ( static_cast<std::uint32_t>( priority ) & 0x3U ) << 16U;
}

std::uint32_t Dma::toAddress( const volatile void *address ) {
	return static_cast<std::uint32_t>(
		reinterpret_cast<std::uintptr_t>( address ) );
}

std::uint32_t Dma::toAddress( const void *address ) {
	return static_cast<std::uint32_t>(
		reinterpret_cast<std::uintptr_t>( address ) );
}

bool Dma::isValidStream( std::uint8_t stream ) {
	return stream < kStreamCount;
}

bool Dma::isValidChannel( std::uint8_t channel ) {
	return channel <= kMaxChannel;
}

std::uint32_t Dma::streamFlagOffset( std::uint8_t stream ) {
	const std::uint8_t local_stream = stream % 4U;

	switch( local_stream ) {
		case 0U:
			return 0U;
		case 1U:
			return 6U;
		case 2U:
			return 16U;
		case 3U:
			return 22U;
		default:
			return 0U;
	}
}

std::uint32_t Dma::allFlagsMask( std::uint8_t stream ) {
	static constexpr std::uint32_t kAllStreamFlagsBaseMask = 0x3DU;
	return kAllStreamFlagsBaseMask << streamFlagOffset( stream );
}

std::uint32_t Dma::singleFlagMask( std::uint8_t stream, DmaFlag flag ) {
	return 1UL << ( streamFlagOffset( stream ) + static_cast<std::uint8_t>( flag ) );
}

std::uint32_t Dma::readInterruptStatusRegister() const {
	if( controller_ == nullptr ) {
		return 0U;
	}

	if( stream_index_ < 4U ) {
		return controller_->LISR;
	}

	return controller_->HISR;
}

volatile std::uint32_t &Dma::interruptFlagClearRegister() const {
	if( stream_index_ < 4U ) {
		return controller_->LIFCR;
	}

	return controller_->HIFCR;
}

void Dma::enableClock() const {
	if( id_ == DmaId::Dma2 ) {
		RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
		(void)RCC->AHB1ENR;
	} else {
		RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
		(void)RCC->AHB1ENR;
	}
}

} // namespace first_project