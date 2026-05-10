#include "flash.h"
#include "stm32f401xe.h"

namespace first_project {

Flash::Flash() = default;

Flash::Flash( FlashSector storage_sector )
	: storage_sector_( storage_sector ) {}

bool Flash::unlock() {
	if( !isLocked() ) {
		return true;
	}

	FLASH->KEYR = 0x45670123UL;
	FLASH->KEYR = 0xCDEF89ABUL;

	return !isLocked();
}

bool Flash::lock() {
	FLASH->CR |= FLASH_CR_LOCK;
	return isLocked();
}

bool Flash::isLocked() const {
	return ( FLASH->CR & FLASH_CR_LOCK ) != 0U;
}

FlashStatus Flash::erase() {
	return erase( storage_sector_ );
}

FlashStatus Flash::erase( FlashSector sector ) {
	if( isLocked() ) {
		return FlashStatus::Locked;
	}

	FlashStatus status = waitWhileBusy();

	if( status != FlashStatus::Ok ) {
		return status;
	}

	clearErrorFlags();

	FLASH->CR &= ~FLASH_CR_PG;
	FLASH->CR &= ~( 0xFUL << 3U );
	FLASH->CR |= FLASH_CR_SER;
	FLASH->CR |= static_cast<std::uint32_t>( sector ) << 3U;
	FLASH->CR |= FLASH_CR_STRT;

	status = waitWhileBusy();

	FLASH->CR &= ~FLASH_CR_SER;
	FLASH->CR &= ~( 0xFUL << 3U );

	if( status != FlashStatus::Ok ) {
		return status;
	}

	if( ( FLASH->SR & (
#ifdef FLASH_SR_WRPERR
						  FLASH_SR_WRPERR |
#endif
#ifdef FLASH_SR_PGAERR
						  FLASH_SR_PGAERR |
#endif
#ifdef FLASH_SR_PGPERR
						  FLASH_SR_PGPERR |
#endif
#ifdef FLASH_SR_PGSERR
						  FLASH_SR_PGSERR |
#endif
						  0U ) ) != 0U ) {
		return FlashStatus::EraseError;
	}

	return FlashStatus::Ok;
}

FlashStatus Flash::read(
	std::uint32_t offset,
	void *data,
	std::size_t size ) const {

	if( data == nullptr && size > 0U ) {
		return FlashStatus::InvalidArgument;
	}

	const std::uint32_t base = storageBaseAddress();
	const std::uint32_t capacity = storageSize();

	if( !containsRange( base, capacity, offset, size ) ) {
		return FlashStatus::OutOfRange;
	}

	auto *destination = static_cast<std::uint8_t *>( data );
	const auto *source = reinterpret_cast<const std::uint8_t *>( base + offset );

	for( std::size_t i = 0U; i < size; ++i ) {
		destination[i] = source[i];
	}

	return FlashStatus::Ok;
}

FlashStatus Flash::write(
	std::uint32_t offset,
	const void *data,
	std::size_t size ) {

	if( data == nullptr && size > 0U ) {
		return FlashStatus::InvalidArgument;
	}

	if( isLocked() ) {
		return FlashStatus::Locked;
	}

	const std::uint32_t base = storageBaseAddress();
	const std::uint32_t capacity = storageSize();

	if( !containsRange( base, capacity, offset, size ) ) {
		return FlashStatus::OutOfRange;
	}

	const std::uint32_t address = base + offset;
	const auto *source = static_cast<const std::uint8_t *>( data );

	if( !canProgramWithoutErase( address, source, size ) ) {
		return FlashStatus::ProgramError;
	}

	FlashStatus status = programBytes( address, source, size );

	if( status != FlashStatus::Ok ) {
		return status;
	}

	if( !verifyBytes( address, source, size ) ) {
		return FlashStatus::VerifyError;
	}

	return FlashStatus::Ok;
}

FlashStatus Flash::update(
	std::uint32_t offset,
	const void *data,
	std::size_t size ) {

	if( data == nullptr && size > 0U ) {
		return FlashStatus::InvalidArgument;
	}

	const std::uint32_t base = storageBaseAddress();
	const std::uint32_t capacity = storageSize();

	if( !containsRange( base, capacity, offset, size ) ) {
		return FlashStatus::OutOfRange;
	}

	FlashStatus status = erase();

	if( status != FlashStatus::Ok ) {
		return status;
	}

	return write( offset, data, size );
}

std::uint32_t Flash::storageBaseAddress() const {
	return sectorBaseAddress( storage_sector_ );
}

std::uint32_t Flash::storageSize() const {
	return sectorSize( storage_sector_ );
}

FlashSector Flash::storageSector() const {
	return storage_sector_;
}

std::uint32_t Flash::sectorBaseAddress( FlashSector sector ) {
	switch( sector ) {
		case FlashSector::Sector0:
			return 0x08000000UL;
		case FlashSector::Sector1:
			return 0x08004000UL;
		case FlashSector::Sector2:
			return 0x08008000UL;
		case FlashSector::Sector3:
			return 0x0800C000UL;
		case FlashSector::Sector4:
			return 0x08010000UL;
		case FlashSector::Sector5:
			return 0x08020000UL;
		case FlashSector::Sector6:
			return 0x08040000UL;
		case FlashSector::Sector7:
			return 0x08060000UL;
		default:
			return 0x08060000UL;
	}
}

std::uint32_t Flash::sectorSize( FlashSector sector ) {
	switch( sector ) {
		case FlashSector::Sector0:
		case FlashSector::Sector1:
		case FlashSector::Sector2:
		case FlashSector::Sector3:
			return 16UL * 1024UL;

		case FlashSector::Sector4:
			return 64UL * 1024UL;

		case FlashSector::Sector5:
		case FlashSector::Sector6:
		case FlashSector::Sector7:
			return 128UL * 1024UL;

		default:
			return 0U;
	}
}

bool Flash::isValidSectorNumber( int sector ) {
	return sector >= 0 && sector <= 7;
}

void Flash::Unlock() {
	(void)unlock();
}

void Flash::block() {
	(void)lock();
}

void Flash::erase( int s ) {
	if( !isValidSectorNumber( s ) ) {
		return;
	}

	(void)erase( sectorFromNumber( s ) );
}

void Flash::read( int bytes, void *dat ) {
	if( bytes <= 0 ) {
		return;
	}

	(void)read(
		0U,
		dat,
		static_cast<std::size_t>( bytes ) );
}

void Flash::record( int bytes, void *dat ) {
	if( bytes <= 0 ) {
		return;
	}

	if( unlock() ) {
		(void)update(
			0U,
			dat,
			static_cast<std::size_t>( bytes ) );

		(void)lock();
	}
}

FlashSector Flash::sectorFromNumber( int sector ) {
	switch( sector ) {
		case 0:
			return FlashSector::Sector0;
		case 1:
			return FlashSector::Sector1;
		case 2:
			return FlashSector::Sector2;
		case 3:
			return FlashSector::Sector3;
		case 4:
			return FlashSector::Sector4;
		case 5:
			return FlashSector::Sector5;
		case 6:
			return FlashSector::Sector6;
		case 7:
			return FlashSector::Sector7;
		default:
			return kDefaultStorageSector;
	}
}

FlashStatus Flash::waitWhileBusy() {
	std::uint32_t timeout = kTimeoutIterations;

	while( ( FLASH->SR & FLASH_SR_BSY ) != 0U ) {
		if( timeout-- == 0U ) {
			return FlashStatus::Timeout;
		}
	}

	return FlashStatus::Ok;
}

void Flash::clearErrorFlags() {
	std::uint32_t flags = 0U;

#ifdef FLASH_SR_EOP
	flags |= FLASH_SR_EOP;
#endif

#ifdef FLASH_SR_OPERR
	flags |= FLASH_SR_OPERR;
#endif

#ifdef FLASH_SR_WRPERR
	flags |= FLASH_SR_WRPERR;
#endif

#ifdef FLASH_SR_PGAERR
	flags |= FLASH_SR_PGAERR;
#endif

#ifdef FLASH_SR_PGPERR
	flags |= FLASH_SR_PGPERR;
#endif

#ifdef FLASH_SR_PGSERR
	flags |= FLASH_SR_PGSERR;
#endif

#ifdef FLASH_SR_RDERR
	flags |= FLASH_SR_RDERR;
#endif

	if( flags != 0U ) {
		FLASH->SR = flags;
	}
}

bool Flash::containsRange(
	std::uint32_t base,
	std::uint32_t capacity,
	std::uint32_t offset,
	std::size_t size ) {

	(void)base;

	if( offset > capacity ) {
		return false;
	}

	if( size > static_cast<std::size_t>( capacity - offset ) ) {
		return false;
	}

	return true;
}

bool Flash::canProgramWithoutErase(
	std::uint32_t address,
	const std::uint8_t *data,
	std::size_t size ) {

	const auto *destination = reinterpret_cast<const std::uint8_t *>( address );

	for( std::size_t i = 0U; i < size; ++i ) {
		if( ( destination[i] & data[i] ) != data[i] ) {
			return false;
		}
	}

	return true;
}

FlashStatus Flash::programBytes(
	std::uint32_t address,
	const std::uint8_t *data,
	std::size_t size ) {

	FlashStatus status = waitWhileBusy();

	if( status != FlashStatus::Ok ) {
		return status;
	}

	clearErrorFlags();

	FLASH->CR &= ~( 0x3UL << 8U );
	FLASH->CR |= FLASH_CR_PG;

	for( std::size_t i = 0U; i < size; ++i ) {
		*reinterpret_cast<volatile std::uint8_t *>( address + i ) = data[i];

		status = waitWhileBusy();

		if( status != FlashStatus::Ok ) {
			FLASH->CR &= ~FLASH_CR_PG;
			return status;
		}

		if( *reinterpret_cast<volatile std::uint8_t *>( address + i ) != data[i] ) {
			FLASH->CR &= ~FLASH_CR_PG;
			return FlashStatus::VerifyError;
		}
	}

	FLASH->CR &= ~FLASH_CR_PG;

	if( ( FLASH->SR & (
#ifdef FLASH_SR_WRPERR
						  FLASH_SR_WRPERR |
#endif
#ifdef FLASH_SR_PGAERR
						  FLASH_SR_PGAERR |
#endif
#ifdef FLASH_SR_PGPERR
						  FLASH_SR_PGPERR |
#endif
#ifdef FLASH_SR_PGSERR
						  FLASH_SR_PGSERR |
#endif
						  0U ) ) != 0U ) {
		return FlashStatus::ProgramError;
	}

	return FlashStatus::Ok;
}

bool Flash::verifyBytes(
	std::uint32_t address,
	const std::uint8_t *data,
	std::size_t size ) {

	const auto *destination = reinterpret_cast<const std::uint8_t *>( address );

	for( std::size_t i = 0U; i < size; ++i ) {
		if( destination[i] != data[i] ) {
			return false;
		}
	}

	return true;
}

} // namespace first_project