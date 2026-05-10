#pragma once

#include <cstddef>
#include <cstdint>

namespace first_project {

enum class FlashStatus : std::uint8_t {
	Ok,
	Busy,
	Locked,
	InvalidArgument,
	OutOfRange,
	ProgramError,
	EraseError,
	VerifyError,
	Timeout
};

enum class FlashSector : std::uint8_t {
	Sector0 = 0U,
	Sector1 = 1U,
	Sector2 = 2U,
	Sector3 = 3U,
	Sector4 = 4U,
	Sector5 = 5U,
	Sector6 = 6U,
	Sector7 = 7U
};

class Flash {
	public:
		static constexpr std::uint32_t kFlashBaseAddress = 0x08000000UL;
		static constexpr std::uint32_t kFlashEndAddress = 0x08080000UL;
		static constexpr std::uint32_t kTimeoutIterations = 1000000UL;

		static constexpr FlashSector kDefaultStorageSector = FlashSector::Sector7;

		Flash();
		explicit Flash( FlashSector storage_sector );

		bool unlock();
		bool lock();
		bool isLocked() const;

		FlashStatus erase();
		FlashStatus erase( FlashSector sector );

		FlashStatus read(
			std::uint32_t offset,
			void *data,
			std::size_t size ) const;

		FlashStatus write(
			std::uint32_t offset,
			const void *data,
			std::size_t size );

		FlashStatus update(
			std::uint32_t offset,
			const void *data,
			std::size_t size );

		std::uint32_t storageBaseAddress() const;
		std::uint32_t storageSize() const;
		FlashSector storageSector() const;

		static std::uint32_t sectorBaseAddress( FlashSector sector );
		static std::uint32_t sectorSize( FlashSector sector );
		static bool isValidSectorNumber( int sector );

		// Legacy compatibility API.
		void Unlock();
		void block();
		void erase( int s );
		void read( int bytes, void *dat );
		void record( int bytes, void *dat );

	private:
		FlashSector storage_sector_ = kDefaultStorageSector;

		static FlashSector sectorFromNumber( int sector );

		static FlashStatus waitWhileBusy();
		static void clearErrorFlags();

		static bool containsRange(
			std::uint32_t base,
			std::uint32_t capacity,
			std::uint32_t offset,
			std::size_t size );

		static bool canProgramWithoutErase(
			std::uint32_t address,
			const std::uint8_t *data,
			std::size_t size );

		static FlashStatus programBytes(
			std::uint32_t address,
			const std::uint8_t *data,
			std::size_t size );

		static bool verifyBytes(
			std::uint32_t address,
			const std::uint8_t *data,
			std::size_t size );
};

} // namespace first_project