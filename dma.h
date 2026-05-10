#pragma once

#include <cstdint>
#include <cstddef>

#include "stm32f401xe.h"

namespace first_project {

enum : std::uint8_t {
	PerToMem = 0U,
	MemToPer = 1U,
	MemToMem = 2U
};

enum class DmaId : std::uint8_t {
	Dma1 = 1U,
	Dma2 = 2U
};

enum class DmaDirection : std::uint8_t {
	PeripheralToMemory = 0U,
	MemoryToPeripheral = 1U,
	MemoryToMemory = 2U
};

enum class DmaDataSize : std::uint8_t {
	Byte = 0U,
	HalfWord = 1U,
	Word = 2U
};

enum class DmaPriority : std::uint8_t {
	Low = 0U,
	Medium = 1U,
	High = 2U,
	VeryHigh = 3U
};

enum class DmaStatus : std::uint8_t {
	Ok,
	InvalidArgument,
	InvalidStream,
	Busy,
	Timeout
};

enum class DmaFlag : std::uint8_t {
	FifoError = 0U,
	DirectModeError = 2U,
	TransferError = 3U,
	HalfTransfer = 4U,
	TransferComplete = 5U
};

struct DmaConfig {
		std::uint8_t channel = 0U;
		DmaDirection direction = DmaDirection::PeripheralToMemory;

		std::uint32_t peripheral_address = 0U;
		std::uint32_t memory0_address = 0U;
		std::uint32_t memory1_address = 0U;

		std::uint16_t length = 0U;

		bool circular = false;
		bool peripheral_increment = false;
		bool memory_increment = true;
		bool peripheral_flow_controller = false;
		bool double_buffer = false;

		DmaDataSize peripheral_size = DmaDataSize::Byte;
		DmaDataSize memory_size = DmaDataSize::Byte;
		DmaPriority priority = DmaPriority::Low;

		bool fifo_enabled = false;
};

class Dma {
	public:
		static constexpr std::uint32_t kTimeoutIterations = 100000U;
		static constexpr std::uint8_t kStreamCount = 8U;
		static constexpr std::uint8_t kMaxChannel = 7U;

		Dma() = default;
		Dma( DmaId id, std::uint8_t stream );

		bool setController( DmaId id );
		bool setStream( std::uint8_t stream );

		DmaStatus configure( const DmaConfig &config );

		bool enable();
		bool disable();
		bool isEnabled() const;

		void clearFlags();
		bool getFlag( DmaFlag flag ) const;

		bool transferComplete() const;
		bool halfTransfer() const;
		bool transferError() const;
		bool directModeError() const;
		bool fifoError() const;

		bool setChannel( std::uint8_t channel );
		bool setDirection( DmaDirection direction );
		bool setCircularMode( bool enable );
		bool setPeripheralIncrement( bool enable );
		bool setMemoryIncrement( bool enable );
		bool setPeripheralFlowController( bool enable );
		bool setPeripheralDataSize( DmaDataSize size );
		bool setMemoryDataSize( DmaDataSize size );
		bool setPriority( DmaPriority priority );

		bool setLength( std::uint16_t length );
		bool setPeripheralAddress( const volatile void *address );
		bool setMemoryAddress( const void *address );
		bool setMemoryAddress( const void *address0, const void *address1 );

		DMA_Stream_TypeDef *stream() const;
		DMA_TypeDef *controller() const;

		// Legacy compatibility API.
		void SetDma( int d = 1 );
		void SetStream( int s = 0 );
		void Enabled( bool c = false );
		void TransDirection( unsigned char d = MemToMem );
		void Circular( bool c = false );
		void PerInc( bool i = false );
		void MemInc( bool i = false );
		void PerCont( bool c = false );
		void PerSize( int s = 8 );
		void MemSize( int s = 8 );
		void SetChannel( int c = 0 );
		void setLon( unsigned short l );
		void SetPerDir( unsigned int dir );
		void SetMemDir( unsigned int dir );
		void SetMemDir( unsigned int dir0, unsigned int dir1 );

	private:
		DmaId id_ = DmaId::Dma1;
		std::uint8_t stream_index_ = 0U;

		DMA_TypeDef *controller_ = DMA1;
		DMA_Stream_TypeDef *stream_ = DMA1_Stream0;

		static DMA_TypeDef *controllerFromId( DmaId id );
		static DMA_Stream_TypeDef *streamFromIndex( DmaId id, std::uint8_t stream );

		static std::uint32_t encodeDirection( DmaDirection direction );
		static std::uint32_t encodeDataSize( DmaDataSize size );
		static std::uint32_t encodePriority( DmaPriority priority );

		static std::uint32_t toAddress( const volatile void *address );
		static std::uint32_t toAddress( const void *address );

		static bool isValidStream( std::uint8_t stream );
		static bool isValidChannel( std::uint8_t channel );

		static std::uint32_t streamFlagOffset( std::uint8_t stream );
		static std::uint32_t allFlagsMask( std::uint8_t stream );
		static std::uint32_t singleFlagMask( std::uint8_t stream, DmaFlag flag );

		std::uint32_t readInterruptStatusRegister() const;
		volatile std::uint32_t &interruptFlagClearRegister() const;

		void enableClock() const;
};

} // namespace first_project