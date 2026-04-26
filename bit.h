#pragma once

namespace first_project {

class Bit {

	protected:
		bool e_{ false };

	public:
		// Default constructor.
		Bit() noexcept = default;

		// State constructor.
		explicit Bit( bool e ) noexcept : e_{ e } {}

		// Logic state writer.
		void setBit( bool e ) noexcept {
			e_ = e;
		}

		// Logic state reader.
		[[nodiscard]] bool getBit() const noexcept {
			return e_;
		}

		// Assignment from bool.
		Bit &operator=( bool e ) noexcept {
			setBit( e );
			return *this;
		}

		// Assignment from another Bit.
		Bit &operator=( const Bit &b ) noexcept {
			setBit( b.getBit() );
			return *this;
		}

		// Boolean conversion.
		[[nodiscard]] explicit operator bool() const noexcept {
			return getBit();
		}

		// NOT operation.
		[[nodiscard]] bool operator!() const noexcept {
			return !getBit();
		}

		// AND operation.
		[[nodiscard]] bool operator&( bool e ) const noexcept {
			return getBit() & e;
		}

		// OR operation.
		[[nodiscard]] bool operator|( bool e ) const noexcept {
			return getBit() | e;
		}

		// XOR operation.
		[[nodiscard]] bool operator^( bool e ) const noexcept {
			return getBit() ^ e;
		}

		// Equal AND operation.
		Bit &operator&=( bool e ) noexcept {
			setBit( getBit() & e );
			return *this;
		}

		// Equal OR operation.
		Bit &operator|=( bool e ) noexcept {
			setBit( getBit() | e );
			return *this;
		}

		// Equal XOR operation.
		Bit &operator^=( bool e ) noexcept {
			setBit( getBit() ^ e );
			return *this;
		}
};

} // namespace first_project