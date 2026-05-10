#include "magnetic_encoder_manager.h"
#include "delay.h"

#include <cstring>

namespace first_project {

MagneticEncoderManager::MagneticEncoderManager(
	MagneticAngleSensor &sensor,
	Flash &flash,
	MagneticSensorType sensor_type,
	std::uint32_t flash_offset )
	: sensor_( sensor ),
	  flash_( flash ),
	  sensor_type_( sensor_type ),
	  flash_offset_( flash_offset ) {}

MagneticEncoderManager::Status MagneticEncoderManager::begin(
	bool load_from_flash ) {

	const SensorStatus sensor_status = sensor_.begin();
	sample_.sensor_status = sensor_status;

	if( sensor_status != SensorStatus::Ok ) {
		return Status::SensorError;
	}

	if( load_from_flash ) {
		const Status load_status = loadFromFlash();

		if( load_status != Status::Ok ) {
			storage_valid_ = false;
		}
	}

	MagneticAngleDiagnostic diagnostic{};
	const SensorStatus diagnostic_status = sensor_.readDiagnostic( diagnostic );

	if( diagnostic_status == SensorStatus::Ok ) {
		sample_.diagnostic = diagnostic;
	}

	return Status::Ok;
}

MagneticEncoderManager::Status MagneticEncoderManager::update() {
	return update( micros() );
}

MagneticEncoderManager::Status MagneticEncoderManager::update(
	std::uint32_t timestamp_us ) {

	std::uint16_t raw_count = 0U;

	const SensorStatus sensor_status = sensor_.readRawAngle( raw_count );
	sample_.sensor_status = sensor_status;

	if( sensor_status != SensorStatus::Ok ) {
		sample_.valid = false;
		return Status::SensorError;
	}

	MagneticAngleDiagnostic diagnostic{};
	const SensorStatus diagnostic_status = sensor_.readDiagnostic( diagnostic );

	if( diagnostic_status == SensorStatus::Ok ) {
		sample_.diagnostic = diagnostic;
	} else {
		sample_.diagnostic.communication_ok = false;
	}

	const bool diagnostic_valid = diagnosticIsValid( sample_.diagnostic );
	const bool calibrated = isCalibrated();

	const float raw_angle = rawCountToRad( raw_count );
	const float calibrated_angle = applyCalibration( raw_angle );
	const float corrected_angle = applyLinearityCorrection( calibrated_angle );

	float unwrapped_angle = corrected_angle;
	float raw_velocity = 0.0F;
	float filtered_velocity = velocity_filter_state_;

	if( runtime_initialized_ ) {
		const std::uint32_t dt_us = timestamp_us - last_timestamp_us_;

		if( dt_us > 0U ) {
			const float dt_s = static_cast<float>( dt_us ) * 1.0e-6F;
			const float delta_angle = wrapPi( corrected_angle - last_angle_rad_ );

			unwrapped_angle = sample_.unwrapped_angle_rad + delta_angle;
			raw_velocity = delta_angle / dt_s;

			filtered_velocity =
				velocity_filter_state_ +
				velocity_alpha_ * ( raw_velocity - velocity_filter_state_ );
		} else {
			unwrapped_angle = sample_.unwrapped_angle_rad;
			raw_velocity = sample_.raw_velocity_rad_s;
			filtered_velocity = sample_.filtered_velocity_rad_s;
		}
	} else {
		runtime_initialized_ = true;
		unwrapped_angle = corrected_angle;
		raw_velocity = 0.0F;
		filtered_velocity = 0.0F;
	}

	last_angle_rad_ = corrected_angle;
	last_timestamp_us_ = timestamp_us;
	velocity_filter_state_ = filtered_velocity;

	sample_.raw_count = raw_count;
	sample_.raw_angle_rad = raw_angle;
	sample_.calibrated_angle_rad = calibrated_angle;
	sample_.corrected_angle_rad = corrected_angle;
	sample_.unwrapped_angle_rad = unwrapped_angle;
	sample_.raw_velocity_rad_s = raw_velocity;
	sample_.filtered_velocity_rad_s = filtered_velocity;
	sample_.timestamp_us = timestamp_us;
	sample_.calibrated = calibrated;

	sample_.valid =
		diagnostic_valid &&
		calibrated &&
		isInsideLimits( corrected_angle );

	if( !diagnostic_valid ) {
		return Status::SensorInvalid;
	}

	if( !calibrated ) {
		return Status::NotCalibrated;
	}

	if( !isInsideLimits( corrected_angle ) ) {
		return Status::OutOfLimits;
	}

	return Status::Ok;
}

MagneticEncoderManager::Status MagneticEncoderManager::calibrateZero(
	float known_joint_angle_rad ) {

	std::uint16_t raw_count = 0U;

	const SensorStatus status = sensor_.readRawAngle( raw_count );

	if( status != SensorStatus::Ok ) {
		return Status::SensorError;
	}

	const float raw_angle = rawCountToRad( raw_count );
	const float direction = normalizeDirection( calibration_.direction );

	calibration_.zero_offset_rad =
		wrapTwoPi( raw_angle - direction * known_joint_angle_rad );

	calibration_.direction = direction;
	calibration_.calibrated = 1U;

	resetRuntimeState();

	dirty_ = true;

	return Status::Ok;
}

MagneticEncoderManager::Status MagneticEncoderManager::setDirection(
	float direction ) {

	calibration_.direction = normalizeDirection( direction );
	resetRuntimeState();

	dirty_ = true;

	return Status::Ok;
}

MagneticEncoderManager::Status MagneticEncoderManager::setLimits(
	float min_angle_rad,
	float max_angle_rad ) {

	if( min_angle_rad >= max_angle_rad ) {
		return Status::InvalidArgument;
	}

	calibration_.min_angle_rad = min_angle_rad;
	calibration_.max_angle_rad = max_angle_rad;

	dirty_ = true;

	return Status::Ok;
}

MagneticEncoderManager::Status MagneticEncoderManager::setVelocityFilterAlpha(
	float alpha ) {

	velocity_alpha_ = clamp( alpha, 0.0F, 1.0F );
	return Status::Ok;
}

MagneticEncoderManager::Status MagneticEncoderManager::setLinearityCorrectionTable(
	const CorrectionPoint *points,
	std::uint8_t count ) {

	if( points == nullptr && count > 0U ) {
		return Status::InvalidArgument;
	}

	if( count > kMaxCorrectionPoints ) {
		return Status::InvalidArgument;
	}

	calibration_.correction_count = count;

	for( std::uint8_t i = 0U; i < count; ++i ) {
		calibration_.correction_points[i] = points[i];
	}

	for( std::uint8_t i = count; i < kMaxCorrectionPoints; ++i ) {
		calibration_.correction_points[i] = CorrectionPoint{};
	}

	resetRuntimeState();

	dirty_ = true;

	return Status::Ok;
}

MagneticEncoderManager::Status MagneticEncoderManager::clearLinearityCorrectionTable() {
	calibration_.correction_count = 0U;

	for( std::uint8_t i = 0U; i < kMaxCorrectionPoints; ++i ) {
		calibration_.correction_points[i] = CorrectionPoint{};
	}

	resetRuntimeState();

	dirty_ = true;

	return Status::Ok;
}

MagneticEncoderManager::Status MagneticEncoderManager::loadFromFlash() {
	StorageBlock block{};

	const FlashStatus flash_status = flash_.read(
		flash_offset_,
		&block,
		sizeof( block ) );

	if( flash_status != FlashStatus::Ok ) {
		storage_valid_ = false;
		return Status::FlashError;
	}

	if( block.magic != kStorageMagic ||
		block.version != kStorageVersion ||
		block.sensor_type != static_cast<std::uint32_t>( sensor_type_ ) ||
		block.counts_per_revolution != sensor_.countsPerRevolution() ) {
		storage_valid_ = false;
		return Status::StorageInvalid;
	}

	const std::uint32_t expected_crc = calculateStorageCrc( block );

	if( expected_crc != block.crc32 ) {
		storage_valid_ = false;
		return Status::StorageInvalid;
	}

	const Status status = applyStorageBlock( block );

	if( status != Status::Ok ) {
		storage_valid_ = false;
		return status;
	}

	storage_sequence_ = block.sequence;
	storage_valid_ = true;
	dirty_ = false;

	resetRuntimeState();

	return Status::Ok;
}

MagneticEncoderManager::Status MagneticEncoderManager::saveToFlash() {
	StorageBlock block = buildStorageBlock();

	if( !flash_.unlock() ) {
		return Status::FlashError;
	}

	const FlashStatus flash_status = flash_.update(
		flash_offset_,
		&block,
		sizeof( block ) );

	(void)flash_.lock();

	if( flash_status != FlashStatus::Ok ) {
		return Status::FlashError;
	}

	storage_sequence_ = block.sequence;
	storage_valid_ = true;
	dirty_ = false;

	return Status::Ok;
}

void MagneticEncoderManager::clearCalibration() {
	calibration_ = Calibration{};
	calibration_.zero_offset_rad = 0.0F;
	calibration_.direction = 1.0F;
	calibration_.min_angle_rad = -kPi;
	calibration_.max_angle_rad = kPi;
	calibration_.calibrated = 0U;
	calibration_.correction_count = 0U;

	storage_valid_ = false;
	dirty_ = true;

	resetRuntimeState();
}

void MagneticEncoderManager::resetRuntimeState() {
	velocity_filter_state_ = 0.0F;
	last_angle_rad_ = 0.0F;
	last_timestamp_us_ = 0U;
	runtime_initialized_ = false;

	sample_ = Sample{};
}

bool MagneticEncoderManager::isStorageValid() const {
	return storage_valid_;
}

bool MagneticEncoderManager::isDirty() const {
	return dirty_;
}

bool MagneticEncoderManager::isCalibrated() const {
	return calibration_.calibrated != 0U;
}

bool MagneticEncoderManager::isValid() const {
	return sample_.valid;
}

const MagneticEncoderManager::Sample &MagneticEncoderManager::sample() const {
	return sample_;
}

const MagneticEncoderManager::Calibration &MagneticEncoderManager::calibration() const {
	return calibration_;
}

const MagneticAngleDiagnostic &MagneticEncoderManager::diagnostic() const {
	return sample_.diagnostic;
}

float MagneticEncoderManager::angleRad() const {
	return sample_.corrected_angle_rad;
}

float MagneticEncoderManager::unwrappedAngleRad() const {
	return sample_.unwrapped_angle_rad;
}

float MagneticEncoderManager::velocityRadS() const {
	return sample_.filtered_velocity_rad_s;
}

float MagneticEncoderManager::rawVelocityRadS() const {
	return sample_.raw_velocity_rad_s;
}

const char *MagneticEncoderManager::sensorName() const {
	return sensor_.name();
}

MagneticSensorType MagneticEncoderManager::sensorType() const {
	return sensor_type_;
}

float MagneticEncoderManager::clamp(
	float value,
	float min_value,
	float max_value ) {

	if( value < min_value ) {
		return min_value;
	}

	if( value > max_value ) {
		return max_value;
	}

	return value;
}

float MagneticEncoderManager::wrapTwoPi( float angle_rad ) {
	while( angle_rad >= kTwoPi ) {
		angle_rad -= kTwoPi;
	}

	while( angle_rad < 0.0F ) {
		angle_rad += kTwoPi;
	}

	return angle_rad;
}

float MagneticEncoderManager::wrapPi( float angle_rad ) {
	while( angle_rad > kPi ) {
		angle_rad -= kTwoPi;
	}

	while( angle_rad < -kPi ) {
		angle_rad += kTwoPi;
	}

	return angle_rad;
}

float MagneticEncoderManager::normalizeDirection( float direction ) {
	if( direction < 0.0F ) {
		return -1.0F;
	}

	return 1.0F;
}

std::uint32_t MagneticEncoderManager::calculateCrc32(
	const std::uint8_t *data,
	std::size_t length ) {

	std::uint32_t crc = 0xFFFFFFFFUL;

	for( std::size_t i = 0U; i < length; ++i ) {
		crc ^= data[i];

		for( std::uint8_t bit = 0U; bit < 8U; ++bit ) {
			if( ( crc & 1UL ) != 0UL ) {
				crc = ( crc >> 1U ) ^ 0xEDB88320UL;
			} else {
				crc >>= 1U;
			}
		}
	}

	return ~crc;
}

std::uint32_t MagneticEncoderManager::calculateStorageCrc(
	const StorageBlock &block ) {

	StorageBlock copy = block;
	copy.crc32 = 0U;

	return calculateCrc32(
		reinterpret_cast<const std::uint8_t *>( &copy ),
		sizeof( copy ) );
}

bool MagneticEncoderManager::diagnosticIsValid(
	const MagneticAngleDiagnostic &diagnostic ) {

	if( !diagnostic.communication_ok ) {
		return false;
	}

	if( !diagnostic.diagnostic_supported ) {
		return true;
	}

	if( !diagnostic.magnet_detected ) {
		return false;
	}

	if( diagnostic.magnet_too_weak ) {
		return false;
	}

	if( diagnostic.magnet_too_strong ) {
		return false;
	}

	return true;
}

float MagneticEncoderManager::rawCountToRad( std::uint16_t raw_count ) const {
	return static_cast<float>( raw_count ) *
		   ( kTwoPi / static_cast<float>( sensor_.countsPerRevolution() ) );
}

float MagneticEncoderManager::applyCalibration(
	float raw_angle_rad ) const {

	const float direction = normalizeDirection( calibration_.direction );
	const float relative = wrapPi( raw_angle_rad - calibration_.zero_offset_rad );

	return direction * relative;
}

float MagneticEncoderManager::applyLinearityCorrection(
	float calibrated_angle_rad ) const {

	const std::uint8_t count = calibration_.correction_count;

	if( count == 0U ) {
		return calibrated_angle_rad;
	}

	if( count == 1U ) {
		return calibrated_angle_rad +
			   calibration_.correction_points[0U].correction_rad;
	}

	const CorrectionPoint *points = calibration_.correction_points;

	if( calibrated_angle_rad <= points[0U].measured_angle_rad ) {
		return calibrated_angle_rad + points[0U].correction_rad;
	}

	for( std::uint8_t i = 0U; i < static_cast<std::uint8_t>( count - 1U ); ++i ) {
		const CorrectionPoint &a = points[i];
		const CorrectionPoint &b = points[i + 1U];

		if( calibrated_angle_rad >= a.measured_angle_rad &&
			calibrated_angle_rad <= b.measured_angle_rad ) {

			const float span = b.measured_angle_rad - a.measured_angle_rad;

			if( span == 0.0F ) {
				return calibrated_angle_rad + a.correction_rad;
			}

			const float t = ( calibrated_angle_rad - a.measured_angle_rad ) / span;

			const float correction =
				a.correction_rad +
				t * ( b.correction_rad - a.correction_rad );

			return calibrated_angle_rad + correction;
		}
	}

	return calibrated_angle_rad + points[count - 1U].correction_rad;
}

bool MagneticEncoderManager::isInsideLimits( float angle_rad ) const {
	return angle_rad >= calibration_.min_angle_rad &&
		   angle_rad <= calibration_.max_angle_rad;
}

MagneticEncoderManager::StorageBlock MagneticEncoderManager::buildStorageBlock() const {
	StorageBlock block{};

	std::memset( &block, 0, sizeof( block ) );

	block.magic = kStorageMagic;
	block.version = kStorageVersion;
	block.sensor_type = static_cast<std::uint32_t>( sensor_type_ );
	block.counts_per_revolution = sensor_.countsPerRevolution();
	block.sequence = storage_sequence_ + 1U;
	block.calibration = calibration_;

	block.crc32 = 0U;
	block.crc32 = calculateStorageCrc( block );

	return block;
}

MagneticEncoderManager::Status MagneticEncoderManager::applyStorageBlock(
	const StorageBlock &block ) {

	const Calibration &cal = block.calibration;

	if( cal.correction_count > kMaxCorrectionPoints ) {
		return Status::StorageInvalid;
	}

	if( cal.direction != 1.0F && cal.direction != -1.0F ) {
		return Status::StorageInvalid;
	}

	if( cal.min_angle_rad >= cal.max_angle_rad ) {
		return Status::StorageInvalid;
	}

	calibration_ = cal;

	return Status::Ok;
}

} // namespace first_project