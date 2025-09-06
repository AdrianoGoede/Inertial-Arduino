#include <cmath>
#include "MatrixMath.h"
#include "Inertial.h"

bool InertialPositionTracker::initialize() {
  for (size_t i = 0; i < STATE_COVARIANCE_SIZE; i++)
    for (size_t j = 0; j < STATE_COVARIANCE_SIZE; j++) {
      this->covariance_matrix[i][j] = (i == j ? COVARIANCE_INITIAL_VALUE : 0.0);
      this->process_noise_matrix[i][j] = (i == j ? PROCESS_NOISE_INITIAL_VALUE : 0.0);
    }

  for (size_t i = 0; i < NOISE_MATRIX_SIZE; i++)
    for (size_t j = 0; j < NOISE_MATRIX_SIZE; j++)
      this->measurement_noise_matrix[i][j] = (i == j ? MEASUREMENT_NOISE_INITIAL_VALUE : 0.0);
  
  if (!this->initializeMPU()) return false;
  this->calibrateMPU();
  
  this->initializeMagnetometer();

  return true;
}

bool InertialPositionTracker::initializeMPU() {
  if (!this->mpu.begin()) return false;
  this->mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  this->mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  this->mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  this->mpu.setI2CBypass(true);
  delay(100);
  return true;
}

void InertialPositionTracker::calibrateMPU() {
  mtx_type ax, ay, az, gx, gy, gz;
  mtx_type sum_ax = 0, sum_ay = 0, sum_az = 0, sum_gx = 0, sum_gy = 0, sum_gz = 0;
  
  for (uint32_t i = 0; i < MPU_CALIBRATION_SAMPLES; i++) {
    this->getMpuReading(&ax, &ay, &az, &gx, &gy, &gz);
    sum_ax += ax;
    sum_ay += ay;
    sum_az += az;
    sum_gx += gx;
    sum_gy += gy;
    sum_gz += gz;
    delay(10);
  }

  this->state_vector[10] = (sum_gx / MPU_CALIBRATION_SAMPLES);
  this->state_vector[11] = (sum_gy / MPU_CALIBRATION_SAMPLES);
  this->state_vector[12] = (sum_gz / MPU_CALIBRATION_SAMPLES);

  this->state_vector[13] = (sum_ax / MPU_CALIBRATION_SAMPLES);
  this->state_vector[14] = (sum_ay / MPU_CALIBRATION_SAMPLES);
  this->state_vector[15] = (sum_az / MPU_CALIBRATION_SAMPLES);
}

void InertialPositionTracker::initializeMagnetometer() {
  this->magnetometer.init();
}

void InertialPositionTracker::updateState(float delta_time) {
  mtx_type ax, ay, az, gx, gy, gz, mx, my, mz;
  this->getMpuReading(&ax, &ay, &az, &gx, &gy, &gz);
  this->getMagnetometerReading(&mx, &my, &mz);
  

}

void InertialPositionTracker::predictState(float delta_time, const mtx_type accelerometer[3], const mtx_type gyroscope[3]) {
  mtx_type angular_velocity[4];
  this->quaternionFromAngularVelocity(delta_time, gyroscope, angular_velocity);

  mtx_type predicted_angular_state[4];
  this->quaternionMultiply(&this->state_vector[6], angular_velocity, predicted_angular_state);

  this->normalizeQuaternion(predicted_angular_state);
  for (uint8_t i = 0; i < 4; i++)
    this->state_vector[6 + i] = predicted_angular_state[i];
  
  const mtx_type accelerometer_corrected[3] {
    (accelerometer[0] - this->state_vector[13]),
    (accelerometer[1] - this->state_vector[14]),
    (accelerometer[2] - this->state_vector[15])
  };
  mtx_type global_acceleration[3]{ accelerometer[0], accelerometer[1], accelerometer[2] };
  this->rotateVector(accelerometer_corrected, predicted_angular_state, global_acceleration);
  global_acceleration[2] -= GRAVITY_ACCELERATION;

  for (uint8_t i = 0; i < 3; i++) {
    this->state_vector[i + 3] += (global_acceleration[i] * delta_time);
    this->state_vector[i] += (this->state_vector[i + 3] * delta_time);
  }
}

void InertialPositionTracker::updateState(float delta_time, const mtx_type accelerometer[3], const mtx_type gyroscope[3]) {
  
}

void InertialPositionTracker::quaternionFromAngularVelocity(float delta_time, const mtx_type gyroscope[3], mtx_type quaternion[4]) {
  const mtx_type gyroscope_corrected[3] {
    ((gyroscope[0] - this->state_vector[10]) * delta_time / 2.0),
    ((gyroscope[1] - this->state_vector[11]) * delta_time / 2.0),
    ((gyroscope[2] - this->state_vector[12]) * delta_time / 2.0)
  };

  float magnitude = sqrt(
    (gyroscope_corrected[0] * gyroscope_corrected[0]) +
    (gyroscope_corrected[1] * gyroscope_corrected[1]) +
    (gyroscope_corrected[2] * gyroscope_corrected[2])
  );
  if (magnitude < MINIMUM_MAGNITUDE_THRESHOLD) {
    quaternion[0] = 1.0;
    quaternion[1] = quaternion[2] = quaternion[3] = 0.0;
    return;
  }

  quaternion[0] = cos(magnitude);
  quaternion[1] = ((gyroscope_corrected[0] / magnitude) * sin(magnitude));
  quaternion[2] = ((gyroscope_corrected[1] / magnitude) * sin(magnitude));
  quaternion[3] = ((gyroscope_corrected[2] / magnitude) * sin(magnitude));
}

void InertialPositionTracker::quaternionMultiply(const mtx_type first[4], const mtx_type second[4], mtx_type output[4]) {
  output[0] = ((first[0] * second[0]) - (first[1] * second[1]) - (first[2] * second[2]) - (first[3] * second[3]));
  output[1] = ((first[0] * second[1]) + (first[1] * second[0]) + (first[2] * second[3]) - (first[3] * second[2]));
  output[2] = ((first[0] * second[2]) - (first[1] * second[3]) + (first[2] * second[0]) + (first[3] * second[1]));
  output[3] = ((first[0] * second[3]) + (first[1] * second[2]) - (first[2] * second[1]) + (first[3] * second[0]));
}

void InertialPositionTracker::rotateVector(const mtx_type vector[3], const mtx_type quaternion[4], mtx_type output[3]) {
  mtx_type conjugate[4];
  conjugate[0] =  quaternion[0];
  conjugate[1] = -quaternion[1];
  conjugate[2] = -quaternion[2];
  conjugate[3] = -quaternion[3];

  mtx_type vector_quaternion[4] = {0.0, vector[0], vector[1], vector[2]};

  mtx_type temp_quaternion[4];
  this->quaternionMultiply(quaternion, vector_quaternion, temp_quaternion);
  this->quaternionMultiply(temp_quaternion, conjugate, temp_quaternion);

  output[0] = temp_quaternion[1];
  output[1] = temp_quaternion[2];
  output[2] = temp_quaternion[3];
}

void InertialPositionTracker::normalizeQuaternion(mtx_type quaternion[4]) {
  mtx_type magnitude = sqrt(
    (quaternion[0] * quaternion[0]) +
    (quaternion[1] * quaternion[1]) +
    (quaternion[2] * quaternion[2]) +
    (quaternion[3] * quaternion[3]) 
  );

  if (magnitude > MINIMUM_MAGNITUDE_THRESHOLD) {
    quaternion[0] /= magnitude;
    quaternion[1] /= magnitude;
    quaternion[2] /= magnitude;
    quaternion[3] /= magnitude;
  }
}

void InertialPositionTracker::getMpuReading(mtx_type* accel_x, mtx_type* accel_y, mtx_type* accel_z, mtx_type* gyro_x, mtx_type* gyro_y, mtx_type* gyro_z) {
  sensors_event_t accelerator, gyroscope;
  mpu.getEvent(&accelerator, &gyroscope, nullptr);

  *accel_x = (mtx_type)accelerator.acceleration.x;
  *accel_y = (mtx_type)accelerator.acceleration.y;
  *accel_z = (mtx_type)accelerator.acceleration.z;

  *gyro_x = (mtx_type)gyroscope.gyro.x;
  *gyro_y = (mtx_type)gyroscope.gyro.y;
  *gyro_z = (mtx_type)gyroscope.gyro.z;
}

void InertialPositionTracker::getMagnetometerReading(mtx_type* mag_x, mtx_type* mag_y, mtx_type* mag_z) {
  this->magnetometer.read();
  *mag_x = (mtx_type)this->magnetometer.getX();
  *mag_y = (mtx_type)this->magnetometer.getY();
  *mag_z = (mtx_type)this->magnetometer.getZ();
}

inertial_tracker_state_t InertialPositionTracker::getCurrentState() {
  return inertial_tracker_state_t {
    // this->accel_x, this->accel_y, this->accel_z,
    // this->gyro_x, this->gyro_y, this->gyro_z,
    // this->mag_x, this->mag_y, this->mag_z,
    // this->filter.getRoll(), this->filter.getPitch(), this->filter.getYaw(),
    // this->vel_x, this->vel_y, this->vel_z,
    // this->pos_x, this->pos_y, this->pos_z
  };
}