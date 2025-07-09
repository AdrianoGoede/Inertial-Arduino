#include "Inertial.h"

bool InertialPositionTracker::initialize() {
  if (!this->initializeMPU()) return false;
  this->calibrateMPU();
  this->filter.begin(FILTER_SAMPLING_RATE);
  return true;
}

float InertialPositionTracker::getAccelerometerX() { return this->accel_x; }

float InertialPositionTracker::getAccelerometerY() { return this->accel_y; }

float InertialPositionTracker::getAccelerometerZ() { return this->accel_z; }

float InertialPositionTracker::getGyroscopeX() { return this->gyro_x; }

float InertialPositionTracker::getGyroscopeY() { return this->gyro_y; }

float InertialPositionTracker::getGyroscopeZ() { return this->gyro_z; }

float InertialPositionTracker::getRoll() { return this->filter.getRoll(); }

float InertialPositionTracker::getPitch() { return this->filter.getPitch(); }

float InertialPositionTracker::getYaw() { return this->filter.getYaw(); }

float InertialPositionTracker::getVelocityX() { return this->vel_x; }

float InertialPositionTracker::getVelocityY() { return this->vel_y; }

float InertialPositionTracker::getVelocityZ() { return this->vel_z; }

float InertialPositionTracker::getPositionX() { return this->pos_x; }

float InertialPositionTracker::getPositionY() { return this->pos_y; }

float InertialPositionTracker::getPositionZ() { return this->pos_z; }

bool InertialPositionTracker::initializeMPU() {
  if (!this->mpu.begin()) return false;
  this->mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  this->mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  this->mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  return true;
}

void InertialPositionTracker::calibrateMPU() {
  double ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  
  for (uint32_t i = 0; i < MPU_CALIBRATION_SAMPLES; i++) {
    this->getMpuReading();
    ax += (double)this->accel_x;
    ay += (double)this->accel_y;
    az += (double)this->accel_z;
    gx += (double)this->gyro_x;
    gy += (double)this->gyro_y;
    gz += (double)this->gyro_z;
    delay((int)FILTER_SAMPLING_RATE);
  }

  this->accel_x_bias = (float)(ax / MPU_CALIBRATION_SAMPLES);
  this->accel_y_bias = (float)(ay / MPU_CALIBRATION_SAMPLES);
  this->accel_z_bias = (float)(az / MPU_CALIBRATION_SAMPLES);
  this->gyro_x_bias = (float)(gx / MPU_CALIBRATION_SAMPLES);
  this->gyro_y_bias = (float)(gy / MPU_CALIBRATION_SAMPLES);
  this->gyro_z_bias = (float)(gz / MPU_CALIBRATION_SAMPLES);
}

void InertialPositionTracker::updateState(float delta_time) {
  this->getMpuReading();
  this->filter.updateIMU(this->gyro_x, this->gyro_y, this->gyro_z, this->accel_x, this->accel_y, this->accel_z);

  float sin_roll  = sin(this->filter.getRoll()),  cos_roll  = cos(this->filter.getRoll());
  float sin_pitch = sin(this->filter.getPitch()), cos_pitch = cos(this->filter.getPitch());
  float sin_yaw   = sin(this->filter.getYaw()),   cos_yaw   = cos(this->filter.getYaw());

  float ax_world = this->accel_x * (cos_yaw * cos_pitch) +
                   this->accel_y * (cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll) +
                   this->accel_z * (cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll);
  ax_world = (abs(ax_world) >= ZUPT_THRESHOLD ? ax_world : 0.0);

  float ay_world = this->accel_x * (sin_yaw * cos_pitch) +
                   this->accel_y * (sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll) +
                   this->accel_z * (sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll);
  ay_world = (abs(ay_world) >= ZUPT_THRESHOLD ? ay_world : 0.0);

  float az_world = this->accel_x * (-sin_pitch) +
                   this->accel_y * (cos_pitch * sin_roll) +
                   this->accel_z * (cos_pitch * cos_roll);
  az_world = (abs(az_world) >= ZUPT_THRESHOLD ? az_world : 0.0);

  this->accel_x_filtered = (ACCELEROMETER_FILTER_ALPHA * this->accel_x_filtered + (1.0 - ACCELEROMETER_FILTER_ALPHA) * ax_world);
  this->accel_y_filtered = (ACCELEROMETER_FILTER_ALPHA * this->accel_y_filtered + (1.0 - ACCELEROMETER_FILTER_ALPHA) * ay_world);
  this->accel_z_filtered = (ACCELEROMETER_FILTER_ALPHA * this->accel_z_filtered + (1.0 - ACCELEROMETER_FILTER_ALPHA) * az_world);

  this->vel_x += (this->accel_x_filtered * 9.81 * delta_time);
  this->pos_x += (this->vel_x * delta_time);

  this->vel_y += (this->accel_y_filtered * 9.81 * delta_time);
  this->pos_y += (this->vel_y * delta_time);

  this->vel_z += (this->accel_z_filtered * 9.81 * delta_time);
  this->pos_z += (this->vel_z * delta_time);
}

void InertialPositionTracker::getMpuReading() {
  sensors_event_t accelerator, gyroscope;
  mpu.getEvent(&accelerator, &gyroscope, nullptr);

  this->accel_x = (accelerator.acceleration.x - this->accel_x_bias);
  this->accel_x = (abs(this->accel_x) >= ZUPT_THRESHOLD ? this->accel_x : 0.0);
  this->accel_y = (accelerator.acceleration.y - this->accel_y_bias);
  this->accel_y = (abs(this->accel_y) >= ZUPT_THRESHOLD ? this->accel_y : 0.0);
  this->accel_z = (accelerator.acceleration.z - this->accel_z_bias);
  this->accel_z = (abs(this->accel_z) >= ZUPT_THRESHOLD ? this->accel_z : 0.0);

  this->gyro_x = (gyroscope.gyro.x - this->gyro_x_bias);
  this->gyro_x = (abs(this->gyro_x) >= ZUPT_THRESHOLD ? this->gyro_x : 0.0);
  this->gyro_y = (gyroscope.gyro.y - this->gyro_y_bias);
  this->gyro_y = (abs(this->gyro_y) >= ZUPT_THRESHOLD ? this->gyro_y : 0.0);
  this->gyro_z = (gyroscope.gyro.z - this->gyro_z_bias);
  this->gyro_z = (abs(this->gyro_z) >= ZUPT_THRESHOLD ? this->gyro_z : 0.0);
}