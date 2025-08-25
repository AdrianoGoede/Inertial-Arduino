#include "Inertial.h"
#include <ArduinoJson.h>

bool InertialPositionTracker::initialize() {
  if (!this->initializeMPU()) return false;
  this->calibrateMPU();
  this->filter.begin(FILTER_SAMPLING_RATE);
  
  this->initializeMagnetometer();

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
  this->mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  this->mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  this->mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  this->mpu.setI2CBypass(true);
  delay(100);
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
    delay(10);
  }

  this->accel_x_bias = (float)(ax / MPU_CALIBRATION_SAMPLES);
  this->accel_y_bias = (float)(ay / MPU_CALIBRATION_SAMPLES);
  this->accel_z_bias = (float)(az / MPU_CALIBRATION_SAMPLES);
  this->gyro_x_bias = (float)(gx / MPU_CALIBRATION_SAMPLES);
  this->gyro_y_bias = (float)(gy / MPU_CALIBRATION_SAMPLES);
  this->gyro_z_bias = (float)(gz / MPU_CALIBRATION_SAMPLES);
}

void InertialPositionTracker::initializeMagnetometer() {
  this->magnetometer.init();
}

void InertialPositionTracker::updateState(float delta_time) {
  this->getMpuReading();
  this->getMagnetometerReading();
  this->filter.update(
    this->gyro_x, this->gyro_y, this->gyro_z,
    this->accel_x, this->accel_y, this->accel_z,
    this->mag_x, this->mag_y, this->mag_z
  );

  float sin_roll  = sin(this->filter.getRoll()),  cos_roll  = cos(this->filter.getRoll());
  float sin_pitch = sin(this->filter.getPitch()), cos_pitch = cos(this->filter.getPitch());
  float sin_yaw   = sin(this->filter.getYaw()),   cos_yaw   = cos(this->filter.getYaw());

  float ax_world = this->accel_x * (cos_yaw * cos_pitch) +
                   this->accel_y * (cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll) +
                   this->accel_z * (cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll);

  float ay_world = this->accel_x * (sin_yaw * cos_pitch) +
                   this->accel_y * (sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll) +
                   this->accel_z * (sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll);

  float az_world = this->accel_x * (-sin_pitch) +
                   this->accel_y * (cos_pitch * sin_roll) +
                   this->accel_z * (cos_pitch * cos_roll);

  float total_accel_magnitude = sqrt(
    ax_world * ax_world +
    ay_world * ay_world +
    az_world * az_world
  );

  this->vel_x = ((total_accel_magnitude >= STATIONARY_THRESHOLD) ? (this->vel_x + ax_world * delta_time) : 0);
  this->vel_y = ((total_accel_magnitude >= STATIONARY_THRESHOLD) ? (this->vel_y + ay_world * delta_time) : 0);
  this->vel_z = ((total_accel_magnitude >= STATIONARY_THRESHOLD) ? (this->vel_z + az_world * delta_time) : 0);

  this->pos_x += (this->vel_x * delta_time);
  this->pos_y += (this->vel_y * delta_time);
  this->pos_z += (this->vel_z * delta_time);
}

void InertialPositionTracker::getMpuReading() {
  sensors_event_t accelerator, gyroscope;
  mpu.getEvent(&accelerator, &gyroscope, nullptr);

  this->accel_x = (accelerator.acceleration.x - this->accel_x_bias);
  this->accel_y = (accelerator.acceleration.y - this->accel_y_bias);
  this->accel_z = (accelerator.acceleration.z - this->accel_z_bias);

  this->gyro_x = (gyroscope.gyro.x - this->gyro_x_bias);
  this->gyro_y = (gyroscope.gyro.y - this->gyro_y_bias);
  this->gyro_z = (gyroscope.gyro.z - this->gyro_z_bias);
}

void InertialPositionTracker::getMagnetometerReading() {
  this->magnetometer.read();
  this->mag_x = (float)this->magnetometer.getX();
  this->mag_y = (float)this->magnetometer.getY();
  this->mag_z = (float)this->magnetometer.getZ();
}

String InertialPositionTracker::getStateJson() {
  String result;
  JsonDocument document;
  document["Velocity"]["X"] = this->getVelocityX();
  document["Velocity"]["Y"] = this->getVelocityY();
  document["Velocity"]["Z"] = this->getVelocityZ();
  document["Position"]["X"] = this->getPositionX();
  document["Position"]["Y"] = this->getPositionY();
  document["Position"]["Z"] = this->getPositionZ();
  serializeJson(document, result);
  return result;
}