#ifndef __INERTIAL__
#define __INERTIAL__

#include <Adafruit_MPU6050.h>
#include <QMC5883LCompass.h>
#include <MatrixMath.h>
#include "settings.h"

typedef struct inertial_tracker_state {
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
  float mag_x, mag_y, mag_z;
  float roll, pitch, yaw;
  float vel_x, vel_y, vel_z;
  float pos_x, pos_y, pos_z;
} inertial_tracker_state_t;

class InertialPositionTracker {
private:
  Adafruit_MPU6050 mpu;
  QMC5883LCompass magnetometer;
  mtx_type state_vector[STATE_COVARIANCE_SIZE]{ INITIAL_STATE_VECTOR };
  mtx_type covariance_matrix[STATE_COVARIANCE_SIZE][STATE_COVARIANCE_SIZE];
  mtx_type process_noise_matrix[STATE_COVARIANCE_SIZE][STATE_COVARIANCE_SIZE];
  mtx_type measurement_noise_matrix[NOISE_MATRIX_SIZE][NOISE_MATRIX_SIZE];
  bool initializeMPU();
  void initializeMagnetometer();
  void calibrateMPU();
  void getMpuReading(mtx_type* accel_x, mtx_type* accel_y, mtx_type* accel_z, mtx_type* gyro_x, mtx_type* gyro_y, mtx_type* gyro_z);
  void getMagnetometerReading(mtx_type* mag_x, mtx_type* mag_y, mtx_type* mag_z);
  void predictState(float delta_time, const mtx_type accelerometer[3], const mtx_type gyroscope[3]);
  void updateState(float delta_time, const mtx_type accelerometer[3], const mtx_type gyroscope[3]);
  void quaternionFromAngularVelocity(float delta_time, const mtx_type gyroscope[3], mtx_type quaternion[4]);
  void quaternionMultiply(const mtx_type first[4], const mtx_type second[4], mtx_type output[4]);
  void rotateVector(const mtx_type vector[3], const mtx_type quaternion[4], mtx_type output[3]);
  void normalizeQuaternion(mtx_type quaternion[4]);
public:
  bool initialize();
  void updateState(float delta_time);
  inertial_tracker_state_t getCurrentState();
};

#endif