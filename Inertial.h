#ifndef __INERTIAL__
#define __INERTIAL__
#include "settings.h"
#include <Adafruit_MPU6050.h>
#include <QMC5883LCompass.h>
#include <MadgwickAHRS.h>

class InertialPositionTracker {
private:
  Adafruit_MPU6050 mpu;
  QMC5883LCompass magnetometer;
  Madgwick filter;
  float vel_x = 0, vel_y = 0, vel_z = 0;
  float pos_x = 0, pos_y = 0, pos_z = 0;
  float accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z;
  float accel_x_filtered = 0, accel_y_filtered = 0, accel_z_filtered = 0;
  float accel_x_bias = 0, accel_y_bias = 0, accel_z_bias = 0, gyro_x_bias = 0, gyro_y_bias = 0, gyro_z_bias = 0;
  bool initializeMPU();
  void initializeMagnetometer();
  void calibrateMPU();
  void getMpuReading();
  void getMagnetometerReading();

public:
  bool initialize();
  void updateState(float delta_time);
  String getStateJson();
  float getAccelerometerX();
  float getAccelerometerY();
  float getAccelerometerZ();
  float getGyroscopeX();
  float getGyroscopeY();
  float getGyroscopeZ();
  float getRoll();
  float getPitch();
  float getYaw();
  float getVelocityX();
  float getVelocityY();
  float getVelocityZ();
  float getPositionX();
  float getPositionY();
  float getPositionZ();
};

#endif