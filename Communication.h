#include "WiFiSSLClient.h"
#include <sys/_stdint.h>
#ifndef __COMMUNICATION__
#define __COMMUNICATION__

#include <Arduino_FreeRTOS.h>
#include <ArduinoMqttClient.h>
#include <WiFiS3.h>
#include "Inertial.h"

enum RequestCodes {
  getPosition = 0,
  getVelocity = 1,
  getPositionAndVelocity = 2,
  getEulerAngles = 3,
  getAccelerometer = 4,
  getGyroscope = 5,
  getMagnetometer = 6,
  getFullState = 7
};

class CommServer {
private:
  static char strBuffer[STRING_BUFFER_SIZE];
  static WiFiClient wifiClient;
  static MqttClient mqttClient;
  static SemaphoreHandle_t semaphore;
  static InertialPositionTracker* positionTracker;
  static uint64_t last_timestamp;
  static bool connectWiFi();
  static bool connectMqttBroker();
  static void publishVelocityAndPositionData(const inertial_tracker_state_t& trackerState);
  static void publishSensorStateData(const inertial_tracker_state_t& trackerState);
  static void publishEulerAnglesData(const inertial_tracker_state_t& trackerState);
  static void publishData(const char* payload, const char* topic);
public:
  static bool initialize(InertialPositionTracker* positionTracker, SemaphoreHandle_t semaphore);
  static void runServer();
};

#endif