#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>
#include <WiFiS3.h>
#include "Inertial.h"
#include "Communication.h"
#include "settings.h"

char CommServer::strBuffer[STRING_BUFFER_SIZE];
WiFiClient CommServer::wifiClient;
MqttClient CommServer::mqttClient(CommServer::wifiClient);
InertialPositionTracker* CommServer::positionTracker = nullptr;
SemaphoreHandle_t CommServer::semaphore;
uint64_t CommServer::last_timestamp = 0;

bool CommServer::initialize(InertialPositionTracker* positionTracker, SemaphoreHandle_t semaphore) {
  if (!positionTracker) {
    Serial.println("Position tracker cannot be null");
    return false;
  }
  CommServer::positionTracker = positionTracker;
  CommServer::semaphore = semaphore;
  return (CommServer::connectWiFi() && CommServer::connectMqttBroker());
}

bool CommServer::connectWiFi() {
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    return false;
  }

  Serial.print("Attempting to connect to WiFi network");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    delay(5000);
  }
  
  snprintf(
    CommServer::strBuffer,
    STRING_BUFFER_SIZE,
    " Successful! IP address: %s",
    WiFi.localIP().toString().c_str()
  );
  Serial.println(CommServer::strBuffer);
  return true;
}

bool CommServer::connectMqttBroker() {
  snprintf(
    CommServer::strBuffer,
    STRING_BUFFER_SIZE,
    "Attempting to connect to MQTT broker at %s:%i... ",
    MQTT_BROKER_ADDRESS,
    MQTT_BROKER_PORT
  );
  Serial.print(CommServer::strBuffer);

  if (!CommServer::mqttClient.connect(MQTT_BROKER_ADDRESS, MQTT_BROKER_PORT)) {
    snprintf(
      CommServer::strBuffer,
      STRING_BUFFER_SIZE,
      "Failed with code %i",
      CommServer::mqttClient.connectError()
    );
    Serial.println(CommServer::strBuffer);
    return false;
  }

  Serial.println("Success!");
  return true;
}

void CommServer::runServer() {
  if (!CommServer::mqttClient.connected()) {
    Serial.println("Connection to broker lost, attempting to restore it...");
    if (!CommServer::connectMqttBroker()) return;
    Serial.println("Connection to broker restored!");
  }
  CommServer::mqttClient.poll();

  uint64_t current_timestamp = millis();
  
  if ((current_timestamp - last_timestamp) < MQTT_PUBLISH_INTERVAL) return;
  last_timestamp = current_timestamp;

  if (!xSemaphoreTake(CommServer::semaphore, pdMS_TO_TICKS(10))) return;
  inertial_tracker_state_t current_state = CommServer::positionTracker->getCurrentState();
  xSemaphoreGive(CommServer::semaphore);
  
  CommServer::publishVelocityAndPositionData(current_state);
  CommServer::publishSensorStateData(current_state);
  CommServer::publishEulerAnglesData(current_state);
}

void CommServer::publishVelocityAndPositionData(const inertial_tracker_state_t& trackerState) {
  String output;
  JsonDocument document;

  document["Pos"]["X"] = trackerState.pos_x;
  document["Pos"]["Y"] = trackerState.pos_y;
  document["Pos"]["Z"] = trackerState.pos_z;
  document["Vel"]["X"] = trackerState.vel_x;
  document["Vel"]["Y"] = trackerState.vel_y;
  document["Vel"]["Z"] = trackerState.vel_z;

  serializeJson(document, output);
  CommServer::publishData(output.c_str(), MQTT_VELOCITY_POSITION_TOPIC);
}

void CommServer::publishSensorStateData(const inertial_tracker_state_t& trackerState) {
  String output;
  JsonDocument document;

  document["Accel"]["X"] = trackerState.accel_x;
  document["Accel"]["Y"] = trackerState.accel_x;
  document["Accel"]["Z"] = trackerState.accel_x;
  document["Gyro"]["X"] = trackerState.gyro_x;
  document["Gyro"]["Y"] = trackerState.gyro_y;
  document["Gyro"]["Z"] = trackerState.gyro_z;
  document["Mag"]["X"] = trackerState.mag_x;
  document["Mag"]["Y"] = trackerState.mag_y;
  document["Mag"]["Z"] = trackerState.mag_z;

  serializeJson(document, output);
  CommServer::publishData(output.c_str(), MQTT_SENSOR_STATE_TOPIC);
}

void CommServer::publishEulerAnglesData(const inertial_tracker_state_t& trackerState) {
  String output;
  JsonDocument document;

  document["Roll"] = trackerState.roll;
  document["Pitch"] = trackerState.pitch;
  document["Yaw"] = trackerState.yaw;

  serializeJson(document, output);
  CommServer::publishData(output.c_str(), MQTT_EULER_ANGLES_TOPIC);
}

void CommServer::publishData(const char* payload, const char* topic) {
  CommServer::mqttClient.beginMessage(topic);
  CommServer::mqttClient.print(payload);
  CommServer::mqttClient.endMessage();
}