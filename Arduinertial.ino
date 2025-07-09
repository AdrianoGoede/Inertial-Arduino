#include "Inertial.h"

InertialPositionTracker positionTracker;
uint64_t last_update = millis();
char strBuffer[100];

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(200);

  if (!positionTracker.initialize()) {
    Serial.println("Error initializing tracker");
    while (true);
  }
}

void loop() {
  uint64_t now = millis();
  float dt = ((now - last_update) / 1000.0);
  if (dt < (1.0 / FILTER_SAMPLING_RATE)) return;
  last_update = now;
  
  positionTracker.updateState(dt);
  
  sprintf(
    strBuffer,
    "%.5f,%.5f,%.5f",
    positionTracker.getPositionX(),
    positionTracker.getPositionY(),
    positionTracker.getPositionZ()
  );
  Serial.println(strBuffer);
}