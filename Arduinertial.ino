#include <Arduino_FreeRTOS.h>
#include "Inertial.h"
#include "Communication.h"

InertialPositionTracker* positionTracker;
SemaphoreHandle_t semaphore;

void intertialTask(void* pvParameters) {
  uint64_t now, last_update = millis();
  float dt;

  while (true) {
    now = millis();
    dt = ((now - last_update) / 1000.0);
    
    if (dt < (1.0 / FILTER_SAMPLING_RATE) || xSemaphoreTake(semaphore, pdMS_TO_TICKS(10)) != pdTRUE) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
    
    positionTracker->updateState(dt);
    xSemaphoreGive(semaphore);
    last_update = now;
  }
}

void communicationTask(void* pvParameters) {
  while (true) {
    CommServer::runServer();
    vTaskDelay(pdMS_TO_TICKS(1)); 
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial) delay(200);

  semaphore = xSemaphoreCreateBinary();
  if (!semaphore) {
    Serial.println("Could not create semaphore, aborting...");
    while (true);
  }
  xSemaphoreGive(semaphore);
  Serial.println("Semaphore initialized");

  positionTracker = new InertialPositionTracker();
  if (!positionTracker->initialize()) {
    Serial.println("Could not initialize inertial position tracker, aborting...");
    while (true);
  }
  Serial.println("Inertial position tracker initialized");

  if (!CommServer::initialize(positionTracker, semaphore)) {
    Serial.println("Could not initialize communication interface, aborting...");
    while (true);
  }

  xTaskCreate(
    intertialTask,
    "Inertial position tracker task",
    INERTIAL_TRACKING_TASK_STACK_SIZE,
    nullptr,
    INERTIAL_TRACKING_TASK_PRIORITY,
    nullptr
  );

  xTaskCreate(
    communicationTask,
    "Communication task",
    COMMUNICATION_TASK_STACK_SIZE,
    nullptr,
    COMMUNICATION_TASK_PRIORITY,
    nullptr
  );

  Serial.println("All set! Starting...\n");
  vTaskStartScheduler();
}

void loop() { }