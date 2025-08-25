#include <Arduino_FreeRTOS.h>
#include "Inertial.h"

InertialPositionTracker positionTracker;
SemaphoreHandle_t semaphore;

void intertialTask(void* pvParameters) {
  uint64_t now, last_update = millis();
  float dt;

  while (true) {
    now = millis();
    dt = ((now - last_update) / 1000.0);
    
    if (dt < (1.0 / FILTER_SAMPLING_RATE) || xSemaphoreTake(semaphore, portMAX_DELAY) != pdTRUE) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
    
    positionTracker.updateState(dt);
    xSemaphoreGive(semaphore);
    last_update = now;
  }
}

// For test only!
void printingTask(void* pvParameters) {
  char strBuffer[100];
  
  while (true) {
    if (xSemaphoreTake(semaphore, portMAX_DELAY) != pdTRUE)
      continue;

    snprintf(
      strBuffer,
      100,
      "%.5f,%.5f,%.5f",
      positionTracker.getPositionX(),
      positionTracker.getPositionY(),
      positionTracker.getPositionZ()
    );
    Serial.println(strBuffer);

    xSemaphoreGive(semaphore);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial) delay(200);

  semaphore = xSemaphoreCreateBinary();
  if (semaphore == nullptr) {
    Serial.println("Error creating Mutex");
    while (true);
  }
  xSemaphoreGive(semaphore);
  Serial.println("Semaphore initialized");

  while (!positionTracker.initialize()) {
    Serial.println("Error initializing position semaphore");
    while (true);
  }
  Serial.println("Tracker initialized");

  xTaskCreate(
    intertialTask,
    "Inertial position tracker task",
    INERTIAL_TRACKING_TASK_STACK_SIZE,
    nullptr,
    INERTIAL_TRACKING_TASK_PRIORITY,
    nullptr
  );

  // For test only!
  xTaskCreate(
    printingTask,
    "Test task",
    256,
    nullptr,
    1,
    nullptr
  );

  Serial.println("Starting...");
  vTaskStartScheduler();
}

void loop() { }