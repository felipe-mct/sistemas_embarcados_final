#include "esp32-hal-ledc.h"
#include <Arduino.h>

// Define RGB LED pins
#define RED 2
#define GREEN 4
#define BLUE 15

// Potentiometer pin
#define POT_PIN 34

// Variables for PWM
int incr = 5;
int DutyRED = 0;
int DutyGREEN = 0;
int DutyBLUE = 0;

// FreeRTOS task handles
TaskHandle_t TaskReadPotentiometer;
TaskHandle_t TaskSetRGB;

// FreeRTOS queue handle
QueueHandle_t potQueue;

// Function to increment PWM values
int increment(int duty, int incr, int coef) {
  duty += incr * coef;
  if (duty > 255) {
    duty = 0;
  }
  return duty;
}

// Task to read potentiometer value
void TaskReadPotentiometerCode(void *pvParameters) {
  for (;;) {
    int potValue = analogRead(POT_PIN);
    potValue = map(potValue, 0, 4095, 0, 255); // Map potentiometer value to 0-255

    // Send potentiometer value to the queue
    if (xQueueSend(potQueue, &potValue, portMAX_DELAY) != pdPASS) {
      Serial.println("Failed to send to queue");
    }

    vTaskDelay(100 / portTICK_PERIOD_MS); // Delay for 100ms
  }
}

// Task to set RGB LED color based on potentiometer value
void TaskSetRGBCode(void *pvParameters) {
  int potValue = 0;

  for (;;) {
    // Receive potentiometer value from the queue
    if (xQueueReceive(potQueue, &potValue, portMAX_DELAY) == pdPASS) {
      // Use potentiometer value to set brightness
      DutyRED = increment(DutyRED, incr, 2);
      DutyGREEN = increment(DutyGREEN, incr, 1);
      DutyBLUE = increment(DutyBLUE, incr, 3);

      ledcWrite(RED, DutyRED * potValue / 255);
      ledcWrite(GREEN, DutyGREEN * potValue / 255);
      ledcWrite(BLUE, DutyBLUE * potValue / 255);

      Serial.print("Potentiometer Value: ");
      Serial.println(potValue);
      Serial.print("DutyRED: ");
      Serial.println(DutyRED);
      Serial.print("DutyGREEN: ");
      Serial.println(DutyGREEN);
      Serial.print("DutyBLUE: ");
      Serial.println(DutyBLUE);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(10);

  // Attach LED pins to PWM channels
  ledcAttach(RED, 5000, 8);
  ledcAttach(GREEN, 5000, 8);
  ledcAttach(BLUE, 5000, 8);

  // Create the FreeRTOS queue
  potQueue = xQueueCreate(10, sizeof(int)); // Queue size: 10 items, each of type `int`
  if (potQueue == NULL) {
    Serial.println("Error creating the queue");
    while (1);
  }

  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(TaskReadPotentiometerCode, "ReadPot", 1000, NULL, 1, &TaskReadPotentiometer, 0);
  xTaskCreatePinnedToCore(TaskSetRGBCode, "SetRGB", 1000, NULL, 1, &TaskSetRGB, 1);
}

void loop() {
  // FreeRTOS tasks handle everything, so the loop remains empty
}
