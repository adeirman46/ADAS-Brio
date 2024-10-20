#include <Arduino_FreeRTOS.h>
#include "../lib/config.hpp"
#include "../lib/gps.hpp"

void setup() {
    Serial.begin(115200);  // Initialize Serial for debugging
    while (!Serial) {
        ; // Wait for Serial to connect
    }
    // Serial.println("Setup started");

    gpsModule.init();
    // Serial.println("GPS module initialized");

    // Serial.println("Creating GPS task...");
    BaseType_t taskCreated = xTaskCreate(
        GPSModule::gpsTask, 
        "GPSTask", 
        GPS_TASK_STACK_SIZE, 
        NULL, 
        GPS_TASK_PRIORITY, 
        NULL
    );

    // if (taskCreated == pdPASS) {
    //     Serial.println("GPS task created successfully");
    // } else {
    //     Serial.println("Failed to create GPS task");
    //     while(1);  // Halt if task creation failed
    // }

    // Serial.println("Starting FreeRTOS scheduler");
    vTaskStartScheduler();

    // Serial.println("Scheduler started - should never reach here");
}

void loop() {
    // Empty. Tasks are handled by FreeRTOS
}