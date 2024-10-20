#include "../lib/gps.hpp"
#include "../lib/config.hpp"
#include <Arduino.h>

GPSModule gpsModule;

GPSModule::GPSModule() : latitude(0), longitude(0), locationValid(false) {
    dataMutex = xSemaphoreCreateMutex();
}

void GPSModule::init() {
    // Serial.println("Initializing GPS module");
    Serial1.begin(GPS_BAUD_RATE);  // GPS connected to Serial1 (TX1=GPS_TX_PIN, RX1=GPS_RX_PIN)
    // Serial.println("GPS Serial initialized");
}

void GPSModule::update() {
    while (Serial1.available() > 0) {
        if (gps.encode(Serial1.read())) {
            if (gps.location.isValid()) {
                xSemaphoreTake(dataMutex, portMAX_DELAY);
                latitude = gps.location.lat();
                longitude = gps.location.lng();
                locationValid = true;
                xSemaphoreGive(dataMutex);
                // Serial.println("Valid GPS data received");
            } else {
                xSemaphoreTake(dataMutex, portMAX_DELAY);
                locationValid = false;
                xSemaphoreGive(dataMutex);
                // Serial.println("Invalid GPS data");
            }
        }
    }
}

bool GPSModule::isLocationValid() const {
    bool valid;
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    valid = locationValid;
    xSemaphoreGive(dataMutex);
    return valid;
}

float GPSModule::getLatitude() const {
    float lat;
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    lat = latitude;
    xSemaphoreGive(dataMutex);
    return lat;
}

float GPSModule::getLongitude() const {
    float lon;
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    lon = longitude;
    xSemaphoreGive(dataMutex);
    return lon;
}

void GPSModule::gpsTask(void* pvParameters) {
    (void)pvParameters;
    // Serial.println("GPS task started");
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(GPS_UPDATE_FREQUENCY);
    xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        // Serial.println("Updating GPS data...");
        gpsModule.update();
        
        if (gpsModule.isLocationValid()) {
            Serial.print("LAT=");
            Serial.print(gpsModule.getLatitude(), 6);
            Serial.print(", LON=");
            Serial.println(gpsModule.getLongitude(), 6);
        } else {
            Serial.print("LAT=");
            Serial.print("NaN");
            Serial.print(", LON=");
            Serial.println("NaN");
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}