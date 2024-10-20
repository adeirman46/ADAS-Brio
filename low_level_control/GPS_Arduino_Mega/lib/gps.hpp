#ifndef GPS_HPP
#define GPS_HPP

#include <TinyGPS++.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>

class GPSModule {
public:
    GPSModule();
    void init();
    void update();
    bool isLocationValid() const;
    float getLatitude() const;
    float getLongitude() const;

    static void gpsTask(void* pvParameters);

private:
    TinyGPSPlus gps;
    float latitude;
    float longitude;
    bool locationValid;
    SemaphoreHandle_t dataMutex;
};

extern GPSModule gpsModule;

#endif // GPS_HPP