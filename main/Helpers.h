#pragma once

#include <Arduino_MKRGPS.h>

#define i16 int
#define u16 unsigned int
#define u32 unsigned long
#define i32 long
#define f32 float

bool PrintGPSStatus(GPSClass* const gps) {
    if (!gps->available())
    {
        return false;
    }
    // read GPS values
    const f32 latitude   = gps->latitude();
    const f32 longitude  = gps->longitude();
    const f32 altitude   = gps->altitude();
    const f32 speed      = gps->speed();
    const i32 satellites = gps->satellites();

    // print GPS values
    Serial.print("Location: ");
    Serial.print(latitude, 7);
    Serial.print(", ");
    Serial.println(longitude, 7);

    Serial.print("Altitude: ");
    Serial.print(altitude);
    Serial.println("m");

    Serial.print("Ground speed: ");
    Serial.print(speed);
    Serial.println(" km/h");

    Serial.print("Number of satellites: ");
    Serial.println(satellites);

    Serial.println();
    return true;
}

void printStatusInterval(GPSClass* const gps) {
    const static size_t REFRESH_INTERVAL = 2000;
    static size_t lastRefreshTime = 0;
    if(millis() - lastRefreshTime > REFRESH_INTERVAL) {
        lastRefreshTime = millis();
        // Check if there is new GPS data available
        PrintGPSStatus(gps);
    }
}