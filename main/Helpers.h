#pragma once

#include <Arduino_MKRGPS.h>

#define i16 int
#define u16 unsigned int
#define u32 unsigned long
#define i32 long
#define f32 float

void printGPSStatus() {
    if (GPS.available()) {
        // read GPS values
        f32 latitude   = GPS.latitude();
        f32 longitude  = GPS.longitude();
        f32 altitude   = GPS.altitude();
        f32 speed      = GPS.speed();
        i32 satellites = GPS.satellites();

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
    }
    else {
        Serial.println("GPS not available!");
    }
}

void printStatusInterval() {
    const static size_t REFRESH_INTERVAL = 2000;
    static size_t lastRefreshTime = 0;
    if(millis() - lastRefreshTime > REFRESH_INTERVAL) {
        lastRefreshTime = millis();
        // Check if there is new GPS data available
        printGPSStatus();
    }
}