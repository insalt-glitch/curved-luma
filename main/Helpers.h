#pragma once

#include <type_traits>

#include <Arduino_MKRGPS.h>
#include <SPI.h>
#include <SD.h>
#include "LinearAlgebra.h"

#define i16 int
#define u16 unsigned int
#define u32 unsigned long
#define i32 long
#define f32 float

#ifdef PRINT_TO_SDCARD
    File sd_file;
    bool SDCARD_ENABLED = false;
#endif

f32 sign(f32 x) { return (x > 0.0f) - (x < 0.0f); }

void PrintFormatted(const char* const fmt, ...)
{
    static char buffer[256];
    va_list va;
    va_start(va, fmt);
    vsprintf(&buffer[0], fmt, va);
#ifdef PRINT_TO_SDCARD
    if (SDCARD_ENABLED) sd_file.print(buffer);
#endif  // PRINT_TO_SDCARD
    Serial.print(buffer);
    va_end (va);
}

template<typename T>
void PrintWrapper(T value) {
#ifdef PRINT_TO_SDCARD
    if (SDCARD_ENABLED) sd_file.print(value);
#endif  // PRINT_TO_SDCARD
    Serial.print(value);
}

template<typename DerivedType, int Rows, int Cols, typename DType>
void PrintMatrix(const BLA::MatrixBase<DerivedType, Rows, Cols, DType> &mat)
{
    PrintWrapper("[");
    for (u32 i = 0; i < Rows; i++)
    {
        PrintWrapper("[");
        for (u32 j = 0; j < Cols; j++)
        {
            PrintWrapper(mat(i,j));
            PrintWrapper((j == Cols - 1) ? "]" : ",");
        }
        PrintWrapper((i == Rows - 1) ? "]" : ",");
    }
}

void BasicPrint() {};

template<typename DerivedType, int Rows, int Cols, typename DType, typename... Other>
void BasicPrint(const BLA::MatrixBase<DerivedType, Rows, Cols, DType> &mat, Other... args)
{
    PrintMatrix(mat);
    BasicPrint(args...);
}

template<typename T, typename... Other>
typename std::enable_if<std::is_fundamental<T>::value>::type
BasicPrint(T value, Other... args)
{
    PrintWrapper(value);
    BasicPrint(args...);
}

template<typename... Other>
void BasicPrint(const char* const value, Other... args)
{
    PrintWrapper(value);
    BasicPrint(args...);
}

void PrintFlush()
{
#ifdef PRINT_TO_SDCARD
    sd_file.flush();
#endif  // PRINT_TO_SDCARD
    Serial.flush();
}

void InitLogging(const u32 baud_rate)
{
    // initialize serial communications and wait for port to open:
    Serial.begin(baud_rate);
    // wait for serial port to connect. Needed for native USB port only
    while (!Serial);
#ifdef PRINT_TO_SDCARD
    if (!SD.begin())
    {
        PrintFormatted("Failed to initialize SD card...");
        return;
    }
    u32 log_id = 0;
    char file_name[16];
    sprintf(file_name, "%d.log", log_id);
    while(SD.exists(file_name))
    {
        log_id += 1;
        sprintf(file_name, "%d.log", log_id);
    }
    sd_file = SD.open(file_name, FILE_WRITE);
    if (!sd_file)
    {
        PrintFormatted("Failed to create file '%s'", sd_file);
        return;
    }
    SDCARD_ENABLED = true;
#endif  // PRINT_TO_SDCARD
}

bool PrintGPSStatus(GPSClass* const gps)
{
    if (!gps->available()) return false;
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

void printStatusInterval(GPSClass* const gps)
{
    const static size_t REFRESH_INTERVAL = 2000;
    static size_t lastRefreshTime = 0;
    if(millis() - lastRefreshTime > REFRESH_INTERVAL)
    {
        lastRefreshTime = millis();
        // Check if there is new GPS data available
        PrintGPSStatus(gps);
    }
}