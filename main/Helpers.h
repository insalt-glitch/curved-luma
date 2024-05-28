#pragma once

#include <type_traits>

#include <Arduino_MKRGPS.h>
#include <SPI.h>
#include <SD.h>
#include "LinearAlgebra.h"
#include "FilterTypes.h"

#define i16 int
#define u16 unsigned int
#define u32 unsigned long
#define i32 long
#define f32 float

#ifdef PRINT_TO_SDCARD
    File sd_file;
    bool SDCARD_ENABLED = false;
#endif

static constexpr f32 sign(const f32 x) { return (x > 0.0f) - (x < 0.0f); }

static f32 Clamp(const f32 x, const f32 a, const f32 b)
{
    if (x < a) return a;
    if (x > b) return b;
    return x;
}

template <typename T>
static constexpr T RadToDeg(const T& x) { return ((float)(180 / M_PI)) * x; }

template <typename T>
static constexpr T DegToRad(const T& x) { return ((float)(M_PI / 180)) * x; }

EulerAngles ToEulerAngles(const Quaternion& q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    const double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    const double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);
    // pitch (y-axis rotation)
    const double sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    const double cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    angles.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;
    // yaw (z-axis rotation)
    const double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

void PrintFormatted(const char* const fmt, ...)
{
    static char buffer[256];
    va_list va;
    va_start(va, fmt);
    vsprintf(&buffer[0], fmt, va);
#ifdef PRINT_TO_SDCARD
    if (SDCARD_ENABLED) sd_file.print(buffer);
#endif  // PRINT_TO_SDCARD
    // Serial.print(buffer);
    va_end (va);
}

template <typename T>
using IsFloat = std::is_same<std::remove_const<std::remove_reference<T>>, float>;

void PrintWrapper(const f32 value) {
#ifdef PRINT_TO_SDCARD
    if (SDCARD_ENABLED) sd_file.print(value, 6);
#endif  // PRINT_TO_SDCARD
    // Serial.print(value, 6);
}

template<typename T>
typename std::enable_if<!IsFloat<T>::value>::type PrintWrapper(const T value) {
#ifdef PRINT_TO_SDCARD
    if (SDCARD_ENABLED) sd_file.print(value);
#endif  // PRINT_TO_SDCARD
    // Serial.print(value);
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

template<typename DerivedType, int Rows, int Cols, typename DType, typename... Other>
void PrintMatrixRaw(const BLA::MatrixBase<DerivedType, Rows, Cols, DType> &mat)
{
    for (u32 i = 0; i < Rows; i++)
    {
        for (u32 j = 0; j < Cols; j++)
        {
            PrintWrapper(mat(i,j));
            if (!((i == Rows - 1) && (j == Cols - 1))) PrintWrapper(";");
        }
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
    // Serial.flush();
}

void Assert(const bool condition, const char* const message = nullptr)
{
    if (!condition)
    {
        if (message != nullptr)
        {
            PrintWrapper(message);
            PrintFlush();
        }
        while (true);
    }
}

void InitLogging(const u32 baud_rate)
{
    // initialize serial communications and wait for port to open:
    // Serial.begin(baud_rate);
    // wait for serial port to connect. Needed for native USB port only
    // while (!Serial);
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
    // if (!gps->available()) return false;
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

void BlinkTest()
{
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT);
    pinMode(15, INPUT);

    digitalWrite(6, LOW);
    digitalWrite(7, HIGH);
    bool led1State = false;
    bool led2State = true;

    while (true)
    {
        for (u32 i = 0; i < 4; i++)
        {
            for (u32 j = 0; j < 6; j++)
            {

                digitalWrite(j    , HIGH);
                digitalWrite(i+ 17, HIGH);
                digitalWrite(6, led1State);
                digitalWrite(7, led2State);
                led1State = !led1State;
                led2State = !led2State;
                delay(200);
                digitalWrite(j    , LOW);
                digitalWrite(i+ 17, LOW);
            }
        }
    }
}
