#include <math.h>
#include <stdio.h>
#include <stdarg.h>

#include <ArduinoLowPower.h>
#include <Arduino_MKRGPS.h>
#include "ICM_20948.h"
#include "LinearAlgebra.h"

#define i16 int16_t
#define u16 uint16_t
#define u32 uint32_t
#define i32 int32_t
#define f32 float

// #define PRINT_TO_SDCARD
#define SERIAL_BAUD_RATE 115200

#include "Helpers.h"
#include "Sensors.h"
#include "EKF_Kalman_Filter.h"
#include "FilterTypes.h"
#include "FilterImplementation.h"
#include "LedControl.h"

MeasurementVector measurement;
BLA::Matrix<7,1> partial_measurement;
StateVector mu_prediction, mu_update;
BLA::Matrix<STATE_SIZE, STATE_SIZE> cov_prediction, cov_update;
f32 dt, t_last, t_now;

constexpr u32 PIN_POWERED_LED = 7;
constexpr u32 PIN_GPS_LED =6;
constexpr u32 PIN_BUTTON = 8;

void EnableDeepSleep()
{
    digitalWrite(PIN_POWERED_LED, LOW);
    digitalWrite(PIN_GPS_LED, LOW);
    LED::PowerOff();
    // wait for the button to be released
    while (digitalRead(PIN_BUTTON) == HIGH) { delay(10); }
    attachInterrupt(digitalPinToInterrupt(PIN_BUTTON), WakeupFromDeepSleep, HIGH);
    LowPower.deepSleep();
    detachInterrupt(digitalPinToInterrupt(PIN_BUTTON));
    digitalWrite(PIN_POWERED_LED, HIGH);
}

void WakeupFromDeepSleep() {}

void setup()
{
    InitLogging(SERIAL_BAUD_RATE);
    // initialize GPS, IMU and LEDs
    InitGPS();
    InitIMU();
    LED::InitMatrix();
    pinMode(PIN_POWERED_LED, OUTPUT);
    pinMode(PIN_GPS_LED, OUTPUT);
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    digitalWrite(PIN_POWERED_LED, HIGH);
    digitalWrite(PIN_GPS_LED, LOW);
    // initialize Kalman filter
    InitializeFilter(&IMU, &GPS, &mu_update, &cov_update);
    BasicPrint("\nQuat: ", mu_update.quat.vec);
    BasicPrint("  |  ", BLA::Norm(mu_update.quat.vec));
}

void loop()
{
    if (ReadMeasurements(&IMU, &GPS, &measurement, &dt)) {
        // BasicPrint("\n\nLinear  Acceleration (m/s)  : ", measurement.acc.vec);
        // BasicPrint("\nAngular Acceleration (rad/s): ", measurement.gyr.vec);
        // BasicPrint("\nMagnetic field       (muT)  : ", measurement.mag.vec);
        // BasicPrint("\nGPS velocity         (m/s)  : ", measurement.speed);

        Kalman::PredictionStepEKF(
            mu_update, cov_update, measurement, cov_motion_noise, dt,
            &motion_model, &mu_prediction, &cov_prediction);
        ExtractPartialMeasurement(measurement, &partial_measurement);
        Kalman::UpdateStepEKF(
            mu_prediction, cov_prediction, partial_measurement, cov_measurement_noise,
            &measurement_model, &mu_update, &cov_update);
        // Normailze quaterion
        mu_update.quat.vec /= BLA::Norm(mu_update.quat.vec) * sign(mu_update.quat.w);
        const EulerAngles e_angles = ToEulerAngles(mu_update.quat);
        constexpr f32 assumed_velocity = 10.0f / 3.6f;
        LED::UpdateMatrix(assumed_velocity, e_angles.pitch);
        // printing
        BasicPrint("EulerAngles: ", RadToDeg(e_angles.vec));
        PrintFlush();
    }
    if (digitalRead(PIN_BUTTON) == HIGH) EnableDeepSleep();
}
