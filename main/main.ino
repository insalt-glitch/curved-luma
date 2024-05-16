#include <math.h>
#include <stdio.h>
#include <stdarg.h>

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
#include "FilterImplementation.h"

MeasurementVector measurement;
StateVector mu_prediction, mu_update;
BLA::Matrix<STATE_SIZE, STATE_SIZE> cov_prediction, cov_update;
f32 dt, t_last, t_now;

void setup()
{
    InitLogging(SERIAL_BAUD_RATE);
    // initialize GPS, IMU and DMP
    InitGPS();
    InitIMU();
    InitializeFilter(&IMU, &GPS, &mu_update, &cov_update);
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
            &motion_model, &mu_prediction, &cov_prediction
        );
        BLA::Matrix<7,1> partial_measurement;
        partial_measurement(0,0) = measurement.speed;
        partial_measurement.Submatrix<3,1>(1,0) = measurement.acc.vec;
        partial_measurement.Submatrix<3,1>(4,0) = measurement.gyr.vec;
        Kalman::UpdateStepEKF(
            mu_prediction, cov_prediction, partial_measurement, cov_measurement_noise,
            &measurement_model, &mu_update, &cov_update
        );
        // Normailze quaterion
        mu_update.quat.vec /= BLA::Norm(mu_update.quat.vec);
        BasicPrint("\n\nQuaternion: ", mu_update.quat.vec);
        BasicPrint("\nVelocity: ", mu_update.speed);
        BasicPrint("\nAcceleration: ", mu_update.acceleration);
        PrintFlush();
    }
}
