#include <math.h>

#include <Arduino_MKRGPS.h>
#include "ICM_20948.h"
#include "LinearAlgebra.h"

#include "Sensors.h"
#include "Helpers.h"
#include "EKF_Kalman_Filter.h"

#define i16 int16_t
#define u16 uint16_t
#define u32 uint32_t
#define i32 int32_t
#define f32 float

#define SERIAL_BAUD_RATE 115200

const BLA::Matrix<STATE_SIZE, STATE_SIZE, float> cov_motion_noise =
{
    2.0f * PI / 180.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 2.0f * PI / 180.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 2.0f * PI / 180.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 2.0f * PI / 180.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f,               0.5f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f,               1.0f,
};
const BLA::Matrix<7, 7, float> cov_measurement_noise =
{
    0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
};

MeasurementVector measurement;
StateVector mu_prediction, mu_update;
BLA::Matrix<STATE_SIZE, STATE_SIZE> cov_prediction, cov_update;
f32 dt, t_last, t_now;


f32 sign(f32 x) { return (x > 0.0f) - (x < 0.0f); }

void setup() {
    // initialize serial communications and wait for port to open:
    Serial.begin(SERIAL_BAUD_RATE);
    // wait for serial port to connect. Needed for native USB port only
    while (!Serial);
    // initialize GPS, IMU and DMP
    initGPS();
    initIMU();
    // initDMP();

    // Wait for first measurement
    while (!ReadMeasurements(&IMU, &GPS, &measurement));
    t_last = millis();
    // compute initial state
    mu_update.speed = 0.0f;
    mu_update.acceleration = BLA::Norm(measurement.acc.vec);
    // rotation state
    measurement.acc.vec /= BLA::Norm(measurement.acc.vec);
    measurement.mag.vec /= BLA::Norm(measurement.mag.vec);
    const BLA::Matrix<3,3> C =
        BLA::CrossProduct(BLA::CrossProduct(measurement.acc.vec, measurement.mag.vec), measurement.acc.vec)
        || BLA::CrossProduct(measurement.acc.vec, measurement.mag.vec)
        || measurement.acc.vec;
    mu_update.quat.vec = {
                                sqrt(C(0,0) + C(1,1) + C(2,2) + 1),
        sign(C(2,1) - C(1,2)) * sqrt(C(0,0) - C(1,1) - C(2,2) + 1),
        sign(C(0,2) - C(2,0)) * sqrt(C(1,1) - C(0,0) - C(2,2) + 1),
        sign(C(1,0) - C(0,1)) * sqrt(C(2,2) - C(1,1) - C(0,0) + 1),
    };
    mu_update.quat.vec *= 0.5f;
    cov_update = BLA::Eye<6,6>();
}

// TODO(Nils): Time-dependent noise??

void motion_model(
    const MeasurementVector& y,
    const StateVector& prior,
    const f32 dt,
    StateVector* const f_x,
    BLA::Matrix<STATE_SIZE, STATE_SIZE>* const f_jacobian)
{
    // prepare matrix
    BLA::Matrix<4, 4> scaled_Omega =
    {
              0, -y.gyr.x, -y.gyr.y, -y.gyr.z,
        y.gyr.x,        0,  y.gyr.z, -y.gyr.y,
        y.gyr.y, -y.gyr.z,        0,  y.gyr.x,
        y.gyr.z,  y.gyr.y, -y.gyr.x,        0,
    };
    scaled_Omega *= dt / 2;
    // compute f(x) and H_f(x)
    if (f_x != nullptr)
    {
        // quaternion (update from https://ahrs.readthedocs.io/en/latest/filters/ekf.html#prediction-step)
        f_x->quat.vec     = (BLA::Eye<4, 4>() + scaled_Omega) * prior.quat.vec;
        // acceleration
        f_x->acceleration = prior.acceleration;
        // speed
        f_x->speed        = prior.speed + dt * prior.acceleration;
    }
    if (f_jacobian != nullptr)
    {
        (*f_jacobian) = BLA::Eye<6, 6>();
        f_jacobian->Submatrix<4,4>(0,0) = f_jacobian->Submatrix<4,4>(0,0) + scaled_Omega;
        (*f_jacobian)(5,6) = dt;
    }
}

void measurement_model(
    const StateVector& prior,
    BLA::Matrix<7,1>* const h_x,
    BLA::Matrix<7, STATE_SIZE>* const h_jacobian)
{
    // magnetic dip angle (https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml#declination)
    static constexpr f32 mag_theta = 4.83 * PI / 180.0;
    static BLA::Matrix<3> r = {cos(mag_theta), 0, sin(mag_theta)};
    // g in NED-frame
    static const BLA::Matrix<3,1,float> g = {0.0f, 0.0f, -9.81f};
    // compute h(x) and jacobian
    if (h_x != nullptr)
    {
        // prepare rotation matrix
        const float qxx = prior.quat.x * prior.quat.x;
        const float qyy = prior.quat.y * prior.quat.y;
        const float qzz = prior.quat.z * prior.quat.z;
        const float qwx = prior.quat.w * prior.quat.x;
        const float qwy = prior.quat.w * prior.quat.y;
        const float qwz = prior.quat.w * prior.quat.z;
        const float qxy = prior.quat.x * prior.quat.y;
        const float qxz = prior.quat.x * prior.quat.z;
        const float qyz = prior.quat.y * prior.quat.z;
        const BLA::Matrix<3,3> C_global_to_sensor_T =
        {
            1.0f - 2 * (qyy + qzz),        2 * (qxy + qwz),        2 * (qxz - qwy),
                   2 * (qxy - qwz), 1.0f - 2 * (qxx + qzz),        2 * (qyz + qwx),
                   2 * (qxz + qwy),        2 * (qyz - qwx), 1.0f - 2 * (qxx + qyy),
        };
        // (&& means vertical concatenation)
        (*h_x)(0) = prior.speed;
        h_x->Submatrix<6,1>(1,0) = (C_global_to_sensor_T * g) && (C_global_to_sensor_T * r);
    }
    if (h_jacobian != nullptr)
    {
        h_jacobian->Fill(0);
        // speed
        (*h_jacobian)(0,4) = 1;
        // rotation
        const BLA::RefMatrix<const BLA::Matrix<4,1>, 3, 1> q_v = prior.quat.vec.Submatrix<3,1>(1,0);
        const BLA::Matrix<3> u_g = BLA::CrossProduct(g, q_v) + g * prior.quat.w;
        const BLA::Matrix<3,3> G = ((~q_v) * g)(0) * BLA::Eye<3,3>() +
        BLA::Matrix<3,3>{
                  0.0f, -u_g(2),  u_g(1),
             u_g(2),       0.0f, -u_g(0),
            -u_g(1),  u_g(0),       0.0f,
        };
        const BLA::Matrix<3> u_r = BLA::CrossProduct(r, q_v) + prior.quat.w * r;
        const BLA::Matrix<3,3> R = ((~q_v) * r)(0) * BLA::Eye<3,3>() +
        BLA::Matrix<3,3>{
                  0.0f, -u_r(2),  u_r(1),
             u_r(2),       0.0f, -u_r(0),
            -u_r(1),  u_r(0),       0.0f,
        };
        h_jacobian->Submatrix<6, 4>(1,0) = 2.0f * ((u_g || G) && (u_r || R));
    }
}


void loop()
{
    if (ReadMeasurements(&IMU, &GPS, &measurement)) {
        t_now = millis();
        dt = (t_now - t_last) / 1000;
        t_last = t_now;

        Serial.print("\n\nLinear  Acceleration (m/s)  : ");
        Serial << measurement.acc.vec;
        Serial.print("\nAngular Acceleration (rad/s): ");
        Serial << measurement.gyr.vec;
        Serial.print("\nMagnetic field       (muT)  : ");
        Serial << measurement.mag.vec;
        Serial.print("\nGPS velocity         (m/s)  : ");
        Serial.print(measurement.speed);
        Serial.flush();

        Kalman::PredictionStepEKF(
            mu_update, cov_update, measurement, cov_motion_noise, dt,
            &motion_model, &mu_prediction, &cov_prediction
        );
        BLA::Matrix<7,1> partial_measurement;
        partial_measurement.Submatrix<4,1>(0,0) = measurement.vec.Submatrix<4,1>(0,0);
        partial_measurement.Submatrix<3,1>(4,0) = measurement.gyr.vec;
        Kalman::UpdateStepEKF(
            mu_prediction, cov_prediction, partial_measurement, cov_measurement_noise,
            &measurement_model, &mu_update, &cov_update
        );
        // Normailze quaterion
        mu_prediction.quat.vec /= BLA::Norm(mu_prediction.quat.vec);
    }
    // get velocity measurement
    // define measurement function
    // define motion function
    // define measurement noise functions
}
