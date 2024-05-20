#pragma once

#include <Arduino_MKRGPS.h>
#include "ICM_20948.h"
#include "LinearAlgebra.h"

#include "Helpers.h"
#include "Sensors.h"

static const BLA::Matrix<STATE_SIZE, STATE_SIZE, float> cov_motion_noise =
{
    2.0f * M_PI / 180.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 2.0f * M_PI / 180.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 2.0f * M_PI / 180.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 2.0f * M_PI / 180.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f,                 0.5f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f,                 1.0f,
};
static const BLA::Matrix<7, 7, float> cov_measurement_noise =
{
    0.5f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 5.69746e-3f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 5.0626e-3f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 9.17358e-3f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.81258423f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.76833002f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.6285494f,
};

void InitializeFilter(
    ICM_20948_I2C* const imu, GPSClass* const gps,
    StateVector* const mu_initial, BLA::Matrix<STATE_SIZE, STATE_SIZE>* const cov_initial)
{
    MeasurementVector measurement;
    // Wait for first measurement
    f32 dt;
    while (!ReadMeasurements(imu, gps, &measurement, &dt));
    // compute initial state
    mu_initial->speed = 0.0f;
    mu_initial->acceleration = BLA::Norm(measurement.acc.vec);
    // rotation state
    measurement.acc.vec /= BLA::Norm(measurement.acc.vec);
    measurement.mag.vec /= BLA::Norm(measurement.mag.vec);
    const BLA::Matrix<3,3> C =
        BLA::CrossProduct(BLA::CrossProduct(measurement.acc.vec, measurement.mag.vec), measurement.acc.vec)
        || BLA::CrossProduct(measurement.acc.vec, measurement.mag.vec)
        || measurement.acc.vec;
    mu_initial->quat.vec = {
                                sqrt(C(0,0) + C(1,1) + C(2,2) + 1),
        sign(C(2,1) - C(1,2)) * sqrt(C(0,0) - C(1,1) - C(2,2) + 1),
        sign(C(0,2) - C(2,0)) * sqrt(C(1,1) - C(0,0) - C(2,2) + 1),
        sign(C(1,0) - C(0,1)) * sqrt(C(2,2) - C(1,1) - C(0,0) + 1),
    };
    mu_initial->quat.vec *= 0.5f;
    (*cov_initial) = BLA::Eye<6,6>();
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
    BLA::Matrix<4,4> scaled_Omega =
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
        f_x->quat.vec     = (BLA::Eye<4,4>() + scaled_Omega) * prior.quat.vec;
        // acceleration
        f_x->acceleration = prior.acceleration;
        // speed
        f_x->speed        = prior.speed + dt * prior.acceleration;
    }
    if (f_jacobian != nullptr)
    {
        (*f_jacobian) = BLA::Eye<STATE_SIZE, STATE_SIZE>();
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
    static constexpr f32 mag_theta = 71.6 * M_PI / 180.0;
    static BLA::Matrix<3> r = {cos(mag_theta), 0, sin(mag_theta)};
    // g in NED-frame
    static const BLA::Matrix<3,1,float> g = {0.0f, 0.0f, -1.0f};
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

void ExtractPartialMeasurement(
    const MeasurementVector& measurement,
    BLA::Matrix<7,1>* const partial_m)
{
    (*partial_m)(0,0) = measurement.speed;
    // normalize acceleration and magnetometer readings
    partial_m->Submatrix<3,1>(1,0) =
        measurement.acc.vec / BLA::Norm(measurement.acc.vec);
    partial_m->Submatrix<3,1>(4,0) =
        measurement.mag.vec / BLA::Norm(measurement.mag.vec);
}
