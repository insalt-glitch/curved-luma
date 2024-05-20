#pragma once

#include "LinearAlgebra.h"

#define i16 int16_t
#define u16 uint16_t
#define u32 uint32_t
#define i32 int32_t
#define f32 float

#define COMPONENT(name, start)   \
struct                           \
{                                \
    f32 _pad[start];             \
    union                        \
    {                            \
        BLA::Matrix<3> vec;      \
        struct { f32 x, y, z; }; \
    };                           \
} name

constexpr u32 STATE_SIZE = 6;
constexpr u32 MEASUREMENT_SIZE = 10;

union EulerAngles
{
    BLA::Matrix<3> vec;
    struct { f32 roll, pitch, yaw; };
};

union Quaternion
{
    BLA::Matrix<4> vec;
    struct { f32 w, x, y, z; };
};

typedef union
{
    BLA::Matrix<STATE_SIZE> vec;
    Quaternion quat;
    struct { f32 _pad[4]; f32 speed; f32 acceleration; };

    // COMPONENT(acc, 5);
    // COMPONENT(gyr, 9);
    // COMPONENT(mag, 12);
    // COMPONENT(acc_bias, 14);
    // COMPONENT(gyr_bias, 17);
    // COMPONENT(mag_bias, 20);
} StateVector;

typedef union
{
    BLA::Matrix<MEASUREMENT_SIZE> vec;
    f32 speed;
    COMPONENT(acc, 1);
    COMPONENT(gyr, 4);
    COMPONENT(mag, 7);
} MeasurementVector;

#undef COMPONENT