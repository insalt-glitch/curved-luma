#pragma once

#include "LinearAlgebra.h"

#define i16 int16_t
#define u16 uint16_t
#define u32 uint32_t
#define i32 int32_t
#define f32 float

// Pins:
// 0...5   -> column
// 17...20 -> row
static const BLA::Matrix<10> LED_PINS{0,1,2,3,4,5,17,18,19,20};

static void ConfigurePins()
{
    for (u32 i = 0; i < LED_PINS.Rows; i++)
    {
        pinMode(LED_PINS(i), OUTPUT);
        digitalWrite(LED_PINS(i), LOW);
    }
}

static void LedSegmentState(const int pin_status, const u32 row_index, const u32 column_index)
{
    digitalWrite(column_index         , pin_status);
    digitalWrite(column_index      + 1, pin_status);
    digitalWrite(row_index    + 17    , pin_status);
    digitalWrite(row_index    + 17 + 1, pin_status);
}

static void LedMatrixState(const u32 row_index, const u32 column_index)
// (0,0) == (top, left)
// (-1,-1) == OFF
{
    static u32 prev_row_index    = -1;
    static u32 prev_column_index = -1;
    // assert(row_index < 4 || row_index == -1); -> Too high
    // assert(column_index < 6 || row_index == -1); -> Too high
    // assert(!((row_index == -1) ^ (column_index == -1))) Only one is off -> invalid

    if ((row_index == prev_row_index) && (column_index == prev_column_index)) return;
    if (prev_row_index != -1) LedSegmentState(LOW , prev_row_index, prev_column_index);
    if (     row_index != -1) LedSegmentState(HIGH,      row_index,      column_index);
    prev_row_index    = row_index;
    prev_column_index = column_index;
}

static f32 ClampZero(const f32 x) { return (x < 0.0f) ? 0.0f : x; }

static void UpdateLedActivation(f32 velocity, f32 roll)
// hysteresis
// which angles??
// which velocity
// make it such that the roll is 0 if the bike goes straight
{
    // assert(velocity >= 0)
    static constexpr f32 g = 10.076878519224737;
    static constexpr f32 assumed_bicycle_length = 1.5f;
    static constexpr f32 critical_velocity = 9.0f;
    static constexpr f32 critical_angle = 13.0 * M_PI / 180.0;
    // radius of the hypothetical curve that the bicycle is taking
    const f32 curve_radius =  velocity * velocity / (roll * g);
    // steering angle of the bicycle
    const f32 delta_angle = atan(assumed_bicycle_length / curve_radius) / 2;
    const u32 row_index = (u32)(velocity / critical_velocity);
    const u32 column_index = (u32)(ClampZero(2.0f * critical_angle + delta_angle) / critical_angle);
    LedMatrixState(row_index, column_index);
}
