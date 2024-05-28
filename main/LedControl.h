#pragma once

#include "LinearAlgebra.h"
#include "Helpers.h"

#define i16 int16_t
#define u16 uint16_t
#define u32 uint32_t
#define i32 int32_t
#define f32 float

namespace LED
{
// Pins:
// 0...5   -> column
// 17...20 -> row
static const BLA::Matrix<10> LED_PINS{0,1,2,3,4,5,17,18,19,20};

static void PowerOff()
{
    for (u32 i = 0; i < LED_PINS.Rows; i++) digitalWrite(LED_PINS(i), LOW);
}

static void InitMatrix()
{
    for (u32 i = 0; i < LED_PINS.Rows; i++)
    {
        pinMode(LED_PINS(i), OUTPUT);
        digitalWrite(LED_PINS(i), LOW);
    }
}

static void SetSegmentState(const int pin_status, const u32 row_index, const u32 column_index)
{
    digitalWrite(column_index         , pin_status);
    digitalWrite(column_index      + 1, pin_status);
    digitalWrite(row_index    + 17    , pin_status);
    digitalWrite(row_index    + 17 + 1, pin_status);
}

static void SetMatrixState(const u32 row_index, const u32 column_index)
// (0,0) == (top, left)
// (-1,-1) == OFF
{
    static u32 prev_row_index    = -1;
    static u32 prev_column_index = -1;
    Assert(row_index < 3 || row_index == -1, "LED-matrix row-index invalid.");
    Assert(column_index < 5 || row_index == -1, "LED-matrix column-index invalid.");
    Assert(!((row_index == -1) ^ (column_index == -1)), "LED-matrix only one index marked as disabled.");

    if (prev_row_index != -1) PowerOff();
    if (     row_index != -1) SetSegmentState(HIGH,      row_index,      column_index);
    prev_row_index    = row_index;
    prev_column_index = column_index;
}

static f32 ComputeSteeringAngle(const f32 velocity, const f32 roll)
{
    Assert(velocity >= 0, "Velocity should be a magnitude (v > 0).");
    static constexpr f32 g = 10.0f;
    static constexpr f32 assumed_bicycle_length = 1.5f;
    // radius of the hypothetical curve that the bicycle is taking
    const f32 curve_radius =  - velocity * velocity / (roll * g);
    // steering angle of the bicycle
    const f32 steering_angle = atan(assumed_bicycle_length / curve_radius) / 2;
    return steering_angle;
}

static u32 ComputeCorrectSegment(
    const f32 value, const u32 prev_index,
    const f32 region_width, const f32 hysteresis)
{
    Assert(value >= 0.0f, "Value cannot be negative");
    const u32 current_index = static_cast<u32>(value / region_width);
    if ((prev_index == -1) ||         // was not initialized before
        (prev_index == current_index) // segment has not changed
        ) return current_index;
    const u32 segment_distance = abs(current_index - prev_index);
    if (segment_distance > 1) return current_index;
    const f32 normalized_hysteresis = hysteresis / region_width;
    const f32 normalized_value = value / region_width;
    const f32 normalized_distance = abs(normalized_value - round(normalized_value));
    if (normalized_distance > normalized_hysteresis) return current_index;
    return prev_index;
}

static void UpdateMatrix(const f32 velocity, const f32 roll)
{
    static constexpr f32 critical_velocity = 9.0f / 3.6f;  // m/s
    static constexpr f32 critical_angle    = DegToRad(13.0) / 1.2f;
    static constexpr f32 hysteresis_velocity = (2.0f / 3.6f);
    static constexpr f32 hysteresis_angle    = DegToRad(3.0f);
    static u32 row_index    = -1;
    static u32 column_index = -1;
    // steering angle of the bicycle
    const f32 steering_angle = ComputeSteeringAngle(velocity, roll);
    // clamped quantities for segement calculations
    const f32 limited_velocity = Clamp(velocity, 0.0f, 2.9f * critical_velocity);
    const f32 shifted_angle = Clamp(2.5f * critical_angle + steering_angle, 0.0f, 4.9f * critical_angle);
    // compute spegement position
    row_index    = ComputeCorrectSegment(limited_velocity, row_index, critical_velocity, hysteresis_velocity);
    column_index = ComputeCorrectSegment(shifted_angle, column_index, critical_angle, hysteresis_angle);
    SetMatrixState(row_index, column_index);
}

}  // namespace LED
