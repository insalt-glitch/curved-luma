#pragma once

#include <Arduino_MKRGPS.h>
#include <ICM_20948.h>

#define i16 int16_t
#define u16 uint16_t
#define u32 uint32_t
#define i32 int32_t
#define f32 float

// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define IMU_I2C_ADRESS 1
typedef icm_20948_DMP_data_t IMUData;

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

typedef union
{
    BLA::Matrix<STATE_SIZE> vec;
    union
    {
        BLA::Matrix<4> vec;
        struct
        {
            f32 w, x, y, z;
        };
    } quat;
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

ICM_20948_I2C IMU; // Create an ICM_20948_I2C object

void initGPS()
{
    if (!GPS.begin(GPS_MODE_SHIELD)) {
        Serial.println("Failed to initialize GPS!");
        // TODO(Nils): Should this be blocking?
        while (true);
    }
    Serial.println("Initialized GPS successfully!");
}

void initIMU()
{
    Wire.begin();
    Wire.setClock(400000);

    while (true)
    {
        // Initialize the IMU (ICM-20948)
        // If the DMP is enabled, .begin performs a minimal startup.
        // We need to configure the sample mode etc. manually.
        IMU.begin(Wire, IMU_I2C_ADRESS);
        if (IMU.status == ICM_20948_Stat_Ok)
        {
            Serial.println("Initialized IMU successfully!");
            break;
        }
        Serial.println("Failed to initialize IMU! Trying again...");
        delay(500);
        continue;
    }
}

void initDMP()
{
    bool success = true; // Use success to show if the DMP configuration was successful
    success &= (IMU.initializeDMP() == ICM_20948_Stat_Ok);
    // DMP sensor options are defined in ICM_20948_DMP.h
    //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
    //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
    //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
    //    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
    //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
    //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
    //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
    //    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
    //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
    //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
    //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
    //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)
    // Enable the DMP accelerometer
    success &= (IMU.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
    // Configuring DMP to output data at multiple ODRs:
    // DMP is capable of outputting multiple sensor data at different rates to FIFO.
    // Setting value can be calculated as follows:
    // Value = (DMP running rate / ODR ) - 1
    // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
    success &= (IMU.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // Set to the maximum

    // Enable the FIFO & DMP then reset
    success &= (IMU.enableFIFO() == ICM_20948_Stat_Ok);
    success &= (IMU.enableDMP() == ICM_20948_Stat_Ok);
    success &= (IMU.resetDMP() == ICM_20948_Stat_Ok);
    success &= (IMU.resetFIFO() == ICM_20948_Stat_Ok);
    // Check success
    if (success)
    {
        Serial.println("Initialized DMP successfully!");
    } else
    {
        Serial.println("Failed to initialize DMP!");
        Serial.println("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h...");
        while (true);
    }
}

f32 ToAcceleration(const ICM_20948_I2C* const IMU, i16 axis_value)
{
  switch (IMU->agmt.fss.a)
  {
  case 0:
    return ((float)axis_value) / 16384.0;
  case 1:
    return ((float)axis_value) / 8192.0;
  case 2:
    return ((float)axis_value) / 4096.0;
  case 3:
    return ((float)axis_value) / 2048.0;
  default:
    return 0;
  }
}

f32 ToAngularVelocity(const ICM_20948_I2C* const IMU, i16 axis_value)
{
    switch (IMU->agmt.fss.g)
    {
        case 0:
            return ((float)axis_value) * PI / (180 * 131);
        case 1:
            return ((float)axis_value) * PI / (180 * 65.5);
        case 2:
            return ((float)axis_value) * PI / (180 * 32.8);
        case 3:
            return ((float)axis_value) * PI / (180 * 16.4);
        default:
            return 0;
    }
}

f32 ToMagneticFieldStrength(const ICM_20948_I2C* const IMU, i16 axis_value)
{
    return ((float)axis_value) * 0.15; // muT
}

f32 ToMetersPerSecond(f32 speed)
{
    return speed / 3.6;
}

void ReadMeasurements(
    const ICM_20948_I2C* const IMU, GPSClass* GPS,
    MeasurementVector* const measurement)
{
    // accelerometer
    measurement->acc.x = ToAcceleration(IMU, IMU->agmt.acc.axes.x);
    measurement->acc.y = ToAcceleration(IMU, IMU->agmt.acc.axes.y);
    measurement->acc.z = ToAcceleration(IMU, IMU->agmt.acc.axes.z);
    // gyroscope
    measurement->gyr.x = ToAngularVelocity(IMU, IMU->agmt.gyr.axes.x);
    measurement->gyr.y = ToAngularVelocity(IMU, IMU->agmt.gyr.axes.y);
    measurement->gyr.z = ToAngularVelocity(IMU, IMU->agmt.gyr.axes.z);
    // magnetometer
    measurement->mag.x = ToMagneticFieldStrength(IMU, IMU->agmt.mag.axes.x);
    measurement->mag.y = ToMagneticFieldStrength(IMU, IMU->agmt.mag.axes.y);
    measurement->mag.z = ToMagneticFieldStrength(IMU, IMU->agmt.mag.axes.z);
    // gps
    if (GPS->available())
    {
        // read GPS values
        measurement->speed = ToMetersPerSecond(GPS->speed());
    } else
    {
        measurement->speed = 0.0;
    }
}

bool readIMU(ICM_20948_I2C* const IMU, IMUData* data)
{
    bool success = false;
    IMU->readDMPdataFromFIFO(data);
    // Was valid data available?
    if ((IMU->status == ICM_20948_Stat_Ok) ||
        (IMU->status == ICM_20948_Stat_FIFOMoreDataAvail)) success = true;
    // TODO(Nils): This should not be in here (but should be done (maybe??))
    // If more data is available then read it right away (no delay)
    // if (IMU.status != ICM_20948_Stat_FIFOMoreDataAvail) delay(10);
    return success;
}

bool extractAcceleration(
    const ICM_20948_I2C* const IMU,
    const IMUData& data,
    BLA::Matrix<3,1>* const acceleration)
{
    if ((data.header & DMP_header_bitmap_Accel) > 0)
    {
        (*acceleration)(0) = ToAcceleration(IMU, data.Raw_Accel.Data.X);
        (*acceleration)(1) = ToAcceleration(IMU, data.Raw_Accel.Data.Y);
        (*acceleration)(2) = ToAcceleration(IMU, data.Raw_Accel.Data.Z);
        return true;
    }
    return false;
}

bool extractQuaternion(
    const IMUData& data,
    BLA::Matrix<4,1>* const quaternion)
{
    if ((data.header & DMP_header_bitmap_Quat9) > 0)
    {
        float q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
        float q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
        float q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
        // double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
        (*quaternion)(1) = q1;
        (*quaternion)(2) = q2;
        (*quaternion)(3) = q3;
        (*quaternion)(0) = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
        return true;
    }
    return false;
}