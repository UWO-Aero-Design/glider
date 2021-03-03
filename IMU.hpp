#pragma once
#include "config.hpp"

// Include for sensor interfaces; imu needed
#include "src/ICM20948/Teensy-ICM-20948.h"

class IMU {

  public:

    int heading = 0;
    float acceleration = 0;




    IMU(){

    }

    ~IMU(){


    }

    bool init(TwoWire &wirePort) {
      icm20948.init(icmSettings, wirePort);
      m_initialized = true;
      return m_initialized;
    }


    bool update(float *accel, float *gyro, float *quat) {
      // If system is not initialized, return error
      if (!m_initialized) {
        return false;
      }


      icm20948.task();

      if (icm20948.gyroDataIsReady())
      {
        icm20948.readGyroData(&gyro[0], &gyro[1], &gyro[2]);
      }

      if (icm20948.accelDataIsReady())
      {
        icm20948.readAccelData(&accel[0], &accel[1], &accel[2]);
      }

      if (icm20948.quatDataIsReady())
      {
        icm20948.readQuatData(&quat[0], &quat[2], &quat[1], &quat[3]);
        quat[3] *= -1.f;
      }

      return true;
    }


  private:
    bool m_initialized = false;

    TeensyICM20948 icm20948;

    TeensyICM20948Settings icmSettings =
    {
      .mode = 1,                     // 0 = low power mode, 1 = high performance mode
      .enable_gyroscope = true,      // Enables gyroscope output
      .enable_accelerometer = true,  // Enables accelerometer output
      .enable_magnetometer = true,   // Enables magnetometer output
      .enable_quaternion = true,     // Enables quaternion output
      .gyroscope_frequency = 200,      // Max frequency = 225, min frequency = 1
      .accelerometer_frequency = 200,  // Max frequency = 225, min frequency = 1
      .magnetometer_frequency = 70,   // Max frequency = 70, min frequency = 1
      .quaternion_frequency = 200     // Max frequency = 225, min frequency = 50
    };

};
