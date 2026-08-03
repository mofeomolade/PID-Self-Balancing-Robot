#ifndef MPU6050_DRIVER_H
#define MPU6050_DRIVER_H

#include <Arduino.h>
#include <Wire.h>

#define MAIN_REGISTER 0x68
#define WAKE_REGISTER 0x6B
#define ACCEL_START_REGISTER 0x3B
#define RAD_TO_DEGREES 180.0 / PI

struct IMU {
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
};

struct Offset {
  float offset_accel_x, offset_accel_y, offset_accel_z;
  float offset_gyro_x, offset_gyro_y, offset_gyro_z;
};

struct Attitude {
  float pitch;
  float roll;
  float yaw;
};

// Function Declarations
int write_burst(uint8_t dev_address, uint8_t reg_address, const uint8_t *data, size_t length);
int write_byte(uint8_t dev_address, uint8_t reg_address, const uint8_t *data);
int read_burst(uint8_t dev_address, uint8_t reg_address, size_t length, uint8_t *buffer);
int read_byte(uint8_t dev_address, uint8_t reg_address, uint8_t *buffer);

int read_IMU(IMU &data);
void offset_test(IMU &data, Offset &calibration);
void correct_IMU(IMU &data, Offset &calibration);
void filter_IMU(IMU &data, Attitude &orientation, float dt);

#endif