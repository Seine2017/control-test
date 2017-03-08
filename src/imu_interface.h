// This file specifies the interface between the IMU interfacing code (written
// by Pawel) and the control code (written by Kier).
// In this file are declared all the necesary function for interfacing
// with the IMU.

#ifndef __IMU_INTERFACE_H
#define __IMU_INTERFACE_H

// Define macros for the successful or failed communication with IMU
#define SUCCESS 0
#define ERROR 1

// A structure to hold all of the data that 'imu_read' retrieves.
// Definition of axes in the body coordinate system:
//   x axis: points directly between rotors A and B ("forwards")
//   y axis: points in the direction of thrust ("upwards")
//   z axis: points directly between rotors B and C ("to the right")
typedef struct {
  // Velocity along the world frame Z axis. Positive indicates upwards (away
  // from the ground). Units are metres/second.
  float z_vel;

  // Roll angle (rotation about the body frame X axis). Positive indicates
  // rolling to the right (the right-hand side of the vehicle is lower than the
  // left-hand side). Units are radians.
  float roll;

  // Pitch angle (rotation about the body frame Y axis). Positive indicates
  // pitching downwards (the front of the vehicle is lower than the rear). Units
  // are radians.
  float pitch;

  // Yaw velocity (angular velocity about the body frame Z axis). Positive
  // indicates counterclockwise rotation when the vehicle is viewed from above.
  // Units are radians/second.
  float yaw_vel;
} measured_state_t;

// Variables
//extern float pitch, roll, pitch_accel, roll_accel;

//uint8_t write_IMU_byte(uint8_t IMU_address, uint8_t address, uint8_t data_8bit);
//uint8_t read_IMU_byte(uint8_t IMU_address, uint8_t address, uint8_t *data_8bit);
//uint8_t read_IMU_bytes(uint8_t IMU_address, uint8_t address, uint8_t count, uint8_t * data);
//void read_IMU_id(void);
//void et_IMU_scales(void);
//void read_raw_gyro(uint16_t *gyro);
//void read_raw_accel(uint16_t *accel);
//void calibrate_IMU(void);
//void reset_IMU();

// Initialise the IMU.
void imu_init();

// Read data from the accelerometer, gyroscope and possibly magnetometer, filter
// and process the data, and store the results into the structure pointed to by
// 'destination'.
void imu_read(measured_state_t *destination);

#endif
