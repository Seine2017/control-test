#ifndef __SETTINGS_H
#define __SETTINGS_H

#include <avr/io.h>

// The frequency at which system clock ticks occur.
// Hence, each tick represents 100 microseconds.
#define CLOCK_TICKS_PER_SECOND 10000
#define SECONDS_PER_CLOCK_TICK (1.0 / CLOCK_TICKS_PER_SECOND)

// The frequency/period at which software PWM ticks occur.
#define US_PER_PWM_TICK 10
#define US_PER_PWM_REP 20000
#define PWM_TICKS_PER_REP (US_PER_PWM_REP / US_PER_PWM_TICK)
#define PWM_TICKS_PER_SECOND (1000000 / US_PER_PWM_TICK)
#define PWM_TICKS_PER_MS (1000 / US_PER_PWM_TICK)

// Z-factor PID parameters.
#define PID_GAIN_Z_P 0
#define PID_GAIN_Z_I 0
#define PID_GAIN_Z_D 0

// Roll-factor PID parameters.
#define PID_GAIN_ROLL_P 0
#define PID_GAIN_ROLL_I 0
#define PID_GAIN_ROLL_D 0

// Pitch-factor PID parameters.
#define PID_GAIN_PITCH_P 0
#define PID_GAIN_PITCH_I 0
#define PID_GAIN_PITCH_D 0

// Yaw-factor PID parameters.
#define PID_GAIN_YAW_P 0
#define PID_GAIN_YAW_I 0
#define PID_GAIN_YAW_D 0

// IMU sensor sensitivities.
#define GYRO_SENSITIVITY 65.5    // = 131 LSB/degrees/sec
#define ACCEL_SENSITIVITY 4096 // = 16384 LSB/g

// Allowable limits for rotor speeds (0.0 = 1ms pulse, 1.0 = 2ms pulse).
#define MIN_ROTOR_SPEED 0.0
#define MAX_ROTOR_SPEED 0.7

// Rotor speeds are filtered to reduce transients. The higher this parameter is,
// the more filtering is applied (but the slower the rotors are to respond to
// changes).
// Range: 0.0 to 1.0
#define ROTOR_SPEED_FILTERING 0.9

// Specification of which pins on the microcontroller are connected to the PWM
// inputs on the ESCS.
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega168P__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
  // Arduino Uno (or compatible).
  #define ESCS_DDR DDRD
  #define ESCS_PORT PORTD
  #define ESCS_BIT_A PD7 // Arduino pin 7
  #define ESCS_BIT_B PD6 // Arduino pin 6
  #define ESCS_BIT_C PD5 // Arduino pin 5
  #define ESCS_BIT_D PD4 // Arduino pin 4
#elif defined(__AVR_ATmega32U4__)
  // Arduino Pro Micro.
  #define ESCS_DDR DDRF
  #define ESCS_PORT PORTF
  #define ESCS_BIT_A PF7 // Arduino pin A0
  #define ESCS_BIT_B PF6 // Arduino pin A1
  #define ESCS_BIT_C PF5 // Arduino pin A2
  #define ESCS_BIT_D PF4 // Arduino pin A3
#else
  #error "Don't know how to set up pin definitions on the target device. Please update settings.h."
#endif


#endif
