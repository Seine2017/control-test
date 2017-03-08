#ifndef __ERROR_H
#define __ERROR_H

typedef uint8_t error_code_t;

// Error code constants:
// No error.
#define ERROR_NONE ((error_code_t) 0)
// Generic I2C bus error.
#define ERROR_I2C ((error_code_t) 1)
// Sending the "who am I" command to the IMU returned an unexpected response.
#define ERROR_BAD_IMU_ID ((error_code_t) 2)

static void error(error_code_t code, const char *message) {
  //TODO: set up serial connection
  //printf("ERROR %d: %s\n", code, message);
}

#endif
