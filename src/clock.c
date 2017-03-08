// clock.c contains routines to manage a monotonic time source; in other words,
// a way of measuring how much time has passed since the processor was switched
// on. This is used by the control algorithm to perform calculus operations
// (integration and differentiation).

#include <avr/interrupt.h>

#include "clock.h"

// The actual number of ticks that have passed since power on.
volatile clock_time_t clock_now = 0;

// Interrupt vector for hardware timer overflow.
ISR(TIMER0_COMPA_vect) {
  clock_now++;
}
