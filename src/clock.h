// clock.h declares the functions and variables defined in clock.c.

// clock.c contains routines to manage a monotonic time source; in other words,
// a way of measuring how much time has passed since the processor was switched
// on. This is used by the control algorithm to perform calculus operations
// (integration and differentiation).

#ifndef __CLOCK_H
#define __CLOCK_H

#include <stdint.h>
#include <avr/io.h>

#include "settings.h"

// A type representing the number of clock 'ticks' since the processor was last
// reset or powered on.
typedef uint16_t clock_time_t;

// A type representing a time interval (the number of 'ticks' between two
// distinct time measurements).
typedef int16_t clock_interval_t;

// The actual number of ticks that have passed since power on.
extern volatile clock_time_t clock_now;

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega168P__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
  #define CLOCK_TIMER_INTERRUPT TIMER1_COMPA_vect
  static void clock_init_timer() {
    // Set up timer 1:
    //   COM1A = 00 (OC1A disconnected)
    //   COM1B = 00 (OC1B disconnected)
    //   WGM1 = 0100 (clear timer on compare match)
    //   CS1 = 001 (no prescaler)
    //   OCIE1A = 1 (interrupt on output compare match A)
    //   OCIE1B = 0 (no interrupt on output compare match B)
    //   TOIE1 = 0 (no interrupt on timer overflow)
    TCCR1A = 0;
    TCCR1B = _BV(WGM12) | _BV(CS10);
    TCCR1C = 0;
    TIMSK1 = _BV(OCIE1A);

    //    F_OVERFLOW = F_CPU / (PRESCALER * (1 + OCR1A))
    // => OCR1A = F_CPU / (PRESCALER * F_OVERFLOW) - 1
    OCR1A = (uint16_t) (((uint32_t) F_CPU) / (((uint32_t) 1) * ((uint32_t) CLOCK_TICKS_PER_SECOND)) - 1);
  }
#elif defined(__AVR_ATmega32U4__)
  #define CLOCK_TIMER_INTERRUPT TIMER1_COMPA_vect
  static void clock_init_timer() {
    // Set up timer 1:
    //   COM1A = 00 (OC1A disconnected)
    //   COM1B = 00 (OC1B disconnected)
    //   COM1C = 00 (OC1C disconnected)
    //   WGM1 = 0100 (clear timer on compare match)
    //   CS1 = 001 (no prescaler)
    //   ICIE1 = 0 (no interrupt on input capture)
    //   OCIE1A = 1 (interrupt on output compare match A)
    //   OCIE1B = 0 (no interrupt on output compare match B)
    //   OCIE1C = 0 (no interrupt on output compare match C)
    //   TOIE1 = 0 (no interrupt on timer overflow)
    TCCR1A = 0;
    TCCR1B = _BV(WGM12) | _BV(CS10);
    TCCR1C = 0;
    TIMSK1 = _BV(OCIE1A);

    //    F_OVERFLOW = F_CPU / (PRESCALER * (1 + OCR1A))
    // => OCR1A = F_CPU / (PRESCALER * F_OVERFLOW) - 1
    OCR1A = (uint16_t) (((uint32_t) F_CPU) / (((uint32_t) 1) * ((uint32_t) CLOCK_TICKS_PER_SECOND)) - 1);
  }
#else
  #error "Don't know how to set up timer on the target device. Please update clock.h."
#endif

// Initialise the clock.
static void clock_init() {
  clock_init_timer();
}

// Check the current time.
static clock_time_t clock_get_time() {
  return clock_now;
}

// Find the interval of time that passed between two distinct time measurements.
static clock_interval_t clock_diff(clock_time_t start, clock_time_t end) {
  return (clock_interval_t) (end - start);
}

#endif
