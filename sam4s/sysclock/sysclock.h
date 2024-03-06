/** @file   sysclock.h
    @author M. P. Hayes, UCECE
    @date   26 December 2023
    @brief
*/
#ifndef SYSCLOCK_H
#define SYSCLOCK_H

#ifdef __cplusplus
extern "C" {
#endif


#include "config.h"

// Number of clocks per millisecond.
#define SYSCLOCK_MS_CLOCKS ((int)(F_CPU * 1e-3))

// Number of clocks per microsecond.
#define SYSCLOCK_US_CLOCKS ((int)(F_CPU * 1e-6))

// This rolls over every 4874 years at 120 MHz (32 bits rolls over
// every 36 s at 120 MHz)
typedef uint64_t sysclock_clocks_t;

typedef void (*sysclock_callback_t) (void);


sysclock_clocks_t sysclock_clocks (void);


uint32_t sysclock_millis (void);


uint32_t sysclock_micros (void);


void sysclock_millis_delay (uint32_t delay);


bool sysclock_millis_elapsed (sysclock_clocks_t from, uint32_t delay);


bool sysclock_micros_elapsed (sysclock_clocks_t from, uint32_t delay);


void sysclock_callback (sysclock_callback_t callback);


/** Initialise sysclock.  */
int sysclock_init (void);


#ifdef __cplusplus
}
#endif
#endif
