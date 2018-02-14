/** @file   pit.h
    @author M. P. Hayes, UCECE
    @date   3 June 2007
    @brief 
*/
#ifndef PIT_H
#define PIT_H

#ifdef __cplusplus
extern "C" {
#endif
    

#include "config.h"

#define PIT_CLOCK_DIVISOR 1

/** Rate in Hz that the pit is incremented.  */
#define PIT_RATE (F_CPU / PIT_CLOCK_DIVISOR)


/** Define pit ticks.  */
typedef uint32_t pit_tick_t;


/** Get current time:
    @return current time in ticks.  */
pit_tick_t pit_get (void);


/** Wait until specified time:
    @param when time to sleep until
    @return current time.  */
pit_tick_t pit_wait_until (pit_tick_t when);


/** Wait for specified period:
    @param period how long to wait
    @return current time.  */
pit_tick_t pit_wait (pit_tick_t period);


/** Initialise pit.  */
int pit_init (void);


#ifdef __cplusplus
}
#endif    
#endif

