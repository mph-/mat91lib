/** @file   tick.h
    @author M. P. Hayes, UCECE
    @date   3 June 2007
    @brief 
*/
#ifndef TICK_H
#define TICK_H

#ifdef __cplusplus
extern "C" {
#endif
    

#include "config.h"
#include "pit.h"

#define TICK_RATE_MIN (F_CPU / 16 / (1 << 20))

/* This macro is used to avoid run-time division.  */
#define TICK_DIVISOR(FREQ) PIT_DIVISOR (FREQ)

#define TICK_READY_P() PIT_COMPARE_P ()

#define TICK_WAIT() while (!TICK_READY_P ()) continue

#define TICK_RESET() PIT_RESET()

#define TICK_INT_ENABLE() PIT_INT_ENABLE()

#define TICK_INT_DISABLE() PIT_INT_DISABLE()

//#define TICK_INTERRUPT SIG_OUTPUT_COMPARE2


#if 0
#define TICK_SLEEP() \
do {set_sleep_mode (SLEEP_MODE_IDLE); sleep_mode ();} while (0)
#endif

extern void
tick_init (uint32_t divisor);


static inline void
tick_wait (void)
{
    /* Wait for next clock tick.  */
    TICK_WAIT ();
}

#ifdef __cplusplus
}
#endif    
#endif

