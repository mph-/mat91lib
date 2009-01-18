/** @file   pacer.h
    @author M. P. Hayes, UCECE
    @date   3 June 2007
    @brief 
*/
#ifndef PACER_H
#define PACER_H

#include "config.h"
#include "pit.h"

#define PACER_RATE_MIN (F_CPU / 16 / (1 << 20))

/** Initialise paced loop timer.  This macro should only be used
    externally for non-integer paced loop rates.  */
#define PACER_INIT(RATE) \
    do {pit_init (); \
        pit_period_set (PIT_DIVISOR (RATE)); \
        pit_start ();} while (0)

/** Wait for next paced loop tick.  */
#define PACER_WAIT() while (!PACER_READY_P ()) continue

#define PACER_READY_P() PIT_COMPARE_P ()

#define PACER_RESET() PIT_RESET()

#define PACER_INT_ENABLE() PIT_INT_ENABLE()

#define PACER_INT_DISABLE() PIT_INT_DISABLE()


#if 0
#define PACER_SLEEP() \
do {set_sleep_mode (SLEEP_MODE_IDLE); sleep_mode ();} while (0)
#endif


/** Initialise paced loop timer.
    @param rate paced loop rate (Hz).  */
static inline void
pacer_init (uint32_t rate)
{
    PACER_INIT (rate);
}


/** Wait for next paced loop tick.  */
static inline void
pacer_wait (void)
{
    /* Wait for next clock tick.  */
    PACER_WAIT ();
}
#endif
