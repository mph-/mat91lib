#include "pit.h"
#include "bits.h"

/* SysTick is a 24 bit down counter that resets to the preloaded value
   in the SYST_RVR register.

   We pretend it is an upcounter counter.

   With a 120 MHz clock the longest delay is (1 << 24) / 120e6 = 0.14 s
   corresponding to a lowest frequency of 7.2 Hz.
 */

/** The maximum overrun (in ticks).  */
#define PIT_OVERRUN_TICKS_MAX 4000000


#define PIT_PERIOD_TICKS_MAX 0x00ffffffu


/** The maximum delay (in ticks).  */
#define PIT_DELAY_TICKS_MAX (PIT_PERIOD_TICKS_MAX - PIT_OVERRUN_TICKS_MAX + 1)


void
pit_start (void)
{
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}


void
pit_stop (void)
{
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}


static uint32_t pit_period_set (uint32_t period)
{
    SysTick->LOAD = period;

    return period;
}


pit_tick_t pit_get (void)
{
    /* Pretend the counter counts up.  */
    return SysTick->LOAD - SysTick->VAL;
}


/** Wait until specified time:
    @param when time to sleep until
    @return current time.  */
pit_tick_t pit_wait_until (pit_tick_t when)
{
    when <<= 8;

    while (1)
    {
        pit_tick_t diff;
        pit_tick_t now;

        now = pit_get () << 8;

        diff = now - when;

        if (diff < (PIT_OVERRUN_TICKS_MAX << 8))
            return now;
    }
}


/** Wait for specified period:
    @param period how long to wait
    @return current time.  */
pit_tick_t pit_wait (pit_tick_t period)
{
    return pit_wait_until (pit_get () + period);
}


int
pit_init (void)
{
    /* Set maximum period.  */
    pit_period_set (PIT_PERIOD_TICKS_MAX);
    pit_start ();
    return 1;
}
