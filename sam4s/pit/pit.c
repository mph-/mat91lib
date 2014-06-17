#include "pit.h"
#include "bits.h"

/* SysTick is a 24 bit down counter that resets
   to the preloaded value in the SYST_RVR register.  */

/** The maximum overrun period (s).  */
#define PIT_OVERRUN_PERIOD 100


/** The maximum overrun (in ticks).  */
#define PIT_OVERRUN_MAX ((pit_tick_t)(PIT_OVERRUN_PERIOD * PIT_RATE))


/** The maximum delay (in ticks).  */
#define PIT_DELAY_MAX (~0u - PIT_OVERRUN_MAX + 1)


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
    if (period >= (1 << 24))
        return 0;

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
    while (1)
    {
        pit_tick_t diff;
        pit_tick_t now;
        
        now = pit_get ();
        
        diff = now - when;

        if (diff < PIT_OVERRUN_MAX)
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
    pit_period_set (0xffffff);
    pit_start ();
    return 1;
}
