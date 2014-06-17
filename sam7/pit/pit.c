#include "pit.h"
#include "bits.h"

/* The PIT has a 20-bit counter clocked at MCLK / 16.  The counter
   counts up until it matches the value in the PIV field of the mode
   register (PIT_MR).  The status bit PITS (in PIT_SR) is then set,
   the counter is reset, and the 12 bit PICNT counter is incremented.
   When PIT_PIVR is read, PIT_SR and PICNT are reset.
   Note if PIT_PIIR is read, PIT_SR and PICNT are not reset.

   PIVR Periodic Interval Value Register
   PIIR Periodic Interval Image Register

   On the SAM4S this is replaced with SysTick a 24 bit counter
   that counts down to zero.
*/

/** The maximum overrun period (s).  */
#define PIT_OVERRUN_PERIOD 100


/** The maximum overrun (in ticks).  */
#define PIT_OVERRUN_MAX ((pit_tick_t)(PIT_OVERRUN_PERIOD * PIT_RATE))


/** The maximum delay (in ticks).  */
#define PIT_DELAY_MAX (~0u - PIT_OVERRUN_MAX + 1)


static AT91S_PITC *pPITC = AT91C_BASE_PITC;
   

void
pit_start (void)
{
    pPITC->PITC_PIMR |= AT91C_PITC_PITEN;        
}


void
pit_stop (void)
{
    pPITC->PITC_PIMR &= ~AT91C_PITC_PITEN;       
}


static uint32_t pit_period_set (uint32_t period)
{
    BITS_INSERT (pPITC->PITC_PIMR, period, 0, 19);

    return period;
}


pit_tick_t pit_get (void)
{
    /* Read the image register (this has no affect on the counters).
       Since the maximum period is selected we can use both the CPIV
       and PICNT fields as a single 32 bit counter.  */
    return pPITC->PITC_PIIR;
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
    pit_period_set (0xfffff);
    pit_start ();
    return 1;
}
