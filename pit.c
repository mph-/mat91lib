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
*/


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


bool
pit_compare_p (void)
{
    return PIT_COMPARE_P ();
}


uint32_t pit_period_set (uint32_t period)
{
    BITS_INSERT (pPITC->PITC_PIMR, period, 0, 19);

    return period;
}


int8_t
pit_init (void)
{
    return 1;
}
