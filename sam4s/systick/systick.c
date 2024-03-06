/** @file   systick.c
    @author M. P. Hayes, UCECE
    @date   26 December 2023
    @brief
*/

#include "systick.h"
#include "bits.h"

/* SysTick is a 24 bit down counter that resets to the preloaded value
   in the SYST_RVR register.

   We pretend it is an upcounter.   It is incremented at the
   system clock rate, F_CPU.

   With a 120 MHz clock the longest delay is (1 << 24) / 120e6 = 0.14 s
   corresponding to a lowest frequency of 7.2 Hz.
 */

void
systick_start (void)
{
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}


void
systick_stop (void)
{
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}


void
systick_period_set (systick_clocks_t period)
{
    SysTick->LOAD = period - 1;
}


systick_clocks_t
systick_period_get (void)
{
    return SysTick->LOAD + 1;
}


systick_clocks_t
systick_clocks_get (void)
{
    /* Pretend the counter counts up.  */
    return SysTick->LOAD - SysTick->VAL;
}


int
systick_init (systick_clocks_t period)
{
    systick_period_set (period);
    // Reset the counter
    SysTick->VAL = 0;
    systick_start ();
    return 1;
}
