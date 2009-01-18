/** @file   tick.c
    @author M. P. Hayes, UCECE
    @date   15 May 2007
    @brief 
*/
#include "pit.h"
#include "tick.h"

void
tick_init (uint32_t divisor)
{
    /* Initialise timer 2 for paced loop.  */
    pit_init ();
    pit_period_set (divisor);
    pit_start ();
}
