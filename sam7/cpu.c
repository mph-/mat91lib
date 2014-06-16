/** @file   cpu.c
    @author M. P. Hayes, UCECE
    @date   04 June 2007
    @brief  CPU routines for AT91SAM7 processors
*/

#include "config.h"
#include "cpu.h"
#include "irq.h"


void cpu_idle (void)
{
    /* Turn off CPU clock after current instruction.  It will be
       re-enabled when an interrupt occurs.  */
    PMC->PMC_SCDR = AT91C_PMC_PCK;

    while ((PMC->PMC_SCSR & AT91C_PMC_PCK) != AT91C_PMC_PCK)
        continue;
}


void
cpu_reset (void)
{
    /* Reset processor and peripherals.  */
    RSTC->RSTC_RCR = AT91C_RSTC_PROCRST | AT91C_RSTC_PERRST 
        | (0xa5 << 24);
}
