/** @file   cpu.c
    @author M. P. Hayes, UCECE
    @date   04 June 2007
    @brief  CPU routines for SAM4S processors
*/

#include "config.h"
#include "cpu.h"
#include "irq.h"


void cpu_idle (void)
{
    /* Turn off CPU clock after current instruction.  It will be
       re-enabled when an interrupt occurs.  */
    PMC->PMC_SCDR = PMC_PCK;

    while ((PMC->PMC_SCSR & PMC_PCK) != PMC_PCK)
        continue;
}


void
cpu_reset (void)
{
    /* Reset processor and peripherals.  */
    RSTC->RSTC_RCR = RSTC_PROCRST | RSTC_PERRST 
        | (0xa5 << 24);
}
