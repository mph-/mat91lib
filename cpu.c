/** @file   cpu.c
    @author M. P. Hayes, UCECE
    @date   04 June 2007
    @brief  CPU routines for AT91SAM7 processors
*/

#include "config.h"
#include "cpu.h"
#include "irq.h"


void
cpu_idle (void)
{
    /* Turn off CPU clock after current instruction.  It will be
       re-enabled when an interrupt occurs.  */
    AT91C_BASE_PMC->PMC_SCDR = AT91C_PMC_PCK;

    while ((AT91C_BASE_PMC->PMC_SCSR & AT91C_PMC_PCK) != AT91C_PMC_PCK)
        continue;
}


void
cpu_reset (void)
{
    /* Reset processor and peripherals.  */
    AT91C_BASE_RSTC->RSTC_RCR = AT91C_RSTC_PROCRST | AT91C_RSTC_PERRST 
        | (0xa5 << 24);
}


void
cpu_power_mode_low (void)
{
    /* Switch from MCK=48MHz to MCK=32kHz.  */

    // MCK = SLCK/2 : change source first from 48 000 000 to 18. / 2 = 9M
    AT91C_BASE_PMC->PMC_MCKR = AT91C_PMC_PRES_CLK_2;
    while(!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY))
        continue;
    
    // MCK=SLCK : then change prescaler
    AT91C_BASE_PMC->PMC_MCKR = AT91C_PMC_CSS_SLOW_CLK;
    while(!( AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY))
        continue;

    /* Disable PLL.  */
    AT91C_BASE_PMC->PMC_PLLR = 0;

    /* Disable Main Oscillator.  */
    AT91C_BASE_PMC->PMC_MOR = 0;

    /* Voltage regulator in standby mode : Enable VREG Low Power Mode.  */
    AT91C_BASE_VREG->VREG_MR |= AT91C_VREG_PSTDBY;
}


void
cpu_power_mode_normal (void)
{
    // Voltage regulator in normal mode : Disable VREG Low Power Mode
    AT91C_BASE_VREG->VREG_MR &= ~AT91C_VREG_PSTDBY;

    cpu_clock_init ();
}


void
cpu_sleep (void)
{
    cpu_power_mode_low ();

    /* Disable processor clock and wait for interrupt.  */
    cpu_idle ();

    cpu_power_mode_normal ();
}



