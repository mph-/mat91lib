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
    AT91C_BASE_UDP->UDP_TXVC |= AT91C_UDP_TXVDIS;  // Disable UDP transceiver

    /* Switch main clock (MCK) from PLLCLK to SLCK.  Note the prescale
       (PRES) and clock source (CSS) fields cannot be changed at the
       same time.  */
    AT91C_BASE_PMC->PMC_MCKR = (AT91C_BASE_PMC->PMC_MCKR & AT91C_PMC_PRES)
        | AT91C_PMC_CSS_SLOW_CLK;

    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY))
        continue;
    
    /* Set prescaler to divide by 64.  */
    AT91C_BASE_PMC->PMC_MCKR = (AT91C_BASE_PMC->PMC_MCKR & AT91C_PMC_CSS)
        | AT91C_PMC_PRES_CLK_64;

    while (!( AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY))
        continue;

    /* Disable PLL.  */
    AT91C_BASE_PMC->PMC_PLLR = 0;

    /* Disable main oscillator.  */
    AT91C_BASE_PMC->PMC_MOR = 0;

    /* Switch voltage regulator to standby (low-power) mode.
       This reduces its static current requirement from 100 uA to 25 uA.  */
    AT91C_BASE_VREG->VREG_MR |= AT91C_VREG_PSTDBY;
}


void
cpu_power_mode_normal (void)
{
    /* Switch voltage regulator to normal mode.  */
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


void
cpu_watchdog_reset (void)
{
    *AT91C_WDTC_WDCR = 0xA5000000 | AT91C_WDTC_WDRSTT;
}


void
cpu_watchdog_enable (void)
{
    /* Enable watchdog with 2s timeout.  */
    *AT91C_WDTC_WDMR = AT91C_WDTC_WDD | AT91C_WDTC_WDRSTEN
        | AT91C_WDTC_WDDBGHLT | 0x200;
    cpu_watchdog_reset ();
}


void
cpu_udp_disable (void)
{
    /* The UDP is enabled by default.  */
    AT91C_BASE_UDP->UDP_TXVC |= AT91C_UDP_TXVDIS;
}


void
cpu_udp_enable (void)
{
    AT91C_BASE_UDP->UDP_TXVC &= ~AT91C_UDP_TXVDIS;
}
