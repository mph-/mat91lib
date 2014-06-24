/** @file   mcu.c
    @author M. P. Hayes, UCECE
    @date   04 June 2007
    @brief  System routines for AT91SAM7 processors
*/

#include "config.h"
#include "mcu.h"
#include "cpu.h"
#include "irq.h"


void
mcu_reset (void)
{
    /* Reset processor and peripherals.  */
    RSTC->RSTC_RCR = AT91C_RSTC_PROCRST | AT91C_RSTC_PERRST 
        | (0xa5 << 24);
}


/* Place this function in SRAM to avoid problem when switching from
   PLLCK to SLCK.  See errata 39.4.4.2.  */
void
mcu_power_mode_low (void)
    __attribute__ ((section(".ramtext")));


void
mcu_power_mode_low (void)
{
    /* Deactivating the brownout detector saves 20 uA; this requires
       programming of the GPNVM bits.  */
       
    /* Disabling the UDP saves ??? uA.  Connecting the USB port pins
       to ground also saves about 100 uA.  */
    mcu_udp_disable ();

    /* Switch main clock (MCK) from PLLCLK to SLCK.  Note the prescale
       (PRES) and clock source (CSS) fields cannot be changed at the
       same time.  We first switch from the PLLCLK to SLCK then set
       the prescaler to divide by 64. */
    PMC->PMC_MCKR = (PMC->PMC_MCKR & AT91C_PMC_PRES)
        | AT91C_PMC_CSS_SLOW_CLK;

    while (!(PMC->PMC_SR & AT91C_PMC_MCKRDY))
        continue;
    
    /* Set prescaler to divide by 64.  */
    PMC->PMC_MCKR = (PMC->PMC_MCKR & AT91C_PMC_CSS)
        | AT91C_PMC_PRES_CLK_64;

    while (!(PMC->PMC_SR & AT91C_PMC_MCKRDY))
        continue;

    /* Disable PLL.  */
    PMC->PMC_PLLR = 0;

    /* Disable main oscillator.  */
    PMC->PMC_MOR = 0;

    /* Switch voltage regulator to standby (low-power) mode.
       This reduces its static current requirement from 100 uA to 25 uA.  */
    VREG->VREG_MR |= AT91C_VREG_PSTDBY;
}


void
mcu_power_mode_normal (void)
{
    /* Switch voltage regulator to normal mode.  */
    VREG->VREG_MR &= ~AT91C_VREG_PSTDBY;

    mcu_clock_init ();
}


void
mcu_sleep (void)
{
    mcu_power_mode_low ();

    /* Disable processor clock and wait for interrupt.  */
    cpu_idle ();

    mcu_power_mode_normal ();
}


void
mcu_watchdog_reset (void)
{
    *AT91C_WDTC_WDCR = 0xA5000000 | AT91C_WDTC_WDRSTT;
}


void
mcu_watchdog_enable (void)
{
    /* Enable watchdog with 2s timeout.  */
    *AT91C_WDTC_WDMR = AT91C_WDTC_WDD | AT91C_WDTC_WDRSTEN
        | AT91C_WDTC_WDDBGHLT | 0x200;
    mcu_watchdog_reset ();
}


void
mcu_udp_disable (void)
{
    /* The UDP is enabled by default.  To disable the UDP it is
       necessary to turn on the UDP clock, disable the UDP, then turn
       the clock off again.  */
    PMC->PMC_PCER |= (1 << AT91C_ID_UDP);

    UDP->UDP_TXVC |= AT91C_UDP_TXVDIS;

    PMC->PMC_PCDR |= (1 << AT91C_ID_UDP);
}


void
mcu_udp_enable (void)
{
    PMC->PMC_PCER |= (1 << AT91C_ID_UDP);
    UDP->UDP_TXVC &= ~AT91C_UDP_TXVDIS;
}


void mcu_cpu_idle (void)
{
    /* Turn off CPU clock after current instruction.  It will be
       re-enabled when an interrupt occurs.  */
    PMC->PMC_SCDR = AT91C_PMC_PCK;

    while ((PMC->PMC_SCSR & AT91C_PMC_PCK) != AT91C_PMC_PCK)
        continue;
}
