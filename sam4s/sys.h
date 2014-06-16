/** @file   sys.h
    @author M. P. Hayes, UCECE
    @date   13 July 2013
    @brief  System routines for SAM4S processors
*/
#ifndef SYS_H
#define SYS_H

#include "config.h"
#include "irq.h"
#include "bits.h"
#include "cpu.h"


#define SYS_FLASH_SPEED 30e6

#ifndef SYS_FLASH_READ_CYCLES 
#define SYS_FLASH_READ_CYCLES 2
// #define SYS_FLASH_READ_CYCLES ((int) ((F_CPU + SYS_FLASH_SPEED - 1) / SYS_FLASH_SPEED))
#endif


#ifdef CPU_PLL_DIV
#define SYS_PLL_DIV CPU_PLL_DIV
#endif

#ifdef CPU_PLL_MUL
#define SYS_PLL_MUL CPU_PLL_MUL
#endif


/* The AT91 Flash is single plane so it is not possible
   to write to it while executing code out of it.  */

/** Initialise flash memory controller.  */
static void
sys_flash_init (void)
{
    switch (SYS_FLASH_READ_CYCLES)
    {
    case 1:
        /* Set 0 flash wait states for reading, 1 for writing.  */
        MC->MC_FMR = MC_FWS_0FWS;
        break;

    case 2:
        /* Set 1 flash wait state for reading, 2 for writing.  */
        MC->MC_FMR = MC_FWS_1FWS;
        break;

    case 3:
        /* Set 2 flash wait states for reading, 3 for writing.  */
        MC->MC_FMR = MC_FWS_2FWS;
        break;

    default:
        /* Set 3 flash wait states for reading, 4 for writing.  */
        MC->MC_FMR = MC_FWS_3FWS;
        break;
    }

    /* Set number of MCK cycles per microsecond for the Flash
       microsecond cycle number (FMCN) field of the Flash mode
       register (FMR).  */
    BITS_INSERT (MC->MC_FMR, (uint16_t) (F_CPU / 1e6), 16, 23);
}


/* Internal slow clock frequency.  */
#define F_SLCK 32768

#define SYS_OS_DELAY 1.5e-3
#define SYS_OS_COUNT ((uint16_t) (SYS_OS_DELAY * F_SLCK + 7)) / 8

#define SYS_PLL_DELAY 0.9e-3
#define SYS_PLL_COUNT (uint16_t) (SYS_PLL_DELAY * F_SLCK)

#define SYS_USB_LOG2_DIV 0

/* The PLL frequency is given by (F_XTAL * SYS_PLL_MUL) / SYS_PLL_DIV.
   This is then divided by the prescaler (assumed 2) for MCK.  */

/** Set up the main clock (MAINCK), PLL clock, and master clock (MCK).   */
static inline void
sys_clock_init (void)
{
    /* To minimize the power required to start up the system, the main
       oscillator is disabled after reset and slow clock is
       selected. 

       There are three clock sources: SLCK (the RC oscillator slow
       clock), MAINCK (the external crystal main clock), and PLLCK (the
       output of the phase locked loop driven by MAINCK).  One of
       these three clock sources can be fed to a prescaler (with
       divisors 2^0 ... 2^6) to drive MCK (master clock).
       
       The main oscillator (external crystal) can range from 3--20 MHz.
       The PLL frequency can range from 80--220 MHz.
    */

    /* Enable the MAINCK oscillator and wait for it to start up.  The
       start delay is SYS_OS_COUNT * 8 SLCK cycles.  */
    PMC->PMC_MOR = BITS (SYS_OS_COUNT, 8, 15) | CKGR_MOSCEN;
    
#ifndef SIM_RUN
    /*  Wait for the oscillator to start up.  */
    while (!(PMC->PMC_SR & PMC_MOSCS))
        continue;
#endif
    
    /* The PLL start delay is SYS_PLL_COUNT SLCK cycles.  */
    PMC->PMC_PLLR = BITS (SYS_PLL_DIV, 0, 7) 
        | BITS (SYS_PLL_MUL - 1, 16, 26)
        | BITS (SYS_PLL_COUNT, 8, 13)
        | BITS (SYS_USB_LOG2_DIV, 28, 29);

#ifndef SIM_RUN
    /*  Wait for the PLL to start up.  */
    while (!(PMC->PMC_SR & PMC_LOCK))
        continue;

    /* Wait for MCK to start up.  */
    while (!(PMC->PMC_SR & PMC_MCKRDY))
        continue;
#endif

    /* Set prescaler so F_MCK = F_PLLCK / 2.  */
    PMC->PMC_MCKR = PMC_PRES_CLK_2;

#ifndef SIM_RUN
    /* Wait for MCK to start up.  */
    while (!(PMC->PMC_SR & PMC_MCKRDY))
        continue;
#endif
 
    /* Switch to PLLCK for MCK.  */
    PMC->PMC_MCKR |= PMC_CSS_PLL_CLK;

#ifndef SIM_RUN
    /* Wait for MCK to start up.  */
    while (!(PMC->PMC_SR & PMC_MCKRDY))
        continue;
#endif
}


extern void _irq_unexpected_handler (void);

extern void _irq_spurious_handler (void);


/* Enable NRST pin.  */
static inline void
sys_reset_enable (void)
{
    RSTC->RSTC_RMR |= RSTC_URSTEN | (0xa5 << 24);
}


/* Disable NRST pin.  */
static inline void
sys_reset_disable (void)
{
    /* Enable NRST pin.  */
    RSTC->RSTC_RMR =
        (RSTC->RSTC_RMR & ~RSTC_URSTEN) | (0xa5 << 24);
}


/** Disable watchdog.  */
__inline void
sys_watchdog_disable (void)
{
    WDTC->WDTC_WDMR = WDTC_WDDIS;
}


/** Initialise flash, disable watchdog, set up clocks.  This and any
    functions it calls must be inline for the C runtime startup to
    work.  */
static inline void 
sys_init (void)
{
    irq_id_t id;

    /* Disable all interrupts to be sure when debugging.  */
    AIC->AIC_IDCR = ~0;
    AIC->AIC_FFDR = ~0;

    sys_flash_init ();

    sys_watchdog_disable ();

    sys_clock_init ();

    sys_reset_enable ();

    /* Enable protect mode.  */
    AIC->AIC_DCR |= AIC_DCR_PROT;

#if 0
    AIC->AIC_SPU = (int) _irq_spurious_handler;

    for (id = IRQ_ID_MIN; id <= IRQ_ID_MAX; id++) 
        irq_vector_set (id, _irq_unexpected_handler);
#endif
}


void
sys_power_mode_low (void);


void
sys_power_mode_normal (void);


void
sys_sleep (void);


void
sys_udp_disable (void);


void
sys_udp_enable (void);


void
sys_watchdog_reset (void);


void
sys_watchdog_enable (void);



#endif /* SYS_H  */
