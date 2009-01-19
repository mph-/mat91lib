/** @file   cpu.c
    @author M. P. Hayes, UCECE
    @date   04 June 2007
    @brief  CPU routines for AT91SAM7 processors
*/

#include "config.h"
#include "cpu.h"
#include "irq.h"

#define F_SLCK 32768

#define OS_DELAY 1.5e-3
#define OS_COUNT ((uint16_t) (OS_DELAY * F_SLCK + 7)) / 8

#define PLL_DELAY 0.9e-3
#define PLL_COUNT (uint16_t) (PLL_DELAY * F_SLCK)


#ifndef CPU_USB_DIV
/* When using USB the PLL clock must be 48, 96, or 192 MHz.  
   Assuming 96 MHz PLL clock then divide by 2 to get 48 MHz USB clock.  */
#define CPU_USB_DIV 2
#endif

#if CPU_USB_DIV == 1
#define USB_LOG2_DIV 0
#elif CPU_USB_DIV == 2
#define USB_LOG2_DIV 1
#elif CPU_USB_DIV == 4
#define USB_LOG2_DIV 2
#else
#error Unsupported divide ratio CPU_USB_DIV
#endif


/* The AT91 Flash is single plane so it is not possible
   to write to it while executing code out of it.  */

/** Initialise flash memory controller.  */
static void cpu_flash_init (void)
{
    /* Set 1 flash wait state for reading, 2 for writing.  */
    AT91C_BASE_MC->MC_FMR = AT91C_MC_FWS_1FWS;

    /* Set number of MCK cycles per microsecond for the Flash
       microsecond cycle number (FMCN) field of the Flash mode
       register (FMR).  */
    BITS_INSERT (AT91C_BASE_MC->MC_FMR, (uint16_t) (F_CPU / 1e6), 16, 23);
}


/** Set up the main clock (MAINCK), PLL clock, and master clock (MCK).   */
static void cpu_clock_init (void)
{

    /* To minimize the power required to start up the system, the main
       oscillator is disabled after reset and slow clock is
       selected. 

       There are three clock sources: SLCK (the RC oscillator slow
       clock), MAINCK (the external crytal main clock), and PLLCK (the
       output of the phase locked loop driven by MAINCK).  One of
       these three clock sources can be fed to a prescaler (with
       divisors 2^0 ... 2^6) to drive MCK (master clock).
    */

    /* Enable the MAINCK oscillator and wait for it to start up.  The
       start delay is OS_COUNT * 8 SLCK cycles.  */
    AT91C_BASE_PMC->PMC_MOR = BITS (OS_COUNT, 8, 15) | AT91C_CKGR_MOSCEN;
    
    /*  Wait for the oscillator to start up.  */
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MOSCS))
	continue;
    
    /* The PLL start delay is PLL_COUNT SLCK cycles.  */
    AT91C_BASE_PMC->PMC_PLLR = BITS (PLL_DIV, 0, 7) 
	| BITS (PLL_MUL - 1, 16, 26)
	| BITS (PLL_COUNT, 8, 13)
	| BITS (USB_LOG2_DIV, 28, 29);

    /*  Wait for the PLL to start up.  */
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_LOCK))
	continue;

    /* Wait for MCK to start up.  */
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY))
	continue;

    /* Set prescaler so F_MCK = F_PLLCK / 2.  */
    AT91C_BASE_PMC->PMC_MCKR = AT91C_PMC_PRES_CLK_2;

    /* Wait for MCK to start up.  */
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY))
	continue;
 
    /* Switch to PLLCK for MCK.  */
    AT91C_BASE_PMC->PMC_MCKR |= AT91C_PMC_CSS_PLL_CLK;

    /* Wait for MCK to start up.  */
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY))
	continue;
}


/** Dummy ISR for unexpected interrupts.  */
static void cpu_dummy_isr (void)
{
    /* Hang.  */
    while (1)
	continue;
}


/** Initialise flash, disable watchdog, set up clocks.  */
void cpu_init (void)
{
    cpu_flash_init ();

    cpu_watchdog_disable ();

    cpu_clock_init ();

    AT91C_BASE_AIC->AIC_SPU = (int) cpu_dummy_isr;

#if defined(RAM_RUN)
    for (irq_id_t id = IRQ_ID_MIN; id <= IRQ_ID_MAX; id++) 
	irq_vector_set (id, cpu_dummy_isr);

    /* Remap SRAM if using RAM model.  */
    cpu_sram_remap ();
#endif
}

