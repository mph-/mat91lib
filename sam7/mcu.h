/** @file   mcu.h
    @author M. P. Hayes, UCECE
    @date   13 July 2013
    @brief  System routines for AT91SAM4S processors
*/
#ifndef MCU_H
#define MCU_H

#ifdef __cplusplus
extern "C" {
#endif
    

#include "config.h"
#include "bits.h"
#include "irq.h"


#ifndef MCU_FLASH_READ_CYCLES 
#define MCU_FLASH_READ_CYCLES 16
#endif


static inline void
mcu_delay_loop (unsigned int loops)
{
#ifdef __THUMBEL__
    __asm__ volatile ("\t sub %0, %0, #1;\n\t bcs . - 2" : "=r" (loops) : "0" (loops));
#else
    __asm__ volatile ("\t subs %0, %0, #1;\n\t bcs . - 4" : "=r" (loops) : "0" (loops));
#endif
}



/** Remap SRAM that SRAM exists at 0x200000 as well as at 0x000000.  On
    reset the Flash at 0x100000 is mapped to address 0x00000 as well as
    0x100000.  Note writing to the remap bit a second time toggles the
    remapping so that the Flash appears at address 0x000000.  This
    causes problems if running from the debugger since the CPU is not
    reset.  Thus we don't toggle the remap bit if the SRAM is already
    remapped.  */
__inline void
mcu_sram_remap (void)
{
    uint32_t tmp;
    uint32_t *p;

    p = (uint32_t *)AT91C_ISRAM + 0x04;

    /* Save the value at position 0x04 in the SRAM (undefined
       instruction vector) then set it to zero.  */
    tmp = *p;
    *p = 0;

    /* If the base vector is now non-zero it is mapped to flash so we
       must map it back to SRAM */
    if (*(uint32_t *)0x04)
        AT91C_BASE_MC->MC_RCR = AT91C_MC_RCB;

    /* Restore vector in SRAM  */
    *p = tmp;
}


/** Unremap SRAM.  */
__inline void 
mcu_sram_unremap (void)
{
    uint32_t tmp;
    uint32_t *p;

    p = (uint32_t *)AT91C_ISRAM + 0x04;

    /* Save the value at position 0x04 in the SRAM (undefined
       instruction vector) then set it to zero.  */
    tmp = *p;
    *p = 0;

    /* If the base vector is zero then it is mapped to SRAM, hence we
       toggle the bit to map it back to flash. */
    if (!*p)
        AT91C_BASE_MC->MC_RCR = AT91C_MC_RCB;

    /* Restore vector in SRAM.  */
    *p = tmp;
}


/** Disable watchdog.  */
__inline void
mcu_watchdog_disable (void)
{
    AT91C_BASE_WDTC->WDTC_WDMR = AT91C_WDTC_WDDIS;
}



/* The AT91 Flash is single plane so it is not possible
   to write to it while executing code out of it.  */

/** Initialise flash memory controller.  */
static void
mcu_flash_init (void)
{
    switch (MCU_FLASH_READ_CYCLES)
    {
    case 1:
        /* Set 0 flash wait states for reading, 1 for writing.  */
        AT91C_BASE_MC->MC_FMR = AT91C_MC_FWS_0FWS;
        break;

    case 2:
        /* Set 1 flash wait state for reading, 2 for writing.  */
        AT91C_BASE_MC->MC_FMR = AT91C_MC_FWS_1FWS;
        break;

    case 3:
        /* Set 2 flash wait states for reading, 3 for writing.  */
        AT91C_BASE_MC->MC_FMR = AT91C_MC_FWS_2FWS;
        break;

    default:
        /* Set 3 flash wait states for reading, 4 for writing.  */
        AT91C_BASE_MC->MC_FMR = AT91C_MC_FWS_3FWS;
        break;
    }

    /* Set number of MCK cycles per microsecond for the Flash
       microsecond cycle number (FMCN) field of the Flash mode
       register (FMR).  */
    BITS_INSERT (AT91C_BASE_MC->MC_FMR, (uint16_t) (F_CPU / 1e6), 16, 23);
}


/* Internal slow clock frequency.  */
#define F_SLCK 32768

#define MCU_OS_DELAY 1.5e-3
#define MCU_OS_COUNT ((uint16_t) (MCU_OS_DELAY * F_SLCK + 7)) / 8

#define MCU_PLL_DELAY 0.9e-3
#define MCU_PLL_COUNT (uint16_t) (MCU_PLL_DELAY * F_SLCK)

#define MCU_USB_LOG2_DIV 0


/** Set up the main clock (MAINCK), PLL clock, and master clock (MCK).   */
static inline void
mcu_clock_init (void)
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
       start delay is MCU_OS_COUNT * 8 SLCK cycles.  */
    AT91C_BASE_PMC->PMC_MOR = BITS (MCU_OS_COUNT, 8, 15) | AT91C_CKGR_MOSCEN;
    
#ifndef SIM_RUN
    /*  Wait for the oscillator to start up.  */
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MOSCS))
        continue;
#endif
    
    /* The PLL start delay is MCU_PLL_COUNT SLCK cycles.  */
    AT91C_BASE_PMC->PMC_PLLR = BITS (MCU_PLL_DIV, 0, 7) 
        | BITS (MCU_PLL_MUL - 1, 16, 26)
        | BITS (MCU_PLL_COUNT, 8, 13)
        | BITS (MCU_USB_LOG2_DIV, 28, 29);

#ifndef SIM_RUN
    /*  Wait for the PLL to start up.  */
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_LOCK))
        continue;

    /* Wait for MCK to start up.  */
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY))
        continue;
#endif

    /* Set prescaler so F_MCK = F_PLLCK / 2.  */
    AT91C_BASE_PMC->PMC_MCKR = AT91C_PMC_PRES_CLK_2;

#ifndef SIM_RUN
    /* Wait for MCK to start up.  */
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY))
        continue;
#endif
 
    /* Switch to PLLCK for MCK.  */
    AT91C_BASE_PMC->PMC_MCKR |= AT91C_PMC_CSS_PLL_CLK;

#ifndef SIM_RUN
    /* Wait for MCK to start up.  */
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY))
        continue;
#endif
}


extern void _irq_unexpected_handler (void);

extern void _irq_spurious_handler (void);


void
mcu_reset (void);


/* Enable NRST pin.  */
static inline void
mcu_reset_enable (void)
{
    AT91C_BASE_RSTC->RSTC_RMR |= AT91C_RSTC_URSTEN | (0xa5 << 24);
}


/* Disable NRST pin.  */
static inline void
mcu_reset_disable (void)
{
    /* Enable NRST pin.  */
    AT91C_BASE_RSTC->RSTC_RMR =
        (AT91C_BASE_RSTC->RSTC_RMR & ~AT91C_RSTC_URSTEN) | (0xa5 << 24);
}


/** Initialise flash, disable watchdog, set up clocks.  This and any
    functions it calls must be inline for the C runtime startup to
    work.  */
static inline void 
mcu_init (void)
{
    irq_id_t id;

    /* Disable all interrupts to be sure when debugging.  */
    AT91C_BASE_AIC->AIC_IDCR = ~0;
    AT91C_BASE_AIC->AIC_FFDR = ~0;

    mcu_flash_init ();

    mcu_watchdog_disable ();

    mcu_clock_init ();

#ifdef RAM
    /* Remap SRAM if using RAM model.  */
    mcu_sram_remap ();
#endif

    mcu_reset_enable ();

    /* Enable protect mode.  */
    AT91C_BASE_AIC->AIC_DCR |= AT91C_AIC_DCR_PROT;

    AT91C_BASE_AIC->AIC_SPU = (int) _irq_spurious_handler;

    for (id = IRQ_ID_MIN; id <= IRQ_ID_MAX; id++) 
        irq_vector_set (id, _irq_unexpected_handler);
}



void
mcu_power_mode_low (void);


void
mcu_power_mode_normal (void);


void
mcu_sleep (void);


void
mcu_udp_disable (void);


void
mcu_udp_enable (void);


void
mcu_watchdog_reset (void);


void
mcu_watchdog_enable (void);


static inline void
mcu_pmc_enable (uint8_t id)
{
    PMC->PMC_PCER = BIT (id);
}


static inline void
mcu_pmc_disable (uint8_t id)
{
    PMC->PMC_PCDR = BIT (id);
}

void
mcu_cpu_idle (void);



#ifdef __cplusplus
}
#endif    
#endif /* MCU_H  */

