/** @file   cpu.c
    @author M. P. Hayes, UCECE
    @date   04 June 2007
    @brief  CPU routines for AT91SAM7 processors
*/
#ifndef CPU_H
#define CPU_H

#include "config.h"
#include "irq.h"
#include "bits.h"


#define CPU_FLASH_SPEED 30e6

#ifndef CPU_FLASH_READ_CYCLES 
#define CPU_FLASH_READ_CYCLES 2
// #define CPU_FLASH_READ_CYCLES ((int) ((F_CPU + CPU_FLASH_SPEED - 1) / CPU_FLASH_SPEED))
#endif


/** The ARM7 has 7 processor modes: user, system, supervisor, abort,
    undefined, interrupt, and fast interrupt.  All except user mode
    are privileged modes and all except user and system modes are
    exception modes.

    System mode is like user mode (it uses the user mode registers)
    but has full supervisor mode priviledges.  System mode is usually
    switched from supervisor mode to access user mode registers, etc.
    Since system mode has full priviledges it is possible to switch
    back from system to supervisor mode.  Once in user mode the only
    way the mode can be changed is with an interrupt.

    User mode is typically used for user applications and supervisor
    mode is typically used by the operating system (kernel mode).
    Without memory management many operating systems (such as ecos)
    will use supervisor mode for everything to reduce the overhead of
    switching modes.
    
    Each mode (except system) has their own banked copies of R13 (link
    register) and R14 (stack pointer) so there can be separate stacks
    for each mode.  FIQ mode also has banked copies of R8--R12 for
    fast context switching.
 */

typedef enum 
{
    CPU_MODE_USR = 0x10,        /** User mode  */
    CPU_MODE_FIQ = 0x11,        /** Fast interrupt mode  */
    CPU_MODE_IRQ = 0x12,        /** Interrupt mode  */
    CPU_MODE_SVC = 0x13,        /** Supervisor mode  */
    CPU_MODE_ABT = 0x17,        /** Abort mode  */
    CPU_MODE_UND = 0x1b,        /** Undefined mode  */
    CPU_MODE_SYS = 0x1f         /** System mode  */
} arm_mode_t;

#define CPU_MODE_MASK 0x1f


/** Current processor status register (CPSR) bits.  */
enum 
{
    CPU_T_BIT = BIT (5),        /** Thumb mode  */
    CPU_F_BIT = BIT (6),        /** Disable FIQ interrupts  */
    CPU_I_BIT = BIT (7)         /** Disable IRQ interrupts  */
};


/** Set the stack pointer of the current mode
    @param stack pointer initial value.  */
__inline __attribute__ ((always_inline)) 
void cpu_sp_set (uint32_t val) 
{
    register uint32_t sp __asm__ ("sp") = val;

    /* Create a dummy use.  */
    __asm__ ("" : : "r" (sp));
}


/** Set the control bits, I, F, T, and M[4:0].  This only works for a
    constant argument and  ARM mode.
    @param CPSR control bit mask.  */
#define cpu_cpsr_c_set_const(cpsr) \
 __asm__ ("\tmsr CPSR_c, %0" : : "i" (cpsr))


/** Set the control bits, I, F, T, and M[4:0] of the CPSR.  This only works 
    in ARM mode.
    @param CPSR control bit mask.  */
__inline __attribute__ ((always_inline)) 
void cpu_cpsr_c_set (uint32_t cpsr) 
{
    __asm__ ("\tmsr CPSR_c, %0" : : "r" (cpsr));
}


/** Set the flag bits of the CPSR.  This only works in ARM mode.
    @param CPSR flag bits mask.  */
__inline __attribute__ ((always_inline)) 
void cpu_cpsr_f_set (uint32_t cpsr) 
{
    __asm__ ("\tmsr CPSR_f, %0" : : "r" (cpsr) : "cc");
}


/** Set all the fields of CPSR (note the s and x fields are currently
   unused).  This only works in ARM mode.
    @param CPSR.  */
__inline __attribute__ ((always_inline)) 
void cpu_cpsr_set (uint32_t cpsr) 
{
    __asm__ ("\tmsr CPSR_fsxc, %0" : : "r" (cpsr) : "cc");
}


/** Get the CPSR (all fields).  This only works in ARM mode.
    @return CPSR.  */
__inline __attribute__ ((always_inline)) 
uint32_t cpu_cpsr_get (void) 
{
    uint32_t cpsr;

    __asm__ ("\tmrs %0, CPSR" : "=r" (cpsr) :);
    return cpsr;
}


/** Set the mode bits M[4:0] of the CPSR.  This only works in ARM mode.
    @param CPSR mode bits mask.  */
__inline __attribute__ ((always_inline)) 
void cpu_mode_set (uint32_t mode) 
{
    uint32_t cpsr;
    
    cpsr = cpu_cpsr_get ();
 
    cpsr &= ~CPU_MODE_MASK;
    cpsr |= mode;

    cpu_cpsr_c_set (cpsr);
}


/** Get the SPSR (all fields).  This only works in ARM mode.
    @return SPSR.  */
__inline __attribute__ ((always_inline)) 
uint32_t cpu_spsr_get (void) 
{
    uint32_t spsr;

    __asm__ ("\tmrs %0, SPSR" : "=r" (spsr) :);
    return spsr;
}



/** Remap SRAM so that SRAM exists at 0x200000 as well as at 0x000000.
    On reset the Flash at 0x100000 is mapped to address 0x00000 as
    well as 0x100000.  Note writing to the remap bit a second time
    toggles the remapping so that the Flash appears at address
    0x000000.  This causes problems if running from the debugger since
    the CPU is not reset.  Thus we don't toggle the remap bit if the
    SRAM is already remapped.  */
__inline void
cpu_sram_remap (void)
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
cpu_sram_unremap (void)
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
cpu_watchdog_disable (void)
{
    AT91C_BASE_WDTC->WDTC_WDMR = AT91C_WDTC_WDDIS;
}



/* The AT91 Flash is single plane so it is not possible
   to write to it while executing code out of it.  */

/** Initialise flash memory controller.  */
static void
cpu_flash_init (void)
{
    switch (CPU_FLASH_READ_CYCLES)
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


#define F_SLCK 32768

#define CPU_OS_DELAY 1.5e-3
#define CPU_OS_COUNT ((uint16_t) (CPU_OS_DELAY * F_SLCK + 7)) / 8

#define CPU_PLL_DELAY 0.9e-3
#define CPU_PLL_COUNT (uint16_t) (CPU_PLL_DELAY * F_SLCK)

#define CPU_USB_LOG2_DIV 0


/** Set up the main clock (MAINCK), PLL clock, and master clock (MCK).   */
static inline void
cpu_clock_init (void)
{
    /* To minimize the power required to start up the system, the main
       oscillator is disabled after reset and slow clock is
       selected. 

       There are three clock sources: SLCK (the RC oscillator slow
       clock), MAINCK (the external crytal main clock), and PLLCK (the
       output of the phase locked loop driven by MAINCK).  One of
       these three clock sources can be fed to a prescaler (with
       divisors 2^0 ... 2^6) to drive MCK (master clock).
       
       The main oscillator (external crystal) can range from 3--20 MHz.
       The PLL frequency can range from 80--220 MHz. 
    */

    /* Enable the MAINCK oscillator and wait for it to start up.  The
       start delay is CPU_OS_COUNT * 8 SLCK cycles.  */
    AT91C_BASE_PMC->PMC_MOR = BITS (CPU_OS_COUNT, 8, 15) | AT91C_CKGR_MOSCEN;
    
#ifndef SIM_RUN
    /*  Wait for the oscillator to start up.  */
    while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MOSCS))
        continue;
#endif
    
    /* The PLL start delay is CPU_PLL_COUNT SLCK cycles.  */
    AT91C_BASE_PMC->PMC_PLLR = BITS (CPU_PLL_DIV, 0, 7) 
        | BITS (CPU_PLL_MUL - 1, 16, 26)
        | BITS (CPU_PLL_COUNT, 8, 13)
        | BITS (CPU_USB_LOG2_DIV, 28, 29);

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


static inline void
cpu_nop (void)
{
    __asm__ ("\tnop");
}


static inline void
cpu_idle (void)
{
    /* Turn off CPU clock after current instruction.  It will be
       re-enabled when an interrupt occurs.  */
    AT91C_BASE_PMC->PMC_SCDR = AT91C_PMC_PCK;
}


static inline void
cpu_reset (void)
{
    /* Reset processor and peripherals.  */
    AT91C_BASE_RSTC->RSTC_RCR = AT91C_RSTC_PROCRST | AT91C_RSTC_PERRST 
        | (0xa5 << 24);
}


/* Enable NRST pin.  */
static inline void
cpu_reset_enable (void)
{
    AT91C_BASE_RSTC->RSTC_RMR |= AT91C_RSTC_URSTEN | (0xa5 << 24);
}


/* Disable NRST pin.  */
static inline void
cpu_reset_disable (void)
{
    /* Enable NRST pin.  */
    AT91C_BASE_RSTC->RSTC_RMR =
        (AT91C_BASE_RSTC->RSTC_RMR & ~AT91C_RSTC_URSTEN) | (0xa5 << 24);
}


static inline uint8_t
cpu_reset_type_get (void)
{
    return (AT91C_BASE_RSTC->RSTC_RSR >> 8) & 0x07;
}


static inline bool
cpu_brownout_detect_get (void)
{
    return (AT91C_BASE_RSTC->RSTC_RSR & AT91C_RSTC_BODSTS) != 0;
}


static inline bool
cpu_reset_detect_get (void)
{
    return (AT91C_BASE_RSTC->RSTC_RSR & AT91C_RSTC_URSTS) != 0;
}



extern void _irq_unexpected_handler (void);

extern void _irq_spurious_handler (void);


/** Initialise flash, disable watchdog, set up clocks.  */
static inline void 
cpu_init (void)
{
    /* Disable all interrupts to be sure when debugging.  */
    AT91C_BASE_AIC->AIC_IDCR = ~0;
    AT91C_BASE_AIC->AIC_FFDR = ~0;

    cpu_flash_init ();

    cpu_watchdog_disable ();

    cpu_clock_init ();

#ifdef RAM_RUN
    /* Remap SRAM if using RAM model.  */
    cpu_sram_remap ();
#endif

    cpu_reset_enable ();

    /* Enable protect mode.  */
    AT91C_BASE_AIC->AIC_DCR |= AT91C_AIC_DCR_PROT;

    AT91C_BASE_AIC->AIC_SPU = (int) _irq_spurious_handler;

    for (irq_id_t id = IRQ_ID_MIN; id <= IRQ_ID_MAX; id++) 
        irq_vector_set (id, _irq_unexpected_handler);
}


static inline void
cpu_sleep (void)
{
    /* Turn off main oscillator.  */
//    AT91C_BASE_PMC->PMC_MOR &= ~AT91C_CKGR_MOSCEN;

    /* Wait for interrupt.  */
    cpu_idle ();
}


/* Globally disable interrupts.  This only works in ARM mode.  */
__inline __attribute__ ((always_inline)) 
void irq_global_disable (void)
{
    uint32_t cpsr;

    cpsr = cpu_cpsr_get ();
    cpsr |= CPU_I_BIT | CPU_F_BIT;
    cpu_cpsr_set (cpsr);
}


/* Globally enable interrupts.  This only works in ARM mode.  */
__inline __attribute__ ((always_inline)) 
void irq_global_enable (void)
{
    uint32_t cpsr;

    cpsr = cpu_cpsr_get ();
    cpsr &= ~(CPU_I_BIT | CPU_F_BIT);
    cpu_cpsr_set (cpsr);
}


#endif /* CPU_H  */
