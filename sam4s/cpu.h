/** @file   cpu.h
    @author M. P. Hayes, UCECE
    @date   04 June 2007
    @brief  CPU routines for SAM4S processors
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


/** Set the stack pointer of the current mode
    @param stack pointer initial value.  */
__inline __attribute__ ((always_inline)) 
void cpu_sp_set (uint32_t val) 
{
    register uint32_t sp __asm__ ("sp") = val;

    /* Create a dummy use.  */
    __asm__ ("" : : "r" (sp));
}


static inline void
cpu_nop (void)
{
    __asm__ ("\tnop");
}


static inline uint8_t
cpu_reset_type_get (void)
{
    return (RSTC->RSTC_RSR >> 8) & 0x07;
}


static inline bool
cpu_brownout_detect_get (void)
{
    return (RSTC->RSTC_RSR & RSTC_BODSTS) != 0;
}


static inline bool
cpu_reset_detect_get (void)
{
    return (RSTC->RSTC_RSR & RSTC_URSTS) != 0;
}


/* Enable NRST pin.  */
static inline void
cpu_reset_enable (void)
{
    RSTC->RSTC_RMR |= RSTC_URSTEN | (0xa5 << 24);
}


/* Disable NRST pin.  */
static inline void
cpu_reset_disable (void)
{
    /* Enable NRST pin.  */
    RSTC->RSTC_RMR =
        (RSTC->RSTC_RMR & ~RSTC_URSTEN) | (0xa5 << 24);
}


/* Globally disable interrupts.  */
__inline __attribute__ ((always_inline)) 
void irq_global_disable (void)
{
    /* TODO.  */
}


/* Globally enable interrupts.  */
__inline __attribute__ ((always_inline)) 
void irq_global_enable (void)
{
    /* TODO.  */
}


void
cpu_idle (void);


void
cpu_reset (void);


void
cpu_power_mode_low (void);


void
cpu_power_mode_normal (void);


void
cpu_sleep (void);


void
cpu_udp_disable (void);


void
cpu_udp_enable (void);


#endif /* CPU_H  */
