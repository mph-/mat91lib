/** @file   cpu.h
    @author M. P. Hayes, UCECE
    @date   04 June 2007
    @brief  CPU routines for AT91SAM7 processors
*/
#ifndef CPU_H
#define CPU_H

#ifdef __cplusplus
extern "C" {
#endif
    

#include "config.h"
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
    constant argument and ARM mode.
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
    return (RSTC->RSTC_RSR & AT91C_RSTC_BODSTS) != 0;
}


static inline bool
cpu_reset_detect_get (void)
{
    return (RSTC->RSTC_RSR & AT91C_RSTC_URSTS) != 0;
}


/* Enable NRST pin.  */
static inline void
cpu_reset_enable (void)
{
    RSTC->RSTC_RMR |= AT91C_RSTC_URSTEN | (0xa5 << 24);
}


/* Disable NRST pin.  */
static inline void
cpu_reset_disable (void)
{
    /* Enable NRST pin.  */
    RSTC->RSTC_RMR =
        (RSTC->RSTC_RMR & ~AT91C_RSTC_URSTEN) | (0xa5 << 24);
}


static inline void
cpu_idle (void)
{
    /* Turn off CPU clock after current instruction.  It will be
       re-enabled when an interrupt occurs.  */
    AT91C_BASE_PMC->PMC_SCDR = AT91C_PMC_PCK;

    while ((AT91C_BASE_PMC->PMC_SCSR & AT91C_PMC_PCK) != AT91C_PMC_PCK)
        continue;
}



#ifdef __cplusplus
}
#endif    
#endif /* CPU_H  */

