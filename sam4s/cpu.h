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

#endif /* CPU_H  */
