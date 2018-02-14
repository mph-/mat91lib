/** @file   cpu.h
    @author M. P. Hayes, UCECE
    @date   04 June 2007
    @brief  CPU routines for SAM4S processors
*/
#ifndef CPU_H
#define CPU_H

#ifdef __cplusplus
extern "C" {
#endif
    

#include "config.h"
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


static inline void
cpu_wfi (void)
{
    __asm__ ("\twfi");
}

#ifdef __cplusplus
}
#endif    
#endif /* CPU_H  */

