/** @file   irq.h
    @author M. P. Hayes, UCECE
    @date   04 June 2007
    @brief 
*/
#ifndef IRQ_H
#define IRQ_H

#ifdef __cplusplus
extern "C" {
#endif
    

#include "config.h"
#include "bits.h"
#include "cpu.h"

typedef void (* irq_vector_t) (void);

typedef uint32_t irq_id_t;

typedef uint32_t irq_type_t;

typedef uint32_t irq_priority_t;


/* NB, The first interrupt vector is for FIQ.  */
enum {IRQ_ID_MIN = 0, IRQ_ID_MAX = 31};


static inline void irq_type_set (irq_id_t id, irq_type_t type)
{
    BITS_INSERT (AT91C_BASE_AIC->AIC_SMR[id], type, 5, 6);
}


static inline void irq_priority_set (irq_id_t id, irq_priority_t priority)
{
    BITS_INSERT (AT91C_BASE_AIC->AIC_SMR[id], priority, 0, 2);
}


static inline void irq_clear (irq_id_t id)
{
    AT91C_BASE_AIC->AIC_ICCR = BIT (id);
}


static inline void irq_enable (irq_id_t id)
{
    AT91C_BASE_AIC->AIC_IECR = BIT (id);
}


static inline bool irq_enabled_p (irq_id_t id)
{
    return (AT91C_BASE_AIC->AIC_IMR & BIT (id)) != 0;
}


static inline void irq_disable (irq_id_t id)
{
    AT91C_BASE_AIC->AIC_IDCR = BIT (id);
}


static inline void irq_trigger (irq_id_t id)
{
    AT91C_BASE_AIC->AIC_ISCR = BIT (id);
}


static inline void irq_vector_set (irq_id_t id, irq_vector_t isr)
{
    AT91C_BASE_AIC->AIC_SVR[id] = (uint32_t) isr;
}


/* This sets up a vector for an IRQ interrupt.  Note that the IRQ
   vectors are stored in special memory-mapped registers and thus do
   not depend on RAM or ROM model.  */
static inline void irq_config (irq_id_t id, irq_priority_t priority,
                               irq_vector_t isr)
{
    irq_disable (id);
    /* Priority is 0 (lowest) to 7 (highest).  */
    irq_priority_set (id, priority);
    irq_type_set (id, AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL);
    irq_vector_set (id, isr);
    irq_clear (id);
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

#ifdef __cplusplus
}
#endif    
#endif /* IRQ_H  */

