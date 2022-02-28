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

typedef void (* irq_vector_t) (void);

typedef uint32_t irq_id_t;

typedef uint32_t irq_type_t;

typedef uint32_t irq_priority_t;

typedef void (*irq_handler_t) (void);

extern irq_handler_t exception_table[];


/* NB, The first interrupt vector is for FIQ.  */
enum {IRQ_ID_MIN = 0, IRQ_ID_MAX = 31};


static inline void irq_type_set (irq_id_t id, irq_type_t type)
{
    /* TODO.  */
}


static inline void irq_priority_set (irq_id_t id, irq_priority_t priority)
{
    // The smaller the number the higher the priority.
    NVIC->IP[id] = priority << 4;
}


static inline void irq_clear (irq_id_t id)
{
    NVIC->ICPR[id >> 5] = BIT (id & 0x1f);
}


static inline void irq_enable (irq_id_t id)
{
    NVIC->ISER[id >> 5] = BIT (id & 0x1f);
}


static inline bool irq_enabled_p (irq_id_t id)
{
    return (NVIC->ISER[id >> 5] & BIT (id & 0x1f)) != 0;
}


static inline void irq_disable (irq_id_t id)
{
    NVIC->ICER[id >> 5] = BIT (id & 0x1f);
}


static inline void irq_trigger (irq_id_t id)
{
    NVIC->ISPR[id >> 5] = BIT (id & 0x1f);
}


static inline void irq_vector_set (irq_id_t id, irq_vector_t isr)
{
    exception_table[id + 16] = isr;
    asm volatile ("" ::: "memory");
}


/* This sets up a vector for an IRQ interrupt.  Note that the IRQ
   vectors are stored in special memory-mapped registers and thus do
   not depend on RAM or ROM model.  */
static inline void irq_config (irq_id_t id, irq_priority_t priority,
                               irq_vector_t isr)
{
    irq_disable (id);
    /* Priority is 0 (highest) to 15 (lowest).  */
    irq_priority_set (id, priority);
    irq_vector_set (id, isr);
    irq_clear (id);
}


/* Return global interrupt status.  */
__inline __attribute__ ((always_inline))
bool irq_global_status (void)
{
    uint32_t status;

    __asm__ volatile ("\tmrs %0, primask" : "=r" (status));

    return status != 0;
}


/* Globally disable interrupts.  */
__inline __attribute__ ((always_inline))
bool irq_global_disable (void)
{
    bool irq_status;

    irq_status = irq_global_status ();
    __asm__ ("\tcpsid i");
    return irq_status;
}


/* Globally enable interrupts.  */
__inline __attribute__ ((always_inline))
void irq_global_enable (void)
{
    __asm__ ("\tcpsie i");
}

#ifdef __cplusplus
}
#endif
#endif /* IRQ_H  */
