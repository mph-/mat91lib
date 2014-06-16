/** @file   irq.h
    @author M. P. Hayes, UCECE
    @date   04 June 2007
    @brief 
*/
#ifndef IRQ_H
#define IRQ_H

#include "config.h"
#include "bits.h"

typedef void (* irq_vector_t) (void);

typedef uint32_t irq_id_t;

typedef uint32_t irq_type_t;

typedef uint32_t irq_priority_t;


/* NB, The first interrupt vector is for FIQ.  */
enum {IRQ_ID_MIN = 0, IRQ_ID_MAX = 31};


static inline void irq_type_set (irq_id_t id, irq_type_t type)
{
    /* TODO.  */
}


static inline void irq_priority_set (irq_id_t id, irq_priority_t priority)
{
    /* TODO.  */
}


static inline void irq_clear (irq_id_t id)
{
    /* TODO.  */
}


static inline void irq_enable (irq_id_t id)
{
    /* TODO.  */
}


static inline bool irq_enabled_p (irq_id_t id)
{
    /* TODO.  */
}


static inline void irq_disable (irq_id_t id)
{
    /* TODO.  */
}


static inline void irq_trigger (irq_id_t id)
{
    /* TODO.  */
}


static inline void irq_vector_set (irq_id_t id, irq_vector_t isr)
{
    /* TODO.  */
}


/* This sets up a vector for an IRQ interrupt.  Note that the IRQ
   vectors are stored in special memory-mapped registers and thus do
   not depend on RAM or ROM model.  */
static inline void irq_config (irq_id_t id, irq_priority_t priority,
                               uint32_t type, irq_vector_t isr)
{
    irq_disable (id);
    /* Priority is 0 (lowest) to 7 (highest).  */
    irq_priority_set (id, priority);
    irq_type_set (id, type);
    irq_vector_set (id, isr);
    irq_clear (id);
}
#endif /* IRQ_H  */
