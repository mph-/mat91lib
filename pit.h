/** @file   pit.h
    @author M. P. Hayes, UCECE
    @date   3 June 2007
    @brief 
*/
#ifndef PIT_H
#define PIT_H

#include "config.h"

/* The PIT is clocked at F_CPU / 16.  */
#define PIT_DIVISOR(FREQ) (((((uint32_t) (F_CPU / (FREQ))) + 8) >> 4) - 1)


/* Reading PITC_PIVR resets the PITS bit.  */
#define PIT_RESET() \
    AT91C_BASE_PITC->PITC_PIVR


#define PIT_COMPARE_P() \
    (AT91C_BASE_PITC->PITC_PISR & AT91C_PITC_PITS \
     ? PIT_RESET (), 1 : 0)

#define PIT_INT_ENABLE() \
    AT91C_BASE_PITC->PITC_PIMR |= AT91C_PITC_PITIEN

#define PIT_INT_DISABLE() \
    AT91C_BASE_PITC->PITC_PIMR &= ~AT91C_PITC_PITIEN


extern void pit_start (void);

extern void pit_stop (void);

extern bool pit_compare_p (void);

extern uint32_t pit_period_set (uint32_t period);

extern int8_t pit_init (void);

#endif
