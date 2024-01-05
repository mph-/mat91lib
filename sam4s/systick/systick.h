/** @file   systick.h
    @author M. P. Hayes, UCECE
    @date   26 December 2023
    @brief
*/
#ifndef SYSTICK_H
#define SYSTICK_H

#ifdef __cplusplus
extern "C" {
#endif


#include "config.h"


/** Define systick ticks.  */
typedef uint32_t systick_clocks_t;


/** Get current number of systics:
    @return current time in ticks.  */
systick_clocks_t systick_clocks_get (void);


void systick_period_set (systick_clocks_t period);


systick_clocks_t systick_period_get (void);



/** Initialise systick.  */
int systick_init (void);


#ifdef __cplusplus
}
#endif
#endif
