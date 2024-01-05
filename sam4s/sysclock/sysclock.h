/** @file   sysclock.h
    @author M. P. Hayes, UCECE
    @date   26 December 2023
    @brief
*/
#ifndef SYSCLOCK_H
#define SYSCLOCK_H

#ifdef __cplusplus
extern "C" {
#endif


#include "config.h"

typedef uint64_t sysclock_clocks_t;


uint32_t sysclock_millis (void);


uint32_t sysclock_micros (void);


void sysclock_millis_delay (uint32_t delay);


bool sysclock_millis_elapsed (uint32_t from, uint32_t delay);


/** Initialise sysclock.  */
int sysclock_init (void);


#ifdef __cplusplus
}
#endif
#endif
