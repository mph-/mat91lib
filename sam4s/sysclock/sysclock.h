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

typedef uint64_t syslock_clocks_t;


uint32_t sysclock_ms_get (void);


void sysclock_ms_delay (uint32_t delay);


bool sysclock_ms_elapsed (uint32_t from, uint32_t delay);


/** Initialise sysclock.  */
int sysclock_init (void);


#ifdef __cplusplus
}
#endif
#endif
