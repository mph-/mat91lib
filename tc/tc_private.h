/** @file   tc_private.h
    @author Michael Hayes
    @date   30 March 2011
    @brief  Private data structure for TC routines. 
*/

#ifndef TC_PRIVATE_H
#define TC_PRIVATE_H

#include "config.h"

typedef struct
{
    TcChannel *base;
    volatile tc_counter_t captureA;
    volatile tc_counter_t captureB;
    volatile tc_counter_t overflows;
    tc_mode_t mode;
    tc_period_t period;         /* Clocks */
    tc_period_t delay;          /* Clocks */
    tc_prescale_t prescale;
    tc_onload_function onload;
} tc_dev_t;

#endif
