/** @file   tc_private.h
    @author Michael Hayes
    @date   30 March 2011
    @brief  Private data structure for TC routines.
*/

#ifndef TC_PRIVATE_H
#define TC_PRIVATE_H

#ifdef __cplusplus
extern "C" {
#endif


#include "config.h"

typedef struct
{
    TcChannel *base;
    volatile tc_counter_t captureA;
    volatile tc_counter_t captureB;
    volatile tc_counter_t overflows;
    tc_mode_t mode;
    tc_mode_t aux_mode;
    tc_period_t period;         /* Clocks */
    tc_period_t delay;          /* Clocks */
    tc_period_t aux_delay;      /* Clocks */
    tc_prescale_t prescale;
    int capture_state;
} tc_dev_t;


#ifdef __cplusplus
}
#endif
#endif
