/** @file   tc.h
    @author M. P. Hayes
    @date   30 November 2010
    @brief  Timer counter routines for AT91SAM7 processors
*/

#ifndef TC_H
#define TC_H

#include "config.h"


typedef enum
{
    TC_CHANNEL_0,
    TC_CHANNEL_1,
    TC_CHANNEL_2,
    TC_CHANNEL_NUM
} tc_channel_t;


typedef enum
{
    TC_PULSE_MODE,
    TC_PULSE_MODE_INVERT,
    TC_PULSE_MODE_ONESHOT,
    TC_PULSE_MODE_ONESHOT_INVERT
} tc_pulse_mode_t;


typedef struct
{
    AT91S_TC *base;
} tc_t;


bool tc_pulse_config (tc_t *tc, tc_pulse_mode_t mode, uint32_t delay, uint32_t pulse_width);


bool tc_start (tc_t *tc);


bool tc_stop (tc_t *tc);
              

tc_t *tc_init (tc_channel_t channel);


void tc_shutdown (void);

#endif






