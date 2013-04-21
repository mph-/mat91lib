/** @file   tc.h
    @author M. P. Hayes
    @date   30 November 2010
    @brief  Timer counter routines for AT91SAM7 processors
*/

#ifndef TC_H
#define TC_H

#include "config.h"
#include "pio.h"

#define TC_PERIOD_DIVISOR(FREQ) ((tc_period_t)(F_CPU / (FREQ)))

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
    TC_PULSE_MODE_ONESHOT_INVERT,
    TC_DELAY_MODE_ONESHOT,
} tc_pulse_mode_t;


/** TC configuration structure.  */
typedef struct
{
    pio_t pio;
} tc_cfg_t;


typedef uint32_t tc_period_t;


/** Include definition of data structures required for the driver
    implementation.  Do not use.  */
#include "tc_private.h"


/** Define datatype for handle to TC functions.  */
typedef tc_dev_t *tc_t;


/** Configure pulse generation with specified mode.   The delay and period
    are in terms of the CPU clock.  The pulse width is delay - period.  */
bool tc_pulse_config (tc_t tc, tc_pulse_mode_t mode, 
                      tc_period_t delay, tc_period_t period);


bool tc_start (tc_t tc);


bool tc_stop (tc_t tc);
              

tc_t tc_init (const tc_cfg_t *cfg);


void tc_shutdown (tc_t tc);


void tc_clock_sync (tc_t tc, tc_period_t period);

#endif






