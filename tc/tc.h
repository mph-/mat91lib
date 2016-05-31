/** @file   tc.h
    @author M. P. Hayes
    @date   30 November 2010
    @brief  Timer counter routines for AT91 processors.
    Although the counters are only 16 bit, this driver synthesises 64 bit
    counters using overflow interrupts.   Even with a 48 MHz clock,
    these 64 bit counters will take 3000 years to overflow!  
*/

#ifndef TC_H
#define TC_H

#include "config.h"
#include "pio.h"

#define TC_CLOCK_FREQUENCY(PRESCALE) (F_CPU / (PRESCALE))

#define TC_PERIOD_DIVISOR(FREQ, PRESCALE) ((tc_period_t)(0.5 + TC_CLOCK_FREQUENCY (PRESCALE) / (FREQ)))


typedef enum
{
    TC_CHANNEL_0,
    TC_CHANNEL_1,
    TC_CHANNEL_2,
    TC_CHANNEL_NUM
} tc_channel_t;


typedef enum
{
    /** Active high repetitive pulse.  */
    TC_MODE_PULSE,
    /** Active low repetitive pulse.  */
    TC_MODE_PULSE_INVERT,
    /** Active high single pulse.  */
    TC_MODE_PULSE_ONESHOT,
    /** Active low single pulse.  */
    TC_MODE_PULSE_ONESHOT_INVERT,
    /** Drive output high after delay.  */
    TC_MODE_DELAY_ONESHOT,
    /** Generate square wave (or close to it).  */
    TC_MODE_CLOCK,
    /** Capture and measure an external signal.  */
    TC_MODE_CAPTURE_RISE_RISE,
    TC_MODE_CAPTURE_RISE_FALL,
    TC_MODE_CAPTURE_FALL_RISE,
    TC_MODE_CAPTURE_FALL_FALL,
    /** Triggering of ADC (this doesn't use the TC pin).  */
    TC_MODE_ADC,
    /** Free running counter (this doesn't use the TC pin).  */
    TC_MODE_COUNTER,
    /** Periodic interrupt (this doesn't use the TC pin).  */
    TC_MODE_INTERRUPT
} tc_mode_t;


typedef enum
{
    TC_CAPTURE_A,
    TC_CAPTURE_B
} tc_capture_t;


typedef enum
{
    TC_OK = 0,
    TC_ERROR_PRESCALE = -1,
    TC_ERROR_CHANNEL = -2,
    TC_ERROR_MODE = -3
} tc_ret_t;


/* The counters are only 16 bit but we synthesise a large counter
   by counting timer overflows.  */
typedef uint64_t tc_counter_t;


typedef uint16_t tc_period_t;


typedef uint32_t tc_frequency_t;


typedef uint16_t tc_capture_mask_t;


typedef uint16_t tc_prescale_t;


typedef void (* tc_onload_function) (int);


/** TC configuration structure.  */
typedef struct
{
    pio_t pio;
    tc_mode_t mode;
    tc_prescale_t prescale;     /* 2, 8, 32, 128.  0 defaults to 2.  */
    /* If frequency is non-zero then it overrides period and delay.  */
    tc_period_t period;         /* Clocks */
    tc_period_t delay;          /* Clocks */
    tc_frequency_t frequency;   /* Hz */
} tc_cfg_t;


/** Include definition of data structures required for the driver
    implementation.  Do not use.  */
#include "tc_private.h"


/** Define datatype for handle to TC functions.  */
typedef tc_dev_t *tc_t;


/** Change TC configuration.  */
tc_ret_t
tc_config_set (tc_t tc, const tc_cfg_t *cfg);


tc_ret_t
tc_start (tc_t tc);


tc_ret_t
tc_stop (tc_t tc);


tc_counter_t
tc_counter_get (tc_t tc);


tc_counter_t
tc_capture_get (tc_t tc, tc_capture_t reg);
              

tc_capture_mask_t
tc_capture_poll (tc_t tc);


/** Get the delay in clocks.  */
tc_period_t
tc_delay_get (tc_t tc);


/** Get the period in clocks.  */
tc_period_t
tc_period_get (tc_t tc);


tc_period_t
tc_period_set (tc_t tc, tc_period_t period);


/** Set the TC output frequency in Hz.  This returns the actual
    frequency, closest to the desired frequency.  It also sets the
    duty factor to 0.5.  */
tc_frequency_t
tc_frequency_set (tc_t tc, tc_frequency_t frequency);


tc_prescale_t
tc_prescale_set (tc_t tc, tc_prescale_t prescale);


tc_t 
tc_init (const tc_cfg_t *cfg);


void
tc_shutdown (tc_t tc);


void
tc_clock_sync (tc_t tc, tc_period_t period);


void
tc_onload_set (tc_t tc, tc_onload_function onload);


void
tc_sync (void);


#endif

