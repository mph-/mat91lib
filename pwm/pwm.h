/** @file   pwm.h
    @author 
    @date   13 February 2008
    @brief  Pulse Width Modulation routines for AT91SAM7 processors.
    @note   The API needs a complete re-write.
*/

#ifndef PWM_H
#define PWM_H

#include "config.h"
#include "pio.h"


#define PWM_PERIOD_DIVISOR(FREQ) ((pwm_period_t)(F_CPU / (FREQ)))

#define PWM_DUTY_DIVISOR(FREQ, DUTY_PERCENT) \
    ((pwm_period_t)((F_CPU * (DUTY_PERCENT)) / ((FREQ) * 100)))


typedef enum
{
    PWM_ALIGN_LEFT,
    PWM_ALIGN_CENTRE
} pwm_align_t;


typedef enum
{
    PWM_POLARITY_LOW,
    PWM_POLARITY_HIGH
} pwm_polarity_t;


typedef struct pwm_dev_struct pwm_dev_t;

typedef pwm_dev_t *pwm_t;

typedef uint32_t pwm_period_t;

typedef uint32_t pwm_frequency_t;

typedef struct pwm_cfg_struct
{
    pio_t pio;
    pwm_period_t period;
    pwm_period_t duty;
    pwm_align_t align;
    pwm_polarity_t polarity;
    pio_config_t stop_state;
    pwm_frequency_t frequency;  /* Hz  (overrides period)  */
    uint16_t duty_ppt;          /* parts per thousand (overrides duty)  */
} pwm_cfg_t;

typedef uint8_t pwm_channel_mask_t;


/** Shutdown clock to PWM peripheral.  */
void
pwm_shutdown (void);


/** Initialises PWM on specified pin.  */
pwm_t
pwm_init (const pwm_cfg_t *cfg);


/** Set waveform period (in CPU clocks).  This will change the 
    prescaler as required.  This will block if the PWM is running until 
    the end of a cycle.  */
pwm_period_t
pwm_period_set (pwm_t pwm, pwm_period_t period);


/** Set waveform frequency in Hz.  */
pwm_frequency_t
pwm_frequency_set (pwm_t pwm, pwm_frequency_t frequency);


/** Return the waveform period (in CPU clocks).  */
pwm_period_t
pwm_duty_get (pwm_t pwm);


/** Set waveform duty (in CPU clocks, not a percentage of the period).
    This will block if the PWM is running until the end of a
    cycle.  */
pwm_period_t
pwm_duty_set (pwm_t pwm, pwm_period_t duty);


/** Return the waveform duty (in CPU clocks).  */
pwm_period_t
pwm_duty_get (pwm_t pwm);


/** Set waveform duty (as a fraction of the period in parts per
    thousand).  This will block if the PWM is running until the end of
    a cycle.  */
unsigned int
pwm_duty_ppt_set (pwm_t pwm, unsigned int duty_ppt);


/** Start selected channel.  */
void
pwm_start (pwm_t pwm);


/** Stop selected channel.  */
void
pwm_stop (pwm_t pwm);


/** Start selected channels simultaneously.  */
void
pwm_channels_start (pwm_channel_mask_t channel_mask);


/** Stop selected channels simultaneously.  */
void
pwm_channels_stop (pwm_channel_mask_t channel_mask);


/** Get channel mask.  */
pwm_channel_mask_t
pwm_channel_mask (pwm_t pwm);

#endif






