/** @file   pwm.h
    @author 
    @date   13 February 2008
    @brief  Pulse Width Modulation routines for AT91SAM7 processors
*/

#ifndef PWM_H
#define PWM_H

#include "config.h"


#define PWM_PERIOD_DIVISOR(FREQ) (F_CPU / (FREQ))
#define PWM_DUTY_DIVISOR(FREQ, DUTY_PERCENT) \
    ((F_CPU * (DUTY_PERCENT)) / ((FREQ) * 100))


typedef enum
{
    PWM_CHANNEL_0,
    PWM_CHANNEL_1,
    PWM_CHANNEL_2,
    PWM_CHANNEL_3,
    PWM_CHANNEL_NUM
} pwm_channel_t;
              

typedef uint8_t pwm_channel_mask_t;


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


/* Currently no support for frequencies below 730Hz */


void
pwm_shutdown (void);


/* Initialises pwm on specified pins */
uint8_t
pwm_init (void);


/* Configures the PWM output The period of the waveform is in number
   of MCK ticks.  The duty can be any number less than the period.
   Support is only for fequencies above 750Hz (no prescaling used
   here).  So for 100kHz, period would be 480 with MCK at 48MHz.  */
uint8_t
pwm_config (pwm_channel_t channel, uint16_t period, uint16_t duty,
            pwm_align_t alignment, pwm_polarity_t polarity);


/* Start selected channels simultaneously.  */
void
pwm_start (pwm_channel_mask_t channel_mask);


/* Stop selected channels simultaneously.  */
void
pwm_stop (pwm_channel_mask_t channel_mask);


void
pwm_enable (pwm_channel_t channel);


void
pwm_disable (pwm_channel_t channel);


/* Updates the waveform period and duty.  */
uint8_t
pwm_update (pwm_channel_t channel, uint16_t new_period, uint16_t new_duty);

#endif






