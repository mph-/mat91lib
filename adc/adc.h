/** @file   adc.h
    @author M. P. Hayes, UCECE
    @date   3 Feb 2005

    @brief Routines to use AT91 onboard ADC in polling mode.
*/

#ifndef ADC_H
#define ADC_H

#include "config.h"

/** ADC channels.  */
typedef enum
{
    ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3, 
    ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7,
    ADC_CHANNEL_NUM
} adc_channel_t;


typedef enum 
{
    ADC_TRIGGER_SW,
    ADC_TRIGGER_EXT,
    ADC_TRIGGER_TC0, ADC_TRIGGER_TC1, ADC_TRIGGER_TC2,
    ADC_TRIGGER_PWM0, ADC_TRIGGER_PWM1
} adc_trigger_t;


/** ADC sample size.  */
typedef uint16_t adc_sample_t;


typedef uint16_t adc_clock_divisor_t;


typedef uint32_t adc_clock_speed_t;


typedef struct adc_dev_struct
{
    adc_channel_t channel;
    adc_trigger_t trigger;
    adc_clock_divisor_t clock_divisor;
    uint8_t bits;
    uint32_t MR;
} adc_dev_t;


typedef adc_dev_t *adc_t;


typedef struct adc_cfg_struct
{
    /* Could have a channels field if need to convert multiple channels
       in a sequence.  */
    adc_channel_t channel;

    /* Conversion bits.  */
    uint8_t bits;

    /* Trigger source.  */
    adc_trigger_t trigger;

    /* Clock speed in kHz (maximum).  */
    adc_clock_speed_t clock_speed_kHz;    
} adc_cfg_t;


void
adc_trigger_set (adc_t adc, adc_trigger_t trigger);


adc_clock_speed_t
adc_clock_speed_kHz_set (adc_t adc, adc_clock_speed_t clock_speed_kHz);


uint8_t
adc_bits_set (adc_t adc, uint8_t bits);


bool
adc_channel_set (adc_t adc, adc_channel_t channel);


/** Configure ADC peripheral registers in preparation for a conversion.
    This is only needed for nefarious purposes since this gets performed
    at the start of adc_read.  */
bool
adc_config (adc_t adc);


/** Returns true if a conversion has finished.  */
bool
adc_ready_p (adc_t adc);


/** Blocking read.  */
int8_t
adc_read (adc_t adc, void *buffer, uint16_t size);


/** Puts ADC into sleep mode.  */
void
adc_sleep (adc_t adc);


Pdc *
adc_pdc_get (adc_t adc);


void
adc_enable (adc_t adc);


void
adc_disable (adc_t adc);


/** Initalises the ADC registers for specified configuration.  */
adc_t 
adc_init (const adc_cfg_t *cfg);


void
adc_shutdown (adc_t adc);

#endif
