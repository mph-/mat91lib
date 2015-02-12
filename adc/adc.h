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

/** ADC reference modes.  */
typedef enum {ADC_REF_EXT = 0} adc_ref_mode_t;


typedef enum 
{
    ADC_TRIGGER_EXT, ADC_TRIGGER_TC0, ADC_TRIGGER_TC1, ADC_TRIGGER_TC2,
    ADC_TRIGGER_PWM0, ADC_TRIGGER_PWM1, ADC_TRIGGER_SW = 7
} adc_trigger_t;


/** ADC sample size.  */
typedef uint16_t adc_sample_t;


typedef uint16_t adc_clock_divisor_t;


typedef uint32_t adc_clock_speed_t;


typedef struct adc_dev_struct
{
    adc_channel_t channel;
    adc_clock_divisor_t clock_divisor;
} adc_dev_t;


typedef adc_dev_t *adc_t;



typedef struct adc_cfg_struct
{
    /* Conversion bits.  */
    uint8_t bits;

    /* Trigger source.  */
    adc_trigger_t trigger;

    /* Clock speed in kHz (maximum).  */
    adc_clock_speed_t clock_speed_kHz;    
} adc_cfg_t;



/** Select ADC reference mode.  */
void 
adc_reference_select (adc_t adc, adc_ref_mode_t mode);


uint8_t
adc_bits_get (adc_t adc);


uint8_t
adc_bits_set (adc_t adc, uint8_t bits);


/** Returns true if a conversion is not in progress.  */
bool
adc_ready_p (adc_t adc);


/** Blocking read.  */
int8_t
adc_read (adc_t adc, adc_sample_t *buffer, uint16_t size);


int8_t
adc_read_channel (adc_t adc, adc_channel_t channel, adc_sample_t *buffer,
                  uint16_t size);


/** Start conversion on selected channel, wait until conversion finished.  */
int8_t
adc_read_wait (adc_t adc, adc_channel_t channel, adc_sample_t *pvalue);


/** Halts any currently running conversion.  */
void 
adc_stop (adc_t adc);


/** Disables the ADC from doing anything.  Requires reinitialisation.  */
void
adc_disable (adc_t adc);


/** Puts ADC into sleep mode.  */
void
adc_sleep (adc_t adc);


/** Initalises the ADC registers for polling operation.  */
adc_t 
adc_init (adc_cfg_t *cfg);

#endif
