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
typedef enum {ADC_REF_EXT = 0}
    adc_ref_mode_t;

/** ADC bits per conversion.  */
enum {ADC_BITS = 10};

/** ADC sample size.  */
typedef uint16_t adc_sample_t;


/** Select ADC reference mode.  */
extern void 
adc_reference_select (adc_ref_mode_t mode);


/** Initalises the ADC registers for polling operation.  */
extern void 
adc_init (uint8_t channels);


/** Starts a conversion in the ADC on the specified channel.  */
extern bool
adc_conversion_start (adc_channel_t channel);


/** Returns true if a conversion is not in progress.  */
extern bool
adc_ready_p (void);


/** Returns 1 if valid sample read.  */
extern int8_t
adc_read (adc_sample_t *value);


/** Start conversion on selected channel, wait until conversion finished.  */
extern int8_t
adc_read_wait (adc_channel_t channel, adc_sample_t *pvalue);


/** Halts any currently running conversion.  */
extern void 
adc_stop (void);


/** Disables the ADC from doing anything.  Requires reinitalisation.  */
extern void
adc_disable (void);


/** Puts ADC into sleep mode.  */
extern void
adc_sleep (void);

#endif
