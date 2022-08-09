/** @file   adc.h
    @author M. P. Hayes, UCECE
    @date   3 Feb 2005

    @brief Routines to use AT91 onboard ADC in polling mode.
*/

#ifndef ADC_H
#define ADC_H

#ifdef __cplusplus
extern "C" {
#endif


#include "config.h"
#include "sys.h"


/** ADC channels.  Note, channel 15 on the SAM4S is the temp. sensor.  */
typedef enum
{
    ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3,
    ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7,
    ADC_CHANNEL_8, ADC_CHANNEL_9, ADC_CHANNEL_10, ADC_CHANNEL_11,
    ADC_CHANNEL_12, ADC_CHANNEL_13, ADC_CHANNEL_14, ADC_CHANNEL_15,
    ADC_CHANNEL_NUM
} adc_channel_t;


typedef enum
{
    ADC_TRIGGER_SW,
    ADC_TRIGGER_EXT,
    ADC_TRIGGER_TC0, ADC_TRIGGER_TC1, ADC_TRIGGER_TC2,
    ADC_TRIGGER_PWM0, ADC_TRIGGER_PWM1
} adc_trigger_t;


typedef enum
{
    ADC_COMPARISON_MODE_LOW,
    ADC_COMPARISON_MODE_HIGH,
    ADC_COMPARISON_MODE_IN,
    ADC_COMPARISON_MODE_OUT
} adc_comparison_mode_t;


/** ADC sample size.  */
typedef uint16_t adc_sample_t;

/** Bit mask to specify enabled channels.  */
typedef uint16_t adc_channels_t;


typedef uint16_t adc_clock_divisor_t;


typedef uint32_t adc_clock_speed_t;


typedef struct adc_dev_struct
{
    adc_channels_t channels;
    adc_trigger_t trigger;
    adc_clock_divisor_t clock_divisor;
    uint8_t bits;
    uint32_t MR;
    uint32_t EMR;
    uint32_t CWR;
    bool tag;
} adc_dev_t;


typedef adc_dev_t *adc_t;


typedef struct adc_cfg_struct
{
    /* This specifies the channel if the channels field is zero.  */
    adc_channel_t channel;

    /* This specifies the channels to convert as a bitmask.  */
    adc_channels_t channels;

    /* Conversion bits.  */
    uint8_t bits;

    /* Trigger source.  */
    adc_trigger_t trigger;

    /* Clock speed in kHz (maximum).  */
    adc_clock_speed_t clock_speed_kHz;
} adc_cfg_t;


/** Set the ADC triggering.  This does not take affect until
    adc_config called.  */
void
adc_trigger_set (adc_t adc, adc_trigger_t trigger);


/** Set clock speed.  This does not take affect until adc_config
    called.  */
adc_clock_speed_t
adc_clock_speed_kHz_set (adc_t adc, adc_clock_speed_t clock_speed_kHz);


/** Set number of bits to convert.  This does not take affect until
    adc_config called.  */
uint8_t
adc_bits_set (adc_t adc, uint8_t bits);


/** Set the channels to convert.  This does not take affect until
    adc_config called.  */
bool
adc_channels_set (adc_t adc, adc_channels_t channels);


/** The ADC can generate an event if the ADC value is above a high
    threshold, below a low threshold, between the thresholds, or
    outside the thresholds.  This does not take affect until
    adc_config called.  */
int8_t
adc_comparison_set (adc_t adc, adc_channel_t channel, bool all_channels,
                    adc_comparison_mode_t mode, adc_sample_t low_threshold,
                    adc_sample_t high_threshold);


/** When set, the channel index is appended to the conversion data in
    the MSBs.  This does not take affect until adc_config called.  */
void
adc_tag_set (adc_t adc, bool tag);


/** Configure ADC peripheral registers in preparation for a conversion.
    This is only needed for nefarious purposes since this gets performed
    at the start of adc_read.  */
bool
adc_config (adc_t adc);


/** Returns true if a conversion has finished.  */
bool
adc_ready_p (adc_t adc);


/** Blocking read.  */
ssize_t
adc_read (adc_t adc, void *buffer, size_t size);


/** Puts ADC into sleep mode.  */
void
adc_sleep (adc_t adc);


Pdc *
adc_pdc_get (adc_t adc);


void
adc_enable (adc_t adc);


void
adc_disable (adc_t adc);


/** Returns true if a comparison event detected.  */
bool
adc_comparison_p (adc_t adc);


/** Perform a calibration cycle.  */
void
adc_calibrate (adc_t adc);


/** Initalises the ADC registers for specified configuration.  */
adc_t
adc_init (const adc_cfg_t *cfg);


void
adc_shutdown (adc_t adc);


void
adc_reset (void);


int16_t *adc_convert_bipolar (adc_sample_t *src, int16_t *dst, uint16_t samples);


#ifdef __cplusplus
}
#endif
#endif
