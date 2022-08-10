/** @file   dac.h
    @author M. P. Hayes, UCECE
    @date   3 Feb 2005

    @brief Routines to use AT91 onboard DAC in polling mode.

    Note, the DAC switches between Vref / 6 to 5 * Vref / 6.

*/

#ifndef DAC_H
#define DAC_H

#ifdef __cplusplus
extern "C" {
#endif


#include "config.h"

/** DAC channels.  */
typedef enum
{
    DAC_CHANNEL_0, DAC_CHANNEL_1, DAC_CHANNEL_NUM
} dac_channel_t;


typedef enum
{
    DAC_TRIGGER_SW,
    DAC_TRIGGER_EXT,
    DAC_TRIGGER_TC0, DAC_TRIGGER_TC1, DAC_TRIGGER_TC2,
    DAC_TRIGGER_PWM0, DAC_TRIGGER_PWM1
} dac_trigger_t;


/** DAC sample size.  */
typedef uint16_t dac_sample_t;

/** Bit mask to specify enabled channels.  */
typedef uint16_t dac_channels_t;


typedef uint16_t dac_clock_divisor_t;


typedef uint32_t dac_clock_speed_t;


typedef struct dac_dev_struct
{
    dac_channels_t channels;
    dac_trigger_t trigger;
    dac_clock_divisor_t clock_divisor;
    uint16_t refresh_clocks;
    uint8_t bits;
    uint32_t MR;
} dac_dev_t;


typedef dac_dev_t *dac_t;


typedef struct dac_cfg_struct
{
    /* This specifies the channel if the channels field is zero.  */
    dac_channel_t channel;

    /* This specifies the channels to convert as a bitmask.  Note,
       if this is non-zero, then tagging is required in the 4 MSBs of the
       data to specify the channel.  */
    dac_channels_t channels;

    /* Conversion bits.  */
    uint8_t bits;

    /* Trigger source.  */
    dac_trigger_t trigger;

    /* Clock speed in kHz (maximum).  */
    dac_clock_speed_t clock_speed_kHz;

    /* Number of clocks between refreshes.  */
    uint16_t refresh_clocks;
} dac_cfg_t;


/** Set the DAC triggering.  This does not take affect until
    dac_config called.  */
void
dac_trigger_set (dac_t dac, dac_trigger_t trigger);


/** Set clock speed.  This does not take affect until dac_config
    called.  */
dac_clock_speed_t
dac_clock_speed_kHz_set (dac_t dac, dac_clock_speed_t clock_speed_kHz);


/** Set number of bits to convert.  This does not take affect until
    dac_config called.  */
uint8_t
dac_bits_set (dac_t dac, uint8_t bits);


/** Set the channels to convert.  This does not take affect until
    dac_config called.  */
bool
dac_channels_set (dac_t dac, dac_channels_t channels);


/** Configure DAC peripheral registers in preparation for a conversion.
    This is only needed for nefarious purposes since this gets performed
    at the start of dac_read.  */
bool
dac_config (dac_t dac);


/** Returns true if FIFO can be written.  */
bool
dac_ready_p (dac_t dac);


/** Returns true if a conversion has finished.  */
bool
dac_conversion_finished_p (dac_t dac);


/** Blocking write.  */
int8_t
dac_write (dac_t dac, void *buffer, uint16_t size);


/** Puts DAC into sleep mode.  */
void
dac_sleep (dac_t dac);


Pdc *
dac_pdc_get (dac_t dac);


void
dac_enable (dac_t dac);


void
dac_disable (dac_t dac);


uint32_t
dac_isr_get (dac_t dac);


/** Initalises the DAC registers for specified configuration.  */
dac_t
dac_init (const dac_cfg_t *cfg);


void
dac_shutdown (dac_t dac);


static inline void
DAC_WRITE (dac_t dac, uint32_t data)
{
    DACC->DACC_CDR = data;
}


#ifdef __cplusplus
}
#endif
#endif
