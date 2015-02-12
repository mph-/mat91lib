/** @file   adc.c
    @author 
    @date   12 February 2008
    @brief  Analogue to digital converter routines for AT91SAM7 processors
*/

#include "adc.h"
#include "bits.h"
#include "mcu.h"

/* On reset the the PIO pins are configured as inputs with pullups.
   To make a PIO pin an ADC pin requires programming ADC_CHER.
   
   SAM4S: 16 channels 10/12 bit (1 MHz max).
   SAM7: 8 channels, 8/10 bits.

   SAM4S: The ADC clock is MCK/2 to MCK/512.
   SAM7: The ADC clock is MCK/2 to MCK/128.

   Each channel has its own channel data register ADC_CDR and an end
   of conversion (EOC) bit in the ADC status register ADC_SR.  When a
   conversion is finished the ADC_CDR is written as well as the last
   converted data register ADC_LCDR.

   The ADC peripheral is designed so that multiple channels can be
   enabled at once.  When it gets a trigger it samples each enabled
   channel in turn and writes to the corresponding ADC_CDR registers.
   It then waits for the next trigger.

   SAM4S: A repetitive 16 channel sequence can be programmed using
   ADC_SEQR1 and ADC_SEQR2 (enabled by USEQ in ADC_MR).

   The minimum impedance (ohms) for the SAM7S driving the ADC is given by:
   
   ZOUT <= (SHTIM - 470) x 10 in 8-bit resolution mode
   ZOUT <= (SHTIM - 589) x 7.69 in 10-bit resolution mode
   
   where SHTIM is the sample/hold time in ns.

   The SAM4S can tolerate higher impedance inputs.
*/


#define ADC_8BIT_CLOCK_MAX 8e6
#define ADC_10BIT_CLOCK_MAX 5e6
#define ADC_STARTUP_TIME_MIN 20e-6
#define ADC_SAMPLE_TIME_MIN 600e-9


#ifndef ADC_8BIT_CLOCK
#define ADC_8BIT_CLOCK ADC_8BIT_CLOCK_MAX
#endif


#ifndef ADC_10BIT_CLOCK
#define ADC_10BIT_CLOCK ADC_10BIT_CLOCK_MAX
#endif


#ifndef ADC_STARTUP_TIME
#define ADC_STARTUP_TIME ADC_STARTUP_TIME_MIN
#endif


#ifndef ADC_SAMPLE_TIME
#define ADC_SAMPLE_TIME ADC_SAMPLE_TIME_MIN
#endif


#define ADC_8BIT_PRESCALE ((uint8_t)(F_CPU / (2 * ADC_8BIT_CLOCK) + 0.5 - 1))
#define ADC_10BIT_PRESCALE ((uint8_t)(F_CPU / (2 * ADC_10BIT_CLOCK) + 0.5 - 1))


/* Determine actual ADC clock.  */
#define ADC_8BIT_CLOCK1 (F_CPU / ((ADC_8BIT_PRESCALE + 1) * 2))
#define ADC_10BIT_CLOCK1 (F_CPU / ((ADC_10BIT_PRESCALE + 1) * 2))

#define ADC_8BIT_STARTUP ((uint8_t)(ADC_STARTUP_TIME * ADC_8BIT_CLOCK1 / 8 + 0.5 - 1))
#define ADC_10BIT_STARTUP ((uint8_t)(ADC_STARTUP_TIME * ADC_10BIT_CLOCK1 / 8 +0.5 - 1))

#define ADC_8BIT_SHTIM ((uint8_t)(ADC_SAMPLE_TIME * ADC_8BIT_CLOCK1 + 0.5 - 1))
#define ADC_10BIT_SHTIM ((uint8_t)(ADC_SAMPLE_TIME * ADC_10BIT_CLOCK1 + 0.5 - 1))

static adc_dev_t adc_dev;


/* Resets ADC.  */
static void
adc_reset (void)
{
    ADC->ADC_CR = ADC_CR_SWRST;
}


/** Puts ADC into sleep mode.
    It should wake on next trigger.  */
void
adc_sleep (adc_t adc)
{
    adc_sample_t dummy;

    /*  Errata for SAM7S256:RevisionB states that the ADC will not be
        placed into sleep mode until a conversion has completed.  */
    adc_init (0);
    ADC->ADC_MR |= ADC_MR_SLEEP;
    adc_read_channel (adc, 0, &dummy, sizeof (dummy));
}


/* Set the ADC triggering.  */
static void
adc_trigger_set (adc_t adc, adc_trigger_t trigger) 
{
    adc->trigger = trigger;

    /* Could also handle FREERUN here where no triggering is
       required.  */

    if (trigger == ADC_TRIGGER_SW)
    {
        BITS_INSERT(ADC->ADC_MR, 0, 0, 1);
    }
    else
    {
        BITS_INSERT(ADC->ADC_MR, (trigger - ADC_TRIGGER_EXT) << 1, 0, 3);
    }
}


/* Set the clock divider (prescaler).  */
static void
adc_clock_divisor_set (adc_t adc, adc_clock_divisor_t clock_divisor) 
{
    BITS_INSERT(ADC->ADC_MR, clock_divisor, 8, 15);
    adc->clock_divisor = clock_divisor;
}


adc_clock_speed_t
adc_clock_speed_kHz_set (adc_t adc, adc_clock_speed_t clock_speed_kHz)
{
    uint32_t clock_speed;
    
    clock_speed = clock_speed_kHz * 1000;
    adc_clock_divisor_set (adc, ((F_CPU / 2) + clock_speed - 1) / clock_speed);
    clock_speed = (F_CPU / 2) / adc->clock_divisor;
    
    return clock_speed / 1000;
}



bool
adc_channel_enable (adc_t adc, adc_channel_t channel)
{
    if (channel >= ADC_CHANNEL_NUM)
	return 0;
    adc->channel = channel;

    ADC->ADC_CHER = BIT (channel);
    return 1;
}


bool
adc_channel_disable (adc_t adc, adc_channel_t channel)
{
    if (channel >= ADC_CHANNEL_NUM)
	return 0;

    ADC->ADC_CHDR = BIT (channel);
    return 1;
}


static void
adc_conversion_start (adc_t adc)
{
    /* Software trigger.  */
    ADC->ADC_CR = ADC_CR_START;
}


/** Select ADC reference mode.  */
void 
adc_reference_select (adc_t adc, adc_ref_mode_t mode __UNUSED__)
{
}


#ifdef __SAM4S__
static bool
adc_resolution_set (adc_t adc, uint8_t resolution)
{
    switch (resolution)
    {
         case 12:

            ADC->ADC_MR &= ~ADC_MR_LOWRES;

             /* Startup time.  */
            BITS_INSERT (ADC->ADC_MR, ADC_10BIT_STARTUP, 16, 20);
            /* Sample and hold time.  */
            BITS_INSERT (ADC->ADC_MR, ADC_10BIT_SHTIM, 24, 27);
            break;

         case 10:
            ADC->ADC_MR |= ADC_MR_LOWRES;

             /* Startup time.  */
            BITS_INSERT (ADC->ADC_MR, ADC_8BIT_STARTUP, 16, 20);
            /* Sample and hold time.  */
            BITS_INSERT (ADC->ADC_MR, ADC_8BIT_SHTIM, 24, 27);
            break;

        default:
            return 0;
            break;
    }
    return resolution;
}
#else
static bool
adc_resolution_set (adc_t adc, uint8_t resolution)
{
    switch (resolution)
    {
         case 10:

            ADC->ADC_MR &= ~ADC_MR_LOWRES;

             /* Startup time.  */
            BITS_INSERT (ADC->ADC_MR, ADC_10BIT_STARTUP, 16, 20);
            /* Sample and hold time.  */
            BITS_INSERT (ADC->ADC_MR, ADC_10BIT_SHTIM, 24, 27);
            break;

         case 8:
            ADC->ADC_MR |= ADC_MR_LOWRES;

             /* Startup time.  */
            BITS_INSERT (ADC->ADC_MR, ADC_8BIT_STARTUP, 16, 20);
            /* Sample and hold time.  */
            BITS_INSERT (ADC->ADC_MR, ADC_8BIT_SHTIM, 24, 27);
            break;

        default:
            return 0;
            break;
    }
    return resolution;
}
#endif


void 
adc_config (adc_t adc, adc_cfg_t *cfg)
{
    adc_resolution_set (adc, cfg->bits);
    adc_trigger_set (adc, cfg->trigger);
    adc_clock_speed_kHz_set (adc, cfg->clock_speed_kHz);
}


/** Initalises the ADC registers for polling operation.  */
adc_t
adc_init (adc_cfg_t *cfg)
{
    adc_sample_t dummy;
    adc_dev_t *adc;

    adc = &adc_dev;

    /* The clock only needs to be enabled when sampling.  The clock is
       automatically started for the SAM7.  */
    mcu_pmc_enable (ID_ADC);

    adc_reset ();

    if (cfg)
    {
        adc_config (adc, cfg);
    }
    else
    {
        adc_resolution_set (adc, 10);
        adc_clock_speed_kHz_set (adc, 5000);
        adc_trigger_set (adc, ADC_TRIGGER_SW);
    }

    /* I'm not sure why a dummy read is required; it is probably a
       quirk of the SAM7.  */
    adc_read_channel (adc, 0, &dummy, sizeof (dummy));

    return adc;
}


/** Returns true if a conversion has finished.  */
bool
adc_ready_p (adc_t adc)
{
    /* FIXME for SAM4S.  */
    return (ADC->ADC_ISR & ADC_ISR_DRDY) != 0;
}


/** Blocking read.  */
int8_t
adc_read (adc_t adc, adc_sample_t *buffer, uint16_t size)
{
    uint16_t i;
    uint16_t samples;

    samples = size / sizeof (adc_sample_t);

    if (!adc_channel_enable (adc, adc->channel))
        return 0;

    for (i = 0; i < samples; i++)
    {
        if (adc->trigger = ADC_TRIGGER_SW)
            adc_conversion_start (adc);

        /* Should have timeout, especially for external trigger.  */
        while (!adc_ready_p (adc))
            continue;

        buffer[i] = ADC->ADC_LCDR;
    }

    /* Disable channel.  */
    ADC->ADC_CHDR = ~0;

    return samples * sizeof (adc_sample_t);
}


/** Blocking read from specified ADC channel.  */
int8_t
adc_read_channel (adc_t adc, adc_channel_t channel, adc_sample_t *buffer,
                  uint16_t size)
{
    adc_channel_enable (adc, channel);

    return adc_read (adc, buffer, size);
}


bool
adc_start (adc_t adc)
{
    return 0;
}


/** Halts any currently running conversion.  */
void 
adc_stop (adc_t adc)
{
}


/** Disables the ADC from doing anything.  */
void
adc_disable (adc_t adc)
{
}
