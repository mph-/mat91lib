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

   The SAM4S requires 20 clocks per sample so for a maximum sample
   rate of 1 MHz then a clock speed of 20 MHz is required.    

   TODO. Add gain setting.
*/


/* There is no limit.  */
#ifndef ADC_DEVICES_NUM
#define ADC_DEVICES_NUM 8
#endif


#ifdef __SAM4S__
#define ADC_STARTUP_TIME_MIN 12e-6
#define ADC_TRACK_TIME_MIN 160e-9
#define ADC_SETTLE_TIME_MIN 200e-9
#else
#define ADC_STARTUP_TIME_MIN 20e-6
#define ADC_TRACK_TIME_MIN 600e-9
#define ADC_SETTLE_TIME_MIN 200e-9
#endif


/* Startup time from standby mode to normal mode.  */
#ifndef ADC_STARTUP_TIME
#define ADC_STARTUP_TIME ADC_STARTUP_TIME_MIN
#endif


/* Track and hold time.  */
#ifndef ADC_TRACK_TIME
#define ADC_TRACK_TIME ADC_TRACK_TIME_MIN
#endif


/* Settling time after changing gain or offset.  */
#ifndef ADC_SETTLE_TIME
#define ADC_SETTLE_TIME ADC_SETTLE_TIME_MIN
#endif


static uint8_t adc_devices_num = 0;
static adc_dev_t adc_devices[ADC_DEVICES_NUM];
static adc_dev_t *adc_config_last = 0;


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
    adc = adc_init (0);
    ADC->ADC_MR |= ADC_MR_SLEEP;
    adc_read (adc, &dummy, sizeof (dummy));
}


/* Set the ADC triggering.  */
void
adc_trigger_set (adc_t adc, adc_trigger_t trigger) 
{
    adc->trigger = trigger;

    /* Could also handle FREERUN here where no triggering is
       required.  */

    if (trigger == ADC_TRIGGER_SW)
    {
        BITS_INSERT (adc->MR, 0, 0, 0);
    }
    else
    {
        /* Select trigger.  */
        BITS_INSERT (adc->MR, trigger - ADC_TRIGGER_EXT, 1, 3);

        /* Enable trigger.  */
        BITS_INSERT (adc->MR, 1, 0, 0);
    }
}


/* Set the clock divider (prescaler).  */
static void
adc_clock_divisor_set (adc_t adc, adc_clock_divisor_t clock_divisor) 
{
    /* The SAM4S requires 20 clocks per sample. 

       ADC_CLOCK = (F_CPU / 2) / clock_divisor.  
    */

    if (clock_divisor >= 256)
        clock_divisor = 256;

    BITS_INSERT (adc->MR, clock_divisor - 1, 8, 15);
    adc->clock_divisor = clock_divisor;
}


adc_clock_speed_t
adc_clock_speed_kHz_set (adc_t adc, adc_clock_speed_t clock_speed_kHz)
{
    uint32_t clock_speed;
    uint16_t settle_clocks;
    uint16_t sample_clocks;
    static const uint8_t adc_settle_table[] = {3, 5, 9, 17};
    static const uint16_t adc_sample_table[] = 
    {0, 8, 16, 24, 64, 80, 96, 112, 512, 576, 640, 704, 768, 832, 896, 960};

    /* For the SAM7 the max clock speed is 5 MHz for 10 bit and 8 MHz
       for 8 bit.  */

    clock_speed = clock_speed_kHz * 1000;
    adc_clock_divisor_set (adc, ((F_CPU / 2) + clock_speed - 1) / clock_speed);
    clock_speed = (F_CPU / 2) / adc->clock_divisor;

    /* STARTUP: With 24 MHz clock need 288 clocks to start up on
       SAM4S.  Let's allocate 512.  TODO, scan through table to find
       appropriate value.  */
    BITS_INSERT (adc->MR, 8, 16, 19);

    /* SETTLING: With 24 MHz clock need 4.8 clocks to settle on SAM4S.
       This is only needed when switching gain or offset, say when
       converting a sequence of channels.  Let's play safe and
       allocate the maximum 17 clocks.  */
    BITS_INSERT (adc->MR, 3, 20, 21);

    /* TRACKTIM: With 24 MHz clock need 3.4 clocks to sample on SAM4S.
       Let's allocate 4.  */
    BITS_INSERT (adc->MR, 3, 24, 27);

    return clock_speed / 1000;
}


/* This does not select the channel until adc_channel_enable called.  */
bool
adc_channel_set (adc_t adc, adc_channel_t channel)
{
    if (channel >= ADC_CHANNEL_NUM)
	return 0;
    adc->channel = channel;

    return 1;
}


static void
adc_channel_enable (adc_t adc)
{
    ADC->ADC_CHER = BIT (adc->channel);
}


static void
adc_channel_disable (adc_t adc)
{
    ADC->ADC_CHDR = BIT (adc->channel);
}


static void
adc_conversion_start (adc_t adc)
{
    /* Software trigger.  */
    ADC->ADC_CR = ADC_CR_START;
}


uint8_t
adc_bits_set (adc_t adc, uint8_t bits)
{
    switch (bits)
    {
#ifdef __SAM4S__
         case 12:
            adc->MR &= ~ADC_MR_LOWRES;
            break;

         case 10:
            adc->MR |= ADC_MR_LOWRES;
            break;
#else
         case 10:

            adc->MR &= ~ADC_MR_LOWRES;
            break;

         case 8:
            adc->MR |= ADC_MR_LOWRES;
            break;            
#endif

        default:
            return 0;
            break;
    }

    adc->bits = bits;
    return bits;
}


bool
adc_config (adc_t adc)
{
    adc_channel_enable (adc);

    if (adc == adc_config_last)
        return 1;
    adc_config_last = adc;

    /* Set mode register.  */
    ADC->ADC_MR = adc->MR;
    return 1;
}


static void
adc_config_set (adc_t adc, const adc_cfg_t *cfg)
{
    adc_bits_set (adc, cfg->bits);
    adc_clock_speed_kHz_set (adc, cfg->clock_speed_kHz);
    adc_trigger_set (adc, cfg->trigger);
    adc_channel_set (adc, cfg->channel);
}


Pdc *
adc_pdc_get (adc_t adc)
{
    return PDC_ADC;
}


void
adc_enable (adc_t adc)
{
    /* Dummy function for symmetry with ssc driver.  */
}


void
adc_disable (adc_t adc)
{
    /* Dummy function for symmetry with ssc driver.  */
}


/** Initalises the ADC registers for polling operation.  */
adc_t
adc_init (const adc_cfg_t *cfg)
{
    adc_sample_t dummy;
    adc_dev_t *adc;
    const adc_cfg_t adc_default_cfg =
        {
            .bits = 10,
            .channel = 0,
            .clock_speed_kHz = 1000
        };
    
    if (adc_devices_num >= ADC_DEVICES_NUM)
        return 0;

    if (adc_devices_num == 0)
    {
        /* The clock only needs to be enabled when sampling.  The clock is
           automatically started for the SAM7.  */
        mcu_pmc_enable (ID_ADC);
        
        adc_reset ();
    }

    adc = adc_devices + adc_devices_num;
    adc_devices_num++;
    
    adc->MR = 0;
    /* The transfer field must have a value of 2.  */
    BITS_INSERT (adc->MR, 2, 28, 29);

    if (!cfg)
        cfg = &adc_default_cfg;

    adc_config_set (adc, cfg);

    /* Note, the ADC is not configured until adc_config is called.  */
    adc_config (adc);

#if 0
    /* I'm not sure why a dummy read is required; it is probably a
       quirk of the SAM7.  This will require a software trigger... */
    adc_read (adc, &dummy, sizeof (dummy));
#endif

    return adc;
}


/** Returns true if a conversion has finished.  */
bool
adc_ready_p (adc_t adc)
{
    return (ADC->ADC_ISR & ADC_ISR_DRDY) != 0;
}


/** Blocking read.  This will hang if a trigger is not supplied
    (except for software triggering mode).  */
int8_t
adc_read (adc_t adc, void *buffer, uint16_t size)
{
    uint16_t i;
    uint16_t samples;
    adc_sample_t *data;

    adc_config (adc);

    samples = size / sizeof (adc_sample_t);
    data = buffer;

    for (i = 0; i < samples; i++)
    {
        /* When the ADC peripheral gets a trigger, it converts all the
           enabled channels consecutively in numerical order.  */

        if (adc->trigger == ADC_TRIGGER_SW)
            adc_conversion_start (adc);

        /* Should have timeout, especially for external trigger.  */
        while (!adc_ready_p (adc))
            continue;

        data[i] = ADC->ADC_LCDR;
    }

    /* Disable channel.  */
    ADC->ADC_CHDR = ~0;

    return samples * sizeof (adc_sample_t);
}


void
adc_shutdown (adc_t adc)
{
    /* TODO.  */
    mcu_pmc_disable (ID_ADC);
}
