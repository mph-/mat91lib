/** @file   pwm.c
    @author M. P. Hayes
    @date   12 February 2008
    @brief  Pulse Width Modulation routines for AT91SAM7/SAM4S processors.
    This only drives the PWMHx signals; there is no support for the PWMLx
    signals.
*/

#include <errno.h>
#include "mcu.h"
#include "pwm.h"
#include "bits.h"
#include "pinmap.h"


struct pwm_dev_struct
{
    /* Channel base.  */
    PwmCh_num *base;
    const pinmap_t *pin;
    pio_config_t stop_state;
    uint8_t prescale;
    pwm_period_t duty;
    pwm_period_t period;
};


#define PWM_DEVICES_NUM 4

static pwm_dev_t pwm_devices[PWM_DEVICES_NUM];


/* Define known PWMH pins, grouped by channel.  */
static const pinmap_t pwm_pins[] =
{
    {0, PA0_PIO, PIO_PERIPH_A, PWM_POLARITY_HIGH},
    {0, PA11_PIO, PIO_PERIPH_B, PWM_POLARITY_HIGH},
    {0, PA23_PIO, PIO_PERIPH_B, PWM_POLARITY_HIGH},

    {1, PA1_PIO, PIO_PERIPH_A, PWM_POLARITY_HIGH},
    {1, PA12_PIO, PIO_PERIPH_B, PWM_POLARITY_HIGH},
    {1, PA24_PIO, PIO_PERIPH_B, PWM_POLARITY_HIGH},

    {2, PA2_PIO, PIO_PERIPH_A, PWM_POLARITY_HIGH},
    {2, PA13_PIO, PIO_PERIPH_B, PWM_POLARITY_HIGH},
    {2, PA25_PIO, PIO_PERIPH_B, PWM_POLARITY_HIGH},

    {3, PA7_PIO, PIO_PERIPH_B, PWM_POLARITY_HIGH},
    {3, PA14_PIO, PIO_PERIPH_B, PWM_POLARITY_HIGH},

#ifdef _SAM4S_
    /* SAM4S  64 pin.  TODO, support PCx on 100 pin devices.  */
    {0, PB0_PIO, PIO_PERIPH_A, PWM_POLARITY_HIGH},
    {1, PB1_PIO, PIO_PERIPH_A, PWM_POLARITY_HIGH},
    {2, PB4_PIO, PIO_PERIPH_B, PWM_POLARITY_HIGH},
    {3, PA17_PIO, PIO_PERIPH_C, PWM_POLARITY_HIGH},
    {3, PB14_PIO, PIO_PERIPH_B, PWM_POLARITY_HIGH},

    {0, PA19_PIO, PIO_PERIPH_B, PWM_POLARITY_LOW},
    {0, PB5_PIO, PIO_PERIPH_B, PWM_POLARITY_LOW},
    {1, PA20_PIO, PIO_PERIPH_B, PWM_POLARITY_LOW},
    {1, PB12_PIO, PIO_PERIPH_A, PWM_POLARITY_LOW},
    {2, PA30_PIO, PIO_PERIPH_A, PWM_POLARITY_LOW},
    {2, PA16_PIO, PIO_PERIPH_C, PWM_POLARITY_LOW},
    {2, PB13_PIO, PIO_PERIPH_A, PWM_POLARITY_LOW},
    {3, PA15_PIO, PIO_PERIPH_C, PWM_POLARITY_LOW},
#endif
};


#define PWM_PINS_NUM ARRAY_SIZE (pwm_pins)



/** Shutdown clock to PWM peripheral.  */
void
pwm_shutdown (void)
{
    /* Disable PWM peripheral clock.  */
    mcu_pmc_disable (ID_PWM);
}


static void
pwm_prescale_set (pwm_t pwm, uint8_t prescale)
{
    /* Configure prescaler.  */
    BITS_INSERT (pwm->base->PWM_CMR, prescale, 0, 3);
    pwm->prescale = prescale;
}


/** Set waveform period (in CPU clocks).  This will change the
    prescaler as required.  This will block if the PWM is running until
    the end of a cycle.  */
pwm_period_t
pwm_period_set (pwm_t pwm, pwm_period_t period)
{
    pwm_channel_mask_t mask;
    uint8_t prescale;

    if (pwm->period != 0)
    {
        pwm_period_t duty;

        /* If change period, need to adjust duty pro-rata.  */
        duty = (period * pwm->duty) / pwm->period;
        pwm_duty_set (pwm, duty);
    }
    pwm->period = period;

    /* If the period is greater than 16-bits then need to select the
       appropriate prescaler.  This can be from 1 to 1024 in powers
       of 2.  */
    for (prescale = 0; prescale < 11 && period >= 65535u; prescale++)
    {
        period >>= 1;
    }

    /* TODO: it is possible to configure CLKA and CLKB to have an even
       lower frequency.  However, these clocks are shared for all
       channels.  */
    if (period > 65535u)
        return 0;

    pwm_prescale_set (pwm, prescale);

    mask = pwm_channel_mask (pwm);

    /* Configure period.  */
    if (PWM->PWM_SR & mask)
    {
        uint8_t status;

        /* The PWM is running.  We need to jump through a hoop to
           update the period register.  This is because the update
           register is shared for updating both the period and for
           the duty.  */

        /* Read update status.  */
        status = BITS_EXTRACT (PWM->PWM_ISR1, 0, 3);

        /* Set mode to update period.  */
        BITS_INSERT (pwm->base->PWM_CMR, 1, 10, 10);

        /* Wait for a new period.  */
        while (!(status & mask))
        {
            status = BITS_EXTRACT (PWM->PWM_ISR1, 0, 3);
        }

        pwm->base->PWM_CPRDUPD = period;
    }
    else
    {
        pwm->base->PWM_CPRD = period;
    }

    return period << pwm->prescale;
}


pwm_period_t
pwm_period_get (pwm_t pwm)
{
    return pwm->base->PWM_CPRD << pwm->prescale;
}


pwm_frequency_t
pwm_frequency_set (pwm_t pwm, pwm_frequency_t frequency)
{
    pwm_period_t period;

    period = PWM_PERIOD_DIVISOR (frequency);
    period = pwm_period_set (pwm, period);

    return F_CPU / period;
}


/** Set waveform duty (in CPU clocks).  This will block if the
    PWM is running until the end of a cycle.  */
pwm_period_t
pwm_duty_set (pwm_t pwm, pwm_period_t duty)
{
    pwm_channel_mask_t mask;

    pwm->duty = duty;
    duty = duty >> pwm->prescale;

    mask = pwm_channel_mask (pwm);

    /* Configure duty.  */
    if (PWM->PWM_SR & mask)
    {
        uint8_t status;

        /* The PWM is running.  We need to jump through a hoop to
           update the duty register.  This is because the update
           register is shared for updating both the period and for
           the duty.  */

        /* Read update status.  */
        status = BITS_EXTRACT (PWM->PWM_ISR1, 0, 3);

        /* Set mode to update duty.  */
        BITS_INSERT (pwm->base->PWM_CMR, 0, 10, 10);

        /* Wait for a new duty.  */
        while (!(status & mask))
        {
            status = BITS_EXTRACT (PWM->PWM_ISR1, 0, 3);
        }

        pwm->base->PWM_CDTYUPD = duty;
    }
    else
    {
        pwm->base->PWM_CDTY = duty;
    }

    return duty << pwm->prescale;
}


pwm_period_t
pwm_duty_get (pwm_t pwm)
{
    return pwm->base->PWM_CDTY << pwm->prescale;
}


/** Set waveform duty (as a fraction of the period in parts per
    thousand).  This will block if the PWM is running until the end of
    a cycle.  */
unsigned int
pwm_duty_ppt_set (pwm_t pwm, unsigned int duty_ppt)
{
    pwm_period_t duty;
    pwm_period_t period;

    period = pwm_period_get (pwm) >> pwm->prescale;
    duty = period * duty_ppt / 1000;

    duty = pwm_duty_set (pwm, duty << pwm->prescale) >> pwm->prescale;

    return duty * 1000 / period;
}


/** Configures the PWM output The period of the waveform is in number
    of MCK ticks.  The duty can be any number less than the period.   */
static uint8_t
pwm_config (pwm_t pwm, pwm_period_t period, pwm_period_t duty,
            pwm_align_t align, pwm_polarity_t polarity)
{
    /* The duty cycle cannot be greater than 100 %.  */
    if (duty > period)
        return 0;

    /* Configure period first since this sets the prescaler.  */
    pwm_period_set (pwm, period);

    pwm_duty_set (pwm, duty);

    /* Polarity and alignment can only be changed when the PWM channel
       is disabled, i.e., is stopped.  */

    /* Configure wave align.  */
    BITS_INSERT (pwm->base->PWM_CMR, align, 8, 8);

    /* Configure polarity.  */
    BITS_INSERT (pwm->base->PWM_CMR, polarity, 9, 9);

    return 1;
}


/** Initialises PWM on specified pin.  */
pwm_t
pwm_init (const pwm_cfg_t *cfg)
{
    const pinmap_t *pin;
    pwm_dev_t *pwm;
    unsigned int i;
    pwm_period_t period;
    pwm_period_t duty;

    /* Find PWM channel matching selected PIO.  */
    pin = 0;
    for (i = 0; i < PWM_PINS_NUM; i++)
    {
        pin = &pwm_pins[i];
        if (pin->pio == cfg->pio)
            break;
    }
    if (!pin)
    {
        errno = ENODEV;
        return 0;
    }

    /* Allow user to override PWM channel.  */
    pwm = &pwm_devices[pin->channel];
    pwm->pin = pin;
    pwm->base = &PWM->PWM_CH_NUM[pin->channel];
    pwm->stop_state = cfg->stop_state;
    pwm->duty = 0;
    pwm->period = 0;

    /* Enable PWM peripheral clock (this is not required to configure
       the PWM).  */
    mcu_pmc_enable (ID_PWM);

    pwm_stop (pwm);

    period = cfg->period;
    if (cfg->frequency)
        period = PWM_PERIOD_DIVISOR (cfg->frequency);

    duty = cfg->duty;
    if (cfg->duty_ppt)
        duty = period * cfg->duty_ppt / 1000;

    pwm_config (pwm, period, duty, cfg->align, cfg->polarity);

    return pwm;
}


/** Get channel mask.  */
pwm_channel_mask_t
pwm_channel_mask (pwm_t pwm)
{
    return BIT (pwm->pin->channel);
}


/** Start selected channels simultaneously.  */
void
pwm_channels_start (pwm_channel_mask_t channel_mask)
{
    int i;

    /* The following code is to handle the case where we want an
       inverted PWM output to be low when it is not running.  This is
       achieved by switching the pin from a PIO to a PWM output.  */
    for (i = 0; i < PWM_DEVICES_NUM; i++)
    {
        pwm_dev_t *pwm;

        if (! (BIT(i) & channel_mask))
            continue;

        pwm = &pwm_devices[i];

        /* Check if trying to start a channel that has not been init.  */
        if (!pwm->pin)
            continue;

        /* Switch PIO so PWM can drive the pin.  */
        pio_config_set (pwm->pin->pio, pwm->pin->periph);
    }

    PWM->PWM_ENA = channel_mask;
}


/** Stop selected channels simultaneously.  */
void
pwm_channels_stop (pwm_channel_mask_t channel_mask)
{
    int i;

    PWM->PWM_DIS = channel_mask;

    /* Switch pins to have desired stop state.  */
    for (i = 0; i < PWM_DEVICES_NUM; i++)
    {
        pwm_dev_t *pwm;

        if (! (BIT(i) & channel_mask))
            continue;

        pwm = &pwm_devices[i];

        /* Check if trying to start a channel that has not been init.  */
        if (!pwm->pin)
            continue;

        if (pwm->stop_state)
            pio_config_set (pwm->pin->pio, pwm->stop_state);
    }
}


/** Start selected channel.  */
void
pwm_start (pwm_t pwm)
{
    pwm_channels_start (pwm_channel_mask (pwm));
}


/** Stop selected channel.  */
void
pwm_stop (pwm_t pwm)
{
    pwm_channels_stop (pwm_channel_mask (pwm));
}
