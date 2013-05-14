/** @file   pwm.c
    @author 
    @date   12 February 2008
    @brief  Pulse Width Modulation routines for AT91SAM7 processors.
*/

#include "pwm.h"
#include "bits.h"
#include "pinmap.h"


struct pwm_dev_struct
{
    const pinmap_t *pin;
    pio_config_t stop_state;
    uint8_t prescale;
};


#define PWM_DEVICES_NUM 4

static pwm_dev_t pwm_devices[PWM_DEVICES_NUM];


/* Define known PWM pins.  */
static const pinmap_t pwm_pins[] = 
{
    {0, PIO_DEFINE (PORT_A, 0), PIO_PERIPH_A},
    {0, PIO_DEFINE (PORT_A, 11), PIO_PERIPH_B},
    {0, PIO_DEFINE (PORT_A, 23), PIO_PERIPH_B},

    {1, PIO_DEFINE (PORT_A, 1), PIO_PERIPH_A},
    {1, PIO_DEFINE (PORT_A, 12), PIO_PERIPH_B},
    {1, PIO_DEFINE (PORT_A, 24), PIO_PERIPH_B},

    {2, PIO_DEFINE (PORT_A, 2), PIO_PERIPH_A},
    {2, PIO_DEFINE (PORT_A, 13), PIO_PERIPH_B},
    {2, PIO_DEFINE (PORT_A, 25), PIO_PERIPH_B},

    {3, PIO_DEFINE (PORT_A, 7), PIO_PERIPH_B},
    {3, PIO_DEFINE (PORT_A, 14), PIO_PERIPH_B},
};


#define PWM_PINS_NUM ARRAY_SIZE (pwm_pins)


static inline AT91S_PWMC_CH *pwm_base (pwm_t pwm)
{
    switch (pwm->pin->channel)
    {
    case 0:
        return AT91C_BASE_PWMC_CH0;
					
    case 1:
        return AT91C_BASE_PWMC_CH1;
					
    case 2:
        return AT91C_BASE_PWMC_CH2;
        
    case 3:
        return AT91C_BASE_PWMC_CH3;
			
    default:
        return 0;
    }
}

/** Shutdown clock to PWM peripheral.  */
void
pwm_shutdown (void)
{
    /* Disable PWM peripheral clock.  */
    AT91C_BASE_PMC->PMC_PCDR = BIT (AT91C_ID_PWMC);
}


static void
pwm_prescale_set (pwm_t pwm, uint8_t prescale)
{
    AT91S_PWMC_CH *pPWM;

    pPWM = pwm_base (pwm);
    if (!pPWM)
        return;

    /* Configure prescaler.  */
    BITS_INSERT (pPWM->PWMC_CMR, prescale, 0, 3);
    pwm->prescale = prescale;
}



pwm_period_t
pwm_period_set (pwm_t pwm, pwm_period_t period)
{
    AT91S_PWMC_CH *pPWM;
    pwm_channel_mask_t mask;
    uint8_t prescale;

    pPWM = pwm_base (pwm);
    if (!pPWM)
        return 0;

    /* If the period is greater than 16-bits then need to select the
       appropriate prescaler.  */
    for (prescale = 0; prescale < 11 && period <= 65535u; prescale++)
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
    if (AT91C_BASE_PWMC->PWMC_SR & mask)
    {
        uint8_t status;

        /* The PWM is running.  We need to jump through a hoop to
           update the period register.  This is because the update
           register is used for the period and for the duty.  */

        /* Read update status.  */
        status = BITS_EXTRACT (AT91C_BASE_PWMC->PWMC_ISR, 0, 3); 

        /* Set mode to update period.  */
        BITS_INSERT (pPWM->PWMC_CMR, 1, 10, 10);
        
        /* Wait for a new period.  */
        while (!(status & mask))
        {
            status = BITS_EXTRACT (AT91C_BASE_PWMC->PWMC_ISR, 0, 3);
        }

        pPWM->PWMC_CUPDR = period;
    }
    else
    {
        pPWM->PWMC_CPRDR = period;
    }

    return period << pwm->prescale;
}


pwm_period_t
pwm_duty_set (pwm_t pwm, pwm_period_t duty)
{
    AT91S_PWMC_CH *pPWM;
    pwm_channel_mask_t mask;

    pPWM = pwm_base (pwm);
    if (!pPWM)
        return 0;

    duty = duty >> pwm->prescale;

    mask = pwm_channel_mask (pwm);

    /* Configure duty.  */
    if (AT91C_BASE_PWMC->PWMC_SR & mask)
    {
        uint8_t status;

        /* The PWM is running.  We need to jump through a hoop to
           update the duty register.  This is because the update
           register is used for the period and for the duty.  */

        /* Read update status.  */
        status = BITS_EXTRACT (AT91C_BASE_PWMC->PWMC_ISR, 0, 3); 

        /* Set mode to update duty.  */
        BITS_INSERT (pPWM->PWMC_CMR, 0, 10, 10);
        
        /* Wait for a new duty.  */
        while (!(status & mask))
        {
            status = BITS_EXTRACT (AT91C_BASE_PWMC->PWMC_ISR, 0, 3);
        }

        pPWM->PWMC_CUPDR = duty;
    }
    else
    {
        pPWM->PWMC_CDTYR = duty;
    }


    return duty << pwm->prescale;
}



/** Configures the PWM output The period of the waveform is in number
    of MCK ticks.  The duty can be any number less than the period.   */
static uint8_t
pwm_config (pwm_t pwm, pwm_period_t period, pwm_period_t duty,
            pwm_align_t align, pwm_polarity_t polarity)
{
    AT91S_PWMC_CH *pPWM;

    /* Select the channel peripheral base address.  */	
    pPWM = pwm_base (pwm);
    
    if (!pPWM)
        return 0;
    
    /* The duty cycle cannot be greater than 100 %.  */
    if (duty > period)
        return 0;

    /* Configure period first since this sets the prescaler.  */
    pwm_period_set (pwm, period);

    pwm_duty_set (pwm, duty);

    /* Polarity and alignment can only be changed when the PWM channel
       is disabled, i.e., is stopped.  */

    /* Configure wave align.  */
    BITS_INSERT (pPWM->PWMC_CMR, align, 8, 8);
    
    /* Configure polarity.  */
    BITS_INSERT (pPWM->PWMC_CMR, polarity, 9, 9);
    
    return 1;
}


/** Initialises PWM on specified pin.  */
pwm_t
pwm_init (const pwm_cfg_t *cfg)
{
    const pinmap_t *pin;
    pwm_dev_t *pwm;
    unsigned int i;

    /* Find PWM channel matching selected PIO.  */
    pin = 0;
    for (i = 0; i < PWM_PINS_NUM; i++)
    {
        pin = &pwm_pins[i]; 
        if (pin->pio == cfg->pio)
            break;
    }
    if (!pin)
        return 0;

    /* Allow user to override PWM channel.  */
    pwm = &pwm_devices[pin->channel];
    pwm->pin = pin;
    pwm->stop_state = cfg->stop_state;
    if (pwm->stop_state)
        pio_config_set (pwm->pin->pio, pwm->stop_state);

    /* Enable PWM peripheral clock (this is not required to configure
       the PWM).  */
    AT91C_BASE_PMC->PMC_PCER = BIT (AT91C_ID_PWMC);

    pwm_config (pwm, cfg->period, cfg->duty, cfg->align, cfg->polarity);

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

    AT91C_BASE_PWMC->PWMC_ENA = channel_mask;
}


/** Stop selected channels simultaneously.  */
void
pwm_channels_stop (pwm_channel_mask_t channel_mask)
{
    int i;

    AT91C_BASE_PWMC->PWMC_DIS = channel_mask;

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
