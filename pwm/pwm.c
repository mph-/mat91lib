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


/** Configures the PWM output The period of the waveform is in number
    of MCK ticks.  The duty can be any number less than the period.   */
static uint8_t
pwm_config (pwm_t pwm, pwm_period_t period, pwm_period_t duty,
            pwm_align_t align, pwm_polarity_t polarity)
{
    AT91S_PWMC_CH *pPWM;

    /* The duty cycle cannot be greater than 100 %.  */
    if (duty > period)
        return 0;

    /* If the period is greater than 16-bits then need to select the
       appropriate prescaler.  TODO.  This restricts the PWM fequency
       to be above 750 Hz.  So for 100 kHz, the period would be 480 with
       MCK at 48 MHz.  */
    if (period >= 65536u)
        return 0;
    
    /* Select the channel peripheral base address.  */	
    pPWM = pwm_base (pwm);
    
    if (!pPWM)
        return 0;
    
    /* Configure period.  */
    pPWM->PWMC_CPRDR = period;
    
    pPWM->PWMC_CDTYR = duty;
    
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


/** Update the waveform period and duty.  */
uint8_t
pwm_update (pwm_t pwm, pwm_period_t new_period, pwm_period_t new_duty)
{
    AT91S_PWMC_CH *pPWM;
    uint8_t status;
    pwm_channel_mask_t mask;
    
    /* Select the channel peripheral base address.  */	
    pPWM = pwm_base (pwm);
	
    if (!pPWM)
        return 0;
    
    /* Check that the duty cycle is okay.  */
    if (new_duty > new_period)
        return 0;

    mask = pwm_channel_mask (pwm);
    
    /* Read update interrupt.  */
    status = BITS_EXTRACT (AT91C_BASE_PWMC->PWMC_ISR, 0, 3); 

    /* Set mode to update duty.  */
    BITS_INSERT (pPWM->PWMC_CMR, 0, 10, 10);
        
    /* Wait for a new period.  */
    while (!(status & mask))
    {
        status = BITS_EXTRACT (AT91C_BASE_PWMC->PWMC_ISR, 0, 3);
    }
        
    /* Write in the new duty.  */
    pPWM->PWMC_CUPDR = new_duty;
        
    /* Read update interrupt.  */
    status = BITS_EXTRACT (AT91C_BASE_PWMC->PWMC_ISR, 0, 3); 

    /* Set mode to update period.  */
    BITS_INSERT (pPWM->PWMC_CMR, 1, 10, 10);
        
    /* Wait for a new period.  */
    while (!(status & mask))
    {
        status = BITS_EXTRACT (AT91C_BASE_PWMC->PWMC_ISR, 0, 3);
    }
        
    /* Write in the new period.  */
    pPWM->PWMC_CUPDR = new_period;

    return 1;
}


