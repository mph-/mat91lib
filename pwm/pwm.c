/** @file   pwm.c
    @author 
    @date   12 February 2008
    @brief  Pulse Width Modulation routines for AT91SAM7 processors.
*/

#include "pwm.h"
#include "bits.h"


typedef struct pwm_pin_struct
{
    uint8_t channel;
    pio_t pio;
    pio_config_t periph;
} pwm_pin_t;


struct pwm_dev_struct
{
    const pwm_pin_t *pin;
};


static pwm_dev_t pwm_devices[4];


#define PWM_PIN(CHANNEL, PIO, PERIPH) \
    {.channel = (CHANNEL), .pio = (PIO), .periph = (PERIPH)}

/* Define known PWM pins.  */
static const pwm_pin_t pwm_pins[] = 
{
    PWM_PIN (0, PIO_DEFINE (PORT_A, 0), PIO_PERIPH_A),
    PWM_PIN (0, PIO_DEFINE (PORT_A, 11), PIO_PERIPH_B),
    PWM_PIN (0, PIO_DEFINE (PORT_A, 23), PIO_PERIPH_B),

    PWM_PIN (1, PIO_DEFINE (PORT_A, 1), PIO_PERIPH_A),
    PWM_PIN (1, PIO_DEFINE (PORT_A, 12), PIO_PERIPH_B),
    PWM_PIN (1, PIO_DEFINE (PORT_A, 24), PIO_PERIPH_B),

    PWM_PIN (2, PIO_DEFINE (PORT_A, 2), PIO_PERIPH_A),
    PWM_PIN (2, PIO_DEFINE (PORT_A, 13), PIO_PERIPH_B),
    PWM_PIN (2, PIO_DEFINE (PORT_A, 24), PIO_PERIPH_B),

    PWM_PIN (3, PIO_DEFINE (PORT_A, 7), PIO_PERIPH_B),
    PWM_PIN (3, PIO_DEFINE (PORT_A, 14), PIO_PERIPH_B),
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
    of MCK ticks.  The duty can be any number less than the period.
    Support is only for fequencies above 750 Hz (no prescaling used
    here).  So for 100 kHz, period would be 480 with MCK at 48
    MHz.  */
static uint8_t
pwm_config (pwm_t pwm, pwm_period_t period, pwm_period_t duty,
            pwm_align_t align, pwm_polarity_t polarity)
{
    AT91S_PWMC_CH *pPWM;
    
    /* Select the channel peripheral base address.  */	
    pPWM = pwm_base (pwm);
    
    if (!pPWM)
        return 0;
    
    /* Configure period.  */
    pPWM->PWMC_CPRDR = period;
    
    /* Check and configure duty cycle.  */
    if (duty > period)
        return 0;
    
    pPWM->PWMC_CDTYR = duty;
    
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
    const pwm_pin_t *pin;
    pwm_dev_t *dev;
    unsigned int i;


    /* Enable PWM peripheral clock.  */
    AT91C_BASE_PMC->PMC_PCER = BIT (AT91C_ID_PWMC);

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
    dev = &pwm_devices[pin->channel];
    dev->pin = pin;

    pwm_config (dev, cfg->period, cfg->duty, cfg->align, cfg->polarity);

    return dev;
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
    AT91C_BASE_PWMC->PWMC_ENA = channel_mask;
}


/** Stop selected channels simultaneously.  */
void
pwm_channels_stop (pwm_channel_mask_t channel_mask)
{
    AT91C_BASE_PWMC->PWMC_DIS = channel_mask;
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


/** Enable PWM to drive pin.  */
void
pwm_enable (pwm_t pwm)
{
    pio_config_set (pwm->pin->pio, pwm->pin->periph);
}


/** Switch pin back to a PIO.  */
void
pwm_disable (pwm_t pwm)
{
    /* Return PIO to previous state.  This is a violation of the PIO
       API.  FIXME.  */
    PIO_PORT_ (pwm->pin->pio)->PIO_PER = PIO_BITMASK_ (pwm->pin->pio);    
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
    
    /* Check that the duty cycle is ok.  */
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


