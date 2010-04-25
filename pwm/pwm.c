/** @file   pwm.c
    @author 
    @date   12 February 2008
    @brief  Pulse Width Modulation routines for AT91SAM7 processors
*/

#include "pwm.h"
#include "bits.h"


/* Currently no support for frequencies below 730Hz.  */

static inline AT91S_PWMC_CH *pwm_base (pwm_channel_t channel)
{
    switch (channel)
    {
    case PWM_CHANNEL_0:
        return AT91C_BASE_PWMC_CH0;
					
    case PWM_CHANNEL_1:
        return AT91C_BASE_PWMC_CH1;
					
    case PWM_CHANNEL_2:
        return AT91C_BASE_PWMC_CH2;
        
    case PWM_CHANNEL_3:
        return AT91C_BASE_PWMC_CH3;
			
    default:
        return 0;
    }
}


void
pwm_shutdown (void)
{
    /* Disable PWM peripheral clock.  */
    AT91C_BASE_PMC->PMC_PCDR = BIT (AT91C_ID_PWMC);
}


/* Initialises pwm on specified pins.  */
uint8_t
pwm_init (void)
{
    /* Enable PWM peripheral clock.  */
    AT91C_BASE_PMC->PMC_PCER = BIT (AT91C_ID_PWMC);

    return 1;
}


/* Configures the PWM output The period of the waveform is in number
   of MCK ticks.  The duty can be any number less than the period.
   Support is only for fequencies above 750Hz (no prescaling used
   here).  So for 100kHz, period would be 480 with MCK at 48MHz.  */
uint8_t
pwm_config (pwm_channel_t channel, uint16_t period, uint16_t duty,
            pwm_align_t alignment, pwm_polarity_t polarity)
{
    AT91S_PWMC_CH *pPWM;
    
    /* Select the channel peripheral base address.  */	
    pPWM = pwm_base (channel);
	
    if (!pPWM)
        return 0;
    
    /* Configure period.  */
    pPWM ->PWMC_CPRDR = period;
    
    /* Check and configure duty cycle.  */
    if (duty > period)
        return 0;
    
    pPWM->PWMC_CDTYR = duty;
    
    /* Configure wave alignment.  */
    BITS_INSERT (pPWM->PWMC_CMR, alignment, 8, 8);
    
    /* Configure polarity.  */
    BITS_INSERT (pPWM->PWMC_CMR, polarity, 9, 9);
    
    return 1;
}


/* Start selected channels simultaneously.  */
void
pwm_channels_start (pwm_channel_mask_t channel_mask)
{
    AT91C_BASE_PWMC->PWMC_ENA = channel_mask;
}


/* Stop selected channels simultaneously.  */
void
pwm_channels_stop (pwm_channel_mask_t channel_mask)
{
    AT91C_BASE_PWMC->PWMC_DIS = channel_mask;
}


/* Start selected channel.  */
void
pwm_start (pwm_channel_t channel)
{
    pwm_channels_start (BIT (channel));
}


/* Stop selected channel.  */
void
pwm_stop (pwm_channel_t channel)
{
    pwm_channels_stop (BIT (channel));
}


void
pwm_enable (pwm_channel_t channel)
{
    switch (channel)
    {
#ifdef PWM0_BIT
    case PWM_CHANNEL_0:
#if PWM0_BIT == 0
        AT91C_BASE_PIOA->PIO_ASR = AT91C_PA0_PWM0;
        AT91C_BASE_PIOA->PIO_PDR = AT91C_PA0_PWM0;
#elif PWM0_BIT == 11
        AT91C_BASE_PIOA->PIO_BSR = AT91C_PA11_PWM0;
        AT91C_BASE_PIOA->PIO_PDR = AT91C_PA11_PWM0;
#elif PWM0_BIT == 23
        AT91C_BASE_PIOA->PIO_BSR = AT91C_PA23_PWM0;
        AT91C_BASE_PIOA->PIO_PDR = AT91C_PA23_PWM0;
#endif
        break;
#endif

#ifdef PWM1_BIT
    case PWM_CHANNEL_1:
#if PWM1_BIT == 1
        AT91C_BASE_PIOA->PIO_ASR = AT91C_PA1_PWM1;
        AT91C_BASE_PIOA->PIO_PDR = AT91C_PA1_PWM1;
#elif PWM1_BIT == 12
        AT91C_BASE_PIOA->PIO_BSR = AT91C_PA12_PWM1;
        AT91C_BASE_PIOA->PIO_PDR = AT91C_PA12_PWM1;
#elif PWM1_BIT == 24
        AT91C_BASE_PIOA->PIO_BSR = AT91C_PA24_PWM1;
        AT91C_BASE_PIOA->PIO_PDR = AT91C_PA24_PWM1;
#endif
        break;
#endif

#ifdef PWM2_BIT
    case PWM_CHANNEL_2:
#if PWM2_BIT == 2
        AT91C_BASE_PIOA->PIO_ASR = AT91C_PA2_PWM2;
        AT91C_BASE_PIOA->PIO_PDR = AT91C_PA2_PWM2;
#elif PWM2_BIT == 13
        AT91C_BASE_PIOA->PIO_BSR = AT91C_PA13_PWM2;
        AT91C_BASE_PIOA->PIO_PDR = AT91C_PA13_PWM2;
#elif PWM2_BIT == 25
        AT91C_BASE_PIOA->PIO_BSR = AT91C_PA25_PWM2;
        AT91C_BASE_PIOA->PIO_PDR = AT91C_PA25_PWM2;
#endif
        break;
#endif

#ifdef PWM3_BIT
    case PWM_CHANNEL_3:
#if PWM3_BIT == 7
        AT91C_BASE_PIOA->PIO_BSR = AT91C_PA7_PWM3;
        AT91C_BASE_PIOA->PIO_PDR = AT91C_PA7_PWM3;
#elif PWM3_BIT == 14
        AT91C_BASE_PIOA->PIO_BSR = AT91C_PA14_PWM3;
        AT91C_BASE_PIOA->PIO_PDR = AT91C_PA14_PWM3;
#endif
        break;
#endif
    default:
        break;
    }
}


void
pwm_disable (pwm_channel_t channel)
{
    switch (channel)
    {
#ifdef PWM0_BIT
    case 0:
#if PWM0_BIT == 0
        AT91C_BASE_PIOA->PIO_PER = AT91C_PA0_PWM0;
#elif PWM0_BIT == 11
        AT91C_BASE_PIOA->PIO_PER = AT91C_PA11_PWM0;
#elif PWM0_BIT == 23
        AT91C_BASE_PIOA->PIO_PER = AT91C_PA23_PWM0;
#endif
        break;
#endif

#ifdef PWM1_BIT
    case 1:
#if PWM1_BIT == 1
        AT91C_BASE_PIOA->PIO_PER = AT91C_PA1_PWM1;
#elif PWM1_BIT == 12
        AT91C_BASE_PIOA->PIO_PER = AT91C_PA12_PWM1;
#elif PWM1_BIT == 24
        AT91C_BASE_PIOA->PIO_PER = AT91C_PA24_PWM1;
#endif
        break;
#endif

#ifdef PWM2_BIT
    case 2:
#if PWM2_BIT == 2
        AT91C_BASE_PIOA->PIO_PER = AT91C_PA2_PWM2;
#elif PWM2_BIT == 13
        AT91C_BASE_PIOA->PIO_PER = AT91C_PA13_PWM2;
#elif PWM2_BIT == 25
        AT91C_BASE_PIOA->PIO_PER = AT91C_PA25_PWM2;
#endif
        break;
#endif

#ifdef PWM3_BIT
    case 3:
#if PWM3_BIT == 7
        AT91C_BASE_PIOA->PIO_PER = AT91C_PA7_PWM3;
#elif PWM3_BIT == 14
        AT91C_BASE_PIOA->PIO_PER = AT91C_PA14_PWM3;
#endif
        break;
#endif
    default:
        break;
    }
}



/* Updates the waveform period and duty.  */
uint8_t
pwm_update (pwm_channel_t channel, uint16_t new_period, uint16_t new_duty)
{
    AT91S_PWMC_CH *pPWM;
    uint8_t status;
    
    /* Select the channel peripheral base address.  */	
    pPWM = pwm_base (channel);
	
    if (!pPWM)
        return 0;
        
    /* Check that the duty cycle is ok.  */
    if (new_duty > new_period)
        return 0;

    /* Read update interrupt.  */
    status = BITS_EXTRACT (AT91C_BASE_PWMC->PWMC_ISR, 0, 3); 

    /* Set mode to update duty.  */
    BITS_INSERT (pPWM->PWMC_CMR, 0, 10, 10);
        
    /* Wait for a new period.  */
    while (!(status & BIT (channel)))
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
    while (!(status & BIT (channel)))
    {
        status = BITS_EXTRACT (AT91C_BASE_PWMC->PWMC_ISR, 0, 3);
    }
        
    /* Write in the new period.  */
    pPWM->PWMC_CUPDR = new_period;

    return 1;
}
