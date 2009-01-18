/** @file   timer.c
    @author 
    @date   12 February 2008
    @brief  Timer counter routines for AT91SAM7 processors
*/

/* DOES NOT SUPPORT WAVEFORM MODE and does not include burst mode,
   clock invert, or external clock configuration.  */


#include "timer.h"
#include "bits.h"

#define TIMER_CHANNEL_0     0
#define TIMER_CHANNEL_1     1
#define TIMER_CHANNEL_2     2

#define EX_TRG_NONE         0
#define EX_TRG_RISING       1
#define EX_TRG_FALLING      2
#define EX_TRG_EACH         3

#define EX_TRG_TIOB         0
#define EX_TRG_TIOA         1

#define RC_COMP_NONE        0
#define RC_COMP_TRG         1

#define LDRA_TIOA_NONE      0
#define LDRA_TIOA_RISING    1    
#define LDRA_TIOA_FALLING   2
#define LDRA_TIOA_EACH      3

#define LDRB_TIOA_NONE      0
#define LDRB_TIOA_RISING    1    
#define LDRB_TIOA_FALLING   2
#define LDRB_TIOA_EACH      3

#define LDRB_CLK_NONE       0
#define LDRB_CLK_STOP       1
#define LDRB_CLK_DIS        2
#define LDRB_CLK_STOP_DIS   3

#define TIMER_TC_CV         0
#define TIMER_TC_RA         1
#define TIMER_TC_RB         2
#define TIMER_TC_RC         3


/* Prescale values:
   2, 8, 32, 128, 1024.

   2^1, 2^3, 2^5, 2^7, 2^10.  */

static const uint8_t log_prescales[] = {1, 3, 5, 7, 10};

static inline AT91S_TC *
timer_base (uint8_t channel)
{
    switch (channel)
    {
    case TIMER_CHANNEL_0:
        return AT91C_BASE_TC0;
					
    case TIMER_CHANNEL_1:
        return AT91C_BASE_TC1;
	
    case TIMER_CHANNEL_2:
        return AT91C_BASE_TC2;
			
    default:
        return 0;
    }
}


uint8_t
timer_init (uint8_t channel)
{
    /* Each timer has its own peripheral clock.  */
    return 1;
}


/* Initialises the timer for capture mode.  */
uint8_t
timer_enable (uint8_t channel)
{
    switch (channel)
    {
    case TIMER_CHANNEL_0:
        /* Enable peripheral pins.  */
        *AT91C_PIOA_PDR = AT91C_PA0_TIOA0 | AT91C_PA1_TIOB0;
        /* Disable pullups.  */
        *AT91C_PIOA_PPUDR = AT91C_PA0_TIOA0 | AT91C_PA1_TIOB0;
        /* Enable peripheral clock.  */
        AT91C_BASE_PMC->PMC_PCER = BIT (AT91C_ID_TC0);
        break;
	
    case TIMER_CHANNEL_1:
        /* Enable peripheral pins.  */
        *AT91C_PIOA_PDR = AT91C_PA15_TIOA1 | AT91C_PA16_TIOB1;
        /* Disable pullups.  */
        *AT91C_PIOA_PPUDR = AT91C_PA15_TIOA1 | AT91C_PA16_TIOB1;
        /* Enable peripheral clock.  */
        AT91C_BASE_PMC->PMC_PCER = BIT (AT91C_ID_TC1);
        break;
	
    case TIMER_CHANNEL_2:
        /* Enable peripheral pins.  */
        *AT91C_PIOA_PDR = AT91C_PA26_TIOA2 | AT91C_PA27_TIOB2;
        /* Disable pullups.  */
        *AT91C_PIOA_PPUDR = AT91C_PA26_TIOA2 | AT91C_PA27_TIOB2;
        /* Enable peripheral clock.  */
        AT91C_BASE_PMC->PMC_PCER = BIT (AT91C_ID_TC2);
        break;
        
    default:
        return 0;
    }
    return 1;
}


/* Disable the selected timer.  */
uint8_t
timer_disable (uint8_t channel)
{
    switch (channel)
    {
    case TIMER_CHANNEL_0:
        /* Disable peripheral pins.  */
        *AT91C_PIOA_PER = AT91C_PA0_TIOA0 | AT91C_PA1_TIOB0;
        /* Disable peripheral clock.  */
        AT91C_BASE_PMC->PMC_PCDR = BIT (AT91C_ID_TC0);
        break;
	
    case TIMER_CHANNEL_1:
        /* Disable peripheral pins.  */
        *AT91C_PIOA_PER = AT91C_PA15_TIOA1 | AT91C_PA16_TIOB1;
        /* Disable peripheral clock.  */
        AT91C_BASE_PMC->PMC_PCDR = BIT (AT91C_ID_TC1);
        break;
	
    case TIMER_CHANNEL_2:
        /* Disable peripheral pins.  */
        *AT91C_PIOA_PER = AT91C_PA26_TIOA2 | AT91C_PA27_TIOB2;
        /* Disable peripheral clock.  */
        AT91C_BASE_PMC->PMC_PCDR = BIT (AT91C_ID_TC2);
        break;
        
    default:
        return 0;
    }
    return 1;
}


/* Selects and applies a suitable prescale for the clock, based on the
   given period.  The period is in (master) clocks.  */
uint8_t
timer_prescaler_set (uint8_t channel, unsigned int period)
{
    uint8_t i;
    uint8_t log_prescale = 0;
	AT91S_TC *pTC;

    /* Choose smallest prescaler so that the period fits within 16 bits.  */
    for (i = 1; i < ARRAY_SIZE (log_prescales) && period > 65535; i++)
    {
        period >>= (log_prescales[i] - log_prescale);
        log_prescale = log_prescales[i];
    }
    
    /* Select the channel peripheral base address.  */	
    pTC = timer_base (channel);
    
    if (!pTC)
        return 0;
    
    /* Set clock prescale.  */
    switch (log_prescale)
    {
    case 1:
        BITS_INSERT (pTC->TC_CMR, AT91C_TC_CLKS_TIMER_DIV1_CLOCK, 0, 2);
        break;
    case 3:
        BITS_INSERT (pTC->TC_CMR, AT91C_TC_CLKS_TIMER_DIV2_CLOCK, 0, 2);
        break;
    case 5:
        BITS_INSERT (pTC->TC_CMR, AT91C_TC_CLKS_TIMER_DIV3_CLOCK, 0, 2);
        break;
    case 7:
        BITS_INSERT (pTC->TC_CMR, AT91C_TC_CLKS_TIMER_DIV4_CLOCK, 0, 2);
        break;
    case 10:
        BITS_INSERT (pTC->TC_CMR, AT91C_TC_CLKS_TIMER_DIV5_CLOCK, 0, 2);
        break;
    default:
        return 0;
    }
    return 1;
}


/* Set the timer to run in capture mode.  */
uint8_t
timer_capture_mode_set (uint8_t channel)
{
    /* Select the channel peripheral base address.  */	
    AT91S_TC *pTC;
    
    pTC = timer_base (channel);
    
    if (!pTC)
        return 0;
    
    /* Enable capture mode.  */
    BITS_INSERT (pTC->TC_CMR, 0, 15, 15);
    return 1;
}



/*     Timer channel external trigger configuration (in capture mode only)      */
/*                                                                              */
/*                          --------OPTIONS--------                             */
/* ex_trg_edge:     EX_TRG_NONE     - No external trigger                       */
/*                  EX_TRG_RISING   - Trigger on signal rising edge             */ 
/*                  EX_TRG_FALLING  - Trigger on signal falling edge            */
/*                  EX_TRG_EACH     - Trigger on both rising and falling edges  */
/* ex_trg:          EX_TRG_TIOB     - Set external trigger to TIOB              */
/*                  EX_TRG_TIOA     - Set external trigger to TIOA              */
/* rc_comp_trg:     RC_COMP_NONE    - RC compare has no effect                  */
/*                  RC_COMP_NONE    - Trigger on RC compare                     */ 

uint8_t
timer_capture_trigger_config (uint8_t channel, uint8_t ex_trg_edge, 
                              uint8_t ex_trg, uint8_t rc_comp_trg)
{
    /* Select the channel peripheral base address.  */	
    AT91S_TC *pTC;
    
    pTC = timer_base (channel);
    
    if (!pTC)
        return 0;
    
    /* Ensure capture mode is in use.  */
    if (BITS_EXTRACT (pTC->TC_CMR, 15, 15))
        return 0;
    
    /* Insert CMR register bits.  */
    BITS_INSERT (pTC->TC_CMR, ex_trg_edge, 8, 9);
    BITS_INSERT (pTC->TC_CMR, ex_trg, 10, 10); 
    BITS_INSERT (pTC->TC_CMR, rc_comp_trg, 14, 14);
    
    return 1;
}


/*       Timer channel register loading configuration (in capture mode only)        */
/*                                                                                  */
/*                          --------OPTIONS--------                                 */
/* ldra:        LDRA_TIOA_NONE    - never load RA                                 */
/*              LDRA_TIOA_RISING  - load RA on TIOA signal rising edge            */
/*              LDRA_TIOA_FALLING - load RA on TIOA signal falling edge           */
/*              LDRA_TIOA_EACH    - load RA on TIOA rising and falling edges      */
/* ldrb:        LDRB_TIOA_NONE    - never load RB                                 */
/*              LDRB_TIOA_RISING  - load RB on TIOA signal rising edge            */
/*              LDRB_TIOA_FALLING - load RB on TIOA signal falling edge           */
/*              LDRB_TIOA_EACH    - load RB on TIOA rising and falling edges      */
/* ldrb_clk:    LDRB_CLK_STOP     - stop the clock when RB is loaded              */
/*              LDRB_CLK_DIS      - disable the clock when RB is loaded           */
/*              LDRB_CLK_STOP_DIS - stop and disable the clock when RB is loaded  */
/*              LDRB_CLK_NONE     - clock continues as normal on RB loading       */ 

/* RA is loaded only if it has not been loaded since the last trigger
 * or if RB has been loaded since  */
/* the last loading of RA.                                                                              */
/* RB is loaded only if RA has been loaded since the last trigger or
 * the last loading of RB.  */
/* Loading RA or RB before the read of the last value loaded sets the
 * Overrun Error Flag (LOVRS)  */
/* in TC_SR (Status Register). In this case, the old value is overwritten.                              */


uint8_t
timer_capture_register_load_config (uint8_t channel, uint8_t ldra, 
                                    uint8_t ldrb, uint8_t ldrb_clk)
{
    AT91S_TC *pTC;
    
    pTC = timer_base (channel);
    
    if (!pTC)
        return 0;
        
    /* Ensure capture mode is in use.  */
    if (BITS_EXTRACT (pTC->TC_CMR, 15, 15))
        return 0;
        
    /* Insert CMR register bits.  */
    BITS_INSERT (pTC->TC_CMR, ldra, 16, 17);
    BITS_INSERT (pTC->TC_CMR, ldrb, 18, 19);
    BITS_INSERT (pTC->TC_CMR, ldrb_clk, 6, 7);
    
    return 1;
}


/* Enable the timer clock.  */
uint8_t
timer_clock_enable (uint8_t channel)
{
    AT91S_TC *pTC;
    
    pTC = timer_base (channel);
	
    if (!pTC)
        return 0;
    
    pTC->TC_CCR = AT91C_TC_CLKEN;
    return 1;
}
    


/*                Timer channel register reading                */
/*                                                              */
/*                  --------OPTIONS--------                     */
/* timer_register:  TIMER_TC_CV       - Current counter value   */
/*                  TIMER_TC_RA       - Current RA value        */ 
/*                  TIMER_TC_RB       - Current RB value        */
/*                  TIMER_TC_RC       - Current RC value        */

uint16_t
timer_read (uint8_t channel, uint8_t timer_reg)
{
    AT91S_TC *pTC;
    
    pTC = timer_base (channel);
	
    if (!pTC)
        return 0;
        
    switch (timer_reg)
    {
        case TIMER_TC_CV:
            return (pTC->TC_CV);
            
        case TIMER_TC_RA:
            /* Check an RA loading has occurred.  */
            if (BITS_EXTRACT (pTC->TC_SR, 5, 5))
                return (pTC->TC_RA);
            else
                return 0;
                
        case TIMER_TC_RB:
            /* Check an RB loading has occurred.  */
            if (BITS_EXTRACT (pTC->TC_SR, 6, 6))
                return (pTC->TC_RA);
            else
                return 0;
                
        case TIMER_TC_RC:
            return (pTC->TC_RC);
        
        default:
            return 0;
    }
}

/* Performs a software trigger.  */
uint8_t
timer_reset (uint8_t channel)
{
    AT91S_TC *pTC;
    
    pTC = timer_base (channel);
	
    if (!pTC)
        return 0;
    
    pTC->TC_CCR = AT91C_TC_SWTRG;
    return 1;
}


/* Disable the timer.  */
uint8_t
timer_clock_disable (uint8_t channel)
{
    AT91S_TC *pTC;
    
    pTC = timer_base (channel);
	
    if (!pTC)
        return 0;
    
    pTC->TC_CCR = AT91C_TC_CLKDIS;
    return 1;
}
       
