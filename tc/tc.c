/** @file   tc.c
    @author M. P. Hayes
    @date   30 November 2010
    @brief  Timer counter routines for AT91SAM7 processors
*/


#include "tc.h"
#include "cpu.h"
#include "irq.h"
#include "bits.h"

/* Each of the three TCs has a 16 bit counter with 3 16 bit registers
   (RA, RB,and RC).  In waveform mode, RA and RB are used to drive
   TIOA and TIOB, respectively.  RC can be used to stop the timer
   clock to provide a one-shot.

   Only TIOAx can be driven at present.
   
   The counter can be clocked with MCK divided by 2, 8, 32, 128, and 1024.
 */

#define TC_CHANNEL(TC) ((TC) - tc_info)


static tc_dev_t tc_info[TC_CHANNEL_NUM];


bool tc_start (tc_t tc)
{
    /* The TC_CCR register is write only.  */
    tc->base->TC_CCR |= (AT91C_TC_CLKEN | AT91C_TC_SWTRG); 
    return 1;
}


bool tc_stop (tc_t tc)
{
    tc->base->TC_CCR |= AT91C_TC_CLKDIS; 
    return 1;
}


uint16_t tc_counter_get (tc_t tc)
{
    return tc->base->TC_CV;
}


bool
tc_pulse_config (tc_t tc, tc_pulse_mode_t mode, uint32_t delay, uint32_t period)
{
    /* Many timer counters can only generate a pulse with a single
       timer clock period.  This timer counter allows the pulse width
       to be varied.  It is specified by period - delay. 
       With WAVESEL_UP_AUTO, the counter is incremented and is reset
       when RC matches.  */
    switch (mode)
    {
    case TC_PULSE_MODE:
        /* Set TIOAx when RA matches and clear TIOAx when RC matches.  */
        tc->base->TC_CMR = AT91C_TC_BURST_NONE | AT91C_TC_WAVE
            | AT91C_TC_WAVESEL_UP_AUTO | AT91C_TC_ACPA_SET | AT91C_TC_ACPC_CLEAR
            | AT91C_TC_ASWTRG_CLEAR;
        break;

    case TC_PULSE_MODE_INVERT:
        /* Clear TIOAx when RA matches and set TIOAx when RC matches.  */
        tc->base->TC_CMR = AT91C_TC_BURST_NONE | AT91C_TC_WAVE
            | AT91C_TC_WAVESEL_UP_AUTO | AT91C_TC_ACPA_CLEAR | AT91C_TC_ACPC_SET
            | AT91C_TC_ASWTRG_SET;
        break;

    case TC_PULSE_MODE_ONESHOT:
        /* Set TIOAx when RA matches and clear TIOAx when RC matches.
           Stop clock when RC matches.   */
        tc->base->TC_CMR = AT91C_TC_BURST_NONE | AT91C_TC_WAVE
            | AT91C_TC_CPCSTOP | AT91C_TC_WAVESEL_UP_AUTO
            | AT91C_TC_ACPA_SET | AT91C_TC_ACPC_CLEAR
            | AT91C_TC_ASWTRG_CLEAR;
        break;

    case TC_PULSE_MODE_ONESHOT_INVERT:
        /* Clear TIOAx when RA matches and set TIOAx when RC matches.
           Stop clock when RC matches.  */
        tc->base->TC_CMR = AT91C_TC_BURST_NONE | AT91C_TC_WAVE
            | AT91C_TC_CPCSTOP | AT91C_TC_WAVESEL_UP_AUTO
            | AT91C_TC_ACPA_CLEAR | AT91C_TC_ACPC_SET
            | AT91C_TC_ASWTRG_SET;
        break;

    case TC_DELAY_MODE_ONESHOT:
        /* Don't change TIOAx.  Stop clock when RC matches.   */
        tc->base->TC_CMR = AT91C_TC_BURST_NONE | AT91C_TC_WAVE
            | AT91C_TC_CPCSTOP | AT91C_TC_WAVESEL_UP_AUTO;
        break;

    default:
        return 0;
    }

    /* If period > 65536 then need to select a slower timer clock.
       For now use MCK / 2.  */
    tc->base->TC_CMR |= AT91C_TC_CLKS_TIMER_DIV1_CLOCK;

    /* These registers are read only when not in wave mode.  */
    tc->base->TC_RA = delay >> 1;
    tc->base->TC_RC = period >> 1;

    /* Generate a software trigger with the clock stopped to hopefully
       set TIOAx to desired state.  Yes, this seems to work.  */
    tc->base->TC_CCR |= (AT91C_TC_CLKDIS | AT91C_TC_SWTRG); 

    /* Make timer pin TIOAx a timer output.  Perhaps we could use
       different logical timer channels to generate pulses on TIOBx
       pins?  */
    switch (TC_CHANNEL (tc))
    {
    case TC_CHANNEL_0:
        /* Switch to peripheral B and disable pin as PIO.  */
        AT91C_BASE_PIOA->PIO_BSR = AT91C_PA0_TIOA0;
        AT91C_BASE_PIOA->PIO_PDR = AT91C_PA0_TIOA0;
        break;

    case TC_CHANNEL_1:
        AT91C_BASE_PIOA->PIO_BSR = AT91C_PA15_TIOA1;
        AT91C_BASE_PIOA->PIO_PDR = AT91C_PA15_TIOA1;
        break;

    case TC_CHANNEL_2:
        AT91C_BASE_PIOA->PIO_BSR = AT91C_PA26_TIOA2;
        AT91C_BASE_PIOA->PIO_PDR = AT91C_PA26_TIOA2;
        break;

    default:
        return 0;
    }

    /* Dummy read of status register.  This helps with debugging 
       since can determine compare status.  */
    tc->base->TC_SR;

    return 1;
}


void
tc_shutdown (tc_t tc)
{
    /* Disable TC0, TC1, TC2 peripheral clocks.  */
    AT91C_BASE_PMC->PMC_PCDR = BIT (AT91C_ID_TC0) 
        | BIT (AT91C_ID_TC1) | BIT (AT91C_ID_TC2);

    /* Perhaps force TC output pins low?  */
}


tc_t 
tc_init (tc_cfg_t *cfg)
{
    tc_t tc;

    switch (cfg->channel)
    {
    case TC_CHANNEL_0:
        /* Enable TC0 peripheral clock.  */
        AT91C_BASE_PMC->PMC_PCER = BIT (AT91C_ID_TC0);
        tc = &tc_info[TC_CHANNEL_0];
        tc->base = AT91C_BASE_TC0;
        break;

    case TC_CHANNEL_1:
        /* Enable TC1 peripheral clock.  */
        AT91C_BASE_PMC->PMC_PCER = BIT (AT91C_ID_TC1);
        tc = &tc_info[TC_CHANNEL_1];
        tc->base = AT91C_BASE_TC1;
        break;

    case TC_CHANNEL_2:
        /* Enable TC2 peripheral clock.  */
        AT91C_BASE_PMC->PMC_PCER = BIT (AT91C_ID_TC2);
        tc = &tc_info[TC_CHANNEL_2];
        tc->base = AT91C_BASE_TC2;
        break;

    default:
        return 0;
    }

    return tc;
}


static void
tc_interrupt_handler (void)
{
    /* Read status register to clear interrupt.  */
    AT91C_BASE_TC0->TC_SR;
    AT91C_BASE_TC1->TC_SR;
    AT91C_BASE_TC2->TC_SR;
}


/* Sleep for specified period.  This is useful for synchronising the
   CPU clock MCK to the timer clock, especially since the fastest
   timer clock is MCK / 2.  */
void
tc_clock_sync (tc_t tc, uint32_t period)
{
    uint32_t id;

    tc_pulse_config (tc, TC_DELAY_MODE_ONESHOT, period, period);

    switch (TC_CHANNEL (tc))
    {
    case TC_CHANNEL_0:
    default:
        id = AT91C_ID_TC0;
        break;

    case TC_CHANNEL_1:
        id = AT91C_ID_TC1;
        break;

    case TC_CHANNEL_2:
        id = AT91C_ID_TC2;
        break;
    }

    irq_config (id, 7, 
                AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE, 
                tc_interrupt_handler);
            
    irq_enable (id);

    /* Enable interrupt when have compare on A.  */
    tc->base->TC_IER = AT91C_TC_CPAS;

    tc_start (tc);
    
    /* Stop CPU clock until interrupt.  FIXME, should disable other
       interrrupts first. */
    cpu_idle ();

    /* Disable interrupt when have compare on A.  */
    tc->base->TC_IDR = AT91C_TC_CPAS;

    irq_disable (id);

    tc_stop (tc);
}
