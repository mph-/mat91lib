/** @file   tc.c
    @author M. P. Hayes
    @date   30 November 2010
    @brief  Timer counter routines for AT91SAM7 processors
*/


#include "tc.h"
#include "bits.h"

/* Each of the three TCs has a 16 bit counter with 3 16 bit registers (RA, RB,and RC).
   In waveform mode, RA and RB are used to drive TIOA and TIOB, respectively.  RC can
   be used to stop the clock to provide a one-shot. 
   
   The counter can be clocked with MCK divided by 2, 8, 32, 128, and 1024.

 */


static tc_t tc_info[TC_CHANNEL_NUM];


bool tc_start (tc_t *tc)
{
    tc->base->TC_CCR |= (AT91C_TC_CLKEN | AT91C_TC_SWTRG); 
    return 1;
}


bool tc_stop (tc_t *tc)
{
    tc->base->TC_CCR |= AT91C_TC_CLKDIS; 
    return 1;
}


bool
tc_one_shot_pulse_config (tc_t *tc, uint32_t delay, uint32_t pulse_width, bool invert)
{
    if (invert)
    {
        /* Clear TIOAx when RA matches and set TIOAx when RC matches.
           Stop clock when RC matches.  Use MCK / 2.  */
        tc->base->TC_CMR = AT91C_TC_CLKS_TIMER_DIV1_CLOCK | AT91C_TC_BURST_NONE
            | AT91C_TC_CPCSTOP | AT91C_TC_WAVESEL
            | AT91C_TC_ACPA_CLEAR | AT91C_TC_ACPC_SET;
    }
    else
    {
        /* Set TIOAx when RA matches and clear TIOAx when RC matches.
           Stop clock when RC matches.  Use MCK / 2.  */
        tc->base->TC_CMR = AT91C_TC_CLKS_TIMER_DIV1_CLOCK | AT91C_TC_BURST_NONE
            | AT91C_TC_CPCSTOP | AT91C_TC_WAVESEL
            | AT91C_TC_ACPA_SET | AT91C_TC_ACPC_CLEAR;
    }

    /* If delay + pulse_width > 65536 then need to select a slower timer clock.  */

    tc->base->TC_RA = delay >> 1;
    tc->base->TC_RC = (delay + pulse_width) >> 1;

    /* Make timer pin TIOAx a timer output.  Perhaps we could use different logical
       timer channels to generate pulses on TIOBx pins?  */
    switch (tc - tc_info)
    {
    case TC_CHANNEL_0:
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

    return 1;
}


void
tc_shutdown (void)
{
    /* Disable TC0, TC1, TC2 peripheral clocks.  */
    AT91C_BASE_PMC->PMC_PCDR = BIT (AT91C_ID_TC0) | BIT (AT91C_ID_TC1) | BIT (AT91C_ID_TC2);

    /* Perhaps force TC output pins low?  */
}


tc_t *
tc_init (tc_channel_t channel)
{
    tc_t *tc;

    switch (channel)
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
