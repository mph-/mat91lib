/** @file   tc.c
    @author M. P. Hayes
    @date   30 November 2010
    @brief  Timer counter routines for AT91SAM7 and SAM4S
    @note   There is no support for capture modes.  Only TIOAx can be
            driven at present.  
*/


#include "tc.h"
#include "irq.h"
#include "pio.h"
#include "mcu.h"
#include "bits.h"
#include "pinmap.h"


/* The SAM7 has one timer/counter controller that supports three timer
   counter channels.  Each of the three TCs has a 16 bit counter with
   3 16 bit registers (RA, RB,and RC).  In waveform mode, RA and RB
   are used to drive TIOA and TIOB, respectively.  RC can be used to
   stop the timer clock to provide a one-shot.
   
   The counter can be clocked with MCK divided by 2, 8, 32, 128, and 1024.

   The 100-pin SAM4S MCUs have two timer/counter controllers.  The
   channels are enumerated TC0, TC1, TC2..., TC5 but perversely the
   controllers are also named TC0 and TC1.  This driver only supports
   TC0, TC1, and TC2.

   In Capture Mode, TIOA and TIOB are configured as inputs.  In
   Waveform Mode, TIOA is always configured to be an output and TIOB
   is an output if it is not selected to be the external trigger.
   However, this driver does not support TIOB.  */

#define TC_CHANNEL(TC) ((TC) - tc_devices)


#ifdef __SAM4S__
#define TC0_BASE (TC0->TC_CHANNEL)
#define TC1_BASE (TC0->TC_CHANNEL + 1)
#define TC2_BASE (TC0->TC_CHANNEL + 2)
#else
#define TC0_BASE TC0
#define TC1_BASE TC1
#define TC2_BASE TC2
#endif


/* Define known TC pins.  */
static const pinmap_t tc_pins[] = 
{
    {0, PA0_PIO, PIO_PERIPH_B},  /* TIOA0 */
    {1, PA15_PIO, PIO_PERIPH_B}, /* TIOA1 */
    {2, PA26_PIO, PIO_PERIPH_B}, /* TIOA2 */
};
#define TC_PINS_NUM ARRAY_SIZE (tc_pins)

#define TC_DEVICES_NUM TC_PINS_NUM

static tc_dev_t tc_devices[TC_DEVICES_NUM];


bool
tc_start (tc_t tc)
{
    /* The TC_CCR register is write only.  */
    tc->base->TC_CCR |= TC_CCR_CLKEN | TC_CCR_SWTRG; 
    return 1;
}


bool
tc_stop (tc_t tc)
{
    tc->base->TC_CCR |= TC_CCR_CLKDIS; 
    return 1;
}


tc_counter_t
tc_counter_get (tc_t tc)
{
    return tc->base->TC_CV;
}


tc_counter_t
tc_capture_get (tc_t tc, tc_capture_t reg)
{
    switch (reg)
    {
    case TC_CAPTURE_A:
        return tc->captureA;

    case TC_CAPTURE_B:
        return tc->captureB;
        
    default:
        return 0;
}


/** Poll to see if the capture registers have been loaded.  */
tc_capture_mask_t
tc_capture_poll (tc_t tc)
{
    uint32_t status = 0;
    int return_val = 0;
    
    /* When the status register is read, the capture status
       flags are cleared!  */
    status = tc->base->TC_SR;
    
    if (status & BIT(6))
    {
        tc->captureA = tc->base->TC_RA;
        return_val |= BIT (TC_CAPTURE_A);
    }
    
    if (status & BIT(5))
    {
        tc->captureB = tc->base->TC_RB;
        return_val |= BIT (TC_CAPTURE_B);
    }
    
    return return_val;
}


/** Configure TC with specified mode.  The delay and period are in
    terms of the CPU clock.  The pulse width is period - delay.  */
bool
tc_config (tc_t tc, tc_mode_t mode, tc_period_t period, 
           tc_period_t delay)
{
    /* Many timer counters can only generate a pulse with a single
       timer clock period.  This timer counter allows the pulse width
       to be varied.  It is specified by period - delay. 

       We configure the TC in mode WAVSEL_UP_RC.  Here the counter
       is incremented and is reset when RC matches.  The output is
       driven when RA matches.  */
    switch (mode)
    {
    case TC_MODE_CLOCK:
        if (!delay)
            delay = period >> 1;

    case TC_MODE_PULSE:
        /* Set TIOAx when RA matches and clear TIOAx when RC matches.  */
        tc->base->TC_CMR = TC_CMR_BURST_NONE | TC_CMR_WAVE
            | TC_CMR_WAVSEL_UP_RC | TC_CMR_ACPA_SET | TC_CMR_ACPC_CLEAR
            | TC_CMR_ASWTRG_CLEAR;
        break;

    case TC_MODE_PULSE_INVERT:
        /* Clear TIOAx when RA matches and set TIOAx when RC matches.  */
        tc->base->TC_CMR = TC_CMR_BURST_NONE | TC_CMR_WAVE
            | TC_CMR_WAVSEL_UP_RC | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET
            | TC_CMR_ASWTRG_SET;
        break;

    case TC_MODE_PULSE_ONESHOT:
        /* Set TIOAx when RA matches and clear TIOAx when RC matches.
           Stop clock when RC matches.   */
        tc->base->TC_CMR = TC_CMR_BURST_NONE | TC_CMR_WAVE
            | TC_CMR_CPCSTOP | TC_CMR_WAVSEL_UP_RC
            | TC_CMR_ACPA_SET | TC_CMR_ACPC_CLEAR
            | TC_CMR_ASWTRG_CLEAR;
        break;

    case TC_MODE_PULSE_ONESHOT_INVERT:
        /* Clear TIOAx when RA matches and set TIOAx when RC matches.
           Stop clock when RC matches.  */
        tc->base->TC_CMR = TC_CMR_BURST_NONE | TC_CMR_WAVE
            | TC_CMR_CPCSTOP | TC_CMR_WAVSEL_UP_RC
            | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET
            | TC_CMR_ASWTRG_SET;
        break;

    case TC_MODE_DELAY_ONESHOT:
        /* Don't change TIOAx.  Stop clock when RC matches.  */
        tc->base->TC_CMR = TC_CMR_BURST_NONE | TC_CMR_WAVE
            | TC_CMR_CPCSTOP | TC_CMR_WAVSEL_UP_RC;
        break;

        /* In the capture modes, the time clock is not stopped or
           disabled when RB loaded.  The capture trigger can only be
           controlled by TIOAx.  Either TIOAx or TIOBx can be used to
           reset the counter (this is called external trigger).
           ABETRG = 1 specifies TIOAx for external trigger.  There are
           many possible combinations of reset and capture.  This
           driver does not support resetting of the counter.  

           The docs say that the external trigger gates the clock but
           it appears that it resets the counter.  bSpecifying
           TC_CMR_ETRGEDG_NONE disables this.  */

    case TC_MODE_CAPTURE_RISE_RISE:
        tc->base->TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1
            | TC_CMR_LDRA_RISING | TC_CMR_LDRB_RISING
            | TC_CMR_ABETRG | TC_CMR_ETRGEDG_NONE;
        break;

    case TC_MODE_CAPTURE_RISE_FALL:
        tc->base->TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1
            | TC_CMR_LDRA_RISING | TC_CMR_LDRB_FALLING
            | TC_CMR_ABETRG | TC_CMR_ETRGEDG_NONE;
        break;

    case TC_MODE_CAPTURE_FALL_RISE:
        tc->base->TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1
            | TC_CMR_LDRA_FALLING | TC_CMR_LDRB_RISING
            | TC_CMR_ABETRG | TC_CMR_ETRGEDG_NONE;
        break;

    case TC_MODE_CAPTURE_FALL_FALL:
        tc->base->TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1
            | TC_CMR_LDRA_FALLING | TC_CMR_LDRB_FALLING
            | TC_CMR_ABETRG | TC_CMR_ETRGEDG_NONE;
        break;
        
    default:
        return 0;
    }

    /* TODO: If period > 65536 then need to select a slower timer
       clock.  For now use MCK / 2.  Other prescale factors are 8, 32,
       128, and 1024.  */
    tc->base->TC_CMR |= TC_CMR_TCCLKS_TIMER_CLOCK1;

    /* These registers are read only when not in wave mode.  */
    tc->base->TC_RA = delay >> 1;
    tc->base->TC_RC = period >> 1;

    /* Generate a software trigger with the clock stopped to set TIOAx
       to desired state.  */
    tc->base->TC_CCR |= TC_CCR_CLKDIS | TC_CCR_SWTRG; 

    /* Make timer pin TIOAx a timer output.  Perhaps we could use
       different logical timer channels to generate pulses on TIOBx
       pins?  */
    switch (TC_CHANNEL (tc))
    {
    case TC_CHANNEL_0:
        /* Switch to peripheral B and disable pin as PIO.  */
        pio_config_set (PA0_PIO, PIO_PERIPH_B);
        break;

    case TC_CHANNEL_1:
        pio_config_set (PA15_PIO, PIO_PERIPH_B);
        break;

    case TC_CHANNEL_2:
        pio_config_set (PA26_PIO, PIO_PERIPH_B);
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
    /* Disable peripheral clock.  */
    mcu_pmc_disable (ID_TC0 + TC_CHANNEL (tc));

    /* Perhaps should force TC output pin low?  */
}


tc_t 
tc_init (const tc_cfg_t *cfg)
{
    tc_t tc;
    const pinmap_t *pin;
    unsigned int i;

    pin = 0;
    for (i = 0; i < TC_PINS_NUM; i++)
    {
        pin = &tc_pins[i]; 
        if (pin->pio == cfg->pio)
            break;
    }
    if (i >= TC_PINS_NUM)
        return 0;

    tc = &tc_devices[pin->channel];

    switch (pin->channel)
    {
    case TC_CHANNEL_0:
        tc->base = TC0_BASE;
        break;

    case TC_CHANNEL_1:
        tc->base = TC1_BASE;
        break;

    case TC_CHANNEL_2:
        tc->base = TC2_BASE;
        break;
    }

    /* Enable TCx peripheral clock.  */
    mcu_pmc_enable (ID_TC0 + pin->channel);
    

    tc_config (tc, cfg->mode, cfg->period, cfg->delay);
   

    return tc;
}


static void
tc_clock_sync_handler (void)
{
    /* Read status register to clear interrupt.  */
    TC0_BASE->TC_SR;
    TC1_BASE->TC_SR;
    TC2_BASE->TC_SR;
}

/* Sleep for specified period.  This is useful for synchronising the
   CPU clock MCK to the timer clock, especially since the fastest
   timer clock is MCK / 2.  */
void
tc_clock_sync (tc_t tc, tc_period_t period)
{
    uint32_t id;

    tc_config (tc, TC_MODE_DELAY_ONESHOT, period, period);

    id = ID_TC0 + TC_CHANNEL (tc);

    irq_config (id, 7, tc_clock_sync_handler);
            
    irq_enable (id);

    /* Enable interrupt when have compare on A.  */
    tc->base->TC_IER = TC_IER_CPAS;

    tc_start (tc);
    
    /* Stop CPU clock until interrupt.  FIXME, should disable other
       interrrupts first. */
    mcu_cpu_idle ();

    /* Disable interrupt when have compare on A.  */
    tc->base->TC_IDR = TC_IDR_CPAS;

    irq_disable (id);

    tc_stop (tc);
}
