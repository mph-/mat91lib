/** @file   mcu_sleep.c
    @author M. P. Hayes, UCECE
    @date   4 May 2015
    @brief  Sleep routines for SAM4S processors
*/


#include "mcu_sleep.h"
#include "cpu.h"
#include "mcu.h"
#include "delay.h"
#include "irq.h"


static bool
mcu_sleep_wakeup_set (pio_t pio, bool active_high)
{
    static const pio_t wakeup_pins[] =
        {
            PA0_PIO,
            PA1_PIO,
            PA2_PIO,
            PA4_PIO,
            PA5_PIO,
            PA8_PIO,
            PA9_PIO,
            PA11_PIO,
            PA14_PIO,
            PA19_PIO,
            PA20_PIO,
            PA30_PIO,
            PB2_PIO,
            PB5_PIO,
            PA15_PIO,
            PA16_PIO
        };

    for (unsigned int i = 0; i < ARRAY_SIZE (wakeup_pins); i++)
    {
        if (wakeup_pins[i] == pio)
        {
            /* Set wakeup inputs register.  */
            SUPC->SUPC_WUIR |= BIT (i);
            if (active_high)
                SUPC->SUPC_WUIR |= BIT (i + 16);
            return 1;
        }
    }
    return 0;
}


void
mcu_sleep (const mcu_sleep_cfg_t *cfg)
{
    for (int i = 0; i < cfg->num_wakeups; i++)
    {
        // Don't sleep if wakeup pin active.
        if (pio_input_get (cfg->wakeups[i].pio) ^ ! cfg->wakeups[i].active_high)
            return;

        mcu_sleep_wakeup_set (cfg->wakeups[i].pio, cfg->wakeups[i].active_high);
    }

    // Set debounce period in slow clock cycles.
    BITS_INSERT (SUPC->SUPC_WUMR, cfg->debounce, 13, 15);

    // Note, the SUPC is powered from VDDIO and the contents
    // of its registers do not seem to be cleared on reset.
    SUPC->SUPC_WUIR = 0;

    switch (cfg->mode)
    {
    case MCU_SLEEP_MODE_BACKUP:

        // Select slow clock, disable PLLA, disable main xtal osc
        mcu_select_slowclock ();

        /* For backup mode, the datasheet says to set DEEPSLEEP bit in
           CPU system control register and set the VROFF bit of
           SUPC_CR.

           This uses the alternative method where DEEPSLEEP is not set
           but WFE is used instead.  */

        // SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
        SUPC->SUPC_CR = SUPC_CR_KEY (0xA5u) | SUPC_CR_VROFF_STOP_VREG;
        // Dummy read
        SUPC->SUPC_MR;
        cpu_wfe ();
        cpu_wfi ();

        /* Exit from backup mode occurs if there is an event on the
           WKUPEN0-15 pins, supply monitor (SM), RTC alarm, or RTT
           alarm.  The supply monitor monitors the voltage on the
           VDDIO pin if it is enabled.  The MCU is reset using
           backup_reset.  */
        break;

    case MCU_SLEEP_MODE_WAIT:

        /* This mode is for fast wakeup.  The clocks of the core,
           peripherals and memories are stopped but these devices are
           still powered.  */

        /* TODO: set fast RC oscillator?  For flash low powr mode, set
           FLPM bitfield in PMC_FSMR, set flash waitstate to 0.  */

        PMC->CKGR_MOR |= CKGR_MOR_KEY (0x37) | CKGR_MOR_WAITMODE;

        /* Exit from backup mode occurs if there is an event on the
           WKUPEN0-15 pins, USB wakeup, RTC alarm, or RTT
           alarm.  The supply monitor monitors the voltage on the
           VDDIO pin if it is enabled.  The MCU is not reset.  */
        break;

    case MCU_SLEEP_MODE_SLEEP:

        /* This mode is for very fast wakeup.  Only the core clock is
           stopped.  The MCU can be woken from an interrupt.  */
        irq_global_enable ();
        cpu_wfi ();
        break;

    default:
        return;
    }
}
