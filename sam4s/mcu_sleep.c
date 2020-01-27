/** @file   mcu_sleep.c
    @author M. P. Hayes, UCECE
    @date   4 May 2015
    @brief  Sleep routines for SAM4S processors
*/


#include "mcu_sleep.h"
#include "cpu.h"
#include "irq.h"


bool
mcu_sleep_wakeup_set (const mcu_sleep_wakeup_cfg_t *cfg)
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

    if (!cfg)
        return 0;

    if (cfg->pio)
    {
        unsigned int i;

        for (i = 0; i < ARRAY_SIZE (wakeup_pins); i++)
        {
            if (wakeup_pins[i] == cfg->pio)
            {
                /* Set wakeup inputs register.  */
                SUPC->SUPC_WUIR |= BIT (i);                
                if (cfg->active_high)
                    SUPC->SUPC_WUIR |= BIT (i + 16);
                return 1;
            }
        }
    }
    return 0;
}


void
mcu_sleep (const mcu_sleep_cfg_t *cfg)
{
    switch (cfg->mode)
    {
    case MCU_SLEEP_MODE_BACKUP:

        /* For backup mode, set DEEPSLEEP bit in CPU system control
           register and set the VROFF bit of SUPC_CR.  The CPU is
           reset when on wake up.  */
        SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
        SUPC->SUPC_CR = SUPC_CR_KEY (0xA5u) | SUPC_CR_VROFF_STOP_VREG;
        
        /* Exit from backup mode occurs if there is an event on the
           WKUPEN0-15 pins, supply monitor (SM), RTC alarm, or RTT
           alarm.  The supply monitor monitors the voltage on the
           VDDIO pin if it is enabled.  The MCU is reset.  */
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

