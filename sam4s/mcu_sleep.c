/** @file   mcu_sleep.c
    @author M. P. Hayes, UCECE
    @date   4 May 2015
    @brief  Sleep routines for SAM4S processors
*/


#include "mcu_sleep.h"
#include "cpu.h"
#include "irq.h"


void
mcu_sleep (const mcu_sleep_cfg_t *cfg)
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
        return;

    if (cfg->pio)
    {
        unsigned int i;

        for (i = 0; i < ARRAY_SIZE (wakeup_pins); i++)
        {
            if (wakeup_pins[i] == cfg->pio)
            {
                if (cfg->active)
                    SUPC->SUPC_WUIR = BIT (i) | BIT (i + 16);
                else
                    SUPC->SUPC_WUIR = BIT (i);

                /* Set debounce period here...  */
                break;
            }
        }
    }
    
    switch (cfg->mode)
    {
    case MCU_SLEEP_MODE_BACKUP:

        /* For backup mode, set DEEPSLEEP bit in CPU system control
           register and set the VROFF bit of SUPC_CR.  The CPU is
           reset when on wake up.  */
        SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
        SUPC->SUPC_CR = SUPC_CR_KEY (0xA5u) | SUPC_CR_VROFF_STOP_VREG;
        irq_global_enable ();
        cpu_wfi ();
        
        /* Exit from backup mode occurs if there is an event on the
           WKUPEN0-15 pins, supply monitor (SM), RTC alarm, or RTT
           alarm.  The supply monitor checks the voltage on the VDDIO pin
           if it is enabled.  */

    default:
        return;
        
    }
}

