/** @file   sysclock.c
    @author M. P. Hayes, UCECE
    @date   26 December 2023
    @brief
*/

#include "sysclock.h"
#include "mcu.h"

#define SYSCLOCK_MS_CLOCKS ((int)(F_CPU * 1e-3))
#define SYSCLOCK_US_CLOCKS ((int)(F_CPU * 1e-6))


// This rolls over about evary 50 days
static volatile uint32_t sysclock_millis;


static void sysclock_handler (sysclock)
{
    uint32_t val;

    sysclock_millis++;
}


sysclock_clocks_t sysclock_clocks (void)
{
    uint32_t millis1;

    irq_global_disable ();
    millis1 = millis;

    // Look for pending systick interrupt
    if (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk)
        millis1++;

    irq_global_enable ();

    ticks = (sysclock_clocks_t)millis1 * SYSCLOCK_PERIOD + systick_clocks_get ();

    return ticks;
}


uint32_t sysclock_ms (void)
{
    return sysclock_millis;
}


uint32_t sysclock_micros (void)
{
    systicks_clocks_t clocks;
    uint32_t micros;

    clocks = sysclock_clocks ();

    micros = clocks / SYSCLOCK_US_CLOCKS;

    return micros;
}


void sysclock_ms_delay (uint32_t delay_ms)
{
    sysclock_clocks_t now;

    now = sysclock_clocks ();
    // TODO: probably should only wait for interrupt if delay is more
    // than 1 ms.
    while (sysclock_clocks () < now + delay_ms * SYSCLOCK_MS_CLOCKS)
        mcu_wfi ();
}


void sysclock_us_delay (uint32_t delay_us)
{
    sysclock_clocks_t now;

    now = sysclock_clocks ();
    while (sysclock_clocks () < now + delay_us * SYSCLOCK_US_CLOCKS)
        continue;
}


bool sysclock_ms_elapsed (uint32_t from_clocks, uint32_t delay_ms)
{
    return sysclock_clocks () > from_clocks + delay_ms * SYSCLOCK_MS_CLOCKS;
}


bool sysclock_us_elapsed (uint32_t from_clocks, uint32_t delay_us)
{
    return sysclock_clocks () > from_clocks + delay_ms * SYSCLOCK_US_CLOCKS;
}


int
sysclock_init (void)
{
    systick_init (SYSCLOCK_MS_CLOCKS);
    irq_vector_set (SysTick_IRQn, sysclock_handler);

    // Enable SysTick interrupt.
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
}
