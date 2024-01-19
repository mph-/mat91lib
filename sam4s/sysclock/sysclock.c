/** @file   sysclock.c
    @author M. P. Hayes, UCECE
    @date   26 December 2023
    @brief This keeps track of time in terms of CPU clocks and
    provides delay functions.  It uses an interrupt every millisecond;
    this can call a heartbeat function.
*/

#include "sysclock.h"
#include "systick.h"
#include "cpu.h"
#include "irq.h"


typedef struct sysclock_dev_struct
{
    // This rolls over about every 50 days
    volatile uint32_t millis;
    sysclock_callback_t callback;
} sysclock_dev_t;


static sysclock_dev_t sysclock_dev;


static void sysclock_handler (void)
{
    // This is called every millisecond.

    sysclock_dev.millis++;
    if (sysclock_dev.callback)
        sysclock_dev.callback ();
}


sysclock_clocks_t sysclock_clocks (void)
{
    uint32_t millis1;
    sysclock_clocks_t clocks;

    irq_global_disable ();
    millis1 = sysclock_dev.millis;

    // Look for pending systick interrupt
    if (SCB->ICSR & SCB_ICSR_PENDSTSET_Msk)
        millis1++;

    irq_global_enable ();

    clocks = (sysclock_clocks_t)millis1 * SYSCLOCK_MS_CLOCKS + systick_clocks_get ();

    return clocks;
}


uint32_t sysclock_millis (void)
{
    return sysclock_dev.millis;
}


uint32_t sysclock_micros (void)
{
    sysclock_clocks_t clocks;
    uint32_t micros;

    clocks = sysclock_clocks ();

    micros = clocks / SYSCLOCK_US_CLOCKS;

    return micros;
}


void sysclock_millis_delay (uint32_t delay_ms)
{
    sysclock_clocks_t now;

    now = sysclock_clocks ();
    // TODO: probably should only wait for interrupt if delay is more
    // than 1 ms.
    while (sysclock_clocks () < now + delay_ms * SYSCLOCK_MS_CLOCKS)
        cpu_wfi ();
}


void sysclock_micros_delay (uint32_t delay_us)
{
    sysclock_clocks_t now;

    now = sysclock_clocks ();
    while (sysclock_clocks () < now + delay_us * SYSCLOCK_US_CLOCKS)
        continue;
}


bool sysclock_millis_elapsed (sysclock_clocks_t from_clocks, uint32_t delay_ms)
{
    sysclock_clocks_t to_clocks;

    to_clocks = from_clocks + delay_ms * SYSCLOCK_MS_CLOCKS;

    return sysclock_clocks () > to_clocks;
}


bool sysclock_micros_elapsed (sysclock_clocks_t from_clocks, uint32_t delay_us)
{
    sysclock_clocks_t to_clocks;

    to_clocks = from_clocks + delay_us * SYSCLOCK_US_CLOCKS;

    return sysclock_clocks () > to_clocks;
}


void sysclock_callback (sysclock_callback_t callback)
{
    // Could register multiple callbacks;
    sysclock_dev.callback = callback;
}


int
sysclock_init (void)
{
    systick_init (SYSCLOCK_MS_CLOCKS);
    irq_vector_set (SysTick_IRQn, sysclock_handler);

    // Enable SysTick interrupt.
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
    return 1;
}
