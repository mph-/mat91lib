/** @file   mcu.c
    @author M. P. Hayes, UCECE
    @date   04 June 2007
    @brief  System routines for SAM4S processors
*/

#include "config.h"
#include "mcu.h"
#include "cpu.h"
#include "irq.h"
#include "delay.h"
#include "pio.h"


#define MCU_FLASH_WAIT_STATES ((MCU_FLASH_READ_CYCLES) - 1)

#define MCU_FLASH_WAIT_STATES_MAX 7


#ifndef MCU_MCK_PRESCALE
#error MCU_MCK_PRESCALE undefined, also check MCU_PLLA_MUL since this no longer assumes a MCK prescale by 2
#define MCU_MCK_PRESCALE 2
#endif

#if MCU_MCK_PRESCALE == 1
#define MCU_MCK_PRESCALER_VALUE 0
#endif
#if MCU_MCK_PRESCALE == 2
#define MCU_MCK_PRESCALER_VALUE 1
#endif
#if MCU_MCK_PRESCALE == 3
#define MCU_MCK_PRESCALER_VALUE 7
#endif
#if MCU_MCK_PRESCALE == 4
#define MCU_MCK_PRESCALER_VALUE 2
#endif
#if MCU_MCK_PRESCALE == 8
#define MCU_MCK_PRESCALER_VALUE 3
#endif
#if MCU_MCK_PRESCALE == 16
#define MCU_MCK_PRESCALER_VALUE 4
#endif
#if MCU_MCK_PRESCALE == 32
#define MCU_MCK_PRESCALER_VALUE 5
#endif
#if MCU_MCK_PRESCALE == 64
#define MCU_MCK_PRESCALER_VALUE 6
#endif

#ifndef MCU_MCK_PRESCALER_VALUE
#error Invalid MCU_MCK_PRESCALE value, must be 1, 2, 3, 4, 8, 16, 32, or 64
#endif

/* CPU_PLL_DIV and CPU_PLL_MUL are for backward compatibility and
   shall be deprecated.  */
#ifdef CPU_PLL_DIV
#define MCU_PLL_DIV CPU_PLL_DIV
#endif

#ifdef CPU_PLL_MUL
#define MCU_PLL_MUL CPU_PLL_MUL
#endif

#ifndef MCU_PLLA_MUL
#define MCU_PLLA_MUL MCU_PLL_MUL
#endif

#ifndef MCU_PLLA_DIV
#define MCU_PLLA_DIV MCU_PLL_DIV
#endif

#if MCU_PLLA_MUL > 62
/* See errata in datasheet.  */
#error MCU_PLLA_MUL must be 62 or smaller
#endif

#ifdef MCU_PLLB_MUL
#if MCU_PLLB_MUL > 62
/* See errata in datasheet.  */
#error MCU_PLLB_MUL must be 62 or smaller
#endif
#endif

#define F_PLLA_IN  (F_XTAL / MCU_PLLA_DIV)
#define F_PLLA_OUT  (F_PLLA_IN * MCU_PLLA_MUL)

/* The PLLA input frequency needs to be between 3 and 32 MHz.  This is
   the frequency after the divider and before the frequency
   multiplier.  */
#define F_PLLA_IN_MIN    3000000
#define F_PLLA_IN_MAX   32000000

/* The PLLA output frequency needs to be between 80 and 240 MHz.   */
#define F_PLLA_OUT_MIN  80000000
#define F_PLLA_OUT_MAX 240000000

#if 0
/* Not all C preprocessors can handle floating point macros.  */
#if F_PLLA_IN < F_PLLA_IN_MIN
#error MCU_PLLA_MUL is too large, the PLLA input frequency is too low
#endif

#if F_PLLA_IN > F_PLLA_IN_MAX
#error MCU_PLLA_MUL is too small, the PLLA input frequency is too high
#endif
#endif


/* Internal slow clock frequency.  */
#define F_SLCK 32678

#define MCU_OS_DELAY 1.5e-3
#define MCU_OS_COUNT ((uint16_t) ((MCU_OS_DELAY * F_SLCK + 7)) / 8)

#ifndef MCU_PLLA_COUNT
#define MCU_PLLA_COUNT 0x3fu
#endif

#ifndef MCU_PLLB_COUNT
#define MCU_PLLB_COUNT 0x3fu
#endif

/* The required number of slow clock cycles (400 Hz) divided by 8.
   The xtal oscillator requires 14.5 ms max to start up.  This is approx
   6 clocks.  */
#define MCU_MAINCK_COUNT 100

#define MCU_USB_LOG2_DIV 0

/* The PLLA frequency is given by (F_XTAL * MCU_PLLA_MUL) / MCU_PLLA_DIV / 2.

   The MCK frequency is given by F_PLLA / MCU_MCK_PRESCALE.
*/


/* The AT91 nor flash memory is single plane so it is not possible
   to write to it while executing code out of it.  */

static void
mcu_flash_fmr_set (uint32_t value)
    __attribute__ ((section(".ramtext")));


static void
mcu_flash_fmr_set (uint32_t fmr)
{
    EFC0->EEFC_FMR = fmr;
}


/** Set number of flash wait states.  */
void
mcu_flash_wait_states_set (uint8_t wait_states)
{
    uint32_t fmr = EFC0->EEFC_FMR & (~EEFC_FMR_FWS_Msk);

    mcu_flash_fmr_set (fmr | EEFC_FMR_FWS (wait_states));
}


/** Initialise flash memory controller.  */
static void
mcu_flash_init (void)
{
    /* No wait states are required when running on MAINCK at reset.  */
}


void
mcu_unique_id (mcu_unique_id_t id)
    __attribute__ ((section(".ramtext")));


/** Return 128-bit unique ID.  */
void
mcu_unique_id (mcu_unique_id_t id)
{
    int i;
    uint32_t *p = (void *)0x400000;

    /* I presume interrupts need to be disabled while this function is
       being executed.  */

    EFC0->EEFC_FCR = EEFC_FCR_FCMD_STUI | EEFC_FCR_FKEY_PASSWD;
    while ((EFC0->EEFC_FSR & EEFC_FSR_FRDY))
        continue;

    for (i = 0; i < 4; i++)
        *id++ = *p++;

    EFC0->EEFC_FCR = EEFC_FCR_FCMD_SPUI | EEFC_FCR_FKEY_PASSWD;
    while (! (EFC0->EEFC_FSR & EEFC_FSR_FRDY))
        continue;
}


uint32_t mcu_chipid (void)
{
    return REG_CHIPID_CIDR;
}


void
mcu_xtal_mainck_start (void)
{
    /* Enable XTAL oscillator.  Be careful with read-modify-writes to
     the CKGR_MOR register since reading of the key field does not
     always return zero.  */
    PMC->CKGR_MOR = (PMC->CKGR_MOR & ~CKGR_MOR_MOSCXTBY) |
        CKGR_MOR_KEY (0x37) | CKGR_MOR_MOSCXTEN |
        CKGR_MOR_MOSCXTST (MCU_MAINCK_COUNT);

    /* Wait for the XTAL oscillator to stabilize.  */
    while (! (PMC->PMC_SR & PMC_SR_MOSCXTS))
        continue;

    /* Select XTAL oscillator for MAINCK.  */
    PMC->CKGR_MOR = (PMC->CKGR_MOR & ~CKGR_MOR_MOSCXTBY) |
        CKGR_MOR_KEY (0x37) | CKGR_MOR_MOSCXTEN |
        CKGR_MOR_MOSCXTST (MCU_MAINCK_COUNT) | CKGR_MOR_MOSCSEL;
}


static void
mcu_fast_rc_mainck_start (void)
{
    /* Enable fast RC oscillator.  Be careful with read-modify-writes
     to the CKGR_MOR register since reading of the key field does not
     return zero.  */
    PMC->CKGR_MOR = CKGR_MOR_KEY (0x37) | CKGR_MOR_MOSCRCEN |
        CKGR_MOR_MOSCXTST (MCU_MAINCK_COUNT);

    /* Select the 12 MHz fast RC oscillator.  There is a choice of 4
       MHz default, 8 MHz and 12 MHz.  The 8 MHz and 12 MHz settings
       are calibrated.  Note, cannot combine with enabling of RC
       oscillator. */
    PMC->CKGR_MOR = CKGR_MOR_KEY (0x37) | CKGR_MOR_MOSCRCEN |
        CKGR_MOR_MOSCXTST (MCU_MAINCK_COUNT) | CKGR_MOR_MOSCRCF_12_MHz;

    /* Wait for the RC oscillator to stabilize.  */
    while (! (PMC->PMC_SR & PMC_SR_MOSCRCS))
        continue;

    /* Select RC oscillator for MAINCK.  */
    PMC->CKGR_MOR = CKGR_MOR_KEY (0x37) | (PMC->CKGR_MOR & ~CKGR_MOR_MOSCSEL);
}


static void
mcu_fast_rc_mainck_stop (void)
{
    /* Disable fast RC oscillator.  Be careful with read-modify-writes
       to the CKGR_MOR register since reading of the key field does not
       return zero.  */
    PMC->CKGR_MOR = CKGR_MOR_KEY (0x37) | (PMC->CKGR_MOR & ~CKGR_MOR_MOSCRCEN);
}


static void
mcu_clock_error (int code)
{
#ifdef LED_ERROR_PIO
    pio_config_set (LED_ERROR_PIO, PIO_OUTPUT_HIGH);
#endif

    while (1);
}


static void
mcu_mck_ready_wait (int code)
{
    int timeout;

    /* Whenever PMC_MCKR is written then PMC_SR_MCKRDY is cleared; it
       gets set when the MCK is established.

       This waits far longer than necessary; only about 5 slow clock
       cycles is required in the worst case.  */

    for (timeout = 2000; timeout && ! (PMC->PMC_SR & PMC_SR_MCKRDY); timeout--)
        continue;

    if (timeout == 0)
        mcu_clock_error (code);
}


static int
mcu_mck_css_get (void)
{
    return PMC->PMC_MCKR & 0x03;
}


static int
mcu_mck_pres_get (void)
{
    return (PMC->PMC_MCKR >> 4) & 0x07;
}


/** Set up the main clock (MAINCK), PLLA clock, and master clock (MCK).   */
static int
mcu_clock_init (void)
{
    /* To minimize the power required to start up the system, the main
       oscillator is disabled after reset and the 4 MHz internal RC
       oscillator is selected.

       There are four clock sources for MCK:
       1. SLCK (the 32 kHz internal RC oscillator or
          32 kHz external crystal slow clock),

       2. MAINCK (the external 3-20 MHz crystal or internal 4/8/12 MHz
       internal fast RC oscillator main clock),

       3. PLLACK,

       4. PLLBCK.

       The PLLs are driven by MAINCK.

       One of these four clock sources can be fed to a prescaler (with
       divisors 2^0 ... 2^6) to drive MCK (master clock).

       The main oscillator (external crystal) can range from 3--20 MHz.
       The PLLA frequency can range from 80--240 MHz.

       Here we assume that an external crystal is used for the MAINCK
       and this is multiplied by PLLA to drive MCK.

       If the USB clock is derived from PLLA, MCK needs to be
       a multiple of 48 MHz required for the USB clock.  This
       restriction can be relaxed using PLLB for the USB.  The UDP
       driver determines the correct prescale for the USB.
    */

#if 0
    /* After programming with OpenOCD, the MCK is still running
       from PLLACK, however, the number of flash wait states is suitable.  */

    if (mcu_mck_css_get () != PMC_MCKR_CSS_MAIN_CLK)
        mcu_clock_error (10);

    if (mcu_mck_pres_get () != 0)
        mcu_clock_error (11);
#endif

#ifdef MCU_12MHZ_RC_OSC
    /* Start fast RC oscillator and select as MAINCK.  */
    mcu_fast_rc_mainck_start ();
#else
    /* Start XTAL oscillator and select as MAINCK.  */
    mcu_xtal_mainck_start ();

    /* TODO: check if XTAL oscillator fails to start; say if XTAL not
       connected and switch to RC oscillator instead.  */

    mcu_fast_rc_mainck_stop ();
#endif

    /* Set prescaler.  */
    if (MCU_MCK_PRESCALER_VALUE)
    {
        PMC->PMC_MCKR = (PMC->PMC_MCKR & ~PMC_MCKR_PRES_Msk) | (MCU_MCK_PRESCALER_VALUE << 4);

        mcu_mck_ready_wait (2);
    }

    /* Disable PLLA if it is running and reset fields.  */
    PMC->CKGR_PLLAR = CKGR_PLLAR_ONE | CKGR_PLLAR_MULA (0);

    DELAY_US (1);

    /* Configure and start PLLA.  The PLLA start delay is MCU_PLLA_COUNT
       SLCK cycles.  Note, PLLA (but not PLLB) needs the mysterious
       bit CKGR_PLLAR_ONE set.  */
    PMC->CKGR_PLLAR = CKGR_PLLAR_MULA (MCU_PLLA_MUL - 1)
        | CKGR_PLLAR_DIVA (MCU_PLLA_DIV)
        | CKGR_PLLAR_PLLACOUNT (MCU_PLLA_COUNT) | CKGR_PLLAR_ONE;

#ifdef MCU_PLLB_MUL
    /* Disable PLLB if it is running and reset fields.  */
    PMC->CKGR_PLLBR = CKGR_PLLBR_MULB (0);

    /* Configure and start PLLB.  The PLLB start delay is MCU_PLLB_COUNT
       SLCK cycles.  */
    PMC->CKGR_PLLBR = CKGR_PLLBR_MULB (MCU_PLLB_MUL - 1)
        | CKGR_PLLBR_DIVB (MCU_PLLB_DIV)
        | CKGR_PLLBR_PLLBCOUNT (MCU_PLLB_COUNT);
#endif

    /* Wait for PLLA to start up.  This will fail if the main XTAL
       oscillator does not have a long enough startup time as
       specified by MCU_MAINCK_COUNT.  */
    while (! (PMC->PMC_SR & PMC_SR_LOCKA))
        continue;

#ifdef MCU_PLLB_MUL
    /* Wait for PLLB to start up.  */
    while (! (PMC->PMC_SR & PMC_SR_LOCKB))
        continue;
#endif

    mcu_flash_wait_states_set (MCU_FLASH_WAIT_STATES);

    /* Switch to PLLA_CLK for MCK.  */
    PMC->PMC_MCKR = (PMC->PMC_MCKR & ~PMC_MCKR_CSS_Msk)
        | PMC_MCKR_CSS_PLLA_CLK;
    mcu_mck_ready_wait (3);

    if (mcu_mck_css_get () != PMC_MCKR_CSS_PLLA_CLK)
        mcu_clock_error (12);

    if (mcu_mck_pres_get () != MCU_MCK_PRESCALER_VALUE)
        mcu_clock_error (13);

    return 1;
}


extern void _irq_unexpected_handler (void);

extern void _irq_spurious_handler (void);


/* Enable NRST pin.  */
void
mcu_reset_enable (void)
{
    RSTC->RSTC_MR |= RSTC_MR_URSTEN | (0xa5 << 24);
}


/* Disable NRST pin.  */
void
mcu_reset_disable (void)
{
    /* Disable NRST pin.  */
    RSTC->RSTC_MR =
        (RSTC->RSTC_MR & ~RSTC_MR_URSTEN) | (0xa5 << 24);
}


uint8_t
mcu_reset_type_get (void)
{
    return (RSTC->RSTC_SR >> 8) & 0x07;
}


/** Disable watchdog.  */
void
mcu_watchdog_disable (void)
{
    WDT->WDT_MR = WDT_MR_WDDIS;
}


void
mcu_bus_fault_enable (void)
{
    SCB->SHCSR |= SCB_SHCSR_BUSFAULTENA_Msk;
}


void
mcu_usage_fault_enable (void)
{
    SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk;
}


void
mcu_mem_fault_enable (void)
{
    SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;
}


/** Disable JTAG but allow SWD.  This frees up PB5 and PB6 as PIO.  */
void
mcu_jtag_disable (void)
{
    /* Allow PB4 and PB5 for general I/O.  */
    REG_CCFG_SYSIO |= (BIT (4) | BIT (5));
}


/** Initialise flash, disable watchdog, set up clocks.  This and any
    functions it calls must be for the C runtime startup to
    work.  */
void
mcu_init (void)
{
    int i;

    mcu_flash_init ();

    /* Disable all interrupts to be sure when debugging.  This is only
       needed if jump to reset.  */
    for (i = 0; i < 8; i++)
        NVIC->ICER[i] = ~0;

    /* Enable exceptions so can debug crash, otherwise hardfault
       handler called for all.  */
    mcu_bus_fault_enable ();

    mcu_usage_fault_enable ();

    mcu_mem_fault_enable ();

    /* Enable reset pin.  */
    mcu_reset_enable ();

    mcu_watchdog_disable ();

    mcu_clock_init ();

#if 0
    /* Enable protect mode.  */
    AIC->AIC_DCR |= AIC_DCR_PROT;
    /* Use MATRIX_WPMR ?  */
#endif

    irq_global_enable ();
}


void
mcu_reset (void)
{
    /* Reset processor and peripherals.  */
    RSTC->RSTC_CR = RSTC_CR_PROCRST | RSTC_CR_PERRST      | (0xa5 << 24);
}


int
mcu_select_slowclock (void)
{
    /* Switch master clock (MCK) from PLLACLK to SLCK.  Note the prescale
       (PRES) and clock source (CSS) fields cannot be changed at the
       same time.  We first switch from the PLLACLK to SLCK then set
       the prescaler.  */

    PMC->PMC_MCKR = (PMC->PMC_MCKR & ~PMC_MCKR_CSS_Msk)
        | PMC_MCKR_CSS_SLOW_CLK;

    mcu_mck_ready_wait (4);

    /* If we prescale by 1, then for some odd reason, wake from backup
       sometimes starts with PLLA as MAINCK with prescale = 1.  This
       clock frequency is too fast and the CPU does not run.  Making
       the prescale 2 reduces the chances of a problem occuring but
       does not fix it. */
    PMC->PMC_MCKR = (PMC->PMC_MCKR & ~PMC_MCKR_PRES_Msk)
        | PMC_MCKR_PRES_CLK_1;

    mcu_mck_ready_wait (5);

    /* Disable PLLA.  */
    PMC->CKGR_PLLAR = CKGR_PLLAR_ONE | CKGR_PLLAR_MULA (0);

    /* Disable main oscillator.  */
    PMC->CKGR_MOR = CKGR_MOR_KEY (0x37);

    return 0;
}


void mcu_cpu_idle (void)
{
    cpu_wfi ();
}


void
mcu_watchdog_reset (void)
{
    WDT->WDT_CR = 0xA5000000 | WDT_CR_WDRSTT;
}


void
mcu_watchdog_enable (void)
{
    /* Enable watchdog with 2s timeout.  */
    WDT->WDT_MR = WDT_MR_WDD(0x200) | WDT_MR_WDRSTEN | WDT_MR_WDDBGHLT;
    mcu_watchdog_reset ();
}
