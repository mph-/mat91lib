/** @file   crt0.c
    @author M. P. Hayes, UCECE
    @date   10 July 2014
    @brief  C run time initialisation for the Atmel AT91SAM4S series 
            of microcontrollers. 
*/

#include "config.h"
#include "sam4s.h"
#include "mcu.h"
#include "irq.h"

/** Symbols defined by linker script.  These are all VMAs except those
    with a _load__ suffix which are LMAs.  */
extern char __stack_start__;    /** Top of stack.  */
extern char __vectors_start__;  /** Start of vector table.  */
extern char __data_load__;      /** Start of initial values for .data (in flash).  */
extern char __data_start__;     /** Start of data (in SRAM).  */
extern char __data_end__;       /** End of data (in SRAM).  */
extern char __bss_start__;      /** Start of uninitialised variables.  */
extern char __bss_end__;        /** End of uninitialised variables.  */


/* TODO: reallocate vector table into SRAM.  */


int main (void);

void __libc_init_array (void);

void reset (void)
    __attribute__ ((alias ("_reset_handler")));

void _reset_handler (void);

void _unexpected_handler (void);


void _nmi_handler (void)
{
    _unexpected_handler ();
}


void _hardfault_handler (void)
{
    /* This is due to an error during exception processing. 
       The reason can be found in SCB_HFSR.  */
    _unexpected_handler ();
}


void _memmanage_handler (void)
{
    _unexpected_handler ();
}


void _busfault_handler (void)
{
    _unexpected_handler ();
}


void _usagefault_handler (void)
{
    _unexpected_handler ();
}


/* Exception table this needs to be mapped into flash.  This table
   does not contain the interrupt vectors.  These are allocated dynamically 
   in the array exception_table.  */
__attribute__ ((section(".vectors")))
irq_handler_t static_exception_table[] =
{
    (irq_handler_t) (&__stack_start__),
    _reset_handler,
    
    _nmi_handler,
    _hardfault_handler,
    _memmanage_handler,
    _busfault_handler,
    _usagefault_handler,
};


/* This needs to be carefully aligned.   */
__attribute__ ((section(".dynamic_vectors")))
irq_handler_t exception_table[] =
{
    (irq_handler_t) (&__stack_start__),
    _reset_handler,
    
    _nmi_handler,
    _hardfault_handler,
    _memmanage_handler,
    _busfault_handler,
    _usagefault_handler,
    0, 0, 0, 0,        /* Reserved */
    _unexpected_handler,
    _unexpected_handler,
    0,                 /* Reserved  */
    _unexpected_handler,
    _unexpected_handler,
    
    /* Configurable interrupts  */
    _unexpected_handler,    /* 0  Supply Controller */
    _unexpected_handler,    /* 1  Reset Controller */
    _unexpected_handler,    /* 2  Real Time Clock */
    _unexpected_handler,    /* 3  Real Time Timer */
    _unexpected_handler,    /* 4  Watchdog Timer */
    _unexpected_handler,    /* 5  PMC */
    _unexpected_handler,    /* 6  EFC0 */
    _unexpected_handler,    /* 7  EFC1 */
    _unexpected_handler,    /* 8  UART0 */
    _unexpected_handler,    /* 9  UART1 */
    _unexpected_handler,    /* 10 SMC */
    _unexpected_handler,    /* 11 Parallel IO Controller A */
    _unexpected_handler,    /* 12 Parallel IO Controller B */
    _unexpected_handler,    /* 13 Parallel IO Controller C */
    _unexpected_handler,    /* 14 USART 0 */
    _unexpected_handler,    /* 15 USART 1 */
    _unexpected_handler,    /* 16 Reserved */
    _unexpected_handler,    /* 17 Reserved */
    _unexpected_handler,    /* 18 HSMCI */
    _unexpected_handler,    /* 19 TWI 0 */
    _unexpected_handler,    /* 20 TWI 1 */
    _unexpected_handler,    /* 21 SPI */
    _unexpected_handler,    /* 22 SSC */
    _unexpected_handler,    /* 23 Timer Counter 0 */
    _unexpected_handler,    /* 24 Timer Counter 1 */
    _unexpected_handler,    /* 25 Timer Counter 2 */
    _unexpected_handler,    /* 26 Timer Counter 3 */
    _unexpected_handler,    /* 27 Timer Counter 4 */
    _unexpected_handler,    /* 28 Timer Counter 5 */
    _unexpected_handler,    /* 29 ADC controller */
    _unexpected_handler,    /* 30 DACC controller */
    _unexpected_handler,    /* 31 PWM */
    _unexpected_handler,    /* 32 CRC Calculation Unit */
    _unexpected_handler,    /* 33 Analog Comparator */
    _unexpected_handler,    /* 34 USB Device Port */
    _unexpected_handler     /* 35 not used */
};


void _reset_handler (void)
{
    char *src;
    char *dst;

    SCB->VTOR = 0;

    /* There's not much frigging around to set things up; the initial
       stack pointer is loaded from the vector table.  At this point
       we are running on the slow clock?  We could crank things up
       before initialising variables etc but this will put constraints
       on the code to set up the clock, etc.  */

    /* Initialise initialised global variables in .data and relocate
       .ramtext function for functions in the ROM model that need
       to execute out of RAM for speed.  */
    for (src = &__data_load__, dst = &__data_start__; dst < &__data_end__; )
        *dst++ = *src++;
    
    /* Zero uninitialised global variables in .bss.  */
    for (dst = &__bss_start__; dst < &__bss_end__; )
        *dst++ = 0;
    
    /* Remap the exception table into SRAM to allow dynamic allocation. 
       This register is zero on reset.  */
    SCB->VTOR = (uint32_t) &exception_table & SCB_VTOR_TBLOFF_Msk;

    /* Set up clocks, etc.  */
    mcu_init ();

    /* Initialise C library.  */
    __libc_init_array ();
    
    main ();
    
    /* Hang.  */
    while (1)
        continue;
}


/** Dummy ISR for unexpected interrupts.  */
void
_unexpected_handler (void)
{
    while (1)
        continue;
}
