/** @file   pio.h
    @author M. P. Hayes, UCECE
    @date   2 Jun 2007
    @brief  PIO abstraction for SAM4S microcontroller.
    @note   Macros and inline functions are used to avoid function
            call overhead and to allow compile-time constant folding. 
*/
#ifndef PIO_H
#define PIO_H

#ifdef __cplusplus
extern "C" {
#endif
    

#include "config.h"
#include "mcu.h"


#define PIO_SAM4S

/* The SAM4S PIO lines are much more configurable than the SAM7.  They
   also can be configured open-drain (multi-drive), have optional
   Schmitt trigger inputs, have internal pulldown resistors, and can
   share the pin with four peripheral signals.  These options are not
   supported.  */


/* The 64 pin MCUs have two PIO ports; the 100 pin ones have three.  */
enum {PORT_A, PORT_B, PORT_C};

/* Enumerate all the PIOs.  */
#define PIO_DEFINE(PORT, PORTBIT) ((pio_t)(((PORT) << 5) + (PORTBIT)))


/** Private macro to lookup bitmask.  */
#define PIO_BITMASK_(PIO) (BIT((PIO) & 0x1F))


/** Private macro to lookup PIO controller base address.  */
#define PIO_PORT(PIO) ((PIO) >> 5)


/** Private macro to lookup PIO controller base address.  */
#define PIO_BASE(PIO) (PIO_PORT (PIO) == PORT_A ? PIOA : PIO_PORT (PIO) == PORT_B ? PIOB : PIOC)


/** Private macro to lookup PIO controller ID.  */
#define PIO_ID(PIO) (PIO_PORT (PIO) == PORT_A ? ID_PIOA : PIO_PORT (PIO) == PORT_B ? ID_PIOB : ID_PIOC)


typedef uint32_t pio_mask_t;

typedef enum pio_config_enum 
{
    PIO_INPUT = 1,          /* Configure as input pin.  */
    PIO_PULLUP,             /* Configure as input pin with pullup.  */
    PIO_PULLDOWN,           /* Configure as input pin with pulldown.  */
    PIO_OUTPUT_LOW,         /* Configure as output, initially low.  */
    PIO_OUTPUT_HIGH,        /* Configure as output, initially high.  */
    PIO_PERIPH_A,
    PIO_PERIPH = PIO_PERIPH_A,
    PIO_PERIPH_B,
    PIO_PERIPH_A_PULLUP,
    PIO_PERIPH_PULLUP = PIO_PERIPH_A_PULLUP,
    PIO_PERIPH_B_PULLUP,
    PIO_PERIPH_C,
    PIO_PERIPH_C_PULLUP
} pio_config_t;


/** Define the pins.  */
#define PA0_PIO PIO_DEFINE(PORT_A, 0)
#define PA1_PIO PIO_DEFINE(PORT_A, 1)
#define PA2_PIO PIO_DEFINE(PORT_A, 2)
#define PA3_PIO PIO_DEFINE(PORT_A, 3)
#define PA4_PIO PIO_DEFINE(PORT_A, 4)
#define PA5_PIO PIO_DEFINE(PORT_A, 5)
#define PA6_PIO PIO_DEFINE(PORT_A, 6)
#define PA7_PIO PIO_DEFINE(PORT_A, 7)
#define PA8_PIO PIO_DEFINE(PORT_A, 8)
#define PA9_PIO PIO_DEFINE(PORT_A, 9)
#define PA10_PIO PIO_DEFINE(PORT_A, 10)
#define PA11_PIO PIO_DEFINE(PORT_A, 11)
#define PA12_PIO PIO_DEFINE(PORT_A, 12)
#define PA13_PIO PIO_DEFINE(PORT_A, 13)
#define PA14_PIO PIO_DEFINE(PORT_A, 14)
#define PA15_PIO PIO_DEFINE(PORT_A, 15)
#define PA16_PIO PIO_DEFINE(PORT_A, 16)
#define PA17_PIO PIO_DEFINE(PORT_A, 17)
#define PA18_PIO PIO_DEFINE(PORT_A, 18)
#define PA19_PIO PIO_DEFINE(PORT_A, 19)
#define PA20_PIO PIO_DEFINE(PORT_A, 20)
#define PA21_PIO PIO_DEFINE(PORT_A, 21)
#define PA22_PIO PIO_DEFINE(PORT_A, 22)
#define PA23_PIO PIO_DEFINE(PORT_A, 23)
#define PA24_PIO PIO_DEFINE(PORT_A, 24)
#define PA25_PIO PIO_DEFINE(PORT_A, 25)
#define PA26_PIO PIO_DEFINE(PORT_A, 26)
#define PA27_PIO PIO_DEFINE(PORT_A, 27)
#define PA28_PIO PIO_DEFINE(PORT_A, 28)
#define PA29_PIO PIO_DEFINE(PORT_A, 29)
#define PA30_PIO PIO_DEFINE(PORT_A, 30)
#define PA31_PIO PIO_DEFINE(PORT_A, 31)

#define PB0_PIO PIO_DEFINE(PORT_B, 0)
#define PB1_PIO PIO_DEFINE(PORT_B, 1)
#define PB2_PIO PIO_DEFINE(PORT_B, 2)
#define PB3_PIO PIO_DEFINE(PORT_B, 3)
#define PB4_PIO PIO_DEFINE(PORT_B, 4)
#define PB5_PIO PIO_DEFINE(PORT_B, 5)
#define PB6_PIO PIO_DEFINE(PORT_B, 6)
#define PB7_PIO PIO_DEFINE(PORT_B, 7)
#define PB8_PIO PIO_DEFINE(PORT_B, 8)
#define PB9_PIO PIO_DEFINE(PORT_B, 9)
#define PB10_PIO PIO_DEFINE(PORT_B, 10)
#define PB11_PIO PIO_DEFINE(PORT_B, 11)
#define PB12_PIO PIO_DEFINE(PORT_B, 12)
#define PB13_PIO PIO_DEFINE(PORT_B, 13)
#define PB14_PIO PIO_DEFINE(PORT_B, 14)
#define PB15_PIO PIO_DEFINE(PORT_B, 15)
#define PB16_PIO PIO_DEFINE(PORT_B, 16)
#define PB17_PIO PIO_DEFINE(PORT_B, 17)
#define PB18_PIO PIO_DEFINE(PORT_B, 18)
#define PB19_PIO PIO_DEFINE(PORT_B, 19)
#define PB20_PIO PIO_DEFINE(PORT_B, 20)
#define PB21_PIO PIO_DEFINE(PORT_B, 21)
#define PB22_PIO PIO_DEFINE(PORT_B, 22)
#define PB23_PIO PIO_DEFINE(PORT_B, 23)
#define PB24_PIO PIO_DEFINE(PORT_B, 24)
#define PB25_PIO PIO_DEFINE(PORT_B, 25)
#define PB26_PIO PIO_DEFINE(PORT_B, 26)
#define PB27_PIO PIO_DEFINE(PORT_B, 27)
#define PB28_PIO PIO_DEFINE(PORT_B, 28)
#define PB29_PIO PIO_DEFINE(PORT_B, 29)
#define PB30_PIO PIO_DEFINE(PORT_B, 30)
#define PB31_PIO PIO_DEFINE(PORT_B, 31)

/* SPI  */
#define MOSI0_PIO PA13_PIO
#define MOSI0_PERIPH PIO_PERIPH_A
#define MISO0_PIO PA12_PIO
#define MISO0_PERIPH PIO_PERIPH_A
#define SPCK0_PIO PA14_PIO
#define SPCK0_PERIPH PIO_PERIPH_A

/* SSC  */
#define RD_PIO PA18_PIO
#define RD_PERIPH PIO_PERIPH_A
#define RK_PIO PA19_PIO
#define RK_PERIPH PIO_PERIPH_A
#define RF_PIO PA20_PIO
#define RF_PERIPH PIO_PERIPH_A

#define TD_PIO PA17_PIO
#define TD_PERIPH PIO_PERIPH_A
#define TK_PIO PA16_PIO
#define TK_PERIPH PIO_PERIPH_A
#define TF_PIO PA15_PIO
#define TF_PERIPH PIO_PERIPH_A

/* TC  */
#define TIOA0_PIO PA0_PIO
#define TIOA0_PERIPH PIO_PERIPH_B
#define TIOA1_PIO PA15_PIO
#define TIOA1_PERIPH PIO_PERIPH_B
#define TIOA2_PIO PA26_PIO
#define TIOA2_PERIPH PIO_PERIPH_B

#define TIOB0_PIO PA1_PIO
#define TIOB0_PERIPH PIO_PERIPH_B
#define TIOB1_PIO PA16_PIO
#define TIOB1_PERIPH PIO_PERIPH_B
#define TIOB2_PIO PA27_PIO
#define TIOB2_PERIPH PIO_PERIPH_B    

/* TWI  */
#define TWD0_PIO PA3_PIO
#define TWD0_PERIPH PIO_PERIPH_A
#define TWCK0_PIO PA4_PIO
#define TWCK0_PERIPH PIO_PERIPH_A
#define TWD1_PIO PB4_PIO
#define TWD1_PERIPH PIO_PERIPH_A
#define TWCK1_PIO PB5_PIO
#define TWCK1_PERIPH PIO_PERIPH_A

/* UART0  */
#define URXD0_PIO PA9_PIO
#define URXD0_PERIPH PIO_PERIPH_A
#define UTXD0_PIO PA10_PIO
#define UTXD0_PERIPH PIO_PERIPH_A

/* UART1  */
#define URXD1_PIO PB2_PIO
#define URXD1_PERIPH PIO_PERIPH_A
#define UTXD1_PIO PB3_PIO
#define UTXD1_PERIPH PIO_PERIPH_A

/* USART0  */
#define RXD0_PIO PA5_PIO
#define RXD0_PERIPH PIO_PERIPH_A
#define TXD0_PIO PA6_PIO
#define TXD0_PERIPH PIO_PERIPH_A
#define RTS0_PIO PA7_PIO
#define RTS0_PERIPH PIO_PERIPH_A
#define CTS0_PIO PA8_PIO
#define CTS0_PERIPH PIO_PERIPH_A

/* USART1  */
#define RXD1_PIO PA21_PIO
#define RXD1_PERIPH PIO_PERIPH_A
#define TXD1_PIO PA22_PIO
#define TXD1_PERIPH PIO_PERIPH_A
#define RTS1_PIO PA24_PIO
#define RTS1_PERIPH PIO_PERIPH_A
#define CTS1_PIO PA25_PIO
#define CTS1_PERIPH PIO_PERIPH_A


typedef uint32_t pio_t;


/** Enable the clock for the port.  This is required for input
    operations.  */
static inline void
pio_init (pio_t pio)
{
    mcu_pmc_enable (PIO_ID (pio));
}


/** Disable the clock for the port.  */
static inline void
pio_shutdown (pio_t pio)
{
    mcu_pmc_disable (PIO_ID (pio));
}


/** Configure PIO
    @param pio  */
static inline __attribute__((optimize (2))) bool
pio_config_set (pio_t pio, pio_config_t config)
{
    switch (config)
    {
    case PIO_OUTPUT_HIGH:
        PIO_BASE (pio)->PIO_SODR = PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_PER = PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_OER = PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_PUDR = PIO_BITMASK_ (pio);
        return 1;

    case PIO_OUTPUT_LOW:
        PIO_BASE (pio)->PIO_CODR = PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_PER = PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_OER = PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_PUDR = PIO_BITMASK_ (pio);
        return 1;

    case PIO_INPUT:
        PIO_BASE (pio)->PIO_ODR = PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_PER = PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_PUDR = PIO_BITMASK_ (pio);
        pio_init (pio);
        return 1;

    case PIO_PULLUP:
        PIO_BASE (pio)->PIO_ODR = PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_PER = PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_PUER = PIO_BITMASK_ (pio);
        pio_init (pio);
        return 1;

    case PIO_PERIPH_A:
        PIO_BASE (pio)->PIO_ABCDSR[0] &= ~PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_ABCDSR[1] &= ~PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_PDR = PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_PUDR = PIO_BITMASK_ (pio);
        return 1;

    case PIO_PERIPH_B:
        PIO_BASE (pio)->PIO_ABCDSR[0] |= PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_ABCDSR[1] &= ~PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_PDR = PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_PUDR = PIO_BITMASK_ (pio);
        return 1;

    case PIO_PERIPH_C:
        PIO_BASE (pio)->PIO_ABCDSR[1] |= PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_ABCDSR[0] &= ~PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_PDR = PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_PUDR = PIO_BITMASK_ (pio);
        return 1;

    case PIO_PERIPH_A_PULLUP:
        PIO_BASE (pio)->PIO_ABCDSR[0] &= ~PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_ABCDSR[1] &= ~PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_PDR = PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_PUER = PIO_BITMASK_ (pio);
        return 1;

    case PIO_PERIPH_B_PULLUP:
        PIO_BASE (pio)->PIO_ABCDSR[0] &= ~PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_ABCDSR[1] |= PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_PDR = PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_PUER = PIO_BITMASK_ (pio);
        return 1;

    case PIO_PERIPH_C_PULLUP:
        PIO_BASE (pio)->PIO_ABCDSR[1] &= ~PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_ABCDSR[0] |= PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_PDR = PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_PUER = PIO_BITMASK_ (pio);
        return 1;

    default:
        return 0;
    }
}


/** Set PIO high.
    @param pio  */
static __always_inline__ __attribute__((optimize (2))) void
pio_output_high (pio_t pio)
{
    PIO_BASE (pio)->PIO_SODR = PIO_BITMASK_ (pio);
}


/** Set PIO low.
    @param pio  */
static __always_inline__ __attribute__((optimize (2))) void 
pio_output_low (pio_t pio)
{
    PIO_BASE (pio)->PIO_CODR = PIO_BITMASK_ (pio);
}


/** Set PIO to desired state.
    @param pio 
    @param state  */
static __always_inline__ __attribute__((optimize (2))) void
pio_output_set (pio_t pio, bool state)
{
    state ? pio_output_high (pio) : pio_output_low (pio);
}


/** Get output state of PIO.
    @param pio
    @return state  */
static __always_inline__ __attribute__((optimize (2))) bool
pio_output_get (pio_t pio)
{
    return (PIO_BASE (pio)->PIO_ODSR & PIO_BITMASK_ (pio)) != 0;
}


/** Read input state of PIO.
    @param pio
    @return state  */
static __always_inline__ __attribute__((optimize (2))) bool
pio_input_get (pio_t pio)
{
    return (PIO_BASE (pio)->PIO_PDSR & PIO_BITMASK_ (pio)) != 0;
}


/** Toggle PIO.
    @param pio  */
static __always_inline__ __attribute__((optimize (2))) void
pio_output_toggle (pio_t pio)
{
    pio_output_get (pio) ? pio_output_low (pio) : pio_output_high (pio);
}


void
pio_init (pio_t pio);


void
pio_shutdown (pio_t pio);


typedef enum pio_irq_config_enum 
{
    PIO_IRQ_FALLING_EDGE = 1, 
    PIO_IRQ_RISING_EDGE, 
    PIO_IRQ_ANY_EDGE, 
    PIO_IRQ_LOW_LEVEL, 
    PIO_IRQ_HIGH_LEVEL
} pio_irq_config_t;


/** Configure PIO for interrupt
    @param pio  */
static inline bool 
pio_irq_config_set (pio_t pio, pio_irq_config_t config)
{

    /* The PIO Controller can be programmed to generate an interrupt
       when it detects an edge or a level on an I/O line, regardless
       of how it is configured: input, output, peripheral, etc.

       For input change detection, the PIO controller clock must be
       enabled.  */

    switch (config)
    {
    case PIO_IRQ_FALLING_EDGE:
        PIO_BASE (pio)->PIO_ESR = PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_FELLSR = PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_AIMER = PIO_BITMASK_ (pio);
        return 1;

    case PIO_IRQ_RISING_EDGE:
        PIO_BASE (pio)->PIO_ESR = PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_REHLSR = PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_AIMER = PIO_BITMASK_ (pio);
        return 1;

    case PIO_IRQ_ANY_EDGE:
        PIO_BASE (pio)->PIO_AIMDR = PIO_BITMASK_ (pio);
        return 1;

    case PIO_IRQ_LOW_LEVEL:
        PIO_BASE (pio)->PIO_LSR = PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_FELLSR = PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_AIMER = PIO_BITMASK_ (pio);
        return 1;

    case PIO_IRQ_HIGH_LEVEL:
        PIO_BASE (pio)->PIO_LSR = PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_REHLSR = PIO_BITMASK_ (pio);
        PIO_BASE (pio)->PIO_AIMER = PIO_BITMASK_ (pio);
        return 1;

    default:
        return 0;
    }
}


/** Enable PIO input change interrupt for specified PIO.  Note, it is
    necessary to read PIO_ISR to clear the interrupt source.
    Unfortunately, this will clear other pending input change
    interrupts from the same PIO controller.  */
static __always_inline__ void
pio_irq_enable (pio_t pio)
{
    PIO_BASE (pio)->PIO_IER = PIO_BITMASK_ (pio);
}


/** Disable PIO input change interrupt for specified PIO.   */
static __always_inline__ void
pio_irq_disable (pio_t pio)
{
    PIO_BASE (pio)->PIO_IDR = PIO_BITMASK_ (pio);
}



/** Clear interrupt for specified PIO.  Unfortunately, this has the
    gnarly side-effect of clearing ALL the PIO interrupts on the same
    port.  The work-around is to have a common interrupt handler that
    delegates to previously registered sub-handlers.  */
static __always_inline__ uint32_t
pio_irq_clear (pio_t pio)
{
    return PIO_BASE (pio)->PIO_ISR;
}



#ifdef __cplusplus
}
#endif    
#endif

