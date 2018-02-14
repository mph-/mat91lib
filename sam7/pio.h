/** @file   pio.h
    @author M. P. Hayes, UCECE
    @date   2 Jun 2007
    @brief  PIO abstraction for AT91 ARM7 microcontroller.
    @note   Macros and inline functions are used to avoid function
            call overhead and to allow compile-time constant folding. 
*/
#ifndef PIO_H
#define PIO_H

#ifdef __cplusplus
extern "C" {
#endif
    

#include "config.h"

#define PIO_AT91

/** Define ports.  */
#ifdef PIOB
/* The AT91SAM7X has two PIO ports.  */
enum {PORT_A, PORT_B};

/* Define a PIO as a structure in terms of a port and a bitmask.
   The AT91SAM7X does not have PA31 or PB31 so use the MSB to select the port.  */
#define PIO_DEFINE(PORT, PORTBIT) (((PORT) << 31) + BIT (PORTBIT))


/** Private macro to lookup bitmask.  */
#define PIO_BITMASK_(PIO) (PIO & 0x7fffffff)


/** Private macro to lookup port register.  */
#define PIO_PORT_(PIO) (((PIO) & 0x80000000) ? PIOB : PIOA)


/** Private macro to lookup PIO controller ID.  */
#define PIO_ID(PIO) (PIO_PORT (PIO) == PORT_A ? AT91C_ID_PIOA : AT91C_ID_PIOB)


#else
/* The AT91SAMSX has one PIO port.  */
enum {PORT_A};

/* Define a PIO as a structure in terms of a bitmask.  */
#define PIO_DEFINE(PORT, PORTBIT) (BIT (PORTBIT))


/** Private macro to lookup bitmask.  */
#define PIO_BITMASK_(PIO) (PIO)


/** Private macro to lookup port register.  */
#define PIO_PORT_(PIO) (PIOA)
#endif


/** Private macro to lookup PIO controller ID.  */
#define PIO_ID(PIO) (AT91C_ID_PIOA)


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
    PIO_PERIPH_B_PULLUP
} pio_config_t;


typedef uint32_t pio_t;


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


#ifdef SPI1
#define MOSI0_PIO PA17_PIO
#define MISO0_PIO PA16_PIO
#define SPCK0_PIO PA18_PIO
#define MOSI1_PIO PA23_PIO
#define MISO1_PIO PA24_PIO
#define SPCK1_PIO PA22_PIO
#else
#define MOSI0_PIO PA13_PIO
#define MOSI0_PERIPH PIO_PERIPH_A
#define MISO0_PIO PA12_PIO
#define MISO0_PERIPH PIO_PERIPH_A
#define SPCK0_PIO PA14_PIO
#define SPCK0_PERIPH PIO_PERIPH_A
#endif


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

/* USART0  */
#define TXD0_PIO PA6_PIO
#define TXD0_PERIPH PIO_PERIPH_A
#define RXD0_PIO PA5_PIO
#define RXD0_PERIPH PIO_PERIPH_A
#define RTS0_PIO PA7_PIO
#define RTS0_PERIPH PIO_PERIPH_A
#define CTS0_PIO PA8_PIO
#define CTS0_PERIPH PIO_PERIPH_A

/* USART1  */
#define TXD1_PIO PA22_PIO
#define TXD1_PERIPH PIO_PERIPH_A
#define RXD1_PIO PA21_PIO
#define RXD1_PERIPH PIO_PERIPH_A
#define RTS1_PIO PA24_PIO
#define RTS1_PERIPH PIO_PERIPH_A
#define CTS1_PIO PA25_PIO
#define CTS1_PERIPH PIO_PERIPH_A

/* UART0  */
#define URXD0_PIO PA9_PIO
#define UTXD0_PIO PA10_PIO

/* TC  */
#define TIOA0_PIO PA0_PIO
#define TIOA0_PERIPH PIO_PERIPH_B
#define TIOB0_PIO PA1_PIO
#define TIOB0_PERIPH PIO_PERIPH_B
#define TIOA1_PIO PA15_PIO
#define TIOA1_PERIPH PIO_PERIPH_B
#define TIOA2_PIO PA26_PIO
#define TIOA2_PERIPH PIO_PERIPH_B



/** Configure PIO
    @param pio  */
static inline
bool pio_config_set (pio_t pio, pio_config_t config)
{
    switch (config)
    {
    case PIO_OUTPUT_HIGH:
        PIO_PORT_ (pio)->PIO_SODR = PIO_BITMASK_ (pio);
        PIO_PORT_ (pio)->PIO_PER = PIO_BITMASK_ (pio);
        PIO_PORT_ (pio)->PIO_OER = PIO_BITMASK_ (pio);
        PIO_PORT_ (pio)->PIO_PPUDR = PIO_BITMASK_ (pio);
        return 1;

    case PIO_OUTPUT_LOW:
        PIO_PORT_ (pio)->PIO_CODR = PIO_BITMASK_ (pio);
        PIO_PORT_ (pio)->PIO_PER = PIO_BITMASK_ (pio);
        PIO_PORT_ (pio)->PIO_OER = PIO_BITMASK_ (pio);
        PIO_PORT_ (pio)->PIO_PPUDR = PIO_BITMASK_ (pio);
        return 1;

    case PIO_INPUT:
        PIO_PORT_ (pio)->PIO_ODR = PIO_BITMASK_ (pio);
        PIO_PORT_ (pio)->PIO_PER = PIO_BITMASK_ (pio);
        PIO_PORT_ (pio)->PIO_PPUDR = PIO_BITMASK_ (pio);
        return 1;

    case PIO_PULLUP:
        PIO_PORT_ (pio)->PIO_ODR = PIO_BITMASK_ (pio);
        PIO_PORT_ (pio)->PIO_PER = PIO_BITMASK_ (pio);
        PIO_PORT_ (pio)->PIO_PPUER = PIO_BITMASK_ (pio);
        return 1;

    case PIO_PERIPH_A:
        PIO_PORT_ (pio)->PIO_ASR = PIO_BITMASK_ (pio);
        PIO_PORT_ (pio)->PIO_PDR = PIO_BITMASK_ (pio);
        PIO_PORT_ (pio)->PIO_PPUDR = PIO_BITMASK_ (pio);
        return 1;

    case PIO_PERIPH_B:
        PIO_PORT_ (pio)->PIO_BSR = PIO_BITMASK_ (pio);
        PIO_PORT_ (pio)->PIO_PDR = PIO_BITMASK_ (pio);
        PIO_PORT_ (pio)->PIO_PPUDR = PIO_BITMASK_ (pio);
        return 1;

    case PIO_PERIPH_A_PULLUP:
        PIO_PORT_ (pio)->PIO_ASR = PIO_BITMASK_ (pio);
        PIO_PORT_ (pio)->PIO_PDR = PIO_BITMASK_ (pio);
        PIO_PORT_ (pio)->PIO_PPUER = PIO_BITMASK_ (pio);
        return 1;

    case PIO_PERIPH_B_PULLUP:
        PIO_PORT_ (pio)->PIO_BSR = PIO_BITMASK_ (pio);
        PIO_PORT_ (pio)->PIO_PDR = PIO_BITMASK_ (pio);
        PIO_PORT_ (pio)->PIO_PPUER = PIO_BITMASK_ (pio);
        return 1;

    default:
        return 0;
    }
}


/** Set PIO high.
    @param pio  */
static inline
void pio_output_high (pio_t pio)
{
    PIO_PORT_ (pio)->PIO_SODR = PIO_BITMASK_ (pio);
}


/** Set PIO low.
    @param pio  */
static inline
void pio_output_low (pio_t pio)
{
    PIO_PORT_ (pio)->PIO_CODR = PIO_BITMASK_ (pio);
}


/** Set PIO to desired state.
    @param pio 
    @param state  */
static inline
void pio_output_set (pio_t pio, bool state)
{
    state ? pio_output_high (pio) : pio_output_low (pio);
}


/** Get output state of PIO.
    @param pio
    @return state  */
static inline
bool pio_output_get (pio_t pio)
{
    return (PIO_PORT_ (pio)->PIO_ODSR & PIO_BITMASK_ (pio)) != 0;
}


/** Read input state of PIO.
    @param pio
    @return state  */
static inline
bool pio_input_get (pio_t pio)
{
    return (PIO_PORT_ (pio)->PIO_PDSR & PIO_BITMASK_ (pio)) != 0;
}


/** Toggle PIO.
    @param pio  */
static inline
void pio_output_toggle (pio_t pio)
{
    pio_output_get (pio) ? pio_output_low (pio) : pio_output_high (pio);
}


/** Enable the clock for the port.  This is required for input
    operations.  FIXME for PIOB.  */
#define pio_init(pio) \
    PMC->PMC_PCER = BIT (AT91C_ID_PIOA)


/** Disable the clock for the port.  FIXME for PIOB.  */
#define pio_shutdown(pio) \
    PMC->PMC_PCDR = BIT (AT91C_ID_PIOA)


/** Enable PIO input change interrupt for specified PIO.  Note, it is
    necessary to read PIO_ISR to clear the interrupt source.
    Unfortunately, this will clear other pending input change
    interrupts from the same PIO controller.  */
static inline void
pio_irq_enable (pio_t pio)
{
    PIO_PORT_ (pio)->PIO_IER = PIO_BITMASK_ (pio);
}


/** Disable PIO input change interrupt for specified PIO.   */
static inline void
pio_irq_disable (pio_t pio)
{
    PIO_PORT_ (pio)->PIO_IDR = PIO_BITMASK_ (pio);
}


/** Clear interrupt for specified PIO.  Unfortunately, this has the
 gnarly side-effect of clearing ALL the PIO interrupts on the same
 port.  */
static inline uint32_t
pio_irq_clear (pio_t pio)
{
    return PIO_PORT_ (pio)->PIO_ISR;
}



#ifdef __cplusplus
}
#endif    
#endif


