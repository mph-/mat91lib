/** @file   pio.h
    @author M. P. Hayes, UCECE
    @date   2 Jun 2007
    @brief  PIO abstraction for AT91 ARM7 microcontroller.
    @note   Macros and inline functions are used to avoid function
            call overhead and to allow compile-time constant folding. 
*/
#ifndef PIO_H
#define PIO_H

#include "config.h"

#define PIO_AT91

/** Define ports.  */
#ifdef AT91C_BASE_PIOB
/* The AT91SAM7X has two PIO ports.  */
#define PORT_A AT91C_BASE_PIOA
#define PORT_B AT91C_BASE_PIOB
#else
#define PORT_A AT91C_BASE_PIOA
#endif

#define PIO_A PORT_A
#define PIO_B PORT_B


typedef AT91S_PIO *pio_port_t;
typedef uint32_t pio_mask_t;

typedef struct
{
    pio_port_t port;
    pio_mask_t bitmask;
} pio_t;


/* Define a PIO as a structure in terms of a port and a bitmask.  */
#define PIO_DEFINE(PORT, PORTBIT) (pio_t){PORT, BIT (PORTBIT)}


typedef enum pio_config_enum 
{
    PIO_INPUT = 1,          /* Configure as input pin.  */
    PIO_PULLUP,             /* Configure as input pin with pullup.  */
    PIO_PULLDOWN,           /* Configure as input pin with pulldown.  */
    PIO_OUTPUT_LOW,         /* Configure as output, initially low.  */
    PIO_OUTPUT_HIGH,        /* Configure as output, initially high.  */
    PIO_PERIPH,
    PIO_PERIPH_A = PIO_PERIPH,
    PIO_PERIPH_B
} pio_config_t;


/** Configure PIO
    @param pio  */
static inline
bool pio_config_set (pio_t pio, pio_config_t config)
{
    switch (config)
    {
    case PIO_OUTPUT_HIGH:
        pio.port->PIO_SODR = pio.bitmask;
        pio.port->PIO_PER = pio.bitmask;
        pio.port->PIO_OER = pio.bitmask;
        return 1;

    case PIO_OUTPUT_LOW:
        pio.port->PIO_CODR = pio.bitmask;
        pio.port->PIO_PER = pio.bitmask;
        pio.port->PIO_OER = pio.bitmask;
        return 1;

    case PIO_INPUT:
        pio.port->PIO_ODR = pio.bitmask;
        pio.port->PIO_PER = pio.bitmask;
        pio.port->PIO_PPUDR = pio.bitmask;
        return 1;

    case PIO_PULLUP:
        pio.port->PIO_ODR = pio.bitmask;
        pio.port->PIO_PER = pio.bitmask;
        pio.port->PIO_PPUER = pio.bitmask;
        return 1;

    case PIO_PERIPH_A:
        pio.port->PIO_PDR = pio.bitmask;
        pio.port->PIO_ASR = pio.bitmask;
        return 1;

    case PIO_PERIPH_B:
        pio.port->PIO_PDR = pio.bitmask;
        pio.port->PIO_BSR = pio.bitmask;
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
    pio.port->PIO_SODR = pio.bitmask;
}


/** Set PIO low.
    @param pio  */
static inline
void pio_output_low (pio_t pio)
{
    pio.port->PIO_CODR = pio.bitmask;
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
    return (pio.port->PIO_ODSR & pio.bitmask) != 0;
}


/** Read input state of PIO.
    @param pio
    @return state  */
static inline
bool pio_input_get (pio_t pio)
{
    return (pio.port->PIO_PDSR & pio.bitmask) != 0;
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
    AT91C_BASE_PMC->PMC_PCER = BIT (AT91C_ID_PIOA)


/** Disable the clock for the port.  FIXME for PIOB.  */
#define pio_shutdown(pio) \
    AT91C_BASE_PMC->PMC_PCDR = BIT (AT91C_ID_PIOA)

#endif
