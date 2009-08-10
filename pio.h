/** @file   pio.h
    @author M. P. Hayes, UCECE
    @date   2 Jun 2007
    @brief  PIO abstraction for AT91 ARM7 microcontroller.
    @note   Macros and inline functions are used to avoid function
            call overhead and to allow compile-time constant folding. 
*/
#ifndef PIO_H
#define PIO_H


/* Note, perhaps add piobus and piogroup support.
   #define DATABUS PIOBUS_DEFINE (PORT, pin1, bits)  */

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


/** Configure selected PIO as output.
    @param pio  */
static inline
void pio_config_output (pio_t pio)
{
    pio.port->PIO_PER = pio.bitmask;
    pio.port->PIO_OER = pio.bitmask;
}


/** Configure selected PIO as input without pullup.
    @param pio  */
static inline
void pio_config_input (pio_t pio)
{
    pio.port->PIO_ODR = pio.bitmask;
    pio.port->PIO_PER = pio.bitmask;
    pio.port->PIO_PPUDR = pio.bitmask;
}


/** Configure selected PIO as input with pullup.
    @param pio  */
static inline
void pio_config_pullup (pio_t pio)
{
    pio.port->PIO_ODR = pio.bitmask;
    pio.port->PIO_PER = pio.bitmask;
    pio.port->PIO_PPUER = pio.bitmask;
}


/** Configure selected pins of port for alternative use such as for
    an internal peripheral.
    @param pio  */
static inline
void pio_config_peripheral (pio_t pio)
{
    pio.port->PIO_PDR = pio.bitmask;
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
void pio_output (pio_t pio, bool state)
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
bool pio_input (pio_t pio)
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
