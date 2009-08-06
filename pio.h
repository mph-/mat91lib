/** @file   pio.h
    @author M. P. Hayes, UCECE
    @date   2 Jun 2007
    @brief  Abstract the I/O ports for AT91 ARM7 microcontroller.
    @note   Macros are used to avoid function call overhead and to allow
            compile-time constant folding. 
*/
#ifndef PIO_H
#define PIO_H

#include "config.h"

#define PIO_ARM7

/** Define ports.  */
#ifdef AT91C_BASE_PIOB
/* The AT91SAM7X has two PIO ports.  */
#define PORT_A AT91C_BASE_PIOA
#define PORT_B AT91C_BASE_PIOB
#else
#define PORT_A AT91C_BASE_PIOA
#endif

#define PORT_A AT91C_BASE_PIOA

typedef AT91S_PIO *pio_port_t;
typedef uint32_t pio_mask_t;

typedef struct
{
    pio_port_t port;
    pio_mask_t bitmask;
} pio_t;


#define PIO_PORT(pio) (pio).port
#define PIO_PINS(pio) (pio).bitmask


/** Read input state from port.  */
#define pio_port_read(port) (port)->PIO_PDSR

/** Get output state for port.  */
#define pio_port_get(port) (port)->PIO_ODSR

/** Write value to port.  */
#define pio_port_write(port, value) \
    do {(port)->PIO_SODR = (value);  \
        (port)->PIO_CODR = ~(value);} while (0)

/** Configure selected PIO as output.  */
__inline __attribute__ ((always_inline))
void pio_config_output (pio_t pio)
{
    PIO_PORT (pio)->PIO_PER = PIO_PINS (pio);
    PIO_PORT (pio)->PIO_OER = PIO_PINS (pio);
}


/** Configure selected PIO as input without pullup.  */
__inline __attribute__ ((always_inline))
void pio_config_input (pio_t pio)
{
    PIO_PORT (pio)->PIO_ODR = PIO_PINS (pio);
    PIO_PORT (pio)->PIO_PER = PIO_PINS (pio);
    PIO_PORT (pio)->PIO_PPUDR = PIO_PINS (pio);
}


/** Configure selected PIO as input with pullup.  */
__inline __attribute__ ((always_inline))
void pio_config_pullup (pio_t pio)
{
    PIO_PORT (pio)->PIO_ODR = PIO_PINS (pio);
    PIO_PORT (pio)->PIO_PER = PIO_PINS (pio);
    PIO_PORT (pio)->PIO_PPUER = PIO_PINS (pio);
}


/** Configure selected pins of port for alternative use such as for
    an internal peripheral.  */
__inline __attribute__ ((always_inline))
void pio_config_peripheral (pio_t pio)
{
    PIO_PORT (pio)->PIO_PDR = PIO_PINS (pio);
}


/** Set PIO high.  */
__inline __attribute__ ((always_inline))
void pio_set_high (pio_t pio)
{
    PIO_PORT (pio)->PIO_SODR = PIO_PINS (pio);
}


/** Set PIO low.  */
__inline __attribute__ ((always_inline))
void pio_set_low (pio_t pio)
{
    PIO_PORT (pio)->PIO_CODR = PIO_PINS (pio);
}


/** Set PIO to desired state.  */
__inline __attribute__ ((always_inline))
void pio_set (pio_t pio, bool state)
{
    state ? pio_set_high (pio) : pio_set_low (pio);
}


/** Get output state of PIO.  */
__inline __attribute__ ((always_inline))
bool pio_get (pio_t pio)
{
    return (pio_port_get (PIO_PORT (pio)) & PIO_PINS (pio)) != 0;
}


/** Read input state of PIO.  */
__inline __attribute__ ((always_inline))
bool pio_read (pio_t pio)
{
    return (pio_port_read (PIO_PORT (pio)) & PIO_PINS (pio)) != 0;
}


/** Toggle PIO.  */
__inline __attribute__ ((always_inline))
void pio_toggle (pio_t pio)
{
    pio_get (pio) ? pio_set_low (pio) : pio_set_high (pio);
}


/** Enable the clock for the port.  FIXME for PIOB.  */
#define pio_init(pio) \
    AT91C_BASE_PMC->PMC_PCER = BIT (AT91C_ID_PIOA)

/** Disable the clock for the port.  FIXME for PIOB.  */
#define pio_shutdown(pio) \
    AT91C_BASE_PMC->PMC_PCDR = BIT (AT91C_ID_PIOA)


/* Define a PIO as a structure in terms of a port and a bitmask.  */
#define PIO(PORT, PORTBIT) (pio_t){PORT, BIT (PORTBIT)}

#endif
