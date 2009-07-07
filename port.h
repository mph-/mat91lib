/** @file   port.h
    @author M. P. Hayes, UCECE
    @date   2 Jun 2007
    @brief  Abstract the I/O ports for AT91 ARM7 microcontroller.
    @note   Macros are used to avoid function call overhead and to allow
            compile-time constant folding. 
*/
#ifndef PORT_H
#define PORT_H

#include "config.h"

#define PORT_ARM7

/** Define ports.  */
#ifdef AT91C_BASE_PIOB
/* The AT91SAM7X has two GPIO ports.  */
#define PORT_A AT91C_BASE_PIOA
#define PORT_B AT91C_BASE_PIOB
#else
#define PORT_A AT91C_BASE_PIOA
#endif

/* By default all ports default to inputs with pullups enabled.  */

#define PORT_BITS_MASK(first, last) ((1 << ((last) + 1)) - (1 << (first)))

typedef AT91S_PIO *port_t;
typedef uint32_t port_bit_t;
typedef uint32_t port_data_t;
typedef uint32_t port_mask_t;


#define PORT_CFG(PORT, PORTBIT) {(PORT), BIT (PORTBIT)}

typedef struct
{
    port_t port;
    port_mask_t bitmask;
} port_cfg_t;


/** Read input state from port.  */
#define port_read(port) (port)->PIO_PDSR

/** Get output state for port.  */
#define port_get(port) (port)->PIO_ODSR

/** Write value to port.  */
#define port_write(port, value) \
    do {(port)->PIO_SODR = (value); \
        (port)->PIO_CODR = ~(value);} while (0)

/** Configure selected pins of port as outputs.  */
#define port_pins_config_output(port, pins) \
    do {(port)->PIO_PER = (pins); (port)->PIO_OER = (pins);} while (0)

/** Configure selected pins of port as inputs without pullup.  */
#define port_pins_config_input(port, pins) \
    do {(port)->PIO_ODR = (pins);               \
        (port)->PIO_PER = (pins);               \
        (port)->PIO_PPUDR = (pins);} while (0)

/** Configure selected pins of port as inputs with pullup.  */
#define port_pins_config_pullup(port, pins) \
    do {(port)->PIO_ODR = (pins);               \
        (port)->PIO_PER = (pins);               \
        (port)->PIO_PPUER = (pins); } while (0)

/** Configure selected pins of port for alternative use such as for
    an internal peripheral.  */
#define port_pins_config_peripheral(port, pins) \
        (port)->PIO_PDR = (pins)


/** Set selected pins of port high.  */
#define port_pins_set_high(port, pins) (port)->PIO_SODR = (pins)

/** Set selected pins of port low.  */
#define port_pins_set_low(port, pins) (port)->PIO_CODR = (pins)

/** Set selected pins of port to desired state.  */
#define port_pins_set(port, pins, state)  \
    do {if (state)                              \
            port_pins_set_high (port, pins);    \
        else                                    \
            port_pins_set_low (port, pins);} while (0)

/** Toggle selected pins of port.  */
#define port_pins_toggle(port, pins) \
    do {port_data_t _tmp = ~port_get (port); \
        port_pins_write (port, pins, _tmp);} while (0)

/** Read input state from selected pins of port.  */
#define port_pins_read(port, pins) (port_read (port) & (pins))

/** Get output state for selected pins of port.  */
#define port_pins_get(port, pins) (port_get (port) & (pins))

/** Write to selected pins of port. */
#define port_pins_write(port, pins, value) \
    do {(port)->PIO_SODR = (pins) & (value); \
        (port)->PIO_CODR = (pins) & ~(value);} while (0)

/** Configure selected pin of port as output.  */
#define port_pin_config_output(port, pin) \
    port_pins_config_output (port, BIT (pin))

/** Configure selected pin of port as input without pullup.  */
#define port_pin_config_input(port, pin) \
    port_pins_config_input (port, BIT (pin))

/** Configure selected pin of port as input with pullup.  */
#define port_pin_config_pullup(port, pin) \
    port_pins_config_pullup (port, BIT (pin))

/** Configure selected pins of port for alternative use such as for
    an internal peripheral.  */
#define port_pin_config_peripheral(port, pin) \
    port_pins_config_peripheral (port, BIT (pin))

/** Set selected pin of port high.  */
#define port_pin_set_high(port, pin) port_pins_set_high (port, BIT (pin))

/** Set selected pin of port low.  */
#define port_pin_set_low(port, pin) port_pins_set_low (port, BIT (pin))

/** Toggle selected pin of port.  */
#define port_pin_toggle(port, pin) port_pins_toggle (port, BIT (pin))

/** Get output state for selected pin of port.  */
#define port_pin_get(port, pin) (port_pins_get (port, BIT (pin)) != 0)

/** Read input state from selected pin of port.  */
#define port_pin_read(port, pin) (port_pins_read (port, BIT (pin)) != 0)

/** Write state of selected pin of port.
   This is equivalent to port_pin_set.  */
#define port_pin_write(port, pin, state)             \
    do                                               \
    {                                                \
        if (state)                                   \
            port_pin_set_high ((port), pin);         \
        else                                         \
            port_pin_set_low ((port), pin);          \
    }                                                \
    while (0)


/** Configure selected bus of port as output.  */
#define port_bus_config_output(port, pin1, pin2) \
    port_pins_config_output (port, PORT_BITS_MASK (pin1, pin2))

/** Configure selected bus of port as input without pullup.  */
#define port_bus_config_input(port, pin1, pin2) \
    port_pins_config_input (port, PORT_BITS_MASK (pin1, pin2))

/** Configure selected bus of port as input with pullup.  */
#define port_bus_config_pullup(port, pin) \
    port_pins_config_pullup (port, PORT_BITS_MASK (pin1, pin2))

/** Write value to selected bus of port.  */
#define port_bus_write(port, pin1, pin2, value)                          \
    port_write ((port), (port_get (port) & ~PORT_BITS_MASK (pin1, pin2)) \
              | (((value) & PORT_BITS_MASK (0, pin2 - pin1)) << (pin1)))

/** Read value from selected bus of port.  */
#define port_bus_read(port, pin1, pin2) \
    ((port_read (port) & PORT_BITS_MASK (pin1, pin2) >> (pin1))

/** Enable the clock for the port.  */
#define port_init(port) \
    AT91C_BASE_PMC->PMC_PCER = BIT (AT91C_ID_PIOA)

/** Disable the clock for the port.  */
#define port_shutdown(port) \
    AT91C_BASE_PMC->PMC_PCDR = BIT (AT91C_ID_PIOA)

#endif
