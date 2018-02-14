/** @file   piobus.h
    @author M. P. Hayes, UCECE
    @date   2 Jun 2007
    @brief  PIOBUS abstraction for AT91 ARM7 microcontroller.
    @note   Macros and inline functions are used to avoid function
            call overhead and to allow compile-time constant folding. 
*/
#ifndef PIOBUS_H
#define PIOBUS_H

#ifdef __cplusplus
extern "C" {
#endif
    

#include "pio.h"

#ifdef PIOB
#error "TODO for SAM7X"
#endif


typedef uint64_t piobus_t;


#define PIOBUS_DEFINE(PORT, LSB, MSB) (((piobus_t)(LSB) << 32) | (BIT ((MSB) + 1) - BIT (LSB)))

/** Private macro to lookup bitmask.  */
#define PIOBUS_BITMASK_(PIOBUS) ((PIOBUS) & 0xffffffff)

/** Private macro to lookup shift.  */
#define PIOBUS_SHIFT_(PIOBUS) ((PIOBUS) >> 32)


/** Private macro to lookup port register.  */
#define PIOBUS_PORT_(PIOBUS) (PIOA)


/** Configure PIOBUS
    @param piobus  */
static inline
bool piobus_config_set (piobus_t piobus, pio_config_t config)
{
    switch (config)
    {
    case PIO_OUTPUT_HIGH:
        PIOBUS_PORT_ (piobus)->PIO_SODR = PIOBUS_BITMASK_ (piobus);
        PIOBUS_PORT_ (piobus)->PIO_PER = PIOBUS_BITMASK_ (piobus);
        PIOBUS_PORT_ (piobus)->PIO_OER = PIOBUS_BITMASK_ (piobus);
        PIOBUS_PORT_ (piobus)->PIO_PPUDR = PIOBUS_BITMASK_ (piobus);
        return 1;

    case PIO_OUTPUT_LOW:
        PIOBUS_PORT_ (piobus)->PIO_CODR = PIOBUS_BITMASK_ (piobus);
        PIOBUS_PORT_ (piobus)->PIO_PER = PIOBUS_BITMASK_ (piobus);
        PIOBUS_PORT_ (piobus)->PIO_OER = PIOBUS_BITMASK_ (piobus);
        PIOBUS_PORT_ (piobus)->PIO_PPUDR = PIOBUS_BITMASK_ (piobus);
        return 1;

    case PIO_INPUT:
        PIOBUS_PORT_ (piobus)->PIO_ODR = PIOBUS_BITMASK_ (piobus);
        PIOBUS_PORT_ (piobus)->PIO_PER = PIOBUS_BITMASK_ (piobus);
        PIOBUS_PORT_ (piobus)->PIO_PPUDR = PIOBUS_BITMASK_ (piobus);
        return 1;

    case PIO_PULLUP:
        PIOBUS_PORT_ (piobus)->PIO_ODR = PIOBUS_BITMASK_ (piobus);
        PIOBUS_PORT_ (piobus)->PIO_PER = PIOBUS_BITMASK_ (piobus);
        PIOBUS_PORT_ (piobus)->PIO_PPUER = PIOBUS_BITMASK_ (piobus);
        return 1;

    default:
        return 0;
    }
}


/** Set PIOBUS high.
    @param piobus  */
static inline
void piobus_output_high (piobus_t piobus)
{
    PIOBUS_PORT_ (piobus)->PIO_SODR = PIOBUS_BITMASK_ (piobus);
}


/** Set PIOBUS low.
    @param piobus  */
static inline
void piobus_output_low (piobus_t piobus)
{
    PIOBUS_PORT_ (piobus)->PIO_CODR = PIOBUS_BITMASK_ (piobus);
}


/** Set PIOBUS to desired state.
    @param piobus 
    @param state  */
static inline
void piobus_output_set (piobus_t piobus, uint32_t value)
{
    PIOBUS_PORT_ (piobus)->PIO_ODSR = (PIOBUS_PORT_ (piobus)->PIO_ODSR & ~PIOBUS_BITMASK_ (piobus)) | (value << PIOBUS_SHIFT_ (piobus));
}


/** Get output state of PIOBUS.
    @param piobus
    @return state  */
static inline
bool piobus_output_get (piobus_t piobus)
{
    return (PIOBUS_PORT_ (piobus)->PIO_ODSR & PIOBUS_BITMASK_ (piobus)) >> PIOBUS_SHIFT_ (piobus);
}


/** Read input state of PIOBUS.
    @param piobus
    @return state  */
static __always_inline__
uint32_t piobus_input_get (piobus_t piobus)
{
    return (PIOBUS_PORT_ (piobus)->PIO_PDSR & PIOBUS_BITMASK_ (piobus)) >> PIOBUS_SHIFT_ (piobus);
}




#ifdef __cplusplus
}
#endif    
#endif

