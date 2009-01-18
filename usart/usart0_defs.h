/** @file   usart0_defs.h
    @author M. P. Hayes, UCECE
    @date   15 May 2007
    @brief 
*/
#ifndef USART0_DEFS_H
#define USART0_DEFS_H

/* You should be using the functions defined in usart0.h.  These
   macros are not for general use except in dire performance
   circumstances.  */

#include "config.h"


#define USART0_READ_READY_P() ((AT91C_BASE_US0->US_CSR & AT91C_US_RXRDY) != 0)

#define USART0_WRITE_READY_P() ((AT91C_BASE_US0->US_CSR & AT91C_US_TXRDY) != 0)

#define USART0_WRITE_FINISHED_P() ((AT91C_BASE_US0->US_CSR & AT91C_US_TXEMPTY) != 0)

#define USART0_READ() AT91C_BASE_US0->US_RHR

#define USART0_WRITE(VAL) AT91C_BASE_US0->US_THR = (VAL)


#define USART0_PUTC(ch)                 \
do                                      \
{                                       \
    while (!USART0_WRITE_READY_P ())    \
        continue;                       \
                                        \
    USART0_WRITE (ch);                  \
} while (0)


/* The US_BRGR register is 32-bit.  The 16-LSBs specify the baud rate
   divisor.  Bits 16--18 specify a fractional divisor.  We set these
   bits to zero to ignore the fractional part.  */
#define USART0_BAUD_DIVISOR_SET(DIVISOR)  AT91C_BASE_US0->US_BRGR = (DIVISOR)


#endif /* USART0_DEFS_H  */
