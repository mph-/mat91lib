/** @file   usart1-defs.h
    @author M. P. Hayes, UCECE
    @date   15 May 2007
    @brief 
*/
#ifndef USART1_DEFS_H
#define USART1_DEFS_H

/* You should be using the functions defined in usart1.h.  These
   macros are not for general use except in dire performance
   circumstances.  */

#include "config.h"


#define USART1_READ_READY_P() ((AT91C_BASE_US1->US_CSR & AT91C_US_RXRDY) != 0)

#define USART1_WRITE_READY_P() ((AT91C_BASE_US1->US_CSR & AT91C_US_TXRDY) != 0)

#define USART1_WRITE_FINISHED_P() ((AT91C_BASE_US1->US_CSR & AT91C_US_TXEMPTY) != 0)

#define USART1_READ() AT91C_BASE_US1->US_RHR

#define USART1_WRITE(VAL) AT91C_BASE_US1->US_THR = (VAL)


#define USART1_PUTC(ch)                 \
do                                      \
{                                       \
    while (!USART1_WRITE_READY_P ())    \
        continue;                       \
                                        \
    USART1_WRITE (ch);                  \
} while (0)


/* The US_BRGR register is 32-bit.  The 16-LSBs specify the baud rate
   divisor.  Bits 16--18 specify a fractional divisor.  We set these
   bits to zero to ignore the fractional part.  */
#define USART1_BAUD_DIVISOR_SET(DIVISOR)  AT91C_BASE_US1->US_BRGR = (DIVISOR)


#define USART1_IRQ_CONFIG(ISR)          \
do                                      \
{                                       \
    USART1_RX_IRQ_ENABLE ();            \
    irq_config (AT91C_ID_US1, 1,        \
                AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE, (ISR));  \
    irq_enable (AT91C_ID_US1);          \
} while (0)


#endif /* USART1_DEFS_H  */
