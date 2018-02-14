/** @file   usart0_defs.h
    @author M. P. Hayes, UCECE
    @date   15 May 2007
    @brief 
*/
#ifndef USART0_DEFS_H
#define USART0_DEFS_H

#ifdef __cplusplus
extern "C" {
#endif
    

/* You should be using the functions defined in usart0.h.  These
   macros are not for general use except in dire performance
   circumstances.  */

#include "config.h"


#define USART0_READ_READY_P() ((USART0->US_CSR & US_CSR_RXRDY) != 0)

#define USART0_WRITE_READY_P() ((USART0->US_CSR & US_CSR_TXRDY) != 0)

#define USART0_WRITE_FINISHED_P() ((USART0->US_CSR & US_CSR_TXEMPTY) != 0)

#define USART0_READ() USART0->US_RHR

#define USART0_WRITE(VAL) USART0->US_THR = (VAL)


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
#define USART0_BAUD_DIVISOR_SET(DIVISOR)  USART0->US_BRGR = (DIVISOR)



#ifdef __cplusplus
}
#endif    
#endif /* USART0_DEFS_H  */

