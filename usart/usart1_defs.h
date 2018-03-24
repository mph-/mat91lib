/** @file   usart1-defs.h
    @author M. P. Hayes, UCECE
    @date   15 May 2007
    @brief 
*/
#ifndef USART1_DEFS_H
#define USART1_DEFS_H

#ifdef __cplusplus
extern "C" {
#endif
    

/* You should be using the functions defined in usart1.h.  These
   macros are not for general use except in dire performance
   circumstances.  */

#include "config.h"


#define USART1_READ_READY_P() ((USART1->US_CSR & US_CSR_RXRDY) != 0)

#define USART1_WRITE_READY_P() ((USART1->US_CSR & US_CSR_TXRDY) != 0)

#define USART1_WRITE_FINISHED_P() ((USART1->US_CSR & US_CSR_TXEMPTY) != 0)

#define USART1_READ() USART1->US_RHR

#define USART1_WRITE(VAL) USART1->US_THR = (VAL)


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
#define USART1_BAUD_DIVISOR_SET(DIVISOR)  USART1->US_BRGR = (DIVISOR)


#ifdef __cplusplus
}
#endif    
#endif /* USART1_DEFS_H  */

