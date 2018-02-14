/** @file   uart0_defs.h
    @author M. P. Hayes, UCECE
    @date   27 May 2015
    @brief 
*/
#ifndef UART0_DEFS_H
#define UART0_DEFS_H

#ifdef __cplusplus
extern "C" {
#endif
    

/* You should be using the functions defined in uart0.h.  These
   macros are not for general use except in dire performance
   circumstances.  */

#include "config.h"


#define UART0_READ_READY_P() ((UART0->UART_SR & UART_SR_RXRDY) != 0)

#define UART0_WRITE_READY_P() ((UART0->UART_SR & UART_SR_TXRDY) != 0)

#define UART0_WRITE_FINISHED_P() ((UART0->UART_SR & UART_SR_TXEMPTY) != 0)

#define UART0_READ() UART0->UART_RHR

#define UART0_WRITE(VAL) UART0->UART_THR = (VAL)


#define UART0_PUTC(ch)                 \
do                                      \
{                                       \
    while (!UART0_WRITE_READY_P ())    \
        continue;                       \
                                        \
    UART0_WRITE (ch);                  \
} while (0)


/* The UART_BRGR register is 32-bit.  The 16-LSBs specify the baud rate
   divisor.  Bits 16--18 specify a fractional divisor.  We set these
   bits to zero to ignore the fractional part.  */
#define UART0_BAUD_DIVISOR_SET(DIVISOR)  UART0->UART_BRGR = (DIVISOR)



#ifdef __cplusplus
}
#endif    
#endif /* UART0_DEFS_H  */

