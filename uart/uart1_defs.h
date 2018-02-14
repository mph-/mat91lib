/** @file   uart1-defs.h
    @author M. P. Hayes, UCECE
    @date   27 May 2015
    @brief 
*/
#ifndef UART1_DEFS_H
#define UART1_DEFS_H

#ifdef __cplusplus
extern "C" {
#endif
    

/* You should be using the functions defined in uart1.h.  These
   macros are not for general use except in dire performance
   circumstances.  */

#include "config.h"


#define UART1_READ_READY_P() ((UART1->UART_SR & UART_SR_RXRDY) != 0)

#define UART1_WRITE_READY_P() ((UART1->UART_SR & UART_SR_TXRDY) != 0)

#define UART1_WRITE_FINISHED_P() ((UART1->UART_SR & UART_SR_TXEMPTY) != 0)

#define UART1_READ() UART1->UART_RHR

#define UART1_WRITE(VAL) UART1->UART_THR = (VAL)


#define UART1_PUTC(ch)                 \
do                                      \
{                                       \
    while (!UART1_WRITE_READY_P ())    \
        continue;                       \
                                        \
    UART1_WRITE (ch);                  \
} while (0)


/* The UART_BRGR register is 32-bit.  The 16-LSBs specify the baud rate
   divisor.  Bits 16--18 specify a fractional divisor.  We set these
   bits to zero to ignore the fractional part.  */
#define UART1_BAUD_DIVISOR_SET(DIVISOR)  UART1->UART_BRGR = (DIVISOR)


#define UART1_IRQ_CONFIG(ISR)          \
do                                      \
{                                       \
    UART1_RX_IRQ_ENABLE ();            \
    irq_config (AT91C_ID_UART1, 1,        \
                AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE, (ISR));  \
    irq_enable (AT91C_ID_UART1);          \
} while (0)



#ifdef __cplusplus
}
#endif    
#endif /* UART1_DEFS_H  */

