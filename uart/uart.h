/** @file   uart.h
    @author M. P. Hayes, UCECE
    @date   21 June 2007
    @brief  Unbuffered UART interface.
*/
#ifndef UART_H
#define UART_H

#include "config.h"
#include "uart0.h"

/** uart configuration structure.  */
typedef struct
{
    /* 0 for UART0, 1 for UART1.  */
    uint8_t channel;
    /* Baud rate.  */
    uint32_t baud_rate;
    /* Baud rate divisor (this is used if baud_rate is zero).  */
    uint32_t baud_divisor;
    /* Non-zero for blocking I/O.  */
    bool block;
}
uart_cfg_t;

typedef struct uart_dev_struct uart_dev_t;

typedef uart_dev_t *uart_t;

#define UART_BAUD_DIVISOR(BAUD_RATE) UART0_BAUD_DIVISOR(BAUD_RATE)


/* Initialise UART for desired channel, baud rate, etc.  */
uart_t 
uart_init (const uart_cfg_t *cfg);


/** Return non-zero if there is a character ready to be read without blocking.  */
bool
uart_read_ready_p (uart_t uart);


/** Return non-zero if a character can be written without blocking.  */
bool
uart_write_ready_p (uart_t uart);


/** Read character.  This blocks.  */
int
uart_getc (uart_t uart);


/** Write character.  This blocks until character written to transmit buffer.  */
int
uart_putc (uart_t uart, char ch);


/** Write string.  This blocks until last character written to transmit buffer.  */
int
uart_puts (uart_t uart, const char *str);


/* Shutdown UART to save power.  */
void
uart_shutdown (uart_t uart);

#endif
