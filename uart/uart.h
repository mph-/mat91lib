/** @file   uart.h
    @author M. P. Hayes, UCECE
    @date   21 June 2007
    @brief  Unbuffered UART interface.
*/
#ifndef UART_H
#define UART_H

#include "config.h"
#include "uart0.h"

typedef struct uart_dev_struct uart_dev_t;

typedef uart_dev_t *uart_t;

/* UART configuration structure.  */
typedef struct uart_cfg_struct 
{
    uint8_t channel;
    uint16_t baud;
    uint8_t bits;
    uart_parity_t parity;
} uart_cfg_t;


/** Initialise UART for desired channel, baud rate, etc.  */
uart_t
uart_init (uart_cfg_t *uart_cfg);


/** Return non-zero if there is a character ready to be read.  */
bool
uart_read_ready_p (uart_t uart);


/** Return non-zero if a character can be written without blocking.  */
bool
uart_write_ready_p (uart_t uart);


/** Return non-zero if transmitter finished.  */
bool
uart_write_finished_p (uart_t uart)


/** Read character.  This blocks.  */
int8_t
uart_getc (uart_t uart);


/** Write character.  This blocks until character written to transmit
    buffer.  */
int8_t
uart_putc (uart_t uart, char ch);


/** Write string.  This blocks until last character written to
    transmit buffer.  */
int8_t
uart_puts (uart_t uart, const char *str);


/** Shutdown UART to save power.  */
void
uart_shutdown (uart_t uart);

#endif
