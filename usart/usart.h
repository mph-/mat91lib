/** @file   usart.h
    @author M. P. Hayes, UCECE
    @date   21 June 2007
    @brief  Unbuffered USART interface.
*/
#ifndef USART_H
#define USART_H

#include "config.h"
#include "usart0.h"

typedef struct usart_dev_struct usart_dev_t;

typedef usart_dev_t *usart_t;

#define USART_BAUD_DIVISOR(BAUD_RATE) USART0_BAUD_DIVISOR(BAUD_RATE)


/* Initialise UART for desired channel, baud rate, etc.  */
usart_t
usart_init (uint8_t channel,
            uint16_t baud_divisor);


/** Return non-zero if there is a character ready to be read without blocking.  */
bool
usart_read_ready_p (usart_t usart);


/** Return non-zero if a character can be written without blocking.  */
bool
usart_write_ready_p (usart_t usart);


/** Read character.  This blocks.  */
int8_t
usart_getc (usart_t usart);


/** Write character.  This blocks until character written to transmit buffer.  */
int8_t
usart_putc (usart_t usart, char ch);


/** Write string.  This blocks until last character written to transmit buffer.  */
int8_t
usart_puts (usart_t usart, const char *str);


/* Shutdown UART to save power.  */
void
usart_shutdown (usart_t usart);

#endif
