/** @file   usart.h
    @author M. P. Hayes, UCECE
    @date   21 June 2007
    @brief  Unbuffered USART interface.
    @note   This driver is a wrapper for USARTx and provides a USART
   independent interface.

   This needs updating to be more general and to provide
   support for synchronous operation. 

   If code memory is at a premium, a USART can be disabled in the
   target.h file, e.g., using #define USART0_ENABLE 0. 

   Both USART0 and USART1 have CTS/RTS flow control pins:

   * RTS is an output from the receiver.  It is driven low when the receiver
   is ready to read.

   * CTS is an input to the transmitter.  The transmitter does not transmit
   until it goes low.

   By default hardware handshaking is not used for flow control.  This
   can be enabled by defining USART0_USE_HANDSHAKING or
   USART1_USE_HANDSHAKING in target.h  */
#ifndef USART_H
#define USART_H

#ifdef __cplusplus
extern "C" {
#endif
    

#include "sys.h"
#include "usart0.h"


/** USART configuration structure.  */
typedef struct
{
    /* 0 for USART0, 1 for USART1.  */
    uint8_t channel;
    /* Baud rate.  */
    uint32_t baud_rate;
    /* Baud rate divisor (this is used if baud_rate is zero).  */
    uint32_t baud_divisor;
    /* Zero for non-blocking I/O.  With blocking I/O, the functions do
       not return until all the I/O has been transferred.  This is not
       very efficient since there is a lot of busy-wait polling on a
       non-multitasked system.  With non-blocking I/O it is necessary
       to check the function returns in case the data was not
       transferred.  In this case the return value is -1 and errno
       is set to EAGAIN.  */
    uint32_t read_timeout_us;
    uint32_t write_timeout_us;
}
usart_cfg_t;

typedef struct usart_dev_struct usart_dev_t;

typedef usart_dev_t *usart_t;

#define USART_BAUD_DIVISOR(BAUD_RATE) USART0_BAUD_DIVISOR(BAUD_RATE)


/* Initialise UART for desired channel, baud rate, etc.  */
usart_t 
usart_init (const usart_cfg_t *cfg);


/** Return non-zero if there is a character ready to be read without blocking.  */
bool
usart_read_ready_p (usart_t usart);


/** Return non-zero if a character can be written without blocking.  */
bool
usart_write_ready_p (usart_t usart);


/** Read size bytes.  */
ssize_t
usart_read (void *usart, void *data, size_t size);


/** Write size bytes.  */
ssize_t
usart_write (void *usart, const void *data, size_t size);


/* Shutdown UART to save power.  */
void
usart_shutdown (usart_t usart);


/** Read character.  */
int
usart_getc (usart_t usart);


/** Write character.  */
int
usart_putc (usart_t usart, char ch);


/** Write string.  In non-blocking mode this is likely to 
    ignore all but the first character.  */
int
usart_puts (usart_t usart, const char *str);    
    

#ifdef __cplusplus
}
#endif    
#endif

