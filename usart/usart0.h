/** @file   usart0.h
    @author Michael Hayes
    @date   10 March 2005
    @brief Routines for interfacing with the usart0 on an AT91 ARM
*/

#ifndef USART0_H
#define USART0_H

#include "config.h"


#define USART0_BAUD_DIVISOR(BAUD_RATE)  ((F_CPU / 16) / (BAUD_RATE))


/* Return non-zero if there is a character ready to be read.  */
extern bool
usart0_read_ready_p (void);

/* Read character from USART0.  This blocks if nothing
   is available to read.  */
extern int8_t
usart0_getc (void);

/* Return non-zero if a character can be written without blocking.  */
extern bool
usart0_write_ready_p (void);

/* Return non-zero if transmitter finished.  */
extern bool
usart0_write_finished_p (void);

/* Write character to USART0.  This returns zero if
   the character could not be written.  */
extern int8_t
usart0_putc (char ch);

/* Write string to USART0.  */
extern void
usart0_puts (const char *str);

/* Initialise USART0 and set baud rate.  */
extern uint8_t
usart0_init (uint16_t baud_divisor);


/* Shutdown USART0 in preparation for sleep.  */
extern void
usart0_shutdown (void);

#endif
