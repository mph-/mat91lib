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
bool
usart0_read_ready_p (void);

/* Read character from USART0.  This blocks if nothing
   is available to read.  */
int
usart0_getc (void);

/* Return non-zero if a character can be written without blocking.  */
bool
usart0_write_ready_p (void);

/* Return non-zero if transmitter finished.  */
bool
usart0_write_finished_p (void);

/* Write character to USART0.  This returns zero if
   the character could not be written.  */
int
usart0_putc (char ch);

/* Write string to USART0.  */
int
usart0_puts (const char *str);

/* Initialise USART0 and set baud rate.  */
uint
usart0_init (uint16_t baud_divisor);


/* Shutdown USART0 in preparation for sleep.  */
void
usart0_shutdown (void);

#endif
