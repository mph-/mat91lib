/** @file   uart0.h
    @author Michael Hayes
    @date   27 May 2015
    @brief Routines for interfacing with the uart0 on an AT91 ARM
*/

#ifndef UART0_H
#define UART0_H

#include "config.h"


#define UART0_BAUD_DIVISOR(BAUD_RATE)  ((F_CPU / 16) / (BAUD_RATE))


/* Return non-zero if there is a character ready to be read.  */
bool
uart0_read_ready_p (void);

/* Read character from UART0.  This blocks if nothing
   is available to read.  */
int
uart0_getc (void);

/* Return non-zero if a character can be written without blocking.  */
bool
uart0_write_ready_p (void);

/* Return non-zero if transmitter finished.  */
bool
uart0_write_finished_p (void);

/* Write character to UART0.  This returns zero if
   the character could not be written.  */
int
uart0_putc (char ch);

/* Write string to UART0.  */
int
uart0_puts (const char *str);

/* Initialise UART0 and set baud rate.  */
int
uart0_init (uint16_t baud_divisor);


/* Shutdown UART0 in preparation for sleep.  */
void
uart0_shutdown (void);

#endif
