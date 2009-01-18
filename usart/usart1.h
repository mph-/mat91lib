/** @file   usart1.h
    @author Michael Hayes
    @date   10 March 2005

   @brief Routines for interfacing with the usart1 on an AT91 ARM
*/

#ifndef USART1_H
#define USART1_H

#include "config.h"


#define USART1_BAUD_DIVISOR(BAUD_RATE)  ((F_CPU / 16) / (BAUD_RATE))


/* Return non-zero if there is a character ready to be read.  */
extern bool
usart1_read_ready_p (void);

/* Read character from USART1.  This blocks if nothing
   is available to read.  */
extern int8_t 
usart1_getc (void);

/* Return non-zero if a character can be written without blocking.  */
extern bool
usart1_write_ready_p (void);

/* Return non-zero if transmitter finished.  */
extern bool
usart1_write_finished_p (void);

/* Write character to USART1.  This returns zero if
   the character could not be written.  */
extern int8_t
usart1_putc (char ch);

/* Write string to USART1.  */
extern void 
usart1_puts (const char *str);

/* Initialise usart1 and set baud rate.  */
extern uint8_t
usart1_init (uint16_t baud_divisor);

#endif
