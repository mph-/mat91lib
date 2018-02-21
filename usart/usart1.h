/** @file   usart1.h
    @author Michael Hayes
    @date   10 March 2005

   @brief Routines for interfacing with the usart1 on an AT91 ARM
*/

#ifndef USART1_H
#define USART1_H

#ifdef __cplusplus
extern "C" {
#endif
    

#include "sys.h"


#define USART1_BAUD_DIVISOR(BAUD_RATE)  ((F_CPU / 16) / (BAUD_RATE))


/* Return non-zero if there is a character ready to be read.  */
bool
usart1_read_ready_p (void);

/* Read character from USART1.  This does not block.   */
int
usart1_getc (void);

/* Return non-zero if a character can be written without blocking.  */
bool
usart1_write_ready_p (void);

/* Return non-zero if transmitter finished.  */
bool
usart1_write_finished_p (void);

/* Write character to USART1.  This does not block.  */
int
usart1_putc (char ch);

/* Initialise USART1 and set baud rate.  */
int
usart1_init (uint16_t baud_divisor);


/* Shutdown USART1 in preparation for sleep.  */
void
usart1_shutdown (void);

#ifdef __cplusplus
}
#endif    
#endif

