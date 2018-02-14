/** @file   usart0_irda.h
    @author M. P. Hayes, UCECE
    @date   18 June 2007
    @brief 
*/
#ifndef USART0_IRDA_H
#define USART0_IRDA_H

#ifdef __cplusplus
extern "C" {
#endif
    

/* Initialise USART for IRDA mode.  */
extern void
usart0_irda_init (void);

/* Switch half-duplex IRDA to transmit mode.  */
extern void
usart0_irda_transmit (void);

/* Switch half-duplex IRDA to receive mode.  */
extern void
usart0_irda_receive (void);


#ifdef __cplusplus
}
#endif    
#endif /* USART0_IRDA_H  */

