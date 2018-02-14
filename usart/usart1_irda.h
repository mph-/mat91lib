/** @file   usart1_irda.h
    @author M. P. Hayes, UCECE
    @date   18 June 2007
    @brief 
*/
#ifndef USART1_IRDA_H
#define USART1_IRDA_H

#ifdef __cplusplus
extern "C" {
#endif
    

/* Initialise USART for IRDA mode.  */
extern void
usart1_irda_init (void);

/* Switch half-duplex IRDA to transmit mode.  */
extern void
usart1_irda_transmit (void);

/* Switch half-duplex IRDA to receive mode.  */
extern void
usart1_irda_receive (void);


#ifdef __cplusplus
}
#endif    
#endif /* USART1_IRDA_H  */

