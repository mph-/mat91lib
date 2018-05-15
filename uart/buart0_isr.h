/** @file   buart0_isr.h
    @author M. P. Hayes, UCECE
    @date   18 June 2007
    @brief 
*/
#ifndef BUART0_ISR_H
#define BUART0_ISR_H

#ifdef __cplusplus
extern "C" {
#endif
    

#include "irq.h"
#include "uart0.h"
#include "uart0_defs.h"


#define UART0_TX_IRQ_ENABLED_P() ((UART0->UART_IMR & UART_SR_TXEMPTY) != 0)

#define UART0_TX_IRQ_DISABLE()  (UART0->UART_IDR = UART_SR_TXEMPTY)

#define UART0_TX_IRQ_ENABLE() (UART0->UART_IER = UART_SR_TXEMPTY)

#define UART0_RX_IRQ_DISABLE() (UART0->UART_IDR = UART_SR_RXRDY)

#define UART0_RX_IRQ_ENABLE() (UART0->UART_IER = UART_SR_RXRDY)


static buart_dev_t buart0_dev;


static void
buart0_tx_irq_enable (void)
{
    if (! UART0_TX_IRQ_ENABLED_P ())
        UART0_TX_IRQ_ENABLE ();
}


static void
buart0_rx_irq_enable (void)
{
    UART0_RX_IRQ_ENABLE ();
}


static bool
buart0_tx_finished_p (void)
{
    return UART0_WRITE_FINISHED_P ();
}


static void
buart0_isr (void)
{
    buart_dev_t *dev = &buart0_dev;    
    uint32_t status;

    status = UART0->UART_SR;
    
    if (UART0_TX_IRQ_ENABLED_P () && ((status & UART_SR_TXRDY) != 0))
    {
        int ret;
        
        ret = ring_getc (&dev->tx_ring);        
        if (ret >= 0)
            UART0_WRITE (ret);
        else
            UART0_TX_IRQ_DISABLE ();
    }

    if ((status & UART_SR_RXRDY) != 0)
    {
        char ch;

        ch = UART0_READ ();

        /* What about buffer overflow?  */
	ring_putc (&dev->rx_ring, ch);
    }
}


static buart_dev_t *
buart0_init (uint16_t baud_divisor)
{
    buart_dev_t *dev = &buart0_dev;

    dev->tx_irq_enable = buart0_tx_irq_enable;
    dev->rx_irq_enable = buart0_rx_irq_enable;
    dev->tx_finished_p = buart0_tx_finished_p;

    uart0_init (baud_divisor);

    irq_config (ID_UART0, 1, buart0_isr);

    irq_enable (ID_UART0);

    return dev;
}


#ifdef __cplusplus
}
#endif    
#endif /* BUART0_ISR_H  */

