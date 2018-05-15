/** @file   buart1_isr.h
    @author M. P. Hayes, UCECE
    @date   18 June 2007
    @brief 
*/
#ifndef BUART1_ISR_H
#define BUART1_ISR_H

#ifdef __cplusplus
extern "C" {
#endif
    

#include "irq.h"
#include "uart1.h"
#include "uart1_defs.h"


#define UART1_TX_IRQ_ENABLED_P() ((UART1->UART_IMR & UART_SR_TXEMPTY) != 0)

#define UART1_TX_IRQ_DISABLE()  (UART1->UART_IDR = UART_SR_TXEMPTY)

#define UART1_TX_IRQ_ENABLE() (UART1->UART_IER = UART_SR_TXEMPTY)

#define UART1_RX_IRQ_DISABLE() (UART1->UART_IDR = UART_SR_RXRDY)

#define UART1_RX_IRQ_ENABLE() (UART1->UART_IER = UART_SR_RXRDY)


static buart_dev_t buart1_dev;


static void
buart1_tx_irq_enable (void)
{
    if (! UART1_TX_IRQ_ENABLED_P ())
        UART1_TX_IRQ_ENABLE ();
}


static void
buart1_rx_irq_enable (void)
{
    UART1_RX_IRQ_ENABLE ();
    UART1_READ ();
}


static bool
buart1_tx_finished_p (void)
{
    return UART1_WRITE_FINISHED_P ();
}


static void
buart1_isr (void)
{
    buart_dev_t *dev = &buart1_dev;
    uint32_t status;

    status = UART1->UART_SR;
    
    if (UART1_TX_IRQ_ENABLED_P () && ((status & UART_SR_TXRDY) != 0))
    {
        int ret;
        
        ret = ring_getc (&dev->tx_ring);        
        if (ret >= 0)
            UART1_WRITE (ret);
        else
            UART1_TX_IRQ_DISABLE ();
    }

    if ((status & UART_SR_RXRDY) != 0)
    {
        char ch;

        ch = UART1_READ ();

        /* What about buffer overflow?  */
	ring_putc (&dev->rx_ring, ch);
    }
}


static buart_dev_t *
buart1_init (uint16_t baud_divisor)
{
    buart_dev_t *dev = &buart1_dev;

    dev->tx_irq_enable = buart1_tx_irq_enable;
    dev->rx_irq_enable = buart1_rx_irq_enable;
    dev->tx_finished_p = buart1_tx_finished_p;

    uart1_init (baud_divisor);

    irq_config (ID_UART1, 1, buart1_isr);

    irq_enable (ID_UART1);

    return dev;
}


#ifdef __cplusplus
}
#endif    
#endif /* BUART1_ISR_H  */

