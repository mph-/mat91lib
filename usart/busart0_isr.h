/** @file   busart0_isr.h
    @author M. P. Hayes, UCECE
    @date   18 June 2007
    @brief 
*/
#ifndef BUSART0_ISR_H
#define BUSART0_ISR_H

#ifdef __cplusplus
extern "C" {
#endif
    

#include "irq.h"
#include "usart0.h"
#include "usart0_defs.h"


#define USART0_TX_IRQ_ENABLED_P() ((USART0->US_IMR & US_CSR_TXEMPTY) != 0)

#define USART0_TX_IRQ_DISABLE()  (USART0->US_IDR = US_CSR_TXEMPTY)

#define USART0_TX_IRQ_ENABLE() (USART0->US_IER = US_CSR_TXEMPTY)

#define USART0_RX_IRQ_DISABLE() (USART0->US_IDR = US_CSR_RXRDY)

#define USART0_RX_IRQ_ENABLE() (USART0->US_IER = US_CSR_RXRDY)


static busart_dev_t busart0_dev;


static void
busart0_tx_irq_enable (void)
{
    if (! USART0_TX_IRQ_ENABLED_P ())
        USART0_TX_IRQ_ENABLE ();
}


static void
busart0_rx_irq_enable (void)
{
    USART0_RX_IRQ_ENABLE ();
}


static bool
busart0_tx_finished_p (void)
{
    return USART0_WRITE_FINISHED_P ();
}


static void
busart0_isr (void)
{
    busart_dev_t *dev = &busart0_dev;

    if (USART0_WRITE_READY_P ())
    {
        int ret;

        ret = ring_getc (&dev->tx_ring);        
        if (ret >= 0)
            USART0_WRITE (ret);
        else
            USART0_TX_IRQ_DISABLE ();
    }

    if (USART0_READ_READY_P ())
    {
	char ch;

	ch = USART0_READ ();

	/* What about buffer overflow?  */
	ring_putc (&dev->rx_ring, ch);
    }
}


static busart_dev_t *
busart0_init (uint16_t baud_divisor)
{
    busart_dev_t *dev = &busart0_dev;

    dev->tx_irq_enable = busart0_tx_irq_enable;
    dev->rx_irq_enable = busart0_rx_irq_enable;
    dev->tx_finished_p = busart0_tx_finished_p;

    usart0_init (baud_divisor);

    irq_config (ID_USART0, 1, busart0_isr);

    irq_enable (ID_USART0);

    return dev;
}


#ifdef __cplusplus
}
#endif    
#endif /* BUSART0_ISR_H  */

