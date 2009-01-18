/** @file   busart0_isr.h
    @author M. P. Hayes, UCECE
    @date   18 June 2007
    @brief 
*/
#ifndef BUSART0_ISR_H
#define BUSART0_ISR_H

#include "irq.h"
#include "usart0.h"
#include "usart0_defs.h"


#define USART0_TX_IRQ_ENABLED_P() ((AT91C_BASE_US0->US_IMR & AT91C_US_TXEMPTY) != 0)

#define USART0_TX_IRQ_DISABLE()  (AT91C_BASE_US0->US_IDR = AT91C_US_TXEMPTY)

#define USART0_TX_IRQ_ENABLE() (AT91C_BASE_US0->US_IER = AT91C_US_TXEMPTY)

#define USART0_RX_IRQ_DISABLE() (AT91C_BASE_US0->US_IDR = AT91C_US_RXRDY)

#define USART0_RX_IRQ_ENABLE() (AT91C_BASE_US0->US_IER = AT91C_US_RXRDY)


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
        char ch;

        ch = ring_getc (&dev->tx_ring);        
        if (ch)
            USART0_WRITE (ch);
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

    irq_config (AT91C_ID_US0, 1,
                AT91C_AIC_SRCTYPE_INT_LEVEL_SENSITIVE, busart0_isr);

    irq_enable (AT91C_ID_US0);

    return dev;
}

#endif /* BUSART0_ISR_H  */
