/** @file   busart1_isr.h
    @author M. P. Hayes, UCECE
    @date   18 June 2007
    @brief 
*/
#ifndef BUSART1_ISR_H
#define BUSART1_ISR_H

#include "irq.h"
#include "usart1.h"
#include "usart1_defs.h"


#define USART1_TX_IRQ_ENABLED_P() ((AT91C_BASE_US1->US_IMR & AT91C_US_TXEMPTY) != 0)

#define USART1_TX_IRQ_DISABLE()  (AT91C_BASE_US1->US_IDR = AT91C_US_TXEMPTY)

#define USART1_TX_IRQ_ENABLE() (AT91C_BASE_US1->US_IER = AT91C_US_TXEMPTY)

#define USART1_RX_IRQ_DISABLE() (AT91C_BASE_US1->US_IDR = AT91C_US_RXRDY)

#define USART1_RX_IRQ_ENABLE() (AT91C_BASE_US1->US_IER = AT91C_US_RXRDY)


static busart_dev_t busart1_dev;


static void
busart1_tx_irq_enable (void)
{
    if (! USART1_TX_IRQ_ENABLED_P ())
        USART1_TX_IRQ_ENABLE ();
}


static void
busart1_rx_irq_enable (void)
{
    USART1_RX_IRQ_ENABLE ();
    USART1_READ ();
}


static bool
busart1_tx_finished_p (void)
{
    return USART1_WRITE_FINISHED_P ();
}


static void
busart1_isr (void)
{
    busart_dev_t *dev = &busart1_dev;

    if (USART1_WRITE_READY_P ())
    {
        char ch;
        
        ch = ring_getc (&dev->tx_ring);        
        if (ch)
            USART1_WRITE (ch);
        else
            USART1_TX_IRQ_DISABLE ();
    }

    if (USART1_READ_READY_P ())
    {
        char ch;

        ch = USART1_READ ();

        /* What about buffer overflow?  */
	ring_putc (&dev->rx_ring, ch);
    }
}


static busart_dev_t *
busart1_init (uint16_t baud_divisor)
{
    busart_dev_t *dev = &busart1_dev;

    dev->tx_irq_enable = busart1_tx_irq_enable;
    dev->rx_irq_enable = busart1_rx_irq_enable;
    dev->tx_finished_p = busart1_tx_finished_p;

    usart1_init (baud_divisor);

    irq_config (AT91C_ID_US1, 1,
                AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL, busart1_isr);

    irq_enable (AT91C_ID_US1);

    return dev;
}

#endif /* BUSART1_ISR_H  */
