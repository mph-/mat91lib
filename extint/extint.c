/** @file   extint.c
    @author M. P. Hayes
    @date   15 April 2013
    @brief  External interrupt handling.
*/

#include "extint.h"
#include "pio.h"
#include "cpu.h"
#include "irq.h"


struct extint_dev_struct
{
    pio_t pio;
    irq_id_t irq_id;
    pio_config_t periph;
};


static extint_dev_t extints[] =
{
    {
        .pio = PIO_DEFINE (PORT_A, 20),
        .irq_id = AT91C_ID_IRQ0,
        .periph = PIO_PERIPH_B_PULLUP,
    },
    {
        .pio = PIO_DEFINE (PORT_A, 30),
        .irq_id = AT91C_ID_IRQ1,
        .periph = PIO_PERIPH_A_PULLUP,
    }
};

#define EXTINT_NUM  ARRAY_SIZE (extints)


static void
extint_default_handler (void)
{
    /* Nothing to do.  */
}


void extint_enable (extint_t extint)
{
    irq_enable (extint->irq_id);
}


void extint_disable (extint_t extint)
{
    irq_disable (extint->irq_id);
}


void extint_sleep (extint_t extint)
{
    extint_enable (extint);
    
    /* Turn off main oscillator, PLL, and master clock, switch to slow
       clock, and sleep until get an external interrupt.  */
    cpu_sleep ();
    
    extint_disable (extint);
}


extint_t extint_init (const extint_cfg_t *cfg)
{
    unsigned int i;
    extint_dev_t *dev;

    for (i = 0; i < EXTINT_NUM; i++)
    {
        dev = &extints[i];
        
        if (dev->pio == cfg->pio)
        {
            void (*handler)(void);

            pio_config_set (dev->pio, dev->periph);

            handler = cfg->handler;
            if (!handler)
                handler = extint_default_handler;
            
            irq_config (dev->irq_id, 1,
                        AT91C_AIC_SRCTYPE_EXT_LOW_LEVEL, 
                        handler);

            return dev;
        }
    }
    return 0;
}


