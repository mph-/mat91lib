#include "pio.h"
#include "mcu.h"

/** Enable the clock for the port.  This is required for input
    operations.  */
void
pio_init (pio_t pio)
{
    mcu_pmc_enable (PIO_ID (pio));
}


/** Disable the clock for the port.  */
void
pio_shutdown (pio_t pio)
{
    mcu_pmc_disable (PIO_ID (pio));
}



