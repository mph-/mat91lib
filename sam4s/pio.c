/**
 * @file irq.c
 * @brief PIO interrupt input
 * @author Ran Bao (rba90@uclive.ac.nz)
 * @date 21 April 2017
 */

#include <stdint.h>
#include "irq.h"
#include "pio.h"

/**
 * Interrupt vector table for each PIO pins
 * According to datasheet, there are 32 pins on each port groups, there are three port groups in
 * total. In sam4s8b, there are 32 pins in Port A and 15 pins in Port B
 */
#warning "Make sure you are using sam4s8b, otherwise you will need to extent pio interrupt vector table"
static irq_handler_t pio_interrupt_vector_table[PB15_PIO - PA0_PIO];

/**
 * Internal symbols that indicates whether interrupt vector handler has been added to the dynamic
 * interrupt vector table (exception_table in crt0.c)
 */
static bool irq_port_registered_a = false;
static bool irq_port_registered_b = false;
static bool irq_port_registered_c = false;

/**
 * Dynamic Vector Table, defined in crt0.c, can be modified at run time
 */
extern irq_handler_t exception_table[];

/**
 * Handle interrupt on specific port groups. PIO IRQs are served and cleared in this function
 * @param port PORT_A or PORT_B or PORT_C
 */
static void pio_irq_common_handler(uint8_t port)
{
	// find base
	Pio *base = PIO_BASE(port);

	// look for those pins with both interrupt is enabled and interrupt stats has changed
	// NOTE: ISR register is cleared after reading
	uint32_t pio_activated = base->PIO_IMR & base->PIO_ISR;

	// to through each bit
	for (uint8_t bit = (uint8_t) IRQ_ID_MIN; bit <= (uint8_t) IRQ_ID_MAX; bit++, pio_activated >>= 1)
	{
		if (pio_activated & 0x1)
		{
			// execute registered interrupt handler
			pio_t pin = PIO_DEFINE(port, bit);
			if (pio_interrupt_vector_table[pin])
			{
				pio_interrupt_vector_table[pin]();
			}
		}
	}
}

static void pio_irq_handler_a(void)
{
	pio_irq_common_handler(PORT_A);
}

static void pio_irq_handler_b(void)
{
	pio_irq_common_handler(PORT_B);
}

static void pio_irq_handler_c(void)
{
	pio_irq_common_handler(PORT_C);
}

static void irq_port_enable(irq_id_t port_id)
{
	// register interrupt handler in dynamic vector table
	switch(port_id)
	{
		case ID_PIOA:
			if (!irq_port_registered_a)
			{
				// disable NVIC interrupts on port groups
				NVIC_DisableIRQ((IRQn_Type) ID_PIOA);

				// insert corresponding handler into exception table
				exception_table[16 + ID_PIOA] = pio_irq_handler_a;

				// clear existing interrupt signal
				NVIC_ClearPendingIRQ((IRQn_Type) ID_PIOA);

				// enable interrupt on port
				NVIC_EnableIRQ((IRQn_Type) ID_PIOA);

				// enable clock signal on specific port
				mcu_pmc_enable(ID_PIOA);

				irq_port_registered_a = true;
			}
			break;
		case ID_PIOB:
			if (!irq_port_registered_b)
			{
				// disable NVIC interrupts on port groups
				NVIC_DisableIRQ((IRQn_Type) ID_PIOB);

				// insert corresponding handler into exception table
				exception_table[16 + ID_PIOB] = pio_irq_handler_b;

				// clear existing interrupt signal
				NVIC_ClearPendingIRQ((IRQn_Type) ID_PIOB);

				// enable interrupt on port
				NVIC_EnableIRQ((IRQn_Type) ID_PIOB);

				// enable clock signal on specific port
				mcu_pmc_enable(ID_PIOB);

				irq_port_registered_b = true;
			}
			break;
		case ID_PIOC:
			if (!irq_port_registered_c)
			{
				// disable NVIC interrupts on port groups
				NVIC_DisableIRQ((IRQn_Type) ID_PIOC);

				// insert corresponding handler into exception table
				exception_table[16 + ID_PIOC] = pio_irq_handler_c;

				// clear existing interrupt signal
				NVIC_ClearPendingIRQ((IRQn_Type) ID_PIOC);

				// enable interrupt on port
				NVIC_EnableIRQ((IRQn_Type) ID_PIOC);

				// enable clock signal on specific port
				mcu_pmc_enable(ID_PIOC);

				irq_port_registered_c = true;
			}
	}
}

/**
 * Initialize PIO pin as interrupt input
 * @param pio PIO pin
 * @param config trigger condition of interrupt
 * @param isr interrupt service routine (interrupt handler) on specific pin
 */
void
pio_irq_init(pio_t pio, pio_irq_config_t config, irq_handler_t isr)
{
	// get port id from pin address
	irq_id_t id = PIO_ID(pio);

	// assign interrupt function
	pio_interrupt_vector_table[pio] = isr;

	// config pin as input
	pio_config_set(pio, PIO_INPUT);

	// config pin as interrupt
	pio_irq_config_set(pio, config);

	// enable port irq
	irq_port_enable(id);

	// enable pin irq
	pio_irq_enable(pio);
}
