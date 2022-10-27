/** @file   spi.c
    @author Michael Hayes
    @date   30 July 2007
    @brief  Routines for interfacing to the SPI bus.
*/

#include <errno.h>
#include "spi.h"
#include "mcu.h"
#include "cpu.h"
#include "pinmap.h"
#include "bits.h"


/* This driver only configures the SPI controller as a master.

   The AT91 SPI peripheral can transmit either 8 or 16 bit data with
   MSB first.  It has 4 separate control status registers and can control
   4 different types of SPI peripheral using the separate chip select
   registers.  Each peripheral can have its own chip select that is
   automatically driven for each transfer.

   The AT91 SPI peripheral has been designed so that blocks of a data
   can be streamed using DMA to multiple devices on the bus (Variable
   Peripheral Selection).  It achieves this by using 32 bits to
   specify the 8 or 16 bit data plus a bit mask specifying which
   peripheral to write to.

   There are 4 control status registers (CSRs); one associated with
   each of the four SPI chip select (CS) signals.  These configure the
   number of bits per transfer and the CS timing for each of the 4
   channels.  The SPI controller chooses one of these 4 CSRs on the
   basis of the channel select mask in the mode register (fixed
   peripheral mode) or in the data (variable peripheral mode).  Now
   this is all fine and dandy if we have 4 SPI devices.  What if we
   want more?  Well one option is to decode the 4 CS signals to
   provide 15 channels; with four per group.  This requires external
   hardware so is not much use.  The alternative is to bit bash
   additional output ports.

   With this driver we can share the 4 channels among multiple SPI
   devices.  However, automatic CS driving can only be performed for
   the few NPCS signals associated with a channel, otherwise the CS
   signals are bit-bashed.  If an NPCS pin is configured as an SPI
   peripheral it will be driven whenever its channel is used.  Thus if
   we want to have other CS signals driven by PIO lines, we need to
   switch the NPCS signals at the start and end of a transfer.

   There are four chip select modes:
   SPI_CS_MODE_FRAME where the CS is asserted for multiple SPI tranmissions,
   SPI_CS_MODE_TOGGLE where the CS is only asserted for each SPI transmission,
   SPI_CS_MODE_HIGH where the CS is kept high,
   SPI_CS_MODE_EXTERNAL where the CS is driven externally, say by a TC peripheral.

   Automatic CS driving is only used for SPI_CS_MODE_TOGGLE.  With
   SPI_CS_MODE_FRAME it is necessary to change the CSAAT bit for the
   ultimate transfer.  The additional fiddling around is not
   worthwhile.  There is only a marginal benefit using automatic chip
   select assertion.  It is primarily designed for use with DMA.

   Note, functions that configure the SPI peripheral (such as
   spi_bits_set) only take effect when spi_config is called (usually
   when some I/O is to be performed).
*/


/* Number of independent SPI controllers.  */
#ifdef SPI1
#define SPI_CONTROLLERS_NUM 2
#else
#define SPI_CONTROLLERS_NUM 1
#endif


/* Currently the maximum of SPI devices is 32 due to the size
   of the mask used for recording active devices.  */
#ifndef SPI_DEVICES_NUM
#define SPI_DEVICES_NUM 8
#endif


#ifndef HOSTED
#define HOSTED 0
#endif


/* Bit masks for the SPI mode settings.  */
enum
{
    SPI_CPOL_MASK = BIT (0),
    SPI_NCPHA_MASK = BIT (1),
    SPI_CSAAT_MASK = BIT (3)
};


/* Macros.  */

#ifndef SPI0
#define SPI0 SPI
#endif

#ifndef SPI1
#define SPI1 0
#endif

#define SPI_BASE_GET(channel) (((channel) < SPI_CHANNELS_NUM) ? SPI0 : SPI1)


#ifdef HOSTED
#define SPI_READY_P(BASE) (HOSTED || ((BASE)->SPI_SR & SPI_SR_RDRF))
#else
#define SPI_READY_P(BASE) ((BASE)->SPI_SR & SPI_SR_RDRF)
#endif

#define SPI_TXEMPTY_P(BASE) ((BASE)->SPI_SR & SPI_SR_TDRE)

/* Set lastxfer bit for use in fixed mode.  */
#define SPI_LASTXFER(BASE) ((BASE)->SPI_CR = SPI_CR_LASTXFER)


/* Read SPI and then send new data.  Blocks while the new data is
   being sent by the spi.  */
#define SPI_XFER(pSPI, txdata, rxdata)                                  \
    do                                                                  \
    {                                                                   \
        /* Dummy read from RDR to ensure RDRF clear.   */               \
        (rxdata) = pSPI->SPI_RDR;                                       \
                                                                        \
        /* Write data to TDR.  */                                       \
        pSPI->SPI_TDR = (txdata);                                       \
                                                                        \
        /* Wait until spi port finishes transmitting/receiving.  */     \
        while (!SPI_READY_P (pSPI))                                     \
            continue;                                                   \
                                                                        \
        /* Read new data from RDR (this clears RDRF).  */               \
        (rxdata) = pSPI->SPI_RDR;                                       \
                                                                        \
    } while (0)


static uint8_t spi_devices_num = 0;
static spi_dev_t spi_devices[SPI_DEVICES_NUM];
static spi_dev_t *spi_config_last = 0;
static uint32_t spi_devices_enabled = 0;


static inline uint32_t
spi_channel_csr_get (spi_t spi)
{
    return spi->base->SPI_CSR[spi->channel & (SPI_CHANNELS_NUM - 1)];
}


static inline void
spi_channel_csr_set (spi_t spi, uint32_t csr)
{
    spi->base->SPI_CSR[spi->channel & (SPI_CHANNELS_NUM - 1)] = csr;
}


/** Set the delay (in clocks * 32) for automatic deassertion of chip select.  */
static void
spi_channel_cs_hold_set (spi_t spi, uint16_t delay)
{
    uint32_t csr;

    csr = spi_channel_csr_get (spi);

    /* Set the DLYBCT (delay between consecutive transfers).  This is
       the only mechanism for automatically delaying the automatic
       deassertion of the chip select.  */
    BITS_INSERT (csr, delay, 24, 31);

    spi_channel_csr_set (spi, csr);
}


/** Set the delay (in clocks) to when the shift clock starts from when
    the transmit data register is written.  */
static void
spi_channel_cs_setup_set (spi_t spi, uint16_t delay)
{
    uint32_t csr;

    csr = spi_channel_csr_get (spi);

    /* Set the DLYBS (delay before SPCK).  */
    BITS_INSERT (csr, delay, 16, 23);

    spi_channel_csr_set (spi, csr);
}


/** The minimum divisor value is 1, this gives the maximum rate of MCK.  */
static void
spi_channel_clock_divisor_set (spi_t spi, spi_clock_divisor_t clock_divisor)
{
    uint32_t csr;

    csr = spi_channel_csr_get (spi);

    BITS_INSERT (csr, clock_divisor, 8, 15);

    spi_channel_csr_set (spi, csr);
}


/* Set number of bits in transfer 8--16.  There is a silicon bug
   (39.2.4.5) where the number of bits cannot be odd if the SPI
   divisor is 1.  */
static void
spi_channel_bits_set (spi_t spi, uint8_t bits)
{
    uint32_t csr;

    csr = spi_channel_csr_get (spi);

    BITS_INSERT (csr, bits - 8, 4, 7);

    spi_channel_csr_set (spi, csr);
}


/* Spi modes:

CPOL defines the clock idle state    (0->L, 1->H)
CPHA defines the clock sampling edge (0->rising, 1->falling)

Mode 	CPOL 	CPHA  NCPHA   Clock idle state  MISO/MOSI sample
0 	0 	0     1       low               rising edge
1 	0 	1     0       low               falling edge
2 	1 	0     1       high              rising edge
3 	1 	1     0       high              falling edge

However, page 512 of the AT91SAM7Sx datasheet say "Note that in SPI
master mode the ATSAM7S512/256/128/64/321/32 does not sample the data
(MISO) on the opposite edge where data clocks out (MOSI) but the same
edge is used as shown in Figure 36-3 and Figure 36-4."  Figure 36-3
shows that CPOL=NCPHA=0 or CPOL=NCPHA=1 samples on the rising edge and
that the data changes sometime after the rising edge (about 2 ns).  To
be consistent with normal SPI operation, it is probably safe to say
that the data changes on the falling edge and should be sampled on the
rising edge.
*/

static void
spi_channel_mode_set (spi_t spi, spi_mode_t mode)
{
    uint32_t csr;

    csr = spi_channel_csr_get (spi);

    csr &= ~(SPI_CPOL_MASK | SPI_NCPHA_MASK);

    switch (mode)
    {
    case SPI_MODE_0:
        /* CPOL = 0, CPHA = 0.  */
        csr |= SPI_NCPHA_MASK;
        break;

    case SPI_MODE_1:
        /* CPOL = 0, CPHA = 1.  */
        break;

    case SPI_MODE_2:
        /* CPOL = 1, CPHA = 0.  */
        csr |= SPI_CPOL_MASK;
        csr |= SPI_NCPHA_MASK;
        break;

    case SPI_MODE_3:
        /* CPOL = 1, CPHA = 1.  */
        csr |= SPI_CPOL_MASK;
        break;
    }

    spi_channel_csr_set (spi, csr);
}


/* NB, if you get `error: initializer element is not constant' then
   need to compile without -std=c99 or -std=gnu99  */

#if SPI_CONTROLLERS_NUM == 2
/* AT91SAM7X  */
static const pinmap_t spi_cs[] =
{
    {0, PA21_PIO, PIO_PERIPH_B, 0},
    {1, PA25_PIO, PIO_PERIPH_B, 0},
    {2, PA26_PIO, PIO_PERIPH_B, 0},
    {3, PA29_PIO, PIO_PERIPH_B, 0},
    {1, PA10_PIO, PIO_PERIPH_B, 0},
    {2, PA11_PIO, PIO_PERIPH_B, 0},
    {3, PA16_PIO, PIO_PERIPH_B, 0},
};

#else
/* AT91SAM7S  */
static const pinmap_t spi_cs[] =
{
    {0, PA11_PIO, PIO_PERIPH_A, 0},
    {1, PA9_PIO, PIO_PERIPH_B, 0},
    {1, PA31_PIO, PIO_PERIPH_A, 0},
    {2, PA10_PIO, PIO_PERIPH_B, 0},
    {2, PA30_PIO, PIO_PERIPH_B, 0},
    {3, PA3_PIO, PIO_PERIPH_B, 0},
    {3, PA5_PIO, PIO_PERIPH_B, 0},
    {3, PA22_PIO, PIO_PERIPH_B, 0}
};

#endif


#define SPI_CS_NUM ARRAY_SIZE (spi_cs)


static pio_config_t
spi_channel_cs_config_get (spi_t spi, pio_t cs)
{
    unsigned int i;

    for (i = 0; i < SPI_CS_NUM; i++)
    {
        if (spi->channel == spi_cs[i].channel
            && cs == spi_cs[i].pio)
        {
            return spi_cs[i].periph;
        }
    }
    return PIO_OUTPUT_HIGH;
}


static void
spi_update (spi_t spi)
{
    /* Need to force an update of the config.  */
    if (spi == spi_config_last)
        spi_config_last = 0;
}


void
spi_clock_divisor_set (spi_t spi, spi_clock_divisor_t clock_divisor)
{
    if (clock_divisor == 0)
        clock_divisor = 1;
    spi->clock_divisor = clock_divisor;
    spi_update (spi);
}


spi_clock_speed_t
spi_clock_speed_kHz_set (spi_t spi, spi_clock_speed_t clock_speed_kHz)
{
    uint32_t clock_speed;
    uint32_t divisor;

    clock_speed = clock_speed_kHz * 1000;

    /* Calculate the appropriate clock divisor. This must be in the range 1 to
     * 255 inclusive. Adding the speed and subtracting 1 rounds up when used
     * with integer division.  */
    divisor = (F_CPU_UL + clock_speed - 1) / clock_speed;
    if (divisor > 255)
        divisor = 255;
    else if (divisor == 0)
        divisor = 1;

    /* Set the divisor and return the actual clock speed. */
    spi_clock_divisor_set (spi, (spi_clock_divisor_t)divisor);
    clock_speed = F_CPU / spi->clock_divisor;
    return clock_speed / 1000;
}


void
spi_cs_hold_set (spi_t spi, uint16_t delay)
{
    spi->cs_hold = delay;
    spi_update (spi);
}


void
spi_cs_setup_set (spi_t spi, uint16_t delay)
{
    spi->cs_setup = delay;
    spi_update (spi);
}


void
spi_bits_set (spi_t spi, uint8_t bits)
{
    spi->bits = bits;
    spi_update (spi);
}


/* This performs a software reset for the specified controller (not an
   individual channel).  It puts the controller in slave mode for each
   channel.  */
static void
spi_reset (Spi *pSPI)
{
    pSPI->SPI_CR = SPI_CR_SWRST;
}


/* This enables the specified controller (not an individual
   channel).  */
static void
spi_enable (Spi *pSPI)
{
    pSPI->SPI_CR = SPI_CR_SPIEN;
}


static void
spi_disable (Spi *pSPI)
{
    pSPI->SPI_CR = SPI_CR_SPIDIS;
}


static void
spi_setup (Spi *pSPI)
{
    /* Desire PS = 0 (fixed peripheral select)
       PCSDEC = 0 (no decoding of chip selects)
       MSTR = 1 (master mode)
       MODFDIS = 1 (mode fault detection disabled)
       CSAAT = 0 (chip select rises after transmission)   */
    pSPI->SPI_MR = SPI_MR_MSTR | SPI_MR_MODFDIS;
}


void
spi_mode_set (spi_t spi, spi_mode_t mode)
{
    spi->mode = mode;
    spi_update (spi);
}


void
spi_cs_mode_set (spi_t spi, spi_cs_mode_t mode)
{
    spi->cs_mode = mode;
    spi_update (spi);
}


/** Enable fixed peripheral select and use specified channel.  When an
    SPI transfer takes place, the channel is used to look up the
    correct CSR.  */
static void
spi_channel_select (spi_t spi)
{
    spi->base->SPI_MR &= ~SPI_MR_PS;

    /* Insert bit pattern to specify which CSR to use and which NPCS
       to drive.  Note, if a value of 0xf is used then the SPI controller
       will hang.  */
    BITS_INSERT (spi->base->SPI_MR, SPI_CHANNEL_MASK (spi->channel), 16, 19);
}


/** Force CS to go low.  */
void
spi_cs_assert (spi_t spi)
{
    pio_output_low (spi->cs);
    spi->cs_active = 1;
}


/** Force CS to go high.  */
void
spi_cs_negate (spi_t spi)
{
    cpu_nop ();
    cpu_nop ();
    cpu_nop ();
    pio_output_high (spi->cs);
    spi->cs_active = 0;
}


/** Enable auto chip select.  Return zero if not possible with selected pio.  */
bool
spi_cs_auto_enable (spi_t spi)
{
    if (spi->cs_config == PIO_OUTPUT_HIGH)
        return 0;

    /* The CS will be automatically driven low when the next transfer
       takes place.  */
    pio_config_set (spi->cs, spi->cs_config);
    return 1;
}


/** Disable auto chip select.   */
void
spi_cs_auto_disable (spi_t spi)
{
    if (spi->cs_mode != SPI_CS_MODE_EXTERNAL)
        pio_config_set (spi->cs, PIO_OUTPUT_HIGH);
}


void
spi_config (spi_t spi)
{
    /* There are 4 sets of SPI registers, one for each channel.  We
       could check which SPI instance is currently using a channel and
       thus avoid reprogramming the registers unnecessarily.  */

    if (spi == spi_config_last)
        return;
    spi_config_last = spi;

    spi_channel_select (spi);
    spi_channel_mode_set (spi, spi->mode);
    spi_channel_bits_set (spi, spi->bits);
    spi_channel_clock_divisor_set (spi, spi->clock_divisor);
    spi_channel_cs_setup_set (spi, spi->cs_setup);
    spi_channel_cs_hold_set (spi, (spi->cs_hold + 31) / 32);
}


static void
spi_wakeup (spi_t spi)
{
    uint8_t dev_num;
    bool wakeup;

    dev_num = spi - spi_devices;

    if (spi_devices_enabled & BIT (dev_num))
        return;

    wakeup = !spi_devices_enabled;

    spi_devices_enabled |= BIT (dev_num);

    if (!wakeup)
        return;

    /* Configure PIO for MISO, MOSI, SPCK.    */
    pio_config_set (MISO0_PIO, MISO0_PERIPH);
    pio_config_set (MOSI0_PIO, MOSI0_PERIPH);
    pio_config_set (SPCK0_PIO, SPCK0_PERIPH);

    /* Enable SPI peripheral clock.  */
    mcu_pmc_enable (ID_SPI);

    spi_reset (SPI0);
    spi_setup (SPI0);
    spi_enable (SPI0);

#if SPI_CONTROLLERS_NUM == 2
    /* Configure PIO for MISO, MOSI, SPCK.    */
    pio_config_set (MISO1_PIO, MISO0_PERIPH);
    pio_config_set (MOSI1_PIO, MOSI1_PERIPH);
    pio_config_set (SPCK1_PIO, SPCK1_PERIPH);

    /* Enable SPI peripheral clock.  */
    mcu_pmc_enable (ID_SPI1);

    spi_reset (SPI1);
    spi_setup (SPI1);
    spi_enable (SPI1);
#endif
}


/** Initialise SPI for master mode.  */
spi_t
spi_init (const spi_cfg_t *cfg)
{
    spi_dev_t *spi;

    if (spi_devices_num >= SPI_DEVICES_NUM)
    {
        errno = EMFILE;
        return 0;
    }

    spi = spi_devices + spi_devices_num;
    spi_devices_num++;

    spi->channel = cfg->channel;
    spi->cs = cfg->cs;

    spi->base = SPI_BASE_GET (spi->channel);

    spi_channel_csr_set (spi, 0);

    spi_cs_mode_set (spi, cfg->cs_mode);

    spi->cs_config = spi_channel_cs_config_get (spi, spi->cs);
    spi_cs_auto_disable (spi);

    spi_cs_setup_set (spi, 0);
    spi_cs_hold_set (spi, 0);
    spi_mode_set (spi, cfg->mode);
    spi_bits_set (spi, cfg->bits ? cfg->bits : 8);
    /* If clock divisor not specified, default to something slow.  */
    spi_clock_speed_kHz_set (spi, cfg->clock_speed_kHz
                             ? cfg->clock_speed_kHz : 100);

    spi_wakeup (spi);
    return spi;
}


/** This only takes effect when the last SPI device is shutdown.  Then
    the CS lines are forced low.  This can only be done when there is
    no more activity on the SPI bus.  */
void
spi_shutdown (spi_t spi)
{
    uint8_t dev_num;
    int i;

    dev_num = spi - spi_devices;

    if (! (spi_devices_enabled & BIT (dev_num)))
        return;

    spi_devices_enabled &= ~BIT (dev_num);

    spi_config_last = 0;

    if (spi_devices_enabled)
        return;

    spi_disable (SPI0);

#if SPI_CONTROLLERS_NUM == 2
    spi_disable (SPI1);

    /* Force lines low to prevent powering devices.  */
    pio_config_set (MISO1_PIO, PIO_OUTPUT_LOW);
    pio_config_set (MOSI1_PIO, PIO_OUTPUT_LOW);
    pio_config_set (SPCK1_PIO, PIO_OUTPUT_LOW);

    /* Disable SPI peripheral clocks.  */
    mcu_pmc_disable (ID_SPI);
    mcu_pmc_disable (ID_SPI1);
#else
    /* Force lines low to prevent powering devices.  */
    pio_config_set (MISO0_PIO, PIO_OUTPUT_LOW);
    pio_config_set (MOSI0_PIO, PIO_OUTPUT_LOW);
    pio_config_set (SPCK0_PIO, PIO_OUTPUT_LOW);

    /* Disable SPI peripheral clock.  */
    mcu_pmc_disable (ID_SPI);
#endif

    /* Set all the chip select pins low.  */
    for (i = 0; i < spi_devices_num; i++)
    {
        if (spi->cs_mode != SPI_CS_MODE_EXTERNAL)
            pio_config_set (spi_devices[i].cs, PIO_OUTPUT_LOW);
    }
}


spi_ret_t
spi_transfer_8 (spi_t spi, const void *txbuffer, void *rxbuffer,
                spi_size_t len, bool terminate)
{
    spi_size_t i;
    const uint8_t *txdata = txbuffer;
    uint8_t *rxdata = rxbuffer;
    uint8_t rx;
    uint8_t tx = 0;

    spi_config (spi);

    i = 0;
    switch (spi->cs_mode)
    {
    case SPI_CS_MODE_TOGGLE:
        if (1 && spi_cs_auto_enable (spi))
        {
            for (i = 0; i < len; i++)
            {
                if (txdata)
                    tx = *txdata++;

                SPI_XFER (spi->base, tx, rx);

                if (rxdata)
                    *rxdata++ = rx;
            }

            spi_cs_auto_disable (spi);
        }
        else
        {
            for (i = 0; i < len; i++)
            {
                /* Defer CS assertion a few cycles.  This can be
                   deferred further with spi_cs_setup_set.
                   This works by delaying the time when the SPI
                   controller starts the shift clock from when the
                   transmit data register is written.  */
                spi_cs_assert (spi);

                if (txdata)
                    tx = *txdata++;

                SPI_XFER (spi->base, tx, rx);

                if (rxdata)
                    *rxdata++ = rx;

                /* Defer CS negation a few cycles.  */
                spi_cs_negate (spi);
            }
        }
        break;

    case SPI_CS_MODE_FRAME:
    case SPI_CS_MODE_HIGH:
        if (len != 0 && spi->cs_mode == SPI_CS_MODE_FRAME)
            spi_cs_assert (spi);

        for (i = 0; i < len; i++)
        {
            if (txdata)
                tx = *txdata++;

            SPI_XFER (spi->base, tx, rx);

            if (rxdata)
                *rxdata++ = rx;
        }

        if (terminate && spi->cs_mode == SPI_CS_MODE_FRAME)
            spi_cs_negate (spi);
        break;

    case SPI_CS_MODE_EXTERNAL:
        for (i = 0; i < len; i++)
        {
            if (txdata)
                tx = *txdata++;

            SPI_XFER (spi->base, tx, rx);

            if (rxdata)
                *rxdata++ = rx;
        }
        break;
    }

    return i;
}


spi_ret_t
spi_transfer_16 (spi_t spi, const void *txbuffer, void *rxbuffer,
                 spi_size_t len, bool terminate)
{
    spi_size_t i;
    const uint16_t *txdata = txbuffer;
    uint16_t *rxdata = rxbuffer;
    uint16_t rx;
    uint16_t tx = 0;

    spi_config (spi);

    i = 0;
    switch (spi->cs_mode)
    {
    case SPI_CS_MODE_TOGGLE:
        if (0 && spi_cs_auto_enable (spi))
        {
            for (i = 0; i < len; i += 2)
            {
                if (txdata)
                    tx = *txdata++;

                SPI_XFER (spi->base, tx, rx);

                if (rxdata)
                    *rxdata++ = rx;
            }

            spi_cs_auto_disable (spi);
        }
        else
        {
            for (i = 0; i < len; i += 2)
            {
                /* Defer CS assertion a few cycles.  This can be
                   deferred further with spi_cs_setup_set.
                   This works by delaying the time when the SPI
                   controller starts the shift clock from when the
                   transmit data register is written.  */
                spi_cs_assert (spi);

                if (txdata)
                    tx = *txdata++;

                SPI_XFER (spi->base, tx, rx);

                if (rxdata)
                    *rxdata++ = rx;

                /* Defer CS negation a few cycles.  */
                spi_cs_negate (spi);
            }
        }
        break;

    case SPI_CS_MODE_FRAME:
    case SPI_CS_MODE_HIGH:
        if (len != 0 && spi->cs_mode == SPI_CS_MODE_FRAME)
            spi_cs_assert (spi);

        for (i = 0; i < len; i += 2)
        {
            if (txdata)
                tx = *txdata++;

            SPI_XFER (spi->base, tx, rx);

            if (rxdata)
                *rxdata++ = rx;
        }

        if (terminate && spi->cs_mode == SPI_CS_MODE_FRAME)
            spi_cs_negate (spi);
        break;

    case SPI_CS_MODE_EXTERNAL:
        for (i = 0; i < len; i += 2)
        {
            if (txdata)
                tx = *txdata++;

            SPI_XFER (spi->base, tx, rx);

            if (rxdata)
                *rxdata++ = rx;
        }
        break;
    }

    return i;
}


spi_ret_t
spi_transfer (spi_t spi, const void *txbuffer, void *rxbuffer,
              spi_size_t len, bool terminate)
{
    /* If terminate is zero we should lock the SPI peripheral
       for this device until terminate is non-zero.  */

    if (spi->bits <= 8)
        return spi_transfer_8 (spi, txbuffer, rxbuffer, len, terminate);
    else
        return spi_transfer_16 (spi, txbuffer, rxbuffer, len, terminate);
}


spi_ret_t
spi_transact (spi_t spi, spi_transfer_t *transfer, uint8_t size)
{
    uint8_t i;
    spi_ret_t bytes;
    spi_ret_t ret;

    bytes = 0;
    for (i = 0; i < size; i++)
    {
        ret = spi_transfer (spi, transfer[i].txbuffer, transfer[i].rxbuffer,
                            transfer[i].size, i == size - 1);
        if (ret < 0)
            return ret;
        bytes += ret;
    }
    return bytes;
}


spi_ret_t
spi_write (spi_t spi, const void *buffer, spi_size_t len, bool terminate)
{
    return spi_transfer (spi, buffer, 0, len, terminate);
}


spi_ret_t
spi_read (spi_t spi, void *buffer, spi_size_t len, bool terminate)
{
    return spi_transfer (spi, 0, buffer, len, terminate);
}


/* Return non-zero if there is a character ready to be read.  */
bool
spi_read_ready_p (spi_t spi)
{
#if HOSTED
    return 1;
#else
    return SPI_READY_P (spi->base);
#endif
}


/* Return non-zero if a character can be written without blocking.  */
bool
spi_write_ready_p (spi_t spi)
{
    return SPI_TXEMPTY_P (spi->base);
}


/* Write character to SPI, return received character.  */
uint8_t
spi_xferc (spi_t spi, char ch)
{
    uint8_t txdata;
    uint8_t rxdata;

    txdata = ch;
    spi_transfer_8 (spi, &txdata, &rxdata, sizeof (txdata), 1);

    return rxdata;
}


/* Read character from SPI by sending a dummy word.  */
uint8_t
spi_getc (spi_t spi)
{
    return spi_xferc (spi, 0);
}


/* Write character to SPI, ignore received character.  */
void
spi_putc (spi_t spi, char ch)
{
    spi_xferc (spi, ch);
}
