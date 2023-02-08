/** @file    ssc.c
    @author  Stuart Duncan, Michael Hayes
    @date    10 January 2012
    @brief   Routines for control and configuration of the AT91SAM SSC

   */

#include "ssc.h"
#include "mcu.h"
#include "pio.h"
#include "bits.h"


/* A word can be transmitted over TD during the frame sync period,
   using the contents of the 16-bit TSHR register (this is a
   preamble).  Similarly, RD can be sampled during the frame sync
   period and stored in the 16-bit RSHR.

   The following data packet can be up to 32 bits long.  Thus if one
   is cunning, a 48 bit word can be transferred: 16 bits in the
   preamble and 32 bits after.  However, the 48 bits cannot be sent by
   DMA.

   The transmitter and receiver can both be programmed to start their
   operations when a start event occurs.  This can be:

   * THR written or when the rx is enabled  (continuous mode)
   * an event on RF/TF pins
   * the receiver matches the preamble

   Whenever the tx has started it will output N bits where N is the
   number of bits per word.  If the TSR is not loaded by writing to
   the THR before the start, the TSR will clock out N zeros.  The end
   of frame occurs when M * N clocks have been output, where M is the
   number of words per frame.

   Whenever the rx has started, it waits for N bits.  The incoming
   data is shifted into the RSR.  When N bits have been received, RSR
   is copied to RHR and the RXRDY flag is set.  If another transfer is
   received before RHR is read, the OVERRUN flag is set and the RSR is
   copied to RHR, losing the previous data.

*/


static ssc_dev_t ssc_dev;


/* Set the clock divider.  */
static void
ssc_clock_divisor_set (ssc_t ssc, ssc_clock_divisor_t clock_divisor)
{
   SSC->SSC_CMR = clock_divisor;
   ssc->clock_divisor = clock_divisor;
}


ssc_clock_speed_t
ssc_clock_speed_kHz_set (ssc_t ssc, ssc_clock_speed_t clock_speed_kHz)
{
    uint32_t clock_speed;

    clock_speed = clock_speed_kHz * 1000;
    ssc_clock_divisor_set (ssc, ((F_CPU_UL / 2) + clock_speed - 1) / clock_speed);
    clock_speed = (F_CPU / 2) / ssc->clock_divisor;

    return clock_speed / 1000;
}


void
ssc_fs_period_set (ssc_t ssc, ssc_fs_period_t fs_period, ssc_module_t module)
{
    uint8_t period;

    if (fs_period > 512)
        fs_period = 512;

    if (fs_period == 0)
        period = 0;
    else
        period = (fs_period >> 1) - 1;

    if (module == SSC_TX)
        BITS_INSERT (SSC->SSC_TCMR, period, 24, 31);
    else
        BITS_INSERT (SSC->SSC_RCMR, period, 24, 31);
}


ssc_fs_period_t
ssc_fs_period_get (ssc_t ssc, ssc_module_t module)
{
    uint8_t period;

    if (module == SSC_TX)
        period = BITS_EXTRACT (SSC->SSC_TCMR, 24, 31);
    else
        period = BITS_EXTRACT (SSC->SSC_RCMR, 24, 31);

    return (period + 1) << 1;
}


void
ssc_tx_start_mode_set (ssc_t ssc, ssc_start_mode_t start_mode)
{
    SSC->SSC_TCMR = (SSC->SSC_TCMR & ~0XF00) | start_mode;
}


void
ssc_rx_start_mode_set (ssc_t ssc, ssc_start_mode_t start_mode)
{
    SSC->SSC_RCMR = (SSC->SSC_RCMR & ~0XF00) | start_mode;
}


/* Configure a module.  */
static uint8_t
ssc_module_config (ssc_t ssc, ssc_module_cfg_t *cfg, ssc_module_t module)
{
    /* Select options to apply to clock mode register.  */
    uint32_t cmr;
    uint32_t fmr;
    uint8_t fslen;
    uint8_t fslen_ext;
    uint8_t datlen;

    if (!cfg)
        return 0;

    ssc_fs_period_set (ssc, cfg->fs_period, module);

    cmr = cfg->start_delay << 16;

    cmr |= cfg->start_mode;
    /* If start mode is SSC_START_COMPARE0 then should load RC0R
       and perhaps RC1R (the compare reigister).  */

    cmr |= cfg->clock_edge;

    /* Select clock source.  */
    cmr |= cfg->clock_select;

    /* Select clock output mode.  */
    cmr |= cfg->clock_out_mode;

    /* Select gating of the clock by the frame sync.  */
    cmr |= cfg->clock_gate_mode;

    if (cfg->fs_length < 1)
        cfg->fs_length = 1;

    fslen = cfg->fs_length - 1;
    fslen_ext = fslen >> 4;
    fslen = fslen & 0x0f;

    /* datlen must be greater than 0 (at least 2 bits transferred).
       If datlen is smaller than 8, data transfers are in bytes, else
       if datlen is smaller than 16, half-words are transferred, otherwise
       32-bit words are transferred.  */
    datlen = cfg->data_length - 1;

    fmr = cfg->fs_mode | (fslen << 16) | (fslen_ext << 28)
        | ((cfg->data_num - 1) << 8) | datlen;

    /* Select frame sync edge that will generate interrupt.  */
    fmr |= cfg->fs_edge;

    if (cfg->data_msb_first)
        fmr |= SSC_RFMR_MSBF;

    /* Apply the configuration to the appropriate module.  */
    if (module == SSC_TX)
    {
        if (cfg->td_default)
            fmr |= SSC_TFMR_DATDEF;

        SSC->SSC_TFMR = fmr | cfg->sync_data_enable;
        SSC->SSC_TCMR = cmr;
    }
    else
    {
        if (cfg->loop_mode)
            fmr |= SSC_RFMR_LOOP;

        SSC->SSC_RFMR = fmr;
        SSC->SSC_RCMR = cmr | cfg->stop_mode;
    }
    return 1;
}


/* Configure the SSC peripheral.  */
static void
ssc_config_set (ssc_t ssc, const ssc_cfg_t *cfg)
{
    /* Enable the peripheral clock.  */
    mcu_pmc_enable (ID_SSC);

    /* Reset receiver and transmitter.  */
    SSC->SSC_CR = SSC_CR_SWRST | SSC_CR_RXDIS | SSC_CR_TXDIS;

    /* Set the clock divider.  */
    ssc_clock_speed_kHz_set (ssc, cfg->clock_speed_kHz);

    /* Configure the receiver module if the configuration exists.  */
    ssc_module_config (ssc, cfg->rx, SSC_RX);

    /* Configure the transmit module if the configuration exists.  */
    ssc_module_config (ssc, cfg->tx, SSC_TX);
}


/* Check if a buffer (tx or rx) is ready (empty or full respectively).  */
static bool
ssc_module_ready_p (ssc_t ssc, ssc_module_t tx_rx)
{
    unsigned int mask = 0;

    switch (tx_rx)
    {
    case SSC_TX:
        mask = SSC_SR_TXRDY;
        break;

    case SSC_RX:
        mask = SSC_SR_RXRDY;
        break;
    }

    return (mask & SSC->SSC_SR) != 0;
}


void
ssc_sync(ssc_t ssc)
{
    int i;

    #define TIMEOUT 10000

    SSC->SSC_CR |= SSC_CR_RXDIS;
    pio_config_set (RF_PIO, PIO_INPUT);

    /* Wait for high transition.  */
    for (i = 0; i < TIMEOUT; i++)
    {
        if (pio_input_get (RF_PIO))
            break;
    }

    /* Wait for low transition.  */
    for (i = 0; i < TIMEOUT; i++)
    {
        if (! pio_input_get (RF_PIO))
            break;
    }

    /* Wait for high transition.  */
    for (i = 0; i < TIMEOUT; i++)
    {
        if (pio_input_get (RF_PIO))
            break;
    }

    pio_config_set (RF_PIO, RF_PERIPH);
    SSC->SSC_CR |= SSC_CR_RXEN;
}


/* Enable an SSC module.  */
static void
ssc_module_enable (ssc_t ssc, ssc_module_t tx_rx)
{
    switch (tx_rx)
    {
    case SSC_TX:
        pio_config_set (TD_PIO, TD_PERIPH);
        pio_config_set (TK_PIO, TK_PERIPH);
        pio_config_set (TF_PIO, TF_PERIPH);

        SSC->SSC_CR |= SSC_CR_TXEN;
        break;

    case SSC_RX:
        pio_config_set (RD_PIO, RD_PERIPH);
        pio_config_set (RK_PIO, RK_PERIPH);
        pio_config_set (RF_PIO, RF_PERIPH);

        SSC->SSC_CR |= SSC_CR_RXEN;
        break;
    }
}


/* Disable an SSC module.  */
static void
ssc_module_disable (ssc_t ssc, ssc_module_t tx_rx)
{
    switch (tx_rx)
    {
    case SSC_TX:
        SSC->SSC_CR |= SSC_CR_TXDIS;

        pio_config_set (TD_PIO, PIO_INPUT);
        pio_config_set (TK_PIO, PIO_INPUT);
        pio_config_set (TF_PIO, PIO_INPUT);
        break;

    case SSC_RX:
        SSC->SSC_CR |= SSC_CR_RXDIS;

        pio_config_set (RD_PIO, PIO_INPUT);
        pio_config_set (RK_PIO, PIO_INPUT);
        pio_config_set (RF_PIO, PIO_INPUT);
        break;
    }
}


/* Disable both rx and tx.  */
void
ssc_disable (ssc_t ssc)
{
    if (ssc->tx)
        ssc_module_disable (ssc, SSC_TX);

    if (ssc->rx)
        ssc_module_disable (ssc, SSC_RX);
}


/* Enable both rx and tx.  */
void
ssc_enable (ssc_t ssc)
{
    if (ssc->tx)
        ssc_module_enable (ssc, SSC_TX);

    if (ssc->rx)
    {
        ssc_module_enable (ssc, SSC_RX);
        /* The RXRDY pin gets set mysteriously but this does not clear it.  */
        ssc_read_value (ssc);
    }
}


/* Read data from the rx buffer.  */
static uint32_t
ssc_read_8 (ssc_t ssc, void *buffer, uint32_t length)
{
    uint8_t *dst = buffer;
    uint32_t i;

    for (i = 0; i < length; i++)
    {
        while (!ssc_read_ready_p (ssc))
            continue;
        *dst++ = ssc_read_value (ssc);
    }
    return length;
}


/* Read data from the rx buffer.  */
static uint32_t
ssc_read_16 (ssc_t ssc, void *buffer, uint32_t length)
{
    uint16_t *dst = buffer;
    uint32_t i;

    for (i = 0; i < length; i++)
    {
        while (!ssc_read_ready_p (ssc))
            continue;
        *dst++ = ssc_read_value (ssc);
    }
    return length << 1;
}


/* Read data from the rx buffer.  */
static uint32_t
ssc_read_32 (ssc_t ssc, void *buffer, uint32_t length)
{
    uint32_t *dst = buffer;
    uint32_t i;

    for (i = 0; i < length; i++)
    {
        while (!ssc_read_ready_p (ssc))
            continue;
        *dst++ = ssc_read_value (ssc);
    }
    return length << 2;
}


/* Read data from the rx buffer.  */
uint32_t
ssc_read (ssc_t ssc, void *buffer, uint32_t bytes)
{
    if (ssc->tx->data_length > 16)
        return ssc_read_32 (ssc, buffer, (bytes + 3) >> 2);
    else if (ssc->tx->data_length > 8)
        return ssc_read_16 (ssc, buffer, (bytes + 1) >> 1);
    else
        return ssc_read_8 (ssc, buffer, bytes);
}


uint32_t
ssc_read_32_unpack_int16_sum (ssc_t ssc, int32_t *buffer, uint32_t length)
{
    uint32_t i;

    for (i = 0; i < length; i++)
    {
        int32_t val;

        while (!ssc_read_ready_p (ssc))
            continue;

        val = ssc_read_value (ssc);
        *buffer++ += val >> 16;
        *buffer++ += (val << 16) >> 16;
    }
    return length;
}


uint32_t
ssc_read_16_add (ssc_t ssc, int32_t *buffer, uint32_t length)
{
    uint32_t i;

    for (i = 0; i < length; i++)
    {
        int16_t val;

        while (!ssc_read_ready_p (ssc))
            continue;

        val = ssc_read_value (ssc);
        *buffer++ += val;
    }
    return length;
}


uint32_t
ssc_read_16_subtract (ssc_t ssc, int32_t *buffer, uint32_t length)
{
    uint32_t i;

    for (i = 0; i < length; i++)
    {
        int16_t val;

        while (!ssc_read_ready_p (ssc))
            continue;

        val = ssc_read_value (ssc);
        *buffer++ -= val;
    }
    return length;
}


uint32_t
ssc_read_ignore (ssc_t ssc, uint32_t length)
{
    uint32_t i;

    for (i = 0; i < length; i++)
    {
        while (!ssc_read_ready_p (ssc))
            continue;

        ssc_read_value (ssc);
    }
    return length;
}


/* Write to the transmit buffer.  */
static uint16_t
ssc_write_8 (ssc_t ssc, void *buffer, uint16_t length)
{
    uint8_t *src = buffer;
    int i;

    for (i = 0; i < length; i++)
    {
        while (!ssc_write_ready_p (ssc))
            continue;
        SSC->SSC_THR = *src++;
    }
    return length;
}


/* Write to the transmit buffer.  */
static uint16_t
ssc_write_16 (ssc_t ssc, void *buffer, uint16_t length)
{
    uint16_t *src = buffer;
    int i;

    for (i = 0; i < length; i++)
    {
        while (!ssc_write_ready_p (ssc))
            continue;
        SSC->SSC_THR = *src++;
    }
    return length << 1;
}


/* Write to the transmit buffer.  */
static uint16_t
ssc_write_32 (ssc_t ssc, void *buffer, uint16_t length)
{
    uint32_t *src = buffer;
    int i;

    for (i = 0; i < length; i++)
    {
        while (!ssc_write_ready_p (ssc))
            continue;
        SSC->SSC_THR = *src++;
    }
    return length << 2;
}


/* Write to the transmit buffer.  */
uint16_t
ssc_write (ssc_t ssc, void *buffer, uint16_t bytes)
{
    if (ssc->tx->data_length > 16)
        return ssc_write_32 (ssc, buffer, (bytes + 3) >> 2);
    else if (ssc->tx->data_length > 8)
        return ssc_write_16 (ssc, buffer, (bytes + 1) >> 1);
    else
        return ssc_write_8 (ssc, buffer, bytes);
}


Pdc *
ssc_pdc_get (ssc_t ssc)
{
    return PDC_SSC;
}


ssc_t
ssc_init (const ssc_cfg_t *cfg)
{
    ssc_t ssc;

    ssc = &ssc_dev;

    ssc_config_set (ssc, cfg);

    ssc->rx = cfg->rx;
    ssc->tx = cfg->tx;

    ssc_enable (ssc);

    return ssc;
}


void
ssc_shutdown (ssc_t ssc)
{
    ssc_disable (ssc);

    /* Perhaps configure as inputs with pullups?  */

    pio_config_set (RD_PIO, PIO_OUTPUT_LOW);
    pio_config_set (RK_PIO, PIO_OUTPUT_LOW);
    pio_config_set (RF_PIO, PIO_OUTPUT_LOW);

    pio_config_set (TD_PIO, PIO_OUTPUT_LOW);
    pio_config_set (TK_PIO, PIO_OUTPUT_LOW);
    pio_config_set (TF_PIO, PIO_OUTPUT_LOW);
}


/* Return non-zero if transmitter finished.  */
bool
ssc_write_finished_p (ssc_t ssc)
{
    return (SSC_SR_TXEMPTY & SSC->SSC_SR) != 0;
}


void
ssc_reset (ssc_t ssc)
{
    SSC->SSC_CR |= BIT(15);
}
