void SPI_ISR (void)
{
    AT91PS_SPI     pSPI = SPI;

    spi_Status = pSPI->SPI_SR;
    
    
    // A data has been recieved and transmitted to the RBR.
    if (spi_Status & AT91C_SPI_RDRF)    
    {
        spi_Rdrf_Flag = 1;
        pSPI->SPI_IDR = AT91C_SPI_RDRF; // Interrupt is disabled to avoid inopportune interrupt 
    }

    // The last data written in the TDR has been transmitted to the serializer
    if (spi_Status & AT91C_SPI_TDRE)    
    {
        spi_Tdre_Flag  = 1;
        pSPI->SPI_IDR = AT91C_SPI_TDRE; // Interrupt is disabled to avoid inopportune interrupt 
    }
    
    // A mode fault has occured (NCS0 was tied low).
    if (spi_Status & AT91C_SPI_MODF)    
    {
        spi_Modf_Flag  = 1;
        pSPI->SPI_IDR = AT91C_SPI_MODF; // Interrupt is disabled to avoid inopportune interrupt 
    }
    
    // An overrun error has occured.
    if (spi_Status & AT91C_SPI_OVRES)   
    {
        spi_Ovred_Flag  = 1;
        pSPI->SPI_IDR = AT91C_SPI_OVRES; // Interrupt is disabled to avoid inopportune interrupt 
    }
    
    // The Receive Counter Register has reached 0 since the last write
    // in SPI_RCR or SPI_RNCR.
    if (spi_Status & AT91C_SPI_ENDRX)   
    {
        spi_Endrx_Flag  = 1;
        pSPI->SPI_IDR = AT91C_SPI_ENDRX; // Interrupt is disabled to avoid inopportune interrupt 
    }
    
    // The Transmit Counter Register has reached 0 since the last
    // write in SPI_TCR or SPI_TNCR.
    if (spi_Status & AT91C_SPI_ENDTX)   
    {
        spi_Endtx_Flag  = 1;
        pSPI->SPI_IDR = AT91C_SPI_ENDTX; // Interrupt is disabled to avoid inopportune interrupt 
    }   
    
    // Both SPI_RCR and SPI_RNCR have a value of 0.
    if (spi_Status & AT91C_SPI_RXBUFF)  
    {
        spi_Rxbuff_Flag  = 1;
        pSPI->SPI_IDR = AT91C_SPI_RXBUFF; // Interrupt is disabled to avoid inopportune interrupt 
    }   
    
    // Both SPI_TCR and SPI_TNCR have a value of 0.
    if (spi_Status & AT91C_SPI_TXBUFE)  
    {
        spi_Txbuff_Flag  = 1;
        pSPI->SPI_IDR = AT91C_SPI_TXBUFE; // Interrupt is disabled to avoid inopportune interrupt 
    }
    
    // NSS has risen
    if (spi_Status & AT91C_SPI_NSSR)    
    {
        spi_Nssr_Flag  = 1;
        pSPI->SPI_IDR = AT91C_SPI_NSSR; // Interrupt is disabled to avoid inopportune interrupt 
    }
    
    // TDR and shift registers are empty
    if (spi_Status & AT91C_SPI_TXEMPTY) 
    {
        spi_Txempty_Flag  = 1;
        pSPI->SPI_IDR = AT91C_SPI_TXEMPTY; // Interrupt is disabled to avoid inopportune interrupt 
    }
}


