MEMORY
{
        /* The 16 kB is ROM for bootloader.  */
        FLASH (rx) : ORIGIN = 0x00400000, LENGTH = 1M
        SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

INCLUDE SAM4SX-ROM.ld
