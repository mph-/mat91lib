# OpenOCD port for GDB communications
target remote tcp:localhost:3333

# Set bit to boot from flash rather than running bootloader
monitor at91sam4 gpnvm set 1
