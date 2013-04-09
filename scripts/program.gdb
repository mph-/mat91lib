# OpenOCD port for GDB communications
target remote tcp:localhost:3333

# Enabling DCC downloads boosted download speed from 26kB/s to 154kB/s
monitor arm7_9 dcc_downloads enable

# Enable fast memory access (may be less stable, comment out if issues arise)
#monitor arm7_9 fast_memory_access enable

# fires the reset-init event which fires the event handler defined in "at91sam7x256.cfg"
monitor reset init

# perform the actual reset (reset init does not do this)
monitor reset

monitor sleep 500

load
