# OpenOCD port for GDB communications
target remote tcp:localhost:3333

# Enable fast memory access (may be less stable, comment out if problems arise)
monitor arm7_9 fast_memory_access enable

monitor halt
