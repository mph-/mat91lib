# OpenOCD port for GDB communications
target remote tcp:localhost:3333

# Reset the target
monitor reset
monitor soft_reset_halt
monitor resume 0
