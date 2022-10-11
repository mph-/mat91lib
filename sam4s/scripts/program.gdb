# OpenOCD port for GDB communications
target remote tcp:localhost:3333

# Fire the reset-init event which fires the event handler defined in "at91sam7x256.cfg"; this programs the clock and sets things up
monitor reset init

# Perform the actual reset (reset init does not do this)
monitor reset halt

monitor sleep 500

load

monitor reset

