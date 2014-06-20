# OpenOCD port for GDB communications
target remote tcp:localhost:3333

define PMC
    p/x *((Pmc    *)0x400E0400U)
end

monitor halt
