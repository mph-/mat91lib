# OpenOCD port for GDB communications
target remote tcp:localhost:3333

define PMC
    p/x *((Pmc    *)0x400E0400U)
end

define SSC
    p/x *((Ssc    *)0x40004000U)
end

monitor halt
