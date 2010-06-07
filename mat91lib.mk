# Need to define:
# ROOT toplevel directory of project
# OPT optimisation level, e.g. -O2
# MCU name of microcontroller
# RUN_MODE either ROM_RUN or RAM_RUN
# MAT91LIB_DIR path to mat91lib
# PERIPHERALS list of peripherals to build

ifndef ROOT
ROOT = .
endif

ifndef MCU
MCU = AT91SAM7S256
endif

ifndef RUN_MODE
RUN_MODE = ROM_RUN	
endif

ifndef OPT
OPT = -Os
endif

ifndef TOOLCHAIN
TOOLCHAIN = arm-eabi
endif

SCRIPTS = $(MAT91LIB_DIR)/scripts
LDSCRIPTS = $(MAT91LIB_DIR)/ldscripts


CC = $(TOOLCHAIN)-gcc
OBJCOPY = $(TOOLCHAIN)-objcopy
SIZE = $(TOOLCHAIN)-size
DEL = rm


CFLAGS += -mcpu=arm7tdmi -Wall -Wstrict-prototypes -W -gdwarf-2 -D$(RUN_MODE) $(INCLUDES) $(OPT) -mthumb-interwork -D$(MCU) 

LDFLAGS += -mthumb-interwork -nostartfiles -lm -lc


ifeq ($(RUN_MODE), RAM_RUN)
LDFLAGS +=-T$(LDSCRIPTS)/$(MCU)-RAM.ld
else 
LDFLAGS +=-T$(LDSCRIPTS)/$(MCU)-ROM.ld
endif


# This cannot be compiled in thumb mode.
objs/crt0.o: crt0.c config.h target.h
	mkdir -p objs
	$(CC) -c $(CFLAGS) -O2 $< -o $@

EXTRA_OBJ = objs/crt0.o objs/cpu.o


include $(MAT91LIB_DIR)/peripherals.mk


# Program the device.
program: $(TARGET_OUT)
	$(TOOLCHAIN)-gdb -batch -x $(SCRIPTS)/program.gdb $^

# Reset the device.
reset: $(TARGET_OUT)
	$(TOOLCHAIN)-gdb -batch -x $(SCRIPTS)/reset.gdb $^

# Attach debugger.
debug:
	$(TOOLCHAIN)-gdb  -x $(SCRIPTS)/debug.gdb $(TARGET_OUT)

