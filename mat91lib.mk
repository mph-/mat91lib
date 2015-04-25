# Need to define:
# OPT optimisation level, e.g. -O2
# MCU name of microcontroller
# RUN_MODE either ROM or RAM
# MAT91LIB_DIR path to mat91lib
# PERIPHERALS list of peripherals to build

ifndef MCU
$(error MCU undefined, this needs to be defined in the Makefile)
endif

ifndef RUN_MODE
RUN_MODE = ROM	
endif

ifndef OPT
OPT = -Os
endif

ifndef TOOLCHAIN
TOOLCHAIN = arm-none-eabi
endif

TARGET_MAP = $(addsuffix .map, $(basename $(TARGET)))

ifneq (, $(findstring SAM7, $(MCU)))
FAMILY = sam7
else
ifneq (, $(findstring SAM4S, $(MCU)))
FAMILY = sam4s
else
$(error unknown family)
endif
endif


all: $(TARGET)

include $(MAT91LIB_DIR)/$(FAMILY)/$(FAMILY).mk

SCRIPTS = $(MAT91LIB_DIR)/$(FAMILY)/scripts
LDSCRIPTS = $(MAT91LIB_DIR)/$(FAMILY)


CC = $(TOOLCHAIN)-gcc
OBJCOPY = $(TOOLCHAIN)-objcopy
SIZE = $(TOOLCHAIN)-size
DEL = rm

INCLUDES += -I.


ifeq ($(RUN_MODE), RAM)
LDFLAGS +=-L$(LDSCRIPTS) -T$(MCU)-RAM.ld
else 
LDFLAGS +=-L$(LDSCRIPTS) -T$(MCU)-ROM.ld
endif

# Hack.  FIXME
SRC += syscalls.c

# Create list of object and dependency files.  Note, sort removes duplicates.
OBJ = $(addprefix objs/, $(sort $(SRC:.c=.o)))
DEPS = $(addprefix deps/, $(sort $(SRC:.c=.d)))

include $(MAT91LIB_DIR)/peripherals.mk

EXTRA_OBJ = objs/crt0.o objs/mcu.o


objs:
	mkdir -p objs

deps:
	mkdir -p deps

objs/%.o: %.c Makefile
	$(CC) -c $(CFLAGS) $< -o $@
# Generate dependencies to see if object file needs recompiling.
	$(CC) -MM $(CFLAGS) $< > deps/$*.d

# Link object files to form output file.
$(TARGET): deps objs $(OBJ) $(EXTRA_OBJ)
	$(CC) $(OBJ) $(EXTRA_OBJ) $(LDFLAGS) -o $@ -lm -Wl,-Map=$(TARGET_MAP),--cref
	$(SIZE) $@

# Include the dependency files.
-include $(DEPS)


# Remove the objs directory
clean-objs:
	-$(DEL) -fr objs

# Rebuild the code, don't delete dependencies.
rebuild: clean-objs $(TARGET_OUT)

# Generate cscope tags file
.PHONY: cscope
cscope:
	cscope -Rb $(INCLUDES)

# Remove non-source files.
.PHONY: clean
clean: 
	-$(DEL) -f *.o *.out *.hex *.bin *.elf *.d *.lst *.map *.sym *.lss *.cfg *.ocd *~
	-$(DEL) -fr objs deps


# Program the device.
program: $(TARGET)
	$(TOOLCHAIN)-gdb -batch -x $(SCRIPTS)/program.gdb $^

# Reset the device.
reset: $(TARGET)
	$(TOOLCHAIN)-gdb -batch -x $(SCRIPTS)/reset.gdb $^

# Attach debugger.
debug:
	$(TOOLCHAIN)-gdb  -x $(SCRIPTS)/debug.gdb $(TARGET)

