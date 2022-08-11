# Need to define:
# OPT optimisation level, e.g. -O2
# MCU name of microcontroller
# RUN_MODE either ROM or RAM
# MAT91LIB_DIR path to mat91lib
# PERIPHERALS list of peripherals to build

# Note.  If weird errors occur it is because the makefile fragments are
# loaded in the wrong order.  mmuclib.mk should be included last.

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

ifneq (, $(shell which $(TOOLCHAIN)-gdb))
GDB = $(TOOLCHAIN)-gdb
else
# This supersedes arm-none-eabi-gdb
GDB = gdb-multiarch
endif

BUILD_DIR ?= .
OBJDIR = $(BUILD_DIR)/objs-$(BOARD)
DEPDIR = $(BUILD_DIR)/deps-$(BOARD)

TARGET := $(BUILD_DIR)/$(TARGET)

all: $(TARGET)

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

MAT91LIB_FAMILY_DIR = $(MAT91LIB_DIR)/$(FAMILY)

include $(MAT91LIB_FAMILY_DIR)/$(FAMILY).mk

SCRIPTS = $(MAT91LIB_FAMILY_DIR)/scripts
LDSCRIPTS = $(MAT91LIB_FAMILY_DIR)

LD = $(TOOLCHAIN)-gcc

CC = $(TOOLCHAIN)-gcc
OBJCOPY = $(TOOLCHAIN)-objcopy
SIZE = $(TOOLCHAIN)-size
DEL = rm -f

ifeq ($(RUN_MODE), RAM)
LDFLAGS +=-L$(LDSCRIPTS) -T$(MCU)-RAM.ld
else
LDFLAGS +=-L$(LDSCRIPTS) -T$(MCU)-ROM.ld
endif

MAT91LIB_SRC = \
	$(MAT91LIB_FAMILY_DIR)/crt0.c \
	$(MAT91LIB_FAMILY_DIR)/mcu.c \
	$(MAT91LIB_DIR)/syscalls.c

ifndef BOARD
BOARD=
endif

SRC += $(notdir $(MAT91LIB_SRC))
OBJS += $(addprefix $(OBJDIR)/, $(patsubst %.c,%.o,$(notdir $(SRC))))
DEPS += $(addprefix $(DEPDIR)/, $(patsubst %.o,%.d,$(notdir $(OBJS))))
VPATH += $(dir $(MAT91LIB_SRC))
INCLUDES += -I. -I"$(MAT91LIB_DIR)"
LDLIBS += -lm -lc

include $(MAT91LIB_DIR)/peripherals.mk

Q=@
ifdef VERBOSE
ifneq ($(VERBOSE),0)
Q=
endif
endif

print-drivers:
	@echo $(DRIVERS)

print-deps:
	@echo $(DEPS)

print-objs:
	@echo $(OBJS)

print-cflags:
	@echo $(CFLAGS)

print-src:
	@echo $(SRC)

print-vpath:
	@echo $(VPATH)

print-includes:
	@echo $(INCLUDES)

# Include the dependency files.
-include $(DEPS)

# Rule to compile .c file to .o file.
$(OBJDIR)/%.o: %.c Makefile
	@mkdir -p "$(@D)"
	$(info CC $<)
	$(Q)$(CC) $(CFLAGS) -o $@ -c $<

# Rule to create .d file from .c file.
$(DEPDIR)/%.d: %.c
	@mkdir -p "$(@D)"
	$(Q)$(CC) -MM -MF "$(@)" -MT "$(OBJDIR)/$(<:.c=.o)" $(CFLAGS) $<

# Link object files to form output file.
$(TARGET): $(OBJS)
	$(info LD $@)
	$(Q)$(LD) $(LDFLAGS) -o $@  $^ $(LDLIBS) -Wl,-Map=$(TARGET_MAP),--cref
	$(SIZE) $@

# Remove non-source files.
.PHONY: clean
clean:
ifeq ($(abspath $(BUILD_DIR)), $(abspath .))
	-$(DEL) *.o *.out *.hex *.bin *.elf *.d *.lst *.map *.sym *.lss *.cfg *.ocd *~
	-$(DEL) -r $(OBJDIR) $(DEPDIR)
else
	-$(DEL) -r $(BUILD_DIR)
endif
ifdef CLEAN_EXTRA_PATHS
	-$(DEL) -r $(CLEAN_EXTRA_PATHS)
endif

# Program the device.
.PHONY: program
program: $(TARGET)
	$(GDB) -batch -x $(SCRIPTS)/program.gdb $<

# Reset the device.
.PHONY: reset
reset:
	$(GDB) -batch -x $(SCRIPTS)/reset.gdb $(TARGET)

# Enable booting from flash.
.PHONY: bootflash
bootflash:
	$(GDB) -batch -x $(SCRIPTS)/bootflash.gdb

# Attach debugger.
.PHONY: debug
debug:
	$(GDB)  -x $(SCRIPTS)/debug.gdb $(TARGET)
