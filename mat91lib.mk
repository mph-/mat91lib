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
OPT = -O1

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

ifndef CXXFLAGS
CXXFLAGS = $(CFLAGS)
endif

all: $(TARGET)

include $(MAT91LIB_DIR)/$(FAMILY)/$(FAMILY).mk

SCRIPTS = $(MAT91LIB_DIR)/$(FAMILY)/scripts
LDSCRIPTS = $(MAT91LIB_DIR)/$(FAMILY)


CC = $(TOOLCHAIN)-gcc
CXX = $(TOOLCHAIN)-g++
LD = $(TOOLCHAIN)-g++
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

CSRC = $(filter %.c, $(SRC))
#CCSRC = $(filter %.cc %.cpp, $(SRC))
CCSRC = $(filter %.cpp, $(SRC))

# Create list of object and dependency files.  Note, sort removes duplicates.
OBJS = $(addprefix objs/, $(notdir $(sort $(CSRC:.c=.o) $(CCSRC:.cpp=.o))))
DEPS = $(addprefix deps/, $(notdir $(sort $(CSRC:.c=.d) $(CCSRC:.cpp=.d))))
SRC_DIRS = $(sort $(dir $(SRC)))


VPATH += $(SRC_DIRS)

include $(MAT91LIB_DIR)/peripherals.mk

EXTRA_OBJS = objs/crt0.o objs/mcu.o


print-deps:
	@echo $(DEPS)

print-objs:
	@echo $(OBJS)

print-src-dirs:
	@echo $(SRC_DIRS)

objs:
	mkdir -p objs

deps:
	mkdir -p deps

objs/%.o: %.cpp Makefile
	$(CXX) -c $(CXXFLAGS) $< -o $@
# Generate dependencies to see if object file needs recompiling.
	$(CXX) -MM $(CXXFLAGS) $< > deps/$*.d

objs/%.o: %.c Makefile
	$(CC) -c $(CFLAGS) $< -o $@
# Generate dependencies to see if object file needs recompiling.
	$(CC) -MM $(CFLAGS) $< > deps/$*.d

# Link object files to form output file.
$(TARGET): deps objs $(OBJS) $(EXTRA_OBJS)
	$(LD) $(OBJS) $(EXTRA_OBJS) $(LDFLAGS) -o $@ -lm -Wl,-Map=$(TARGET_MAP),--cref
	$(SIZE) $@

# Include the dependency files.
-include $(DEPS)


# Remove the objs directory
clean-objs:
	-$(DEL) -fr objs

# Rebuild the code, don't delete dependencies.
rebuild: clean-objs $(TARGET)

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

