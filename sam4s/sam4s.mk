CFLAGS += -mcpu=cortex-m4 -Wall -Wstrict-prototypes -W -gdwarf-2 -D$(RUN_MODE) $(sort $(INCLUDES)) $(OPT) -mthumb -mthumb-interwork -D__$(MCU)__ -D__SAM4S__

LDFLAGS += -mthumb-interwork -nostartfiles -lm -lc

INCLUDES += -I$(MAT91LIB_DIR)/$(FAMILY) -I$(MAT91LIB_DIR)/$(FAMILY)/atmel

VPATH += $(MAT91LIB_DIR)/$(FAMILY)
