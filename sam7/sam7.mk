CFLAGS += -mcpu=arm7tdmi -Wall -Wstrict-prototypes -W -gdwarf-2 -D$(RUN_MODE) $(sort $(INCLUDES)) $(OPT) -mthumb-interwork -D__$(MCU)__ -D__SAM7__ -Wno-unused

LDFLAGS += -mthumb-interwork -nostartfiles -lm -lc

# This cannot be compiled in thumb mode.
objs/crt0.o: crt0.c config.h target.h
	mkdir -p objs
	$(CC) -c $(CFLAGS) -O2 $< -o $@

INCLUDES += -I$(MAT91LIB_DIR)/$(FAMILY) 

VPATH += $(MAT91LIB_DIR)/$(FAMILY)
