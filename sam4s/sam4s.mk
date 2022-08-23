# -g3 saves macro information
CFLAGS += -mcpu=cortex-m4 -Wall -Wstrict-prototypes -W -g3 -D$(RUN_MODE) $(sort $(INCLUDES)) \
$(OPT) -mthumb -D__$(MCU)__ -D__SAM4S__ -Wno-unused -Wno-unused-parameter -DARM_MATH_CM4=true

CXXFLAGS += -mcpu=cortex-m4 -Wall -W -g3 -D$(RUN_MODE) $(sort $(INCLUDES)) \
$(OPT) -mthumb -D__$(MCU)__ -D__SAM4S__ -Wno-unused -Wno-unused-parameter -DARM_MATH_CM4=true -Wno-missing-field-initializers

LDFLAGS += -mthumb -mcpu=cortex-m4

INCLUDES += -I$(MAT91LIB_DIR)/$(FAMILY) -I$(MAT91LIB_DIR)/$(FAMILY)/atmel

VPATH += $(MAT91LIB_DIR)/$(FAMILY)

SRC += mcu_sleep.c


