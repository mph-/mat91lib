include $(foreach peripheral, $(PERIPHERALS), $(MAT91LIB_DIR)/$(peripheral)/$(peripheral).mk)

# Perform second pass for that peripherals that depend on other peripherals
include $(foreach peripheral, $(PERIPHERALS), $(MAT91LIB_DIR)/$(peripheral)/$(peripheral).mk)

VPATH += $(MAT91LIB_DIR)
INCLUDES += -I$(MAT91LIB_DIR)

