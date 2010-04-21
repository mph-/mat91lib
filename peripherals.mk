PERIPHERAL_DIR = $(ARCH_DIR)

include $(foreach peripheral, $(PERIPHERALS), $(ARCH_DIR)/$(peripheral)/$(peripheral).mk)

# Perform second pass for that peripherals that depend on other peripherals
include $(foreach peripheral, $(PERIPHERALS), $(ARCH_DIR)/$(peripheral)/$(peripheral).mk)

VPATH += $(ARCH_DIR)
INCLUDES += -I$(ARCH_DIR)

