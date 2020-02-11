CADC_DIR = $(MAT91LIB_DIR)/cadc

VPATH += $(CADC_DIR)
INCLUDES += -I$(CADC_DIR)

SRC += cadc.c

include $(MAT91LIB_DIR)/adc/adc.mk
include $(MAT91LIB_DIR)/tc/tc.mk
include $(MAT91LIB_DIR)/pdc/pdc.mk


