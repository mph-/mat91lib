SSC_DIR = $(MAT91LIB_DIR)/ssc

VPATH += $(SSC_DIR)
INCLUDES += -I$(SSC_DIR) -I$(SSC_DIR)/../../mmculib/ring/

SRC += ssc.c ssc_dma.c bssc.c ring.c


