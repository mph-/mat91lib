USART_DIR = $(MAT91LIB_DIR)/usart

VPATH += $(USART_DIR)
INCLUDES += -I$(USART_DIR)

SRC += usart.c usart0.c usart1.c
