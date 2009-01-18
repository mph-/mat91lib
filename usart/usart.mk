USART_DIR = $(PERIPHERAL_DIR)/usart

VPATH += $(USART_DIR)
INCLUDES += -I$(USART_DIR)

SRC += usart.c usart0.c usart1.c
