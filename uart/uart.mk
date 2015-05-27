UART_DIR = $(MAT91LIB_DIR)/uart

VPATH += $(UART_DIR)
INCLUDES += -I$(UART_DIR)

SRC += uart.c uart0.c uart1.c

