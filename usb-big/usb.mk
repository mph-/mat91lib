USB_DIR = $(PERIPHERAL_DIR)/usb

VPATH += $(USB_DIR)
INCLUDES += -I$(USB_DIR)

SRC += usb_cdc.c


VPATH += $(USB_DIR)/device/cdc-serial
VPATH += $(USB_DIR)/device/core
VPATH += $(USB_DIR)/common/core
VPATH += $(USB_DIR)/common/cdc

SRC += USBD_OTGHS.c USBD_UDP.c USBD_UDPHS.c USBDDriver.c
SRC += USBDCallbacks_Initialized.c
SRC += USBDCallbacks_Reset.c
SRC += USBDDriverCb_CfgChanged.c
SRC += USBDDriverCb_IfSettingChanged.c
SRC += USBSetAddressRequest.c USBGenericDescriptor.c USBInterfaceRequest.c
SRC += USBGenericRequest.c USBGetDescriptorRequest.c 
SRC += USBSetConfigurationRequest.c USBFeatureRequest.c
SRC += USBEndpointDescriptor.c USBConfigurationDescriptor.c
SRC += CDCDSerialDriver.c CDCDSerialDriverDescriptors.c
SRC += CDCSetControlLineStateRequest.c CDCLineCoding.c

INCLUDES += -I$(USB_DIR)/device/cdc-serial
INCLUDES += -I$(USB_DIR)/device/core
INCLUDES += -I$(USB_DIR)/common/core
INCLUDES += -I$(USB_DIR)/common/cdc

VPATH += $(USB_DIR)/utility
VPATH += $(USB_DIR)/peripherals/aic
VPATH += $(USB_DIR)/peripherals/pio

INCLUDES += -I$(USB_DIR)/peripherals
INCLUDES += -I$(USB_DIR)/utility

SRC += led.c pio.c aic.c

TRACE_LEVEL = 0

