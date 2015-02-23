#include "pio.h"

typedef enum 
{
    MCU_SLEEP_MODE_BACKUP,
    MCU_SLEEP_MODE_WAIT,
    MCU_SLEEP_MODE_SLEEP
} mcu_sleep_mode_t;


typedef struct mcu_sleep_cfg_struct
{
    mcu_sleep_mode_t mode;
    pio_t pio;
    bool active_high;
} mcu_sleep_mode_cfg;


void
mcu_sleep (mcu_sleep_mode_cfg_t *cfg);


