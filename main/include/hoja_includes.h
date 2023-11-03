#ifndef HOJA_INCLUDES_H
#define HOJA_INCLUDES_H

#include <string.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <assert.h>
#include <stdint.h>

#include "hoja.h"

typedef enum
{
    INPUT_MODE_LOAD     = -1,
    INPUT_MODE_SWPRO    = 0,
    INPUT_MODE_XINPUT   = 1,
    INPUT_MODE_GAMECUBE = 2,
    INPUT_MODE_N64      = 3,
    INPUT_MODE_SNES     = 4,
    INPUT_MODE_GCUSB    = 5,
} input_mode_t;

typedef enum
{
    I2CINPUT_ID_INIT    = 0xF2,
    I2CINPUT_ID_INPUT   = 0x01,
    I2CINPUT_ID_STATUS  = 0xF4,
    I2CINPUT_ID_SAVEMAC = 0xF3,
    I2CINPUT_ID_STOP    = 0x02,
} i2cinput_id_t;

typedef struct
{
    union
    {
        struct
        {
            // D-Pad
            uint8_t dpad_up     : 1;
            uint8_t dpad_down   : 1;
            uint8_t dpad_left   : 1;
            uint8_t dpad_right  : 1;
            // Buttons
            uint8_t button_a    : 1;
            uint8_t button_b    : 1;
            uint8_t button_x    : 1;
            uint8_t button_y    : 1;

            // Triggers
            uint8_t trigger_l   : 1;
            uint8_t trigger_zl  : 1;
            uint8_t trigger_r   : 1;
            uint8_t trigger_zr  : 1;

            // Special Functions
            uint8_t button_plus     : 1;
            uint8_t button_minus    : 1;

            // Stick clicks
            uint8_t button_stick_left   : 1;
            uint8_t button_stick_right  : 1;
        };
        uint16_t buttons_all;
    };

    union
    {
        struct
        {
            // Menu buttons (Not remappable by API)
            uint8_t button_capture  : 1;
            uint8_t button_home     : 1;
            uint8_t button_safemode : 1;
            uint8_t padding         : 5;
        };
        uint8_t buttons_system;
    };

    uint16_t lx;
    uint16_t ly;
    uint16_t rx;
    uint16_t ry;
    uint16_t lt;
    uint16_t rt;

    uint16_t ax;
    uint16_t ay;
    uint16_t az;
    uint16_t gx;
    uint16_t gy;
    uint16_t gz;
    
} i2cinput_input_s;

#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_mac.h"

#include "switch_analog.h"
#include "switch_commands.h"
#include "switch_spi.h"

// Bluetooth stuff

#include "esp_hid_common.h"
#include "esp_hidd.h"

#include "esp_hidd_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_gap_bt_api.h"
#include "esp_gap_ble_api.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "core_bt_xinput.h"
#include "core_bt_switch.h"
#include "util_bt_hid.h"

#include "rsc_descriptors.h"

#include "esp_hid_gap.h"
#include "esp_log.h"
#include "esp_err.h"

#include "driver/i2c.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"


#endif 