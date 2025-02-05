#ifndef HOJA_INCLUDES_H
#define HOJA_INCLUDES_H

#define HOJA_BASEBAND_VERSION 0xA022
#define HOJA_MAGIC_NUM 0x83FD
#define HOJA_SETTINGS_NAMESPACE "hsettings"

#include "hoja_types.h"
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <assert.h>
#include <stdint.h>

#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_random.h"
#include "esp_mac.h"
#include "esp_attr.h"
#include "esp_ipc.h"

#include "esp_hid_common.h"
#include "esp_hidd.h"

#include "esp_hidd_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_gap_bt_api.h"
#include "esp_gap_ble_api.h"
#include "esp_timer.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "esp_hid_gap.h"
#include "esp_log.h"
#include "esp_err.h"

#include "soc/i2c_struct.h"
//#include "driver/i2c.h"
#include "mitch_i2c.h"
//#include "driver/i2c_slave.h" BAD

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "imu_tool.h"
#include "interval.h"

typedef enum
{
    INPUT_MODE_LOAD     = -1,
    INPUT_MODE_SWPRO    = 0,
    INPUT_MODE_XINPUT   = 1,
    INPUT_MODE_GCUSB    = 2,
    INPUT_MODE_GAMECUBE = 3,
    INPUT_MODE_N64      = 4,
    INPUT_MODE_SNES     = 5,
    INPUT_MODE_DS4      = 6,
    INPUT_MODE_MAX,
} input_mode_t;

typedef struct
{
    // Buttons
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

    // Buttons system
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

    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t gx;
    int16_t gy;
    int16_t gz;
} __attribute__ ((packed)) i2cinput_input_s;

#include "hoja.h"

#include "switch_analog.h"
#include "switch_commands.h"
#include "switch_spi.h"

#include "core_bt_xinput.h"
#include "core_bt_switch.h"
#include "core_bt_gamecube.h"
#include "core_bt_ds4.h"
#include "util_bt_hid.h"

#include "rsc_descriptors.h"



#endif 