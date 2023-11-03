#ifndef HOJA_H
#define HOJA_H

#include "hoja_includes.h"

typedef struct
{
    // Mac address of this device
    uint8_t device_mac[6];

    // Mac address of the device we are connected to
    uint8_t switch_host_mac[6];

    // Mac address of the device we are paired to
    uint8_t paired_host_mac[6];
} hoja_settings_s;

extern hoja_settings_s global_loaded_settings;

typedef union
  {
    struct
    {
      uint8_t connection  : 4; // 1 is charging, 0 is not connected
      uint8_t bat_lvl     : 4; // 0-8 battery level
    };
    uint8_t bat_status;
  } switch_battery_status_u;

  typedef struct
{
    uint8_t rumble_intensity; // 0-100 value representing rumble intensity
    uint8_t connected_status; // Value representing if the BT is connected
} i2cinput_status_s;

  typedef struct
{
    union
    {
        struct
        {
            // Y and C-Up (N64)
            uint8_t b_y       : 1;

            // X and C-Left (N64)
            uint8_t b_x       : 1;

            uint8_t b_b       : 1;
            uint8_t b_a       : 1;
            uint8_t t_r_sr    : 1;
            uint8_t t_r_sl    : 1;
            uint8_t t_r       : 1;

            // ZR and C-Down (N64)
            uint8_t t_zr      : 1;
        };
        uint8_t right_buttons;
    };
    union
    {
        struct
        {
            // Minus and C-Right (N64)
            uint8_t b_minus     : 1;

            // Plus and Start
            uint8_t b_plus      : 1;

            uint8_t sb_right    : 1;
            uint8_t sb_left     : 1;
            uint8_t b_home      : 1;
            uint8_t b_capture   : 1;
            uint8_t none        : 1;
            uint8_t charge_grip_active : 1;
        };
        uint8_t shared_buttons;
    };
    union
    {
        struct
        {
            uint8_t d_down    : 1;
            uint8_t d_up      : 1;
            uint8_t d_right   : 1;
            uint8_t d_left    : 1;
            uint8_t t_l_sr    : 1;
            uint8_t t_l_sl    : 1;
            uint8_t t_l       : 1;

            // ZL and Z (N64)
            uint8_t t_zl      : 1;

        };
        uint8_t left_buttons;
    };

    uint16_t ls_x;
    uint16_t ls_y;
    uint16_t rs_x;
    uint16_t rs_y;

    uint16_t ax;
    uint16_t ay;
    uint16_t az;
    uint16_t gx;
    uint16_t gy;
    uint16_t gz;

} __attribute__ ((packed)) sw_input_s;

void app_set_rumble(uint8_t intensity);
bool app_compare_mac(uint8_t *mac_1, uint8_t *mac_2);

void app_save_host_mac();

#endif