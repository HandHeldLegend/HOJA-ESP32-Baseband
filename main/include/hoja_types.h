#ifndef HOJA_TYPES_H
#define HOJA_TYPES_H

#include "stdint.h"
#include "stdbool.h"

typedef struct 
{
  uint32_t this_time;
  uint32_t last_time;
} interval_s;

// IMU data structure
typedef struct
{
    union
    {
        struct
        {
            uint8_t ax_8l : 8;
            uint8_t ax_8h : 8;
        };
        int16_t ax;
    };

    union
    {
        struct
        {
            uint8_t ay_8l : 8;
            uint8_t ay_8h : 8;
        };
        int16_t ay;
    };
    
    union
    {
        struct
        {
            uint8_t az_8l : 8;
            uint8_t az_8h : 8;
        };
        int16_t az;
    };
    
    union
    {
        struct
        {
            uint8_t gx_8l : 8;
            uint8_t gx_8h : 8;
        };
        int16_t gx;
    };

    union
    {
        struct
        {
            uint8_t gy_8l : 8;
            uint8_t gy_8h : 8;
        };
        int16_t gy;
    };
    
    union
    {
        struct
        {
            uint8_t gz_8l : 8;
            uint8_t gz_8h : 8;
        };
        int16_t gz;
    };
    
    bool retrieved;
} imu_data_s;

typedef struct
{
    uint16_t magic;

    // Mac address of this device
    uint8_t device_mac_switch[6];

    uint8_t device_mac_gamecube[6];

    uint8_t device_mac_xinput[6];

    // Mac address of the Switch we are paired to
    uint8_t paired_host_switch_mac[6];

    // Mac address of the GameCube we are paired to
    uint8_t paired_host_gamecube_mac[6];

    // Mac address of the XInput we are paired to
    uint8_t paired_host_xinput_mac[6];
} hoja_settings_s;

typedef struct 
{
    uint8_t rgb_gripl[3];
    uint8_t rgb_gripr[3];
    uint8_t rgb_body[3];
    uint8_t rgb_buttons[3];
} hoja_live_s;

extern hoja_settings_s global_loaded_settings;
extern hoja_live_s global_live_data;

typedef union
{
    struct
    {
        uint8_t connection  : 4; // 1 is charging, 0 is not connected
        uint8_t bat_lvl     : 4; // 0-8 battery level
    };
    uint8_t bat_status;
} switch_battery_status_u;

// Status return data types
typedef enum
{
    I2C_STATUS_NULL, // Nothing to report
    I2C_STATUS_HAPTIC_STANDARD, // Standard haptic data
    I2C_STATUS_HAPTIC_SWITCH, // Nintendo Switch haptic data
    I2C_STATUS_FIRMWARE_VERSION, // Report fw version
    I2C_STATUS_CONNECTED_STATUS, // Connected status change
    I2C_STATUS_POWER_CODE, // Change power setting
    I2C_STATUS_MAC_UPDATE, // Update HOST save MAC address
} i2cinput_status_t;

typedef enum
{
    I2C_CMD_STANDARD = 0xFF, // Regular input data
    I2C_CMD_MOTION = 0xFC, // Motion data
    I2C_CMD_START = 0xFE, // Launch BT Mode with parameter
    I2C_CMD_FIRMWARE_VERSION = 0xFD, // Retrieve the firmware version
} i2cinput_cmd_t;

typedef struct
{
    uint8_t cmd;
    uint32_t rand_seed; // Random data to help our CRC
    uint8_t data[10]; // Buffer for related data   
} __attribute__ ((packed)) i2cinput_status_s;

typedef struct
{
    uint8_t haptic_data[11]; // Value representing haptic information
} __attribute__ ((packed)) i2cinput_switch_haptic_s;

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

} __attribute__ ((packed)) sw_input_s;

typedef struct
{
    union
    {
        float raw[4];
        struct {
            float x;
            float y;
            float z;
            float w;
        };
    };
    uint32_t timestamp;
    int16_t ax;
    int16_t ay;
    int16_t az;
} quaternion_s;

typedef struct{
    int16_t y;
    int16_t x;
    int16_t z;
} vector_s;

typedef struct {
    vector_s accel_0;
    uint32_t mode : 2;
    uint32_t max_index : 2;
    uint32_t last_sample_0 : 21;
    uint32_t last_sample_1l : 7;
    uint16_t last_sample_1h : 14;
    uint16_t last_sample_2l : 2;
    vector_s accel_1;
    uint32_t last_sample_2h : 19;
    uint32_t delta_last_first_0 : 13;
    uint16_t delta_last_first_1 : 13;
    uint16_t delta_last_first_2l : 3;
    vector_s accel_2;
    uint32_t delta_last_first_2h : 10;
    uint32_t delta_mid_avg_0 : 7;
    uint32_t delta_mid_avg_1 : 7;
    uint32_t delta_mid_avg_2 : 7;
    uint32_t timestamp_start_l : 1;
    uint16_t timestamp_start_h : 10;
    uint16_t timestamp_count : 6;
} __attribute__ ((packed)) mode_2_s;

typedef struct
{
    float hi_freq_linear;
    float hi_amp_linear;
    float lo_freq_linear;
    float lo_amp_linear;
} __attribute__((packed)) hoja_haptic_frame_linear_s;

typedef struct
{
    float   high_frequency;
    float   high_amplitude;
    float   low_frequency;
    float   low_amplitude;
} __attribute__ ((packed)) hoja_haptic_frame_s;

typedef struct
{
    uint8_t sample_count;
    bool unread;
    hoja_haptic_frame_linear_s linear; // Last known state 
    hoja_haptic_frame_s samples[3];
} __attribute__ ((packed)) hoja_rumble_msg_s;

typedef struct {
    //uint8_t report_id Should be set to 0x01
    uint8_t left_x;        // Left joystick X axis
    uint8_t left_y;        // Left joystick Y axis
    uint8_t right_x;       // Right joystick X axis
    uint8_t right_y;       // Right joystick Y axis
    uint8_t left_trigger;  // Left analog trigger
    uint8_t right_trigger; // Right analog trigger
    struct {
        uint8_t a : 1;
        uint8_t b : 1;
        uint8_t x : 1;
        uint8_t y : 1;
        uint8_t l3 : 1;
        uint8_t r3 : 1;
        uint8_t l : 1; // Mirrored Z button/Switch L Button
        uint8_t r : 1; // GameCube Z Button/Switch R Button
    } buttons1;
    struct {
        uint8_t zl : 1; // GameCube L trigger 
        uint8_t zr : 1; // GameCube R trigger
        uint8_t start : 1;
        uint8_t select : 1;
        uint8_t home : 1;
        uint8_t capture : 1;
        uint8_t reserved : 2;  // Padding bits
    } buttons2;

    struct {
        uint8_t dpad : 4;     // D-pad as hat switch
        uint8_t padding : 4;  // Padding to complete the byte
    } dpad;

} gc_input_s;

typedef struct {
    // uint8_t report_id is 0x02
    struct {
        uint8_t rumble : 1;        // Rumble on/off
        uint8_t player_number : 3; // Player number (0-4)
        uint8_t padding : 4;
    } feedback;
} gc_output_s;

#endif