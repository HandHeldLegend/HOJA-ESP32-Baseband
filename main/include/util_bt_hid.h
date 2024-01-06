#ifndef UTIL_BT_HID_H
#define UTIL_BT_HID_H

#include "hoja_includes.h"

// XInput Hat Codes
typedef enum
{
  XI_HAT_TOP          = 0x01,
  XI_HAT_TOP_RIGHT    = 0x02,
  XI_HAT_RIGHT        = 0x03,
  XI_HAT_BOTTOM_RIGHT = 0x04,
  XI_HAT_BOTTOM       = 0x05,
  XI_HAT_BOTTOM_LEFT  = 0x06,
  XI_HAT_LEFT         = 0x07,
  XI_HAT_TOP_LEFT     = 0x08,
  XI_HAT_CENTER       = 0x00,
} xi_input_hat_dir_t;

typedef enum
{
    UTIL_BT_HID_STATUS_IDLE,
    UTIL_BT_HID_STATUS_INITIALIZED,
    UTIL_BT_HID_STATUS_RUNNING,
    UTIL_BT_HID_STATUS_CONNECTED,
    UTIL_BT_HID_STATUS_PAIRING,
    UTIL_BT_HID_STATUS_MAX
} util_bt_hid_status_t;

typedef enum
{
    UTIL_BT_MODE_CLASSIC,
    UTIL_BT_MODE_BLE,
} util_bt_hid_mode_t;

typedef struct
{
    // CB for BTC
    esp_event_handler_t hidd_cb;
    esp_bt_gap_cb_t gap_cb;

    // CB for BLE
    esp_event_handler_t ble_hidd_cb;
    esp_gap_ble_cb_t ble_gap_cb;
    
    esp_hidd_dev_t *hid_dev;
    esp_bt_mode_t bt_mode;
    uint8_t *uuid128;
    int appearance;
} util_bt_app_params_s;

// Public variables
extern util_bt_hid_status_t util_bt_hid_status;
extern util_bt_hid_mode_t util_bt_hid_mode;

// App params external
extern util_bt_app_params_s switch_app_params;

// Public functions

void util_bt_set_paired(bool paired, uint8_t *host);

bool util_bt_get_paired(void);

int util_bluetooth_init(uint8_t *mac_address);

int util_bluetooth_register_app(util_bt_app_params_s *util_bt_app_params, esp_hid_device_config_t *hidd_device_config);

void util_bluetooth_deinit(void);

void util_bluetooth_connect(void);

#endif