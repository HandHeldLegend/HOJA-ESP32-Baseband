#include "core_bt_xinput.h"
#include "esp_log.h"

#define XI_HID_LEN 16
#define XI_INPUT_REPORT_ID 0x01

/* Thanks to https://github.com/Mystfit/ESP32-BLE-CompositeHID */
#define XINPUT_INPUT_REPORT_ID 0x01
#define XINPUT_EXTRA_INPUT_REPORT_ID 0x02
#define XINPUT_OUTPUT_REPORT_ID 0x03
#define XINPUT_EXTRA_OUTPUT_REPORT_ID 0x04

typedef struct
{
    uint16_t stick_left_x : 16;
    uint16_t stick_left_y : 16;
    uint16_t stick_right_x : 16;
    uint16_t stick_right_y : 16;

    uint16_t analog_trigger_l : 16;
    uint16_t analog_trigger_r : 16;

    uint8_t dpad_hat : 4;
    uint8_t dpad_padding : 4;

    union
    {
        struct
        {
            uint8_t button_a : 1;
            uint8_t button_b : 1;
            uint8_t padding_1 : 1;
            uint8_t button_x : 1;
            uint8_t button_y : 1;
            uint8_t padding_2 : 1;
            uint8_t bumper_l : 1;
            uint8_t bumper_r : 1;
        } __attribute__((packed));
        uint8_t buttons_1 : 8;
    };

    union
    {
        struct
        {
            uint8_t padding_3 : 2;
            uint8_t button_back : 1;
            uint8_t button_menu : 1;
            uint8_t button_guide : 1;
            uint8_t button_stick_l : 1;
            uint8_t button_stick_r : 1;
            uint8_t padding_4 : 1;
        } __attribute__((packed));
        uint8_t buttons_2 : 8;
    };

    uint8_t buttons_blank;

} __attribute__((packed)) xinput_input_report_s;

// Input Report 0x02
typedef struct
{
    uint8_t home_button : 1; // AC Home button (Consumer page)
    uint8_t padding : 7;     // Constant padding bits
} __attribute__((packed)) xinput_consumer_input_report_s;

// Output Report 0x03
typedef struct
{
    uint8_t enable : 4;   // Force feedback enable (0x97 usage)
    uint8_t padding1 : 4; // Constant padding bits

    uint8_t magnitude[4]; // Force feedback magnitude values (0x70 usage) - 0-100 each
    uint8_t duration;     // Duration in centiseconds (0x50 usage) - 0-255
    uint8_t start_delay;  // Start delay (0xA7 usage) - 0-255
    uint8_t loop_count;   // Loop count (0x7C usage) - 0-255
} __attribute__((packed)) xinput_force_feedback_report_s;

// Input report 0x04
typedef struct
{
    uint8_t battery_level; // Battery strength 0-255 (0x00-0xFF)
} __attribute__((packed)) xinput_battery_report_s;


const uint8_t xinput_hid_report_descriptor_broke[334] = {
    0x05, 0x01,                   // Usage Page (Generic Desktop Ctrls)
    0x09, 0x05,                   // Usage (Game Pad)
    0xA1, 0x01,                   // Collection (Application)
    0x85, 0x01,                   //   Report ID (1)
    0x09, 0x01,                   //   Usage (Pointer)
    0xA1, 0x00,                   //   Collection (Physical)
    0x09, 0x30,                   //     Usage (X)
    0x09, 0x31,                   //     Usage (Y)
    0x15, 0x00,                   //     Logical Minimum (0)
    0x27, 0xFF, 0xFF, 0x00, 0x00, //     Logical Maximum (65534)
    0x95, 0x02,                   //     Report Count (2)
    0x75, 0x10,                   //     Report Size (16)
    0x81, 0x02,                   //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,                         //   End Collection
    0x09, 0x01,                   //   Usage (Pointer)
    0xA1, 0x00,                   //   Collection (Physical)
    0x09, 0x32,                   //     Usage (Z)
    0x09, 0x35,                   //     Usage (Rz)
    0x15, 0x00,                   //     Logical Minimum (0)
    0x27, 0xFF, 0xFF, 0x00, 0x00, //     Logical Maximum (65534)
    0x95, 0x02,                   //     Report Count (2)
    0x75, 0x10,                   //     Report Size (16)
    0x81, 0x02,                   //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,                         //   End Collection

    0x05, 0x02,       //   Usage Page (Sim Ctrls)
    0x09, 0xC5,       //   Usage (Brake)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0x03, //   Logical Maximum (1023)
    0x95, 0x01,       //   Report Count (1)
    0x75, 0x0A,       //   Report Size (10)
    0x81, 0x02,       //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x00,       //   Logical Maximum (0)
    0x75, 0x06,       //   Report Size (6)
    0x95, 0x01,       //   Report Count (1)
    0x81, 0x03,       //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x05, 0x02,       //   Usage Page (Sim Ctrls)
    0x09, 0xC4,       //   Usage (Accelerator)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0x03, //   Logical Maximum (1023)
    0x95, 0x01,       //   Report Count (1)
    0x75, 0x0A,       //   Report Size (10)
    0x81, 0x02,       //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x00,       //   Logical Maximum (0)
    0x75, 0x06,       //   Report Size (6)
    0x95, 0x01,       //   Report Count (1)
    0x81, 0x03,       //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    
    0x05, 0x01,       //   Usage Page (Generic Desktop Ctrls)
    0x09, 0x39,       //   Usage (Hat switch)
    0x15, 0x01,       //   Logical Minimum (1)
    0x25, 0x08,       //   Logical Maximum (8)
    0x35, 0x00,       //   Physical Minimum (0)
    0x46, 0x3B, 0x01, //   Physical Maximum (315)
    0x66, 0x14, 0x00, //   Unit (System: English Rotation, Length: Centimeter)
    0x75, 0x04,       //   Report Size (4)
    0x95, 0x01,       //   Report Count (1)
    0x81, 0x42,       //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,Null State)

    0x75, 0x04,       //   Report Size (4)
    0x95, 0x01,       //   Report Count (1)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x00,       //   Logical Maximum (0)
    0x35, 0x00,       //   Physical Minimum (0)
    0x45, 0x00,       //   Physical Maximum (0)
    0x65, 0x00,       //   Unit (None)
    0x81, 0x03,       //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    
    0x05, 0x09,       //   Usage Page (Button)
    0x19, 0x01,       //   Usage Minimum (0x01)
    0x29, 0x0F,       //   Usage Maximum (0x0F)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x01,       //   Logical Maximum (1)
    0x75, 0x01,       //   Report Size (1)
    0x95, 0x0F,       //   Report Count (15)
    0x81, 0x02,       //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x00,       //   Logical Maximum (0)
    0x75, 0x01,       //   Report Size (1)
    0x95, 0x01,       //   Report Count (1)
    0x81, 0x03,       //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    
    0x05, 0x0C,       //   Usage Page (Consumer)
    0x0A, 0x24, 0x02, //   Usage (AC Back)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x01,       //   Logical Maximum (1)
    0x95, 0x01,       //   Report Count (1)
    0x75, 0x01,       //   Report Size (1)
    0x81, 0x02,       //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x00,       //   Logical Maximum (0)
    0x75, 0x07,       //   Report Size (7)
    0x95, 0x01,       //   Report Count (1)
    0x81, 0x03,       //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    

    0x05, 0x0C,       //   Usage Page (Consumer)
    0x09, 0x01,       //   Usage (Consumer Control)


    0x85, 0x02,       //   Report ID (2)
    0xA1, 0x01,       //   Collection (Application)
    0x05, 0x0C,       //     Usage Page (Consumer)
    0x0A, 0x23, 0x02, //     Usage (AC Home)
    0x15, 0x00,       //     Logical Minimum (0)
    0x25, 0x01,       //     Logical Maximum (1)
    0x95, 0x01,       //     Report Count (1)
    0x75, 0x01,       //     Report Size (1)
    0x81, 0x02,       //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x15, 0x00,       //     Logical Minimum (0)
    0x25, 0x00,       //     Logical Maximum (0)
    0x75, 0x07,       //     Report Size (7)
    0x95, 0x01,       //     Report Count (1)
    0x81, 0x03,       //     Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    
    0xC0,             //   End Collection
    0x05, 0x0F,       //   Usage Page (PID Page)
    0x09, 0x21,       //   Usage (0x21)

    0x85, 0x03,       //   Report ID (3)
    0xA1, 0x02,       //   Collection (Logical)
    0x09, 0x97,       //     Usage (0x97)
    0x15, 0x00,       //     Logical Minimum (0)
    0x25, 0x01,       //     Logical Maximum (1)
    0x75, 0x04,       //     Report Size (4)
    0x95, 0x01,       //     Report Count (1)
    0x91, 0x02,       //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x15, 0x00,       //     Logical Minimum (0)
    0x25, 0x00,       //     Logical Maximum (0)
    0x75, 0x04,       //     Report Size (4)
    0x95, 0x01,       //     Report Count (1)
    0x91, 0x03,       //     Output (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x09, 0x70,       //     Usage (0x70)
    0x15, 0x00,       //     Logical Minimum (0)
    0x25, 0x64,       //     Logical Maximum (100)
    0x75, 0x08,       //     Report Size (8)
    0x95, 0x04,       //     Report Count (4)
    0x91, 0x02,       //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x09, 0x50,       //     Usage (0x50)
    0x66, 0x01, 0x10, //     Unit (System: SI Linear, Time: Seconds)
    0x55, 0x0E,       //     Unit Exponent (-2)
    0x15, 0x00,       //     Logical Minimum (0)
    0x26, 0xFF, 0x00, //     Logical Maximum (255)
    0x75, 0x08,       //     Report Size (8)
    0x95, 0x01,       //     Report Count (1)
    0x91, 0x02,       //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x09, 0xA7,       //     Usage (0xA7)
    0x15, 0x00,       //     Logical Minimum (0)
    0x26, 0xFF, 0x00, //     Logical Maximum (255)
    0x75, 0x08,       //     Report Size (8)
    0x95, 0x01,       //     Report Count (1)
    0x91, 0x02,       //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x65, 0x00,       //     Unit (None)
    0x55, 0x00,       //     Unit Exponent (0)
    0x09, 0x7C,       //     Usage (0x7C)
    0x15, 0x00,       //     Logical Minimum (0)
    0x26, 0xFF, 0x00, //     Logical Maximum (255)
    0x75, 0x08,       //     Report Size (8)
    0x95, 0x01,       //     Report Count (1)
    0x91, 0x02,       //     Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0xC0,             //   End Collection

    0x05, 0x06,       //   Usage Page (Generic Dev Ctrls)
    0x09, 0x20,       //   Usage (Battery Strength)
    0x85, 0x04,       //   Report ID (4)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x75, 0x08,       //   Report Size (8)
    0x95, 0x01,       //   Report Count (1)
    0x81, 0x02,       //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,             // End Collection

    // 334 bytes
};

#define XINPUT_HID_REPORT_MAP_LEN 166
const uint8_t xinput_hid_report_descriptor[166] = {
    0x05, 0x01, // Usage Page (Generic Desktop Ctrls)
    0x09, 0x05, // Usage (Gamepad)
    0xA1, 0x01, // Collection (Application)

    // Byte 0
    0x85, 0x01, //   Report ID (1)

    // Byte 1-8
    // --- Analog sticks ---
    0x05, 0x01, // Usage Page (Generic Desktop)
    // Left Stick X
    0x09, 0x30, //   Usage (X)
    // Left Stick Y
    0x09, 0x31, //   Usage (Y)
    // Right Stick X
    0x09, 0x33, //   Usage (Rx)
    // Right Stick Y
    0x09, 0x34, //   Usage (Ry)
    0x15, 0x00,                   //     Logical Minimum (0)
    0x27, 0xFF, 0xFF, 0x00, 0x00, //     Logical Maximum (65534)
    0x95, 0x04,                   //     Report Count (4)
    0x75, 0x10,                   //     Report Size (16)
    0x81, 0x02,                   //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

    // Byte 9-12
    // --- Analog Triggers ---
    0x05, 0x01, // Usage Page (Generic Desktop)
    // Left Trigger
    0x09, 0x32, //   Usage (Z)
    // Right Trigger
    0x09, 0x35, //   Usage (Rz)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0x03, //   Logical Maximum (1023)
    0x95, 0x02,       //   Report Count (2)
    0x75, 0x10,       //   Report Size (16)
    0x81, 0x02,       //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

    // Byte 13
    // D‑pad (Hat switch), 4 bits + 4 bits padding
    0x05, 0x01, // Usage Page (Generic Desktop)
    0x09, 0x39, // Usage (Hat switch)
    0x15, 0x00, //   Logical Minimum (0)
    0x25, 0x07, //   Logical Maximum (7)  – eight directions
    0x75, 0x04, //   Report Size (4 bits)
    0x95, 0x01, //   Report Count (1 field)
    0x81, 0x42, //   Input (Data,Var,Abs,Null)

    // Padding nibble to fill the byte
    0x75, 0x04, //   Report Size (4 bits)
    0x95, 0x01, //   Report Count (1 field)
    0x81, 0x03, //   Input (Const,Var,Abs)

    // Byte 14-15
    // --- 15 buttons ---
    0x05, 0x09,        // Usage Page (Button)
    0x19, 0x01,        //   Usage Minimum (Button 1)
    0x29, 0x0F,        //   Usage Maximum (Button 15)
    0x15, 0x00,        //   Logical Min (0)
    0x25, 0x01,        //   Logical Max (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x0F,        //   Report Count(15)
    0x81, 0x02,        //   Input (Data,Var,Abs)

    // 1 bit padding
    0x75, 0x01, //   Report Size (1 bit)
    0x95, 0x01, //   Report Count (1 field)
    0x81, 0x03, //   Input (Const,Var,Abs)

    // 1 bit button
    0x05, 0x09,        // Usage Page (Button)
    0x19, 0x01,        //   Usage Minimum (Button 1)
    0x29, 0x01,        //   Usage Maximum (Button 1)
    0x15, 0x00,        //   Logical Min (0)
    0x25, 0x01,        //   Logical Max (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x01,        //   Report Count(1)
    0x81, 0x02,        //   Input (Data,Var,Abs)

    // 7 bits padding
    0x75, 0x07, //   Report Size (7 bit)
    0x95, 0x01, //   Report Count (1 field)
    0x81, 0x03, //   Input (Const,Var,Abs)

    // Other reports
    0x06, 0x00, 0xFF,              // Usage Page (Vendor Defined)
    0x09, 0x20,                    //   Usage (Vendor Usage 0x20)
    
    0x06, 0x00, 0xFF,   // Usage Page (Vendor Defined)

    0x85, 0x02,         //   Report ID (2)
    0x09, 0x20,         //   Usage (Vendor Usage 0x20)
    0x15, 0x00,         //   Logical Minimum (0)
    0x26, 0xFF, 0x00,   //   Logical Maximum (255)
    0x75, 0x08,         //   Report Size (8 bits)
    0x95, 0x01,         //   Report Count (1)
    0x81, 0x02,         //   Input (Data,Var,Abs)

    0x85, 0x03,         //   Report ID (3)
    0x09, 0x21,         //   Usage (Vendor Usage 0x21)
    0x15, 0x00,         //   Logical Minimum (0)
    0x26, 0xFF, 0x00,   //   Logical Maximum (255)
    0x75, 0x08,         //   Report Size (8 bits)
    0x95, 0x01,         //   Report Count (8)
    0x91, 0x02,         //   Output (Data,Var,Abs)

    0x85, 0x04,         //   Report ID (2)
    0x09, 0x20,         //   Usage (Vendor Usage 0x20)
    0x15, 0x00,         //   Logical Minimum (0)
    0x26, 0xFF, 0x00,   //   Logical Maximum (255)
    0x75, 0x08,         //   Report Size (8 bits)
    0x95, 0x01,         //   Report Count (1)
    0x81, 0x02,         //   Input (Data,Var,Abs)

    0xC0                           // End Collection 
};
#define XINPUT_DEFAULT_TICK_DELAY (8 / portTICK_PERIOD_MS)
#define XINPUT_DEFAULT_US_DELAY (8 * 1000)
static volatile bool _hid_connected = false;

static volatile bool _xinput_paired = false;

interval_s _xi_interval = {0};

void _xi_reset_report_spacer()
{
    uint64_t delay_time = app_get_report_timer();
    uint64_t timestamp = get_timestamp_us();
    interval_resettable_run(timestamp, delay_time, true, &_xi_interval);
}

bool _xi_send_check_nonblocking()
{
    uint64_t delay_time = app_get_report_timer();
    uint64_t timestamp = get_timestamp_us();
    return interval_run(timestamp, delay_time, &_xi_interval);
}

TaskHandle_t _xinput_bt_task_handle = NULL;

xinput_input_report_s _xinput_input_data = {0};

void _xinput_bt_task(void *parameters);

// XInput BTC GAP Event Callback
void xinput_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    const char *TAG = "xinput_bt_gap_cb";
    switch (event)
    {
    case ESP_BT_GAP_DISC_RES_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_DISC_RES_EVT");
        // esp_log_buffer_hex(TAG, param->disc_res.bda, ESP_BD_ADDR_LEN);
        break;
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_DISC_STATE_CHANGED_EVT");
        break;
    case ESP_BT_GAP_RMT_SRVCS_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_RMT_SRVCS_EVT");
        ESP_LOGI(TAG, "%d", param->rmt_srvcs.num_uuids);
        break;
    case ESP_BT_GAP_RMT_SRVC_REC_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_RMT_SRVC_REC_EVT");
        break;

    case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT:
        ESP_LOGI(TAG, "Setting to non-connectable, non-discoverable");
        esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
        ESP_LOGI(TAG, "ACL Connect Complete.");

        // Start input loop task
        if (_xinput_bt_task_handle == NULL)
        {
            xTaskCreatePinnedToCore(_xinput_bt_task,
                                    "XInput Send Task", 4048,
                                    NULL, 0, &_xinput_bt_task_handle, 0);
        }
        break;

    case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
        ESP_LOGI(TAG, "ACL Disconnect Complete.");

        if (_xinput_bt_task_handle != NULL)
        {
            vTaskDelete(_xinput_bt_task_handle);
            _xinput_bt_task_handle = NULL;
        }

        app_set_connected_status(0);

        if (_xinput_paired)
        {
            ESP_LOGI(TAG, "Setting to non-connectable, non-discoverable, then attempting connection.");
            esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            util_bluetooth_connect(global_loaded_settings.paired_host_xinput_mac);
        }
        else
        {
            ESP_LOGI(TAG, "Setting to connectable, discoverable.");
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        }

        break;

    case ESP_BT_GAP_AUTH_CMPL_EVT:
    {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGI(TAG, "authentication success: %s", param->auth_cmpl.device_name);
            // esp_log_buffer_hex(TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);

            if (!_xinput_paired)
            {
                _xinput_paired = true;
                app_save_host_mac(INPUT_MODE_XINPUT, &param->auth_cmpl.bda[0]);
            }
        }
        else
        {
            ESP_LOGI(TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }

    case ESP_BT_GAP_ENC_CHG_EVT:
    {

        break;
    }

    case ESP_BT_GAP_MODE_CHG_EVT:
    {
        // Depreciated, not needed I guess
        break;
    }

    default:
        ESP_LOGI(TAG, "UNKNOWN GAP EVT: %d", event);
        break;
    }
}

// Handles an OUT report and responds accordingly (if we need to respond)
void _xinput_report_handler(uint8_t report_id, uint8_t *data, uint16_t len)
{
    uint8_t response_dat[63] = {data[0]};
}

// Callbacks for HID report events
void _xinput_bt_hidd_cb(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;
    const char *TAG = "_xinput_bt_hidd_cb";

    switch (event)
    {
    case ESP_HIDD_START_EVENT:
    {
        if (param->start.status == ESP_OK)
        {
            ESP_LOGI(TAG, "START OK");
            if (_xinput_paired)
            {
                ESP_LOGI(TAG, "Setting to non-connectable, non-discoverable, then attempting connection.");
                esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
                util_bluetooth_connect(global_loaded_settings.paired_host_xinput_mac);
            }
            else
            {
                ESP_LOGI(TAG, "Setting to connectable, discoverable.");
                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            }
        }
        break;
    }

    case ESP_HIDD_CONNECT_EVENT:
    {
        if (param->connect.status == ESP_OK)
        {
            _hid_connected = true;
            app_set_connected_status(1);
            ESP_LOGI(TAG, "CONNECT OK");
        }
        else
        {
            ESP_LOGE(TAG, "CONNECT failed!");
        }
        break;
    }

    case ESP_HIDD_PROTOCOL_MODE_EVENT:
    {
        ESP_LOGI(TAG, "PROTOCOL MODE[%u]: %s", param->protocol_mode.map_index, param->protocol_mode.protocol_mode ? "REPORT" : "BOOT");
        break;
    }

    case ESP_HIDD_OUTPUT_EVENT:
    {
        // ESP_LOGI(TAG, "OUTPUT[%u]: %8s ID: %2u, Len: %d, Data:", param->output.map_index, esp_hid_usage_str(param->output.usage), param->output.report_id, param->output.length);
        // ESP_LOG_BUFFER_HEX(TAG, param->output.data, param->output.length);
        _xinput_report_handler(param->output.report_id, param->output.data, param->output.length);
        break;
    }

    case ESP_HIDD_FEATURE_EVENT:
    {
        // ESP_LOGI(TAG, "FEATURE[%u]: %8s ID: %2u, Len: %d, Data:", param->feature.map_index, esp_hid_usage_str(param->feature.usage), param->feature.report_id, param->feature.length);
        ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
        break;
    }

    case ESP_HIDD_DISCONNECT_EVENT:
    {
        if (param->disconnect.status == ESP_OK)
        {
            _hid_connected = false;
            _xi_reset_report_spacer();
            ESP_LOGI(TAG, "DISCONNECT OK");
        }
        else
        {
            ESP_LOGE(TAG, "DISCONNECT failed!");
        }
        break;
    }

    case ESP_HIDD_STOP_EVENT:
    {
        ESP_LOGI(TAG, " HID STOP");
        break;
    }

    default:
        break;
    }
}

// XInput HID report maps
esp_hid_raw_report_map_t xinput_report_maps[1] = {
    {
        .data = xinput_hid_report_descriptor,
        .len = (uint16_t)XINPUT_HID_REPORT_MAP_LEN,
    }};

// Bluetooth App setup data
util_bt_app_params_s xinput_app_params = {
    .hidd_cb = _xinput_bt_hidd_cb,
    .gap_cb = xinput_bt_gap_cb,
    .bt_mode = ESP_BT_MODE_CLASSIC_BT,
    .appearance = ESP_HID_APPEARANCE_GAMEPAD,
};

esp_hid_device_config_t xinput_hidd_config = {
    .vendor_id = 0x045e,
    .product_id = 0x02e0,
    .version = 0x0100,
    .device_name = "XInput Gamepad",
    .manufacturer_name = "HandHeldLegend",
    .serial_number = "000000",
    .report_maps = xinput_report_maps,
    .report_maps_len = 1,
};

// Attempt start of XInput controller core
int core_bt_xinput_start(void)
{
    const char *TAG = "core_bt_xinput_start";
    esp_err_t ret;
    int err;

    err = util_bluetooth_init(global_loaded_settings.device_mac_xinput);

    _xinput_paired = false;

    for (uint8_t i = 0; i < 6; i++)
    {
        if (global_loaded_settings.paired_host_xinput_mac[i] > 0)
            _xinput_paired = true;
    }

    if (_xinput_paired)
    {
        ESP_LOGI(TAG, "Paired host found");
    }

    // Starting bt mode
    err = util_bluetooth_register_app(&xinput_app_params, &xinput_hidd_config);

    return 1;
}

// Stop XInput controller core
void core_bt_xinput_stop(void)
{
    const char *TAG = "core_xi_stop";
    // si_connected = false;
    // si_controller_input_task_set(si_REPORT_MODE_IDLE);
    util_bluetooth_deinit();
}

// XInput Hat Codes
typedef enum
{
    XI_HAT_TOP = 0x01,
    XI_HAT_TOP_RIGHT = 0x02,
    XI_HAT_RIGHT = 0x03,
    XI_HAT_BOTTOM_RIGHT = 0x04,
    XI_HAT_BOTTOM = 0x05,
    XI_HAT_BOTTOM_LEFT = 0x06,
    XI_HAT_LEFT = 0x07,
    XI_HAT_TOP_LEFT = 0x08,
    XI_HAT_CENTER = 0x00,
} xi_input_hat_dir_t;

uint8_t _get_dpad_hat(uint8_t left_right, uint8_t up_down)
{
    uint8_t ret = XI_HAT_CENTER;

    if (left_right == 2)
    {
        ret = XI_HAT_RIGHT;
        if (up_down == 2)
        {
            ret = XI_HAT_TOP_RIGHT;
        }
        else if (up_down == 0)
        {
            ret = XI_HAT_BOTTOM_RIGHT;
        }
    }
    else if (left_right == 0)
    {
        ret = XI_HAT_LEFT;
        if (up_down == 2)
        {
            ret = XI_HAT_TOP_LEFT;
        }
        else if (up_down == 0)
        {
            ret = XI_HAT_BOTTOM_LEFT;
        }
    }

    else if (up_down == 2)
    {
        ret = XI_HAT_TOP;
    }
    else if (up_down == 0)
    {
        ret = XI_HAT_BOTTOM;
    }

    return ret;
}

void _xinput_bt_task(void *parameters)
{
    ESP_LOGI("_xinput_bt_task", "Starting input loop task...");

    _hid_connected = false;
    app_set_report_timer(XINPUT_DEFAULT_US_DELAY);

    for (;;)
    {
        static uint8_t _full_buffer[64] = {0};

        if (_hid_connected)
        {
            if (_xi_send_check_nonblocking())
            {
                if (_hid_connected)
                {
                    // Fill out input data here
                    memcpy(_full_buffer, &_xinput_input_data, sizeof(xinput_input_report_s));
                    esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, XI_INPUT_REPORT_ID, 16, _full_buffer);
                }
            }
        }
        else
        {
            vTaskDelay(16 / portTICK_PERIOD_MS);
        }
    }
}

int16_t _xi_scale_u12_to_s16(uint16_t val)
{
    if (val > 4095) val = 4095; // Clamp just in case

    // Scale: map [0, 4095] → [INT16_MIN, INT16_MAX]
    // The range of INT16 is 65535, so multiply first to preserve precision
    return (int16_t)(((int32_t)val * 65535) / 4095 + INT16_MIN);
}

// Clamp macro for int16_t
#define CLAMP_INT16(x) (((x) > INT16_MAX) ? INT16_MAX : (((x) < INT16_MIN) ? INT16_MIN : (x)))

// Scales 0-4095 to int16 range, centers at 2048, and allows optional inversion
#define SCALE_AXIS(value, invert) \
    CLAMP_INT16(((int32_t)((invert) ? -(value - 2048) : (value - 2048)) * 16))


void xinput_bt_sendinput(i2cinput_input_s *input)
{
    _xinput_input_data.stick_left_x = input->lx << 4;
    _xinput_input_data.stick_left_y = 0xFFFF - (input->ly << 4);
    _xinput_input_data.stick_right_x = input->rx << 4;
    _xinput_input_data.stick_right_y = 0xFFFF - (input->ry << 4);

    _xinput_input_data.analog_trigger_l = input->trigger_zl ? 1020 : (input->lt >> 2);
    _xinput_input_data.analog_trigger_r = input->trigger_zr ? 1020 : (input->rt >> 2);

    _xinput_input_data.bumper_l = input->trigger_l;
    _xinput_input_data.bumper_r = input->trigger_r;

    _xinput_input_data.button_back = input->button_minus;
    _xinput_input_data.button_menu = input->button_plus;

    _xinput_input_data.button_guide = input->button_home;

    _xinput_input_data.button_stick_l = input->button_stick_left;
    _xinput_input_data.button_stick_r = input->button_stick_right;

    _xinput_input_data.button_a = input->button_a;
    _xinput_input_data.button_b = input->button_b;
    _xinput_input_data.button_x = input->button_x;
    _xinput_input_data.button_y = input->button_y;

    uint8_t lr = (1 - input->dpad_left) + input->dpad_right;
    uint8_t ud = (1 - input->dpad_down) + input->dpad_up;

    _xinput_input_data.dpad_hat = _get_dpad_hat(lr, ud);
}