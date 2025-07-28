#include "core_bt_sinput.h"
#include "esp_log.h"

#define REPORT_ID_SINPUT_INPUT         0x01 // Input Report ID, used for SINPUT input data
#define REPORT_ID_SINPUT_INPUT_CMDDAT  0x02 // Input report ID for command replies
#define REPORT_ID_SINPUT_OUTPUT_CMDDAT 0x03 // Output Haptic Report ID, used for haptics and commands


#define SINPUT_COMMAND_HAPTIC       0x01
#define SINPUT_COMMAND_FEATUREFLAGS 0x02
#define SINPUT_COMMAND_PLAYERLED    0x03

#pragma pack(push, 1) // Ensure byte alignment
// Input report (Report ID: 1)
typedef struct
{
    uint8_t plug_status;    // Plug Status Format
    uint8_t charge_percent; // 0-100

    union {
        struct {
            uint8_t button_a   : 1;
            uint8_t button_b   : 1;
            uint8_t button_x   : 1;
            uint8_t button_y   : 1;
            uint8_t dpad_up    : 1;
            uint8_t dpad_down  : 1;
            uint8_t dpad_left  : 1;
            uint8_t dpad_right : 1;
        };
        uint8_t buttons_1;
    };

    union
    {
        struct
        {
            uint8_t button_stick_left : 1;
            uint8_t button_stick_right : 1;
            uint8_t button_l_shoulder : 1;
            uint8_t button_r_shoulder : 1;
            uint8_t button_l_trigger : 1;
            uint8_t button_r_trigger : 1;
            uint8_t button_l_paddle_1 : 1;
            uint8_t button_r_paddle_1 : 1;
        };
        uint8_t buttons_2;
    };

    union
    {
        struct
        {
            uint8_t button_start  : 1;
            uint8_t button_select : 1;
            uint8_t button_guide  : 1;
            uint8_t button_share  : 1;
            uint8_t button_l_paddle_2 : 1;
            uint8_t button_r_paddle_2 : 1;
            uint8_t button_l_touchpad : 1;
            uint8_t button_r_touchpad : 1;
        };
        uint8_t buttons_3;
    };

    union
    {
        struct
        {
            uint8_t button_power   : 1;
            uint8_t button_misc_4  : 1;
            uint8_t button_misc_5  : 1;
            uint8_t button_misc_6  : 1;
            
            // Misc 7 through 10 is unused by
            // SDL currently!
            uint8_t button_misc_7  : 1; 
            uint8_t button_misc_8  : 1;
            uint8_t button_misc_9  : 1;
            uint8_t button_misc_10 : 1;
        };
        uint8_t buttons_4;
    };

    int16_t left_x;             // Left stick X
    int16_t left_y;             // Left stick Y
    int16_t right_x;            // Right stick X
    int16_t right_y;            // Right stick Y
    int16_t trigger_l;          // Left trigger
    int16_t trigger_r;          // Right trigger
    uint32_t imu_timestamp_us;  // Microseconds

    int16_t accel_x;            // Accelerometer X
    int16_t accel_y;            // Accelerometer Y
    int16_t accel_z;            // Accelerometer Z
    int16_t gyro_x;             // Gyroscope X
    int16_t gyro_y;             // Gyroscope Y
    int16_t gyro_z;             // Gyroscope Z

    int16_t touchpad_1_x;       // Touchpad/trackpad
    int16_t touchpad_1_y;
    int16_t touchpad_1_pressure;

    int16_t touchpad_2_x;
    int16_t touchpad_2_y;
    int16_t touchpad_2_pressure;

    uint8_t reserved_bulk[17];  // Reserved for command data
} sinput_input_s;
#pragma pack(pop)

#pragma pack(push, 1) // Ensure byte alignment
typedef struct 
{
    uint8_t type;

    union 
    {
        // Frequency Amplitude pairs
        struct 
        {
            struct
            {
                uint16_t frequency_1;
                uint16_t amplitude_1;
                uint16_t frequency_2;
                uint16_t amplitude_2;
            } left;

            struct
            {
                uint16_t frequency_1;
                uint16_t amplitude_1;
                uint16_t frequency_2;
                uint16_t amplitude_2;
            } right;
            
        } type_1;

        // Basic ERM simulation model
        struct 
        {
            struct 
            {
                uint8_t amplitude;
                bool    brake;
            } left;

            struct 
            {
                uint8_t amplitude;
                bool    brake;
            } right;
            
        } type_2; 
    };
} sinput_haptic_s;
#pragma pack(pop)

#pragma pack(push, 1) // Ensure byte alignment
typedef union
{
    struct
    {
        uint8_t rumble_supported : 1;
        uint8_t player_leds_supported : 1;
        uint8_t accelerometer_supported : 1;
        uint8_t gyroscope_supported : 1;
        uint8_t left_analog_stick_supported : 1;
        uint8_t right_analog_stick_supported : 1;
        uint8_t left_analog_trigger_supported : 1;
        uint8_t right_analog_trigger_supported : 1;
    };
    uint8_t value;
} sinput_featureflags_1_u;
#pragma pack(pop)

#define SINPUT_HID_REPORT_MAP_LEN 139
const uint8_t sinput_hid_report_descriptor[139] = {
    0x05, 0x01,                    // Usage Page (Generic Desktop Ctrls)
    0x09, 0x05,                    // Usage (Gamepad)
    0xA1, 0x01,                    // Collection (Application)
    
    // INPUT REPORT ID 0x01 - Main gamepad data
    0x85, 0x01,                    //   Report ID (1)
    
    // Padding bytes (bytes 2-3) - Plug status and Charge Percent (0-100)
    0x06, 0x00, 0xFF,              //   Usage Page (Vendor Defined)
    0x09, 0x01,                    //   Usage (Vendor Usage 1)
    0x15, 0x00,                    //   Logical Minimum (0)
    0x25, 0xFF,                    //   Logical Maximum (255)
    0x75, 0x08,                    //   Report Size (8)
    0x95, 0x02,                    //   Report Count (2)
    0x81, 0x02,                    //   Input (Data,Var,Abs)

    // --- 32 buttons ---
    0x05, 0x09,        // Usage Page (Button)
    0x19, 0x01,        //   Usage Minimum (Button 1)
    0x29, 0x20,        //   Usage Maximum (Button 32)
    0x15, 0x00,        //   Logical Min (0)
    0x25, 0x01,        //   Logical Max (1)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x20,        //   Report Count (32)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    
    // Analog Sticks and Triggers
    0x05, 0x01,                    // Usage Page (Generic Desktop)
    // Left Stick X (bytes 8-9)
    0x09, 0x30,                    //   Usage (X)
    // Left Stick Y (bytes 10-11)
    0x09, 0x31,                    //   Usage (Y)
    // Right Stick X (bytes 12-13)
    0x09, 0x32,                    //   Usage (Z)
    // Right Stick Y (bytes 14-15)
    0x09, 0x35,                    //   Usage (Rz)
    // Right Trigger (bytes 18-19)
    0x09, 0x33,                    //   Usage (Rx)
    // Left Trigger  (bytes 16-17)
    0x09, 0x34,                     //  Usage (Ry)
    0x16, 0x00, 0x80,              //   Logical Minimum (-32768)
    0x26, 0xFF, 0x7F,              //   Logical Maximum (32767)
    0x75, 0x10,                    //   Report Size (16)
    0x95, 0x06,                    //   Report Count (6)
    0x81, 0x02,                    //   Input (Data,Var,Abs)
    
    // Padding/Reserved data (bytes 20-63) - 44 bytes
    // This includes gyro/accel data and counter that apps can use if supported
    0x06, 0x00, 0xFF,              // Usage Page (Vendor Defined)
    
    // Motion Input Delta (Microseconds, time since last USB report)
    0x09, 0x20,                    //   Usage (Vendor Usage 0x20)
    0x15, 0x00,                    //   Logical Minimum (0)
    0x26, 0xFF, 0xFF,              //   Logical Maximum (655535)
    0x75, 0x10,                    //   Report Size (16)
    0x95, 0x01,                    //   Report Count (1)
    0x81, 0x02,                    //   Input (Data,Var,Abs)

    // Motion Input Accelerometer XYZ (Gs) and Gyroscope XYZ (Degrees Per Second)
    0x09, 0x21,                    //   Usage (Vendor Usage 0x21)
    0x16, 0x00, 0x80,              //   Logical Minimum (-32768)
    0x26, 0xFF, 0x7F,              //   Logical Maximum (32767)
    0x75, 0x10,                    //   Report Size (16)
    0x95, 0x06,                    //   Report Count (6)
    0x81, 0x02,                    //   Input (Data,Var,Abs)

    // Reserved padding (31 bytes blank for now)
    0x09, 0x22,                    //   Usage (Vendor Usage 0x22)
    0x15, 0x00,                    //   Logical Minimum (0)
    0x26, 0xFF, 0x00,              //   Logical Maximum (255)
    0x75, 0x08,                    //   Report Size (8)
    0x95, 0x1F,                    //   Report Count (31)
    0x81, 0x02,                    //   Input (Data,Var,Abs)
    
    // INPUT REPORT ID 0x02 - Vendor COMMAND data
    0x85, 0x02,                    //   Report ID (2)
    0x09, 0x23,                    //   Usage (Vendor Usage 0x23)
    0x15, 0x00,                    //   Logical Minimum (0)
    0x26, 0xFF, 0x00,              //   Logical Maximum (255)
    0x75, 0x08,                    //   Report Size (8 bits)
    0x95, 0x3F,                    //   Report Count (63) - 64 bytes minus report ID
    0x81, 0x02,                    //   Input (Data,Var,Abs)

    // OUTPUT REPORT ID 0x03 - Vendor COMMAND data
    0x85, 0x03,                    //   Report ID (3)
    0x09, 0x24,                    //   Usage (Vendor Usage 0x24)
    0x15, 0x00,                    //   Logical Minimum (0)
    0x26, 0xFF, 0x00,              //   Logical Maximum (255)
    0x75, 0x08,                    //   Report Size (8 bits)
    0x95, 0x2F,                    //   Report Count (47) - 48 bytes minus report ID
    0x91, 0x02,                    //   Output (Data,Var,Abs)

    0xC0                           // End Collection 
};

#define SINPUT_DEFAULT_TICK_DELAY (8/portTICK_PERIOD_MS)
#define SINPUT_DEFAULT_US_DELAY (8*1000)
static volatile bool        _hid_connected    = false;

static volatile bool        _sinput_paired = false;

interval_s _si_interval = {0};
sinput_input_s _si_input = {0};

void _si_fill_features(uint16_t pid, uint8_t sub_id, uint8_t *data)
{
    const uint16_t sinput_protocol_version = 0x0001;
    memcpy(&data[0], &sinput_protocol_version, sizeof(sinput_protocol_version)); 

    sinput_featureflags_1_u feature_flags = {0};
    feature_flags.value = 0x00; // Set default feature flags

    if(pid == 0x10C6) // Generic HOJA Gamepad
    {
        switch(sub_id)
        {
            case 0x00: // Super Gamepad+

                data[4] = 0; // Default type
                data[5] = (3 << 5); // Gamepad Sub-Type and face style

                feature_flags.player_leds_supported = 1;

                uint8_t usage_mask_sgpp[4] = {0b11111111, 0b00001100, 0b00000011, 0b00000000};
                data[12] = usage_mask_sgpp[0];
                data[13] = usage_mask_sgpp[1];
                data[14] = usage_mask_sgpp[2];
                data[15] = usage_mask_sgpp[3];
            break;
        }
    }
    else 
    {
        switch(pid)
        {
            case 0x10DD: // GC Ultimate
                feature_flags.accelerometer_supported   = 1;
                feature_flags.gyroscope_supported       = 1;
                feature_flags.left_analog_stick_supported       = 1;
                feature_flags.right_analog_stick_supported      = 1;
                feature_flags.left_analog_trigger_supported     = 1;
                feature_flags.right_analog_trigger_supported    = 1;
                feature_flags.rumble_supported     = 1;
                feature_flags.player_leds_supported = 1;

                data[4] = 11; // Gamepad Type (GameCube)

                uint8_t usage_mask_gcult[4] = {0b11111111, 0b00111111, 0b00001111, 0b00000001};
                data[12] = usage_mask_gcult[0];
                data[13] = usage_mask_gcult[1];
                data[14] = usage_mask_gcult[2];
                data[15] = usage_mask_gcult[3];
            break;

            case 0x10DF: // ProGCC
                feature_flags.accelerometer_supported   = 1;
                feature_flags.gyroscope_supported       = 1;
                feature_flags.left_analog_stick_supported       = 1;
                feature_flags.right_analog_stick_supported      = 1;
                feature_flags.left_analog_trigger_supported     = 1;
                feature_flags.right_analog_trigger_supported    = 1;
                feature_flags.rumble_supported     = 1;
                feature_flags.player_leds_supported = 1;

                data[4] = 7; // ProCon Type

                uint8_t usage_mask_progcc[4] = {0b11111111, 0b00111111, 0b00001111, 0b00000000};
                data[12] = usage_mask_progcc[0];
                data[13] = usage_mask_progcc[1];
                data[14] = usage_mask_progcc[2];
                data[15] = usage_mask_progcc[3];
            break;
        }
    }

    data[2] = feature_flags.value; // Feature flags value   
    data[3] = 0x00; // Feature flags 2 (unused)

    data[16] = 0;
    data[17] = 0;

    // Set MAC
    data[18] = global_live_data.current_mac[0];
    data[19] = global_live_data.current_mac[1];
    data[20] = global_live_data.current_mac[2];
    data[21] = global_live_data.current_mac[3];
    data[22] = global_live_data.current_mac[4];
    data[23] = global_live_data.current_mac[5];
}

void _si_reset_report_spacer()
{
    uint64_t delay_time = app_get_report_timer();
    uint64_t timestamp = get_timestamp_us();
    interval_resettable_run(timestamp, delay_time, true, &_si_interval);
}

bool _si_send_check_nonblocking()
{
    uint64_t delay_time = app_get_report_timer();
    uint64_t timestamp = get_timestamp_us();
    return interval_run(timestamp, delay_time, &_si_interval);
}

TaskHandle_t _sinput_bt_task_handle = NULL;

sinput_input_s _sinput_input_data = {0};

void _sinput_bt_task(void *parameters);

// sinput BTC GAP Event Callback
void sinput_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    const char *TAG = "sinput_bt_gap_cb";
    switch(event)
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
        if(_sinput_bt_task_handle==NULL)
        {
            xTaskCreatePinnedToCore(_sinput_bt_task,
                                "SInput Send Task", 4048,
                                NULL, 0, &_sinput_bt_task_handle, 0);
        }
        break;

    case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
        ESP_LOGI(TAG, "ACL Disconnect Complete.");
        
        if(_sinput_bt_task_handle!=NULL)
        {
            vTaskDelete(_sinput_bt_task_handle);
            _sinput_bt_task_handle = NULL;
        }

        app_set_connected_status(0);

        app_set_power_setting(POWER_CODE_OFF); // Shut down
        
        /*
        if(_sinput_paired)
        {
            ESP_LOGI(TAG, "Setting to non-connectable, non-discoverable, then attempting connection.");
            esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            util_bluetooth_connect(global_loaded_settings.paired_host_sinput_mac);
        }
        else
        {
            ESP_LOGI(TAG, "Setting to connectable, discoverable.");
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        }*/

        break;
    
    case ESP_BT_GAP_AUTH_CMPL_EVT:
    {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGI(TAG, "authentication success: %s", param->auth_cmpl.device_name);
            //esp_log_buffer_hex(TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
            
            if(!_sinput_paired)
            {
                _sinput_paired = true;
                app_save_host_mac(INPUT_MODE_SINPUT, &param->auth_cmpl.bda[0]);
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

// Get Feature Flags
void _sinput_cmd_get_featureflags(uint8_t *buffer)
{
    uint16_t accel_g_range      = 8; // 8G 
    uint16_t gyro_dps_range     = 2000; // 2000 degrees per second

    buffer[6] = 8;    // Polling rate (ms)
    buffer[7] = 0x00; // Reserved

    memcpy(&buffer[8], &accel_g_range, sizeof(accel_g_range)); // Accelerometer G range
    memcpy(&buffer[10], &gyro_dps_range, sizeof(gyro_dps_range)); // Gyroscope DPS range

    _si_fill_features(global_live_data.product_id, global_live_data.sub_id, buffer);
}

volatile uint8_t _sinput_current_cmd = 0;

// Handles an OUT report and responds accordingly (if we need to respond)
void _sinput_report_handler(uint8_t report_id, uint8_t *data, uint16_t len)
{
    uint8_t response_dat[63] = {data[0]};

    if(report_id == REPORT_ID_SINPUT_OUTPUT_CMDDAT)
    {
        uint8_t cmd = data[0];

        switch(cmd)
        {
            case SINPUT_COMMAND_HAPTIC:
            app_set_sinput_haptic(&data[1], 19);
            break;

            case SINPUT_COMMAND_FEATUREFLAGS:
            ESP_LOGI("_sinput_report_handler", "Feature Flags Command Got");
            _sinput_current_cmd = SINPUT_COMMAND_FEATUREFLAGS;
            break;

            case SINPUT_COMMAND_PLAYERLED:
            ESP_LOGI("_sinput_report_handler", "Player LED Command Got: Player %d", data[1]);
            app_set_connected_status(data[1]);
            break;

            default:
            break;
        }
    }
}

// Callbacks for HID report events
void _sinput_bt_hidd_cb(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;
    const char *TAG = "_sinput_bt_hidd_cb";

    switch(event)
    {
        case ESP_HIDD_START_EVENT:
        {
            if (param->start.status == ESP_OK)
            {
                ESP_LOGI(TAG, "START OK");
                if(_sinput_paired)
                {
                    ESP_LOGI(TAG, "Setting to non-connectable, non-discoverable, then attempting connection.");
                    esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
                    util_bluetooth_connect(global_loaded_settings.paired_host_sinput_mac);
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
            //ESP_LOGI(TAG, "OUTPUT[%u]: %8s ID: %2u, Len: %d, Data:", param->output.map_index, esp_hid_usage_str(param->output.usage), param->output.report_id, param->output.length);
            //ESP_LOG_BUFFER_HEX(TAG, param->output.data, param->output.length);
            _sinput_report_handler(param->output.report_id, param->output.data, param->output.length);
            break;
        }

        case ESP_HIDD_FEATURE_EVENT:
        {
            //ESP_LOGI(TAG, "FEATURE[%u]: %8s ID: %2u, Len: %d, Data:", param->feature.map_index, esp_hid_usage_str(param->feature.usage), param->feature.report_id, param->feature.length);
            ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
            break;
        }

        case ESP_HIDD_DISCONNECT_EVENT:
        {
            if (param->disconnect.status == ESP_OK)
            {
                _hid_connected = false;
                _si_reset_report_spacer();
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

// SInput HID report maps
esp_hid_raw_report_map_t sinput_report_maps[1] = {
    {
        .data = sinput_hid_report_descriptor,
        .len = (uint16_t)SINPUT_HID_REPORT_MAP_LEN,
    }};

// Bluetooth App setup data
util_bt_app_params_s sinput_app_params = {
    .hidd_cb = _sinput_bt_hidd_cb,
    .gap_cb = sinput_bt_gap_cb,
    .bt_mode = ESP_BT_MODE_CLASSIC_BT,
    .appearance = ESP_HID_APPEARANCE_GAMEPAD,
};

esp_hid_device_config_t sinput_hidd_config = {
    .vendor_id = 0x0000,
    .product_id = 0x0000,
    .version = 0x0100,
    .device_name = "SInput Gamepad",
    .manufacturer_name = "",
    .serial_number = "000000",
    .report_maps = sinput_report_maps,
    .report_maps_len = 1,
};

// Attempt start of SInput controller core
int core_bt_sinput_start(void)
{
    const char *TAG = "core_bt_sinput_start";
    esp_err_t ret;
    int err;

    uint8_t tmpmac[6] = {0};
    uint8_t *mac = global_live_data.current_mac;

    if( (mac[0] == 0) && (mac[1] == 0) )
    {
        mac = global_loaded_settings.device_mac_switch;
    }

    memcpy(tmpmac, mac, 6);
    // On ESP32, the Bluetooth address is the base MAC with the last octet +=2
    tmpmac[5] -= 2;
    err = util_bluetooth_init(tmpmac);

    _sinput_paired = false;

    for (uint8_t i = 0; i < 6; i++)
    {
        if (global_loaded_settings.paired_host_sinput_mac[i] > 0)
            _sinput_paired = true;
    }

    if(_sinput_paired)
    {
        ESP_LOGI(TAG, "Paired host found");
    }

    // Load VID/PID
    sinput_hidd_config.vendor_id  = global_live_data.vendor_id;
    sinput_hidd_config.product_id = global_live_data.product_id;

    // Starting bt mode
    err = util_bluetooth_register_app(&sinput_app_params, &sinput_hidd_config);

    return 1;
}

// Stop SInput controller core
void core_bt_sinput_stop(void)
{
    const char *TAG = "core_si_stop";
    // si_connected = false;
    // si_controller_input_task_set(si_REPORT_MODE_IDLE);
    util_bluetooth_deinit();
}

void _sinput_bt_task(void *parameters)
{
    ESP_LOGI("_sinput_bt_task", "Starting input loop task...");

    _hid_connected = false;
    app_set_report_timer(SINPUT_DEFAULT_US_DELAY); 

    for (;;)
    {
        static uint8_t _full_buffer[64] = {0};

        if(_hid_connected)
        {
            if(_si_send_check_nonblocking())
            {
                if(_hid_connected)
                {
                    if(_sinput_current_cmd != 0)
                    {
                        ESP_LOGI("_sinput_bt_task", "Sending response cmd...");
                        switch(_sinput_current_cmd)
                        {
                            case SINPUT_COMMAND_FEATUREFLAGS:
                                memset(_full_buffer, 0, 64);
                                _full_buffer[0] = SINPUT_COMMAND_FEATUREFLAGS;
                                _sinput_cmd_get_featureflags(&(_full_buffer[1]));
                                esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, REPORT_ID_SINPUT_INPUT_CMDDAT, 63, _full_buffer);
                                _sinput_current_cmd = 0;
                            break;
                            
                            default:
                            break;
                        }
                    }
                    else 
                    {
                        // Update battery status 
                        if(global_live_data.bat_status.charging)
                        {
                            _si_input.plug_status = 2;
                        }
                        else 
                        {
                            _si_input.plug_status = 4;
                        }

                        if(global_live_data.bat_status.bat_lvl == 4)
                        {
                            if(global_live_data.bat_status.charging)
                            {
                                _si_input.plug_status = 3;
                            }
                        }

                        _si_input.charge_percent = global_live_data.bat_status.bat_lvl * 25;

                        // Fill out delta time and gyro
                        uint64_t timestamp = get_timestamp_us();

                        _si_input.imu_timestamp_us = (uint32_t) (timestamp & UINT32_MAX);
                        
                        static imu_data_s *imu = NULL;

                        imu = imu_fifo_last();

                        _si_input.accel_x = imu->ax;
                        _si_input.accel_y = imu->ay;
                        _si_input.accel_z = imu->az;

                        _si_input.gyro_x = imu->gx;
                        _si_input.gyro_y = imu->gy;
                        _si_input.gyro_z = imu->gz;

                        // Fill out input data here
                        memcpy(_full_buffer, &_si_input, sizeof(sinput_input_s));
                        esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, REPORT_ID_SINPUT_INPUT, 63, _full_buffer);
                    }
                }   
            }
        }
        else
        {
            vTaskDelay(16/portTICK_PERIOD_MS);
        }
    }
}

int16_t scale_u12_to_s16(uint16_t val)
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

void sinput_bt_sendinput(i2cinput_input_s *input)
{
    _si_input.left_x    = SCALE_AXIS(input->lx, false);
    _si_input.left_y    = SCALE_AXIS(input->ly, true);
    _si_input.right_x   = SCALE_AXIS(input->rx, false);
    _si_input.right_y   = SCALE_AXIS(input->ry, true);

    // Buttons
    _si_input.button_a = input->button_a;
    _si_input.button_b = input->button_b;
    _si_input.button_x = input->button_x;
    _si_input.button_y = input->button_y;

    _si_input.button_stick_left  = input->button_stick_left;
    _si_input.button_stick_right = input->button_stick_right;

    _si_input.button_start  = input->button_plus;
    _si_input.button_select = input->button_minus;
    _si_input.button_guide  = input->button_home;
    _si_input.button_share  = input->button_capture;

    _si_input.dpad_up = input->dpad_up;
    _si_input.dpad_down = input->dpad_down;
    _si_input.dpad_left = input->dpad_left;
    _si_input.dpad_right = input->dpad_right;

    _si_input.button_l_shoulder = input->trigger_l;
    _si_input.button_r_shoulder = input->trigger_r;

    _si_input.button_l_paddle_1 = input->trigger_gl;
    _si_input.button_r_paddle_1 = input->trigger_gr;

    _si_input.button_l_trigger  = input->trigger_zl;
    _si_input.button_r_trigger  = input->trigger_zr;

    _si_input.button_power = input->button_shipping;

    int32_t trigger = -32768;

    _si_input.trigger_l = scale_u12_to_s16(input->lt);    // Scale to 16-bit
    _si_input.trigger_r = scale_u12_to_s16(input->rt);    // Scale to 16-bit
}