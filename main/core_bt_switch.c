#include "core_bt_switch.h"
#include "esp_log.h"

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

#define HID_PROD_NSPRO  0x2009
#define HID_VEND_NSPRO  0x057E
#define PROCON_HID_REPORT_MAP_LEN 170

const uint8_t procon_hid_descriptor[PROCON_HID_REPORT_MAP_LEN] = {
    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
    0x09, 0x05,        // Usage (Game Pad)
    0xA1, 0x01,        // Collection (Application)
    0x06, 0x01, 0xFF,  //   Usage Page (Vendor Defined 0xFF01)

    0x85, 0x21,  //   Report ID (33)
    0x09, 0x21,  //   Usage (0x21)
    0x75, 0x08,  //   Report Size (8)
    0x95, 0x30,  //   Report Count (48)
    0x81, 0x02,  //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null
                 //   Position)

    0x85, 0x30,  //   Report ID (48)
    0x09, 0x30,  //   Usage (0x30)
    0x75, 0x08,  //   Report Size (8)
    0x95, 0x30,  //   Report Count (48)
    0x81, 0x02,  //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null
                 //   Position)

    0x85, 0x31,        //   Report ID (49)
    0x09, 0x31,        //   Usage (0x31)
    0x75, 0x08,        //   Report Size (8)
    0x96, 0x69, 0x01,  //   Report Count (361)
    0x81, 0x02,  //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null
                 //   Position)

    0x85, 0x32,        //   Report ID (50)
    0x09, 0x32,        //   Usage (0x32)
    0x75, 0x08,        //   Report Size (8)
    0x96, 0x69, 0x01,  //   Report Count (361)
    0x81, 0x02,  //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null
                 //   Position)

    0x85, 0x33,        //   Report ID (51)
    0x09, 0x33,        //   Usage (0x33)
    0x75, 0x08,        //   Report Size (8)
    0x96, 0x69, 0x01,  //   Report Count (361)
    0x81, 0x02,  //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null
                 //   Position)

    0x85, 0x3F,  //   Report ID (63)
    0x05, 0x09,  //   Usage Page (Button)
    0x19, 0x01,  //   Usage Minimum (0x01)
    0x29, 0x10,  //   Usage Maximum (0x10)
    0x15, 0x00,  //   Logical Minimum (0)
    0x25, 0x01,  //   Logical Maximum (1)
    0x75, 0x01,  //   Report Size (1)
    0x95, 0x10,  //   Report Count (16)
    0x81, 0x02,  //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null
                 //   Position)
    0x05, 0x01,  //   Usage Page (Generic Desktop Ctrls)
    0x09, 0x39,  //   Usage (Hat switch)
    0x15, 0x00,  //   Logical Minimum (0)
    0x25, 0x07,  //   Logical Maximum (7)
    0x75, 0x04,  //   Report Size (4)
    0x95, 0x01,  //   Report Count (1)
    0x81, 0x42,  //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,Null
                 //   State)
    0x05, 0x09,  //   Usage Page (Button)
    0x75, 0x04,  //   Report Size (4)
    0x95, 0x01,  //   Report Count (1)
    0x81, 0x01,  //   Input (Const,Array,Abs,No Wrap,Linear,Preferred State,No
                 //   Null Position)
    0x05, 0x01,  //   Usage Page (Generic Desktop Ctrls)
    0x09, 0x30,  //   Usage (X)
    0x09, 0x31,  //   Usage (Y)
    0x09, 0x33,  //   Usage (Rx)
    0x09, 0x34,  //   Usage (Ry)
    0x16, 0x00, 0x00,              //   Logical Minimum (0)
    0x27, 0xFF, 0xFF, 0x00, 0x00,  //   Logical Maximum (65534)
    0x75, 0x10,                    //   Report Size (16)
    0x95, 0x04,                    //   Report Count (4)
    0x81, 0x02,  //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null
                 //   Position)
                 
    0x06, 0x01, 0xFF,  //   Usage Page (Vendor Defined 0xFF01)

    0x85, 0x01,  //   Report ID (1)
    0x09, 0x01,  //   Usage (0x01)
    0x75, 0x08,  //   Report Size (8)
    0x95, 0x30,  //   Report Count (48)
    0x91, 0x02,  //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No
                 //   Null Position,Non-volatile)

    0x85, 0x10,  //   Report ID (16)
    0x09, 0x10,  //   Usage (0x10)
    0x75, 0x08,  //   Report Size (8)
    0x95, 0x09,  //   Report Count (9)
    0x91, 0x02,  //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No
                 //   Null Position,Non-volatile)

    0x85, 0x11,  //   Report ID (17)
    0x09, 0x11,  //   Usage (0x11)
    0x75, 0x08,  //   Report Size (8)
    0x95, 0x30,  //   Report Count (48)
    0x91, 0x02,  //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No
                 //   Null Position,Non-volatile)

    0x85, 0x12,  //   Report ID (18)
    0x09, 0x12,  //   Usage (0x12)
    0x75, 0x08,  //   Report Size (8)
    0x95, 0x30,  //   Report Count (48)
    0x91, 0x02,  //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No
                 //   Null Position,Non-volatile)
    0xC0,        // End Collection
};

#define DEFAULT_TICK_DELAY (8/portTICK_PERIOD_MS)
#define DEFAULT_US_DELAY (8*1000)
static volatile bool        _hid_connected = false;

static volatile bool        _switch_paired = false;
uint8_t _switch_imu_mode = 0x00;

interval_s _ns_interval = {0};

sw_input_s _switch_input_data = {.ls_x = 2047, .ls_y = 2047, .rs_x = 2047, .rs_y = 2047};

void ns_set_imu_mode(uint8_t mode)
{
    _switch_imu_mode = mode;
}

void ns_report_setinputreport_full(uint8_t *buffer)
{
  // Set input data
  buffer[2] = _switch_input_data.right_buttons;
  buffer[3] = _switch_input_data.shared_buttons;
  buffer[4] = _switch_input_data.left_buttons;

  // Set sticks directly
  // Saves cycles :)
  buffer[5] = (_switch_input_data.ls_x & 0xFF);
  buffer[6] = (_switch_input_data.ls_x & 0xF00) >> 8;
  // ns_input_report[7] |= (g_stick_data.lsy & 0xF) << 4;
  buffer[7] = (_switch_input_data.ls_y & 0xFF0) >> 4;
  buffer[8] = (_switch_input_data.rs_x & 0xFF);
  buffer[9] = (_switch_input_data.rs_x & 0xF00) >> 8;
  buffer[10] = (_switch_input_data.rs_y & 0xFF0) >> 4;

  if (_switch_imu_mode == 0x01)
  {
    // Set gyro
    // Retrieve and write IMU data
    imu_data_s *_imu_tmp = imu_fifo_last();

    // Group 1
    buffer[12] = _imu_tmp->ay_8l; // Y-axis
    buffer[13] = _imu_tmp->ay_8h;
    buffer[14] = _imu_tmp->ax_8l; // X-axis
    buffer[15] = _imu_tmp->ax_8h;
    buffer[16] = _imu_tmp->az_8l; // Z-axis
    buffer[17] = _imu_tmp->az_8h;

    buffer[18] = _imu_tmp->gy_8l;
    buffer[19] = _imu_tmp->gy_8h;
    buffer[20] = _imu_tmp->gx_8l;
    buffer[21] = _imu_tmp->gx_8h;
    buffer[22] = _imu_tmp->gz_8l;
    buffer[23] = _imu_tmp->gz_8h;

    _imu_tmp = imu_fifo_last();

    // Group 2
    buffer[24] = _imu_tmp->ay_8l; // Y-axis
    buffer[25] = _imu_tmp->ay_8h;
    buffer[26] = _imu_tmp->ax_8l; // X-axis
    buffer[27] = _imu_tmp->ax_8h;
    buffer[28] = _imu_tmp->az_8l; // Z-axis
    buffer[29] = _imu_tmp->az_8h;

    buffer[30] = _imu_tmp->gy_8l;
    buffer[31] = _imu_tmp->gy_8h;
    buffer[32] = _imu_tmp->gx_8l;
    buffer[33] = _imu_tmp->gx_8h;
    buffer[34] = _imu_tmp->gz_8l;
    buffer[35] = _imu_tmp->gz_8h;

    _imu_tmp = imu_fifo_last();

    // Group 3
    buffer[36] = _imu_tmp->ay_8l; // Y-axis
    buffer[37] = _imu_tmp->ay_8h;
    buffer[38] = _imu_tmp->ax_8l; // X-axis
    buffer[39] = _imu_tmp->ax_8h;
    buffer[40] = _imu_tmp->az_8l; // Z-axis
    buffer[41] = _imu_tmp->az_8h;

    buffer[42] = _imu_tmp->gy_8l;
    buffer[43] = _imu_tmp->gy_8h;
    buffer[44] = _imu_tmp->gx_8l;
    buffer[45] = _imu_tmp->gx_8h;
    buffer[46] = _imu_tmp->gz_8l;
    buffer[47] = _imu_tmp->gz_8h;
  }
  else if (_switch_imu_mode == 0x02)
  {
    static mode_2_s mode_2_data = {0};

    imu_pack_quat(&mode_2_data);

    memcpy(&(buffer[12]), &mode_2_data, sizeof(mode_2_s));
  }
}

void _ns_reset_report_spacer()
{
    uint64_t timestamp = get_timestamp_us();
    uint64_t delay_time = app_get_report_timer();
    interval_resettable_run(timestamp, delay_time, true, &_ns_interval);
}

bool _ns_send_check_nonblocking()
{
    uint64_t timestamp = get_timestamp_us();
    uint64_t delay_time = app_get_report_timer();
    return interval_run(timestamp, delay_time, &_ns_interval);
}

/**
 * @brief NS Core Report mode enums
 */
typedef enum
{
    NS_REPORT_MODE_BLANK,
    NS_REPORT_MODE_SIMPLE,
    NS_REPORT_MODE_FULL,
    NS_REPORT_MODE_MAX,
} ns_report_mode_t;

/**
 * @brief NS Core power handle state types
 */
typedef enum
{
    NS_POWER_AWAKE,
    NS_POWER_SLEEP,
} ns_power_handle_t;

/**
 * @brief NS Core Status
 */
typedef enum
{
    NS_STATUS_IDLE,
    NS_STATUS_SUBCORESET,
    NS_STATUS_RUNNING,
} ns_core_status_t;

TaskHandle_t _switch_bt_task_handle = NULL;
ns_power_handle_t _switch_power_state = NS_POWER_AWAKE;


void _switch_bt_task_standard(void *parameters);
void _switch_bt_task_empty(void *parameters);
void _switch_bt_task_short(void *parameters);


void switch_bt_end_task()
{
    if(_switch_bt_task_handle != NULL)
    {
        vTaskDelete(_switch_bt_task_handle);
    }
}

// Unused
void btsnd_hcic_sniff_mode_cb(bool sniff, uint16_t tx_lat, uint16_t rx_lat)
{
    // Ignore all of this for debug
    return;
    if(sniff)
    {
        //_sniff = true;
        //_delay_time_us = rx_lat*1000;//_ns_interval_to_us(rx_lat);
        printf("Delay (ms): %d\n", rx_lat);
    }
    else
    {
        //_sniff = false;
        //_delay_time_us = 8000;
        printf("UnSniff: \n");
    }
    _ns_reset_report_spacer();
}



/* HCI mode defenitions */
#define HCI_MODE_ACTIVE                 0x00
#define HCI_MODE_HOLD                   0x01
#define HCI_MODE_SNIFF                  0x02
#define HCI_MODE_PARK                   0x03

// SWITCH BTC GAP Event Callback
void switch_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    const char *TAG = "switch_bt_gap_cb";
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
        if(_switch_bt_task_handle==NULL)
        {
            xTaskCreatePinnedToCore(_switch_bt_task_standard,
                                "Standard Send Task", 4048,
                                NULL, 0, &_switch_bt_task_handle, 0);
        }
        break;

    case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
        ESP_LOGI(TAG, "ACL Disconnect Complete.");
        
        if(_switch_bt_task_handle!=NULL)
        {
            vTaskDelete(_switch_bt_task_handle);
            _switch_bt_task_handle = NULL;
        }

        app_set_connected_status(0);
        
        if(_switch_paired)
        {
            ESP_LOGI(TAG, "Setting to non-connectable, non-discoverable, then attempting connection.");
            esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            util_bluetooth_connect(global_loaded_settings.paired_host_switch_mac);
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
            //esp_log_buffer_hex(TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
            
            if(!_switch_paired)
            {
                _switch_paired = true;
                app_save_host_mac(INPUT_MODE_SWPRO, &param->auth_cmpl.bda[0]);
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

// Callbacks for HID report events
void switch_bt_hidd_cb(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;
    const char *TAG = "ns_bt_hidd_cb";

    switch (event)
    {
        case ESP_HIDD_START_EVENT:
        {
            if (param->start.status == ESP_OK)
            {
                ESP_LOGI(TAG, "START OK");
                if(_switch_paired)
                {
                    ESP_LOGI(TAG, "Setting to non-connectable, non-discoverable, then attempting connection.");
                    esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
                    util_bluetooth_connect(global_loaded_settings.paired_host_switch_mac);
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
            ns_report_handler(param->output.report_id, param->output.data, param->output.length);
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
                _ns_reset_report_spacer();
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

// Switch HID report maps
esp_hid_raw_report_map_t switch_report_maps[1] = {
    {
        .data = procon_hid_descriptor,
        .len = (uint16_t)PROCON_HID_REPORT_MAP_LEN,
    }};

// Bluetooth App setup data
util_bt_app_params_s switch_app_params = {
    .hidd_cb = switch_bt_hidd_cb,
    .gap_cb = switch_bt_gap_cb,
    .bt_mode = ESP_BT_MODE_CLASSIC_BT,
    .appearance = ESP_HID_APPEARANCE_GAMEPAD,
};

esp_hid_device_config_t switch_hidd_config = {
    .vendor_id = HID_VEND_NSPRO,
    .product_id = HID_PROD_NSPRO,
    .version = 0x0100,
    .device_name = "Pro Controller",
    .manufacturer_name = "Nintendo",
    .serial_number = "000000",
    .report_maps = switch_report_maps,
    .report_maps_len = 1,
};

// Attempt start of Nintendo Switch controller core
int core_bt_switch_start(void)
{
    const char *TAG = "core_bt_switch_start";
    esp_err_t ret;
    int err;

    // Convert calibration data
    switch_analog_calibration_init();

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

    _switch_paired = false;

    for (uint8_t i = 0; i < 6; i++)
    {
        if (global_loaded_settings.paired_host_switch_mac[i] > 0)
            _switch_paired = true;
    }

    if(_switch_paired)
    {
        ESP_LOGI(TAG, "Paired host found");
    }

    // Starting bt mode
    err = util_bluetooth_register_app(&switch_app_params, &switch_hidd_config);

    return 1;
}

// Stop Nintendo Switch controller core
void core_bt_switch_stop(void)
{
    const char *TAG = "core_ns_stop";
    util_bluetooth_deinit();
}

void _switch_bt_task_standard(void *parameters)
{
    ESP_LOGI("_switch_bt_task_standard", "Starting input loop task...");

    static ns_report_mode_t _report_mode = NS_REPORT_MODE_FULL;

    //_report_mode = NS_REPORT_MODE_BLANK;
    _hid_connected = false;
    app_set_report_timer(DEFAULT_US_DELAY); 

    for (;;)
    {
        static uint8_t _full_buffer[64] = {0};
        uint8_t tmp[64] = {0x00, 0x00};

        if(_hid_connected)
        {
            if(_ns_send_check_nonblocking())
            {
                if((_report_mode == NS_REPORT_MODE_FULL))
                {
                    ns_report_clear(_full_buffer, 64);
                    ns_report_setinputreport_full(_full_buffer);
                    ns_report_settimer(_full_buffer);
                    ns_report_setbattconn(_full_buffer);
                    //_full_buffer[12] = 0x70;
                    if(_hid_connected)
                        esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x30, SWITCH_BT_REPORT_SIZE, _full_buffer);
                }
                else
                {
                    esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x00, 1, tmp);
                }
            }
        }
        else
        {
            vTaskDelay(16/portTICK_PERIOD_MS);
        }
    }
}

void switch_bt_sendinput(i2cinput_input_s *input)
{
    _switch_input_data.ls_x = input->lx;
    _switch_input_data.ls_y = input->ly;

    _switch_input_data.rs_x = input->rx;
    _switch_input_data.rs_y = input->ry;

    _switch_input_data.b_a = input->button_a;
    _switch_input_data.b_b = input->button_b;
    _switch_input_data.b_x = input->button_x;
    _switch_input_data.b_y = input->button_y;

    _switch_input_data.d_down   = input->dpad_down;
    _switch_input_data.d_left   = input->dpad_left;
    _switch_input_data.d_right  = input->dpad_right;
    _switch_input_data.d_up     = input->dpad_up;

    _switch_input_data.b_capture    = input->button_capture;
    _switch_input_data.b_home       = input->button_home;
    _switch_input_data.b_minus      = input->button_minus;
    _switch_input_data.b_plus       = input->button_plus;

    _switch_input_data.t_l  = input->trigger_l;
    _switch_input_data.t_r  = input->trigger_r;
    _switch_input_data.t_zl = input->trigger_zl;
    _switch_input_data.t_zr = input->trigger_zr;

    _switch_input_data.sb_left  = input->button_stick_left;
    _switch_input_data.sb_right = input->button_stick_right;
}