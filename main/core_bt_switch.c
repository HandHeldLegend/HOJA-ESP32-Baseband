#include "core_bt_switch.h"
#include "esp_log.h"

#define DEFAULT_TICK_DELAY (8/portTICK_PERIOD_MS)
#define DEFAULT_US_DELAY (8*1000)
static volatile bool        _hid_connected = false;
static volatile uint32_t    _delay_time_us = DEFAULT_US_DELAY;
static volatile uint32_t    _delay_time_ticks = DEFAULT_TICK_DELAY; // 8ms default?
static volatile bool        _sniff = true;

interval_s _ns_interval = {0};

void ns_reset_report_spacer()
{
    uint32_t timestamp = get_timestamp_us();
    interval_resettable_run(timestamp, _delay_time_us, true, &_ns_interval);
}

void ns_send_check_blocking()
{
    bool ok_to_send = false;
    uint32_t timestamp = 0;
    uint32_t fail_timer = 1000;
    while(!ok_to_send)
    {
        timestamp = get_timestamp_us();
        if(interval_run(timestamp, _delay_time_us, &_ns_interval))
        {
            ok_to_send = true;
        }
        else
        {
            fail_timer-=1;
            vTaskDelay(1/portTICK_PERIOD_MS);
        }

        if(!fail_timer)
        {
            ns_reset_report_spacer();
            return;
        }
    }
}

bool ns_send_check_nonblocking()
{
    uint32_t timestamp = get_timestamp_us();
    return interval_run(timestamp, _delay_time_us, &_ns_interval);
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

typedef enum
{
    NS_EVENT_HID_CHANGE,
    NS_EVENT_REPORT_MODE_CHANGE,
    NS_EVENT_SET_SNIFF,
    NS_EVENT_SET_AWAKE,
} ns_event_t;

typedef struct
{
    ns_event_t event_id;
    ns_report_mode_t report_mode;
    uint16_t poll_interval;
    bool hid_connected;
} ns_event_s;

QueueHandle_t ns_event_queue;

TaskHandle_t _switch_bt_task_handle = NULL;
ns_power_handle_t _switch_power_state = NS_POWER_AWAKE;

sw_input_s _switch_input_data = {.ls_x = 2047, .ls_y = 2047, .rs_x = 2047, .rs_y = 2047};

void _switch_bt_task_standard(void *parameters);
void _switch_bt_task_empty(void *parameters);
void _switch_bt_task_short(void *parameters);
void ns_controller_input_task_set(ns_report_mode_t report_mode_type);

void switch_bt_end_task()
{
    if(_switch_bt_task_handle != NULL)
    {
        vTaskDelete(_switch_bt_task_handle);
    }
}

void ns_controller_setinputreportmode(uint8_t report_mode)
{
    return; // Debug do nothing
    char *TAG = "ns_controller_setinputreportmode";

    ESP_LOGI(TAG, "Switching to input mode: %04x", report_mode);
    switch (report_mode)
    {
    // Standard
    case 0x30:
        ESP_LOGI(TAG, "Setting standard report mode.");
        ns_controller_input_task_set(NS_REPORT_MODE_FULL);

        break;

    // SimpleHID. Data pushes only on button press/release
    case 0x3F:
        ESP_LOGI(TAG, "Setting short report mode.");
        //ns_controller_input_task_set(NS_REPORT_MODE_SIMPLE);
        break;

    // NFC/IR
    case 0x31:
    case 0x00 ... 0x03:
    default:
        // ERROR
        break;
    }
}

void ns_controller_input_task_set(ns_report_mode_t report_mode_type)
{
    const char *TAG = "ns_controller_input_task_set";

    ns_event_s event = {.event_id = NS_EVENT_REPORT_MODE_CHANGE};

    switch (report_mode_type)
    {
    default:
        ESP_LOGI(TAG, "Unhandled input task: %d", report_mode_type);
    break;

    case NS_REPORT_MODE_BLANK:
        ESP_LOGI(TAG, "Set Report Mode BLANK");
        event.report_mode = NS_REPORT_MODE_BLANK;

        if(_switch_bt_task_handle==NULL)
        {
            xTaskCreatePinnedToCore(_switch_bt_task_standard,
                                "Standard Send Task", 2048,
                                NULL, 0, &_switch_bt_task_handle, 0);
        }
        
        break;

    /*case NS_REPORT_MODE_SIMPLE:
        ESP_LOGI(TAG, "Start input SIMPLE task...");

        // Set the internal reporting mode.
        _switch_report_mode = NS_REPORT_MODE_SIMPLE;
        xTaskCreatePinnedToCore(_switch_bt_task_short,
                                "Standard Send Task", 2048,
                                NULL, 0, &_switch_bt_task_handle, 0);
        break;*/

    case NS_REPORT_MODE_FULL:
        ESP_LOGI(TAG, "Set Report Mode FULL");
        event.report_mode = NS_REPORT_MODE_FULL;
        break;
    }

    xQueueSend(ns_event_queue, &event, 0);
}

uint32_t interval_to_ticks(uint16_t interval)
{
    float num = 0.625f*(float)interval;
    uint32_t out = (uint32_t) num / portTICK_PERIOD_MS;
    return out;
}

uint32_t interval_to_us(uint16_t interval)
{
    return (uint32_t) interval * 625;
}

void btsnd_hcic_sniff_mode_cb(bool sniff, uint16_t tx_lat, uint16_t rx_lat)
{
    // Ignore all of this for debug
    return;
    if(sniff)
    {
        _sniff = true;
        _delay_time_us = rx_lat*1000;//interval_to_us(rx_lat);
        printf("Delay (ms): %d\n", rx_lat);
    }
    else
    {
        _sniff = false;
        _delay_time_us = 8000;
        printf("UnSniff: \n");
    }
    ns_reset_report_spacer();
}

/* HCI mode defenitions */
#define HCI_MODE_ACTIVE                 0x00
#define HCI_MODE_HOLD                   0x01
#define HCI_MODE_SNIFF                  0x02
#define HCI_MODE_PARK                   0x03

void btm_hcif_mode_change_cb(bool succeeded, uint16_t hci_handle, uint8_t mode, uint16_t interval)
{
    if (!succeeded) {
        printf("HCI mode change event failed\n");
        return;
    }

    switch (mode) {
        case HCI_MODE_ACTIVE:
            printf("Connection handle 0x%04x is in ACTIVE mode.\n", hci_handle);
            // Handle ACTIVE mode
            _sniff = false;
            _delay_time_us = 8000;
            break;

        case HCI_MODE_SNIFF:
            printf("Connection handle 0x%04x is in SNIFF mode. Interval: %d ms\n", hci_handle, interval);
            // Handle SNIFF mode
            
            _sniff = true;
            _delay_time_us = interval*1000;//interval_to_us(interval);
            break;

        default:
            printf("Connection handle 0x%04x is in an unknown mode (%d). Interval: %d slots\n", hci_handle, mode, interval);
            break;
    }
    //vTaskDelay(16/portTICK_PERIOD_MS);
    ns_reset_report_spacer();
}

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
        
        if(util_bt_get_paired())
        {
            ESP_LOGI(TAG, "Setting to non-connectable, non-discoverable, then attempting connection.");
            esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            util_bluetooth_connect();
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
            

            // Set host bluetooth address
            memcpy(&global_loaded_settings.switch_host_mac[0], &param->auth_cmpl.bda[0], ESP_BD_ADDR_LEN);

            // We set pairing address here
            if (!app_compare_mac(global_loaded_settings.switch_host_mac, global_loaded_settings.paired_host_mac))
            {
                app_save_host_mac();
            }

            //ns_controller_input_task_set(NS_REPORT_MODE_BLANK);
        }
        else
        {
            ESP_LOGI(TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }

    case ESP_BT_GAP_MODE_CHG_EVT:
    {
        // This is critical for Nintendo Switch to act upon.
        // If power mode is 0, there should be NO packets sent from the controller until
        // another power mode is initiated by the Nintendo Switch console.
        /*
        ns_event_s event = {.event_id = NS_EVENT_INTERVAL_CHANGE};

        if ((!_hcif_report_interval && !_hcif_report_mode))
        {   
            // set interval to sniff interval
            event.poll_interval = 0;
            //ns_controller_input_task_set(NS_REPORT_MODE_IDLE);
        }
        else
        {
            if(_hcif_report_mode == 2)
            {   
                ESP_LOGI(TAG, "Set Report Interval and non-idle: %d", _hcif_report_interval);
                event.poll_interval = _hcif_report_interval;
            }
        }

        xQueueSend(ns_event_queue, &event, 0);*/
        //ESP_LOGI(TAG, "power mode change: %d", param->mode_chg.mode);

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
                if(util_bt_get_paired())
                {
                    ESP_LOGI(TAG, "Setting to non-connectable, non-discoverable, then attempting connection.");
                    esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
                    util_bluetooth_connect();
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
                ns_event_s connect_event = {.event_id = NS_EVENT_HID_CHANGE, .hid_connected = true};
                _hid_connected = true;
                //xQueueSend(ns_event_queue, &connect_event, 0);
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
                ns_reset_report_spacer();
                //xQueueSend(ns_event_queue, &connect_event, 0);
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

    err = util_bluetooth_init(global_loaded_settings.device_mac);

    bool paired = false;

    for (uint8_t i = 0; i < 6; i++)
    {
        if (global_loaded_settings.paired_host_mac[i] > 0)
            paired = true;
    }

    if(paired)
    {
        ESP_LOGI(TAG, "Paired host found, setting paired in util.");
        util_bt_set_paired(true, global_loaded_settings.paired_host_mac);
    }

    // Starting bt mode
    err = util_bluetooth_register_app(&switch_app_params, &switch_hidd_config);
    if (err == 1)
    {
        vTaskDelay(1500 / portTICK_PERIOD_MS);

        // Set host bluetooth address
        memcpy(&global_loaded_settings.switch_host_mac[0], &global_loaded_settings.paired_host_mac[0], ESP_BD_ADDR_LEN);
    }

    return 1;
}

// Stop Nintendo Switch controller core
void core_bt_switch_stop(void)
{
    const char *TAG = "core_ns_stop";
    // ns_connected = false;
    // ns_controller_input_task_set(NS_REPORT_MODE_IDLE);
    util_bluetooth_deinit();
}

// Save Nintendo Switch bluetooth pairing
void ns_savepairing(uint8_t *host_addr)
{
    const char *TAG = "ns_savepairing";

    if (host_addr == NULL)
    {
        ESP_LOGE(TAG, "Host address is blank.");
        return;
    }

    ESP_LOGI(TAG, "Pairing to Nintendo Switch.");

    // Copy host address into settings memory.
    memcpy(global_loaded_settings.switch_host_mac, host_addr, sizeof(global_loaded_settings.switch_host_mac));

    // Save all settings send pairing info to RP2040
}

void _switch_bt_task_standard(void *parameters)
{
    ESP_LOGI("_switch_bt_task_standard", "Starting input loop task...");

    static ns_report_mode_t _report_mode = NS_REPORT_MODE_FULL;

    //_report_mode = NS_REPORT_MODE_BLANK;
    _hid_connected = false;
    _delay_time_us = DEFAULT_US_DELAY; 
    _sniff = true;

    for (;;)
    {
        static uint8_t _full_buffer[64] = {0};
        uint8_t tmp[64] = {0x00, 0x00};

        if(_hid_connected)
        {
            if(ns_send_check_nonblocking())
            {
                if((_report_mode == NS_REPORT_MODE_FULL))
                {
                    ns_report_clear(_full_buffer, 64);
                    ns_report_setinputreport_full(_full_buffer, &_switch_input_data);
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

#define ANALOG_DIGITAL_THRESH 650

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

    _switch_input_data.d_down = input->dpad_down;
    _switch_input_data.d_left = input->dpad_left;
    _switch_input_data.d_right = input->dpad_right;
    _switch_input_data.d_up = input->dpad_up;

    _switch_input_data.b_capture = input->button_capture;
    _switch_input_data.b_home = input->button_home;
    _switch_input_data.b_minus = input->button_minus;
    _switch_input_data.b_plus = input->button_plus;

    _switch_input_data.t_l = input->trigger_l;
    _switch_input_data.t_r = input->trigger_r;
    _switch_input_data.t_zl = ((input->lt >= ANALOG_DIGITAL_THRESH) ? 1 : 0) | input->trigger_zl;
    _switch_input_data.t_zr = ((input->rt >= ANALOG_DIGITAL_THRESH) ? 1 : 0) | input->trigger_zr;

    _switch_input_data.sb_left = input->button_stick_left;
    _switch_input_data.sb_right = input->button_stick_right;
}