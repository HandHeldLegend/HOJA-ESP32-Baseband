#include "core_bt_gamecube.h"
#include "esp_log.h"

#define DEFAULT_TICK_DELAY (8/portTICK_PERIOD_MS)
#define DEFAULT_US_DELAY (8*1000)
static volatile bool        _hid_connected = false;
static volatile uint32_t    _delay_time_us = DEFAULT_US_DELAY;
static volatile uint32_t    _delay_time_ticks = DEFAULT_TICK_DELAY; // 8ms default?
static volatile bool        _sniff = true;
static volatile bool        _gamecube_paired = false;

interval_s _report_interval = {0};

void gc_reset_report_spacer()
{
    uint32_t timestamp = get_timestamp_us();
    interval_resettable_run(timestamp, _delay_time_us, true, &_report_interval);
}

void gc_send_check_blocking()
{
    bool ok_to_send = false;
    uint32_t timestamp = 0;
    uint32_t fail_timer = 1000;
    while(!ok_to_send)
    {
        timestamp = get_timestamp_us();
        if(interval_run(timestamp, _delay_time_us, &_report_interval))
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

bool gc_send_check_nonblocking()
{
    uint32_t timestamp = get_timestamp_us();
    return interval_run(timestamp, _delay_time_us, &_report_interval);
}

TaskHandle_t _gamecube_bt_task_handle = NULL;

gc_input_s _gamecube_input_data = {.left_x = 128, .left_y = 128, .right_x = 128, .right_y = 128};

void _gamecube_bt_task_standard(void *parameters);

void gc_bt_end_task()
{
    if(_gamecube_bt_task_handle != NULL)
    {
        vTaskDelete(_gamecube_bt_task_handle);
    }
}

uint32_t gc_interval_to_ticks(uint16_t interval)
{
    float num = 0.625f*(float)interval;
    uint32_t out = (uint32_t) num / portTICK_PERIOD_MS;
    return out;
}

uint32_t gc_interval_to_us(uint16_t interval)
{
    return (uint32_t) interval * 625;
}

// GameCube BTC GAP Event Callback
void gamecube_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    const char *TAG = "gc_bt_gap_cb";
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
        if(_gamecube_bt_task_handle==NULL)
        {
            xTaskCreatePinnedToCore(_gamecube_bt_task_standard,
                                "GameCube Send Task", 4048,
                                NULL, 0, &_gamecube_bt_task_handle, 0);
        }
        break;

    case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
        ESP_LOGI(TAG, "ACL Disconnect Complete.");
        
        if(_gamecube_bt_task_handle!=NULL)
        {
            vTaskDelete(_gamecube_bt_task_handle);
            _gamecube_bt_task_handle = NULL;
        }

        app_set_connected_status(0);
        
        if(_gamecube_paired)
        {
            ESP_LOGI(TAG, "Setting to non-connectable, non-discoverable, then attempting connection.");
            esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            util_bluetooth_connect(global_loaded_settings.paired_host_gamecube_mac);
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

            if(!_gamecube_paired)
            {
                _gamecube_paired = true;
                app_save_host_mac(INPUT_MODE_GAMECUBE, &param->auth_cmpl.bda[0]);
            }
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
void gamecube_bt_hidd_cb(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
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
                if(_gamecube_paired)
                {
                    ESP_LOGI(TAG, "Setting to non-connectable, non-discoverable, then attempting connection.");
                    esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
                    util_bluetooth_connect(global_loaded_settings.paired_host_gamecube_mac);
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
                app_set_connected_status(1);
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
            // TODO

            //ESP_LOGI(TAG, "OUTPUT[%u]: %8s ID: %2u, Len: %d, Data:", param->output.map_index, esp_hid_usage_str(param->output.usage), param->output.report_id, param->output.length);
            //ESP_LOG_BUFFER_HEX(TAG, param->output.data, param->output.length);
            //ns_report_handler(param->output.report_id, param->output.data, param->output.length);
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
                //app_set_connected_status(0);
                gc_reset_report_spacer();
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

// GameCube HID report maps
esp_hid_raw_report_map_t gamecube_report_maps[1] = {
    {
        .data = gc_hid_report_descriptor,
        .len = (uint16_t)GC_HID_REPORT_MAP_LEN,
    }
};

// Bluetooth App setup data
util_bt_app_params_s gamecube_app_params = {
    .hidd_cb = gamecube_bt_hidd_cb,
    .gap_cb = gamecube_bt_gap_cb,
    .bt_mode = ESP_BT_MODE_CLASSIC_BT,
    .appearance = ESP_HID_APPEARANCE_GAMEPAD,
};

esp_hid_device_config_t gamecube_hidd_config = {
    .vendor_id = HID_VEND_NSPRO,
    .product_id = HID_PROD_GCA,
    .version = 0x0100,
    .device_name = "OpenGC BT Gamepad",
    .manufacturer_name = "HHL",
    .serial_number = "000000",
    .report_maps = gamecube_report_maps,
    .report_maps_len = 1,
};

// Attempt start of GameCube controller core
int core_bt_gamecube_start(void)
{
    const char *TAG = "core_bt_switch_start";
    esp_err_t ret;
    int err;

    err = util_bluetooth_init(global_loaded_settings.device_mac_gamecube);

    _gamecube_paired = false;

    for (uint8_t i = 0; i < 6; i++)
    {
        if (global_loaded_settings.paired_host_gamecube_mac[i] > 0)
            _gamecube_paired = true;
    }

    if(_gamecube_paired)
    {
        ESP_LOGI(TAG, "Paired host found for GC");
    }

    // Starting bt mode
    err = util_bluetooth_register_app(&gamecube_app_params, &gamecube_hidd_config);

    return 1;
}

// Stop Nintendo Switch controller core
void core_bt_gamecube_stop(void)
{
    const char *TAG = "core_ns_stop";
    // ns_connected = false;
    // ns_controller_input_task_set(NS_REPORT_MODE_IDLE);
    util_bluetooth_deinit();
}

// Save Nintendo Switch bluetooth pairing
void gamecube_savepairing(uint8_t *host_addr)
{
    const char *TAG = "gamecube_savepairing";

    if (host_addr == NULL)
    {
        ESP_LOGE(TAG, "Host address is blank.");
        return;
    }

    ESP_LOGI(TAG, "Pairing to GameCube Host.");

    app_save_host_mac(INPUT_MODE_GAMECUBE, host_addr);
}

void _gamecube_bt_task_standard(void *parameters)
{
    ESP_LOGI("_gamecube_bt_task_standard", "Starting input loop task...");

    //_report_mode = NS_REPORT_MODE_BLANK;
    _hid_connected = false;
    _delay_time_us = DEFAULT_US_DELAY; 

    for (;;)
    {
        static uint8_t _full_buffer[64] = {0};
        memcpy(_full_buffer, &_gamecube_input_data, sizeof(_gamecube_input_data));

        if(_hid_connected)
        {
            if(gc_send_check_nonblocking())
            {
                if(_hid_connected)
                    esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x01, GAMECUBE_BT_REPORT_SIZE, _full_buffer);
            }
        }
        else
        {
            vTaskDelay(16/portTICK_PERIOD_MS);
        }
    }
}

// Function to calculate the D-pad value
uint8_t gc_dpad_value(uint8_t up, uint8_t down, uint8_t left, uint8_t right) {
    if (up) {
        if (right) return 1;   // Up-Right
        if (left)  return 7;   // Up-Left
        return 0;              // Up
    }
    
    if (down) {
        if (right) return 3;   // Down-Right
        if (left)  return 5;   // Down-Left
        return 4;              // Down
    }
    
    if (right) return 2;       // Right
    if (left)  return 6;       // Left
    
    return 8;  // Center (no input)
}

#define CLAMP_0_255(value) ((value) < 0 ? 0 : ((value) > 255 ? 255 : (value)))

void gamecube_bt_sendinput(i2cinput_input_s *input)
{
    _gamecube_input_data.left_x = CLAMP_0_255(input->lx>>4);
    _gamecube_input_data.left_y = CLAMP_0_255(255-(input->ly>>4));

    _gamecube_input_data.right_x = CLAMP_0_255(input->rx>>4);
    _gamecube_input_data.right_y = CLAMP_0_255(input->ry>>4);

    _gamecube_input_data.left_trigger   = CLAMP_0_255(input->lt>>4);
    _gamecube_input_data.right_trigger  = CLAMP_0_255(input->rt>>4);

    _gamecube_input_data.buttons1.a = input->button_a;
    _gamecube_input_data.buttons1.b = input->button_b;
    _gamecube_input_data.buttons1.x = input->button_x;
    _gamecube_input_data.buttons1.y = input->button_y;

    _gamecube_input_data.buttons1.l = input->trigger_l;
    _gamecube_input_data.buttons1.r = input->trigger_r;
    _gamecube_input_data.buttons2.zl = input->trigger_zl;
    _gamecube_input_data.buttons2.zr = input->trigger_zr;

    _gamecube_input_data.buttons2.start = input->button_plus;

    _gamecube_input_data.dpad.dpad = gc_dpad_value(input->dpad_up, input->dpad_down, input->dpad_left, input->dpad_right);

    // Need DPAD implementation
}