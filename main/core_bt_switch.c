#include "core_bt_switch.h"


/**
 * @brief NS Core Report mode enums
*/
typedef enum
{
    NS_REPORT_MODE_IDLE,
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
ns_report_mode_t _switch_report_mode = NS_REPORT_MODE_IDLE;

sw_input_s _switch_input_data = {};

void _switch_bt_task_standard(void * parameters);
void _switch_bt_task_empty(void * parameters);
void _switch_bt_task_short(void * parameters);
void ns_controller_input_task_set(ns_report_mode_t report_mode_type);

void ns_controller_setinputreportmode(uint8_t report_mode)
{
    char* TAG = "ns_controller_setinputreportmode";

    ESP_LOGI(TAG, "Switching to input mode: %04x", report_mode);
    switch(report_mode)
    {
        // Standard
        case 0x30:
            ESP_LOGI(TAG, "Starting standard report mode.");
            ns_controller_input_task_set(NS_REPORT_MODE_FULL);
            break;

        // SimpleHID. Data pushes only on button press/release
        case 0x3F:
            ESP_LOGI(TAG, "Starting short report mode.");
            ns_controller_input_task_set(NS_REPORT_MODE_SIMPLE);
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
    const char* TAG = "ns_controller_input_task_set";
    switch(report_mode_type)
    {
        default:
        case NS_REPORT_MODE_IDLE:
            ESP_LOGI(TAG, "Start input IDLE task...");
            // Just stop all tasks and clear report mode internal.
            if (_switch_bt_task_handle != NULL)
            {
                vTaskDelete(_switch_bt_task_handle);
                _switch_bt_task_handle = NULL;
            }
            _switch_report_mode = NS_REPORT_MODE_IDLE;
            break;

        case NS_REPORT_MODE_BLANK:
            ESP_LOGI(TAG, "Start input BLANK task...");
            if (_switch_bt_task_handle != NULL)
            {
                vTaskDelete(_switch_bt_task_handle);
                _switch_bt_task_handle = NULL;
            }

            _switch_report_mode = NS_REPORT_MODE_BLANK;
            xTaskCreatePinnedToCore(_switch_bt_task_empty, 
                                "Blank Send Task", 2048,
                                NULL, 0, &_switch_bt_task_handle, 0);
            break;

        case NS_REPORT_MODE_SIMPLE:
            ESP_LOGI(TAG, "Start input SIMPLE task...");
            if (_switch_bt_task_handle != NULL)
            {
                vTaskDelete(_switch_bt_task_handle);
                _switch_bt_task_handle = NULL;
            }

            // Set the internal reporting mode.
            _switch_report_mode = NS_REPORT_MODE_SIMPLE;
            xTaskCreatePinnedToCore(_switch_bt_task_short, 
                            "Standard Send Task", 2048,
                            NULL, 0, &_switch_bt_task_handle, 0);
            break;

        case NS_REPORT_MODE_FULL:
            ESP_LOGI(TAG, "Start input FULL task...");
            if (_switch_bt_task_handle != NULL)
            {
                vTaskDelete(_switch_bt_task_handle);
                _switch_bt_task_handle = NULL;
            }
            _switch_report_mode = NS_REPORT_MODE_FULL;
            xTaskCreatePinnedToCore(_switch_bt_task_standard, 
                            "Standard Send Task", 2048,
                            NULL, 0, &_switch_bt_task_handle, 0);
            break;
    }
}

void ns_controller_sleep_handle(ns_power_handle_t power_type)
{
    const char* TAG = "ns_controller_sleep_handle";
    switch(power_type)
    {
        default:
        case NS_POWER_AWAKE:
            ESP_LOGI(TAG, "Controller set to awake.");
            _switch_power_state = NS_POWER_AWAKE;
            ns_controller_input_task_set(_switch_report_mode);
            break;

        case NS_POWER_SLEEP:
            ESP_LOGI(TAG, "Controller set to sleep.");
            _switch_power_state = NS_POWER_SLEEP;
            if (_switch_bt_task_handle != NULL)
            {
                vTaskDelete(_switch_bt_task_handle);
                _switch_bt_task_handle = NULL;
            }
            break;
    }
}

// SWITCH BTC GAP Event Callback
void switch_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    const char* TAG = "switch_bt_gap_cb";
    switch (event) 
    {
        case ESP_BT_GAP_DISC_RES_EVT:
            ESP_LOGI(TAG, "ESP_BT_GAP_DISC_RES_EVT");
            //esp_log_buffer_hex(TAG, param->disc_res.bda, ESP_BD_ADDR_LEN);
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
        case ESP_BT_GAP_AUTH_CMPL_EVT:{
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "authentication success: %s", param->auth_cmpl.device_name);
                esp_log_buffer_hex(TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
                ns_controller_input_task_set(NS_REPORT_MODE_BLANK);

                // Set host bluetooth address
                memcpy(&global_loaded_settings.switch_host_mac[0], &param->auth_cmpl.bda[0], ESP_BD_ADDR_LEN);

            } else {
                ESP_LOGI(TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
            }
            break;
        }

        case ESP_BT_GAP_MODE_CHG_EVT:{
            // This is critical for Nintendo Switch to act upon.
            // If power mode is 0, there should be NO packets sent from the controller until
            // another power mode is initiated by the Nintendo Switch console.
            ESP_LOGI(TAG, "power mode change: %d", param->mode_chg.mode);
            if (param->mode_chg.mode == 0)
            {
                ns_controller_sleep_handle(NS_POWER_SLEEP);
            }
            else
            {
                ns_controller_sleep_handle(NS_POWER_AWAKE);  
            }
            break;
        }
        
        default:
            ESP_LOGI(TAG, "UNKNOWN GAP EVT: %d", event);
            break; 
    }
}

// Callbacks for HID report events
void switch_bt_hidd_cb(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    const char* TAG = "ns_bt_hidd_cb";

    switch (event) {
        case ESP_HIDD_INIT_EVT:
            if (param->init.status == ESP_HIDD_SUCCESS) {
                //ESP_LOGI(TAG, "init hidd success!");
            } else {
                ESP_LOGI(TAG, "init hidd failed!");
            }
            break;
        case ESP_HIDD_DEINIT_EVT:
            break;
        case ESP_HIDD_REGISTER_APP_EVT:
            if (param->register_app.status == ESP_HIDD_SUCCESS) {
                ESP_LOGI(TAG, "Register HIDD app parameters success!");
                //if(param->register_app.bd_addr == NULL)
                //{
                //    ESP_LOGI(TAG, "bd_addr is undefined!");
                //}
            } else {
                ESP_LOGI(TAG, "Register HIDD app parameters failed!");
            }
            break;
        case ESP_HIDD_UNREGISTER_APP_EVT:
            if (param->unregister_app.status == ESP_HIDD_SUCCESS) {
                ESP_LOGI(TAG, "unregister app success!");
            } else {
                ESP_LOGI(TAG, "unregister app failed!");
            }
            break;
        case ESP_HIDD_OPEN_EVT:
            if (param->open.status == ESP_HIDD_SUCCESS) {
                if (param->open.conn_status == ESP_HIDD_CONN_STATE_CONNECTING) {
                    ESP_LOGI(TAG, "connecting...");
                } else if (param->open.conn_status == ESP_HIDD_CONN_STATE_CONNECTED) {

                    ESP_LOGI(TAG, "connected to %02x:%02x:%02x:%02x:%02x:%02x", param->open.bd_addr[0],
                            param->open.bd_addr[1], param->open.bd_addr[2], param->open.bd_addr[3], param->open.bd_addr[4],
                            param->open.bd_addr[5]);
                    ESP_LOGI(TAG, "making self non-discoverable and non-connectable.");
                    esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
                    ns_controller_input_task_set(NS_REPORT_MODE_SIMPLE);

                } else {
                    ESP_LOGI(TAG, "unknown connection status");
                    ESP_LOGI(TAG, "making self non-discoverable and non-connectable.");
                    esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
                }
            } else {
                ESP_LOGI(TAG, "open failed!");
                ESP_LOGI(TAG, "making self non-discoverable and non-connectable.");
                esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            }
            break;
        case ESP_HIDD_CLOSE_EVT:
            ESP_LOGI(TAG, "ESP_HIDD_CLOSE_EVT");
            if (param->close.status == ESP_HIDD_SUCCESS) {
                if (param->close.conn_status == ESP_HIDD_CONN_STATE_DISCONNECTING) {
                    ESP_LOGI(TAG, "disconnecting...");
                } else if (param->close.conn_status == ESP_HIDD_CONN_STATE_DISCONNECTED) {
                    ESP_LOGI(TAG, "disconnected!");
                    app_set_connected(0);
                    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
                } else {
                    ESP_LOGI(TAG, "unknown connection status");
                }
            } else {
                ESP_LOGI(TAG, "close failed!");
                ESP_LOGI(TAG, "making self non-discoverable and non-connectable.");
                esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            }
            break;
        case ESP_HIDD_SEND_REPORT_EVT:
            break;
        case ESP_HIDD_REPORT_ERR_EVT:
            ESP_LOGI(TAG, "ESP_HIDD_REPORT_ERR_EVT");
            break;
        case ESP_HIDD_GET_REPORT_EVT:
            ESP_LOGI(TAG, "ESP_HIDD_GET_REPORT_EVT id:0x%02x, type:%d, size:%d", param->get_report.report_id,
                    param->get_report.report_type, param->get_report.buffer_size);
            break;
        case ESP_HIDD_SET_REPORT_EVT:
            ESP_LOGI(TAG, "ESP_HIDD_SET_REPORT_EVT");
            break;
        case ESP_HIDD_SET_PROTOCOL_EVT:
            ESP_LOGI(TAG, "ESP_HIDD_SET_PROTOCOL_EVT");
            break;
        case ESP_HIDD_INTR_DATA_EVT:
            // Send interrupt data to command handler
            ns_report_handler(param->intr_data.report_id, param->intr_data.data, param->intr_data.len);
            break;
        case ESP_HIDD_VC_UNPLUG_EVT:
            ESP_LOGI(TAG, "ESP_HIDD_VC_UNPLUG_EVT");
            if (param->vc_unplug.status == ESP_HIDD_SUCCESS) {
                if (param->close.conn_status == ESP_HIDD_CONN_STATE_DISCONNECTED) {
                    app_set_connected(0);
                    ESP_LOGI(TAG, "disconnected!");
                } else {
                    ESP_LOGI(TAG, "unknown connection status");
                }
            } else {
                ESP_LOGI(TAG, "close failed!");
                ESP_LOGI(TAG, "making self non-discoverable and non-connectable.");
                esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            }
            break;
        default:
            ESP_LOGI(TAG, "UNKNOWN EVENT: %d", event);

            break;
        }
}

// Switch HID report maps
esp_hid_raw_report_map_t switch_report_maps[1] = {
    {
        .data = procon_hid_descriptor,
        .len = (uint16_t) PROCON_HID_REPORT_MAP_LEN,
    }
};

// Bluetooth App setup data
util_bt_app_params_s switch_app_params = {
    .hidd_cb            = switch_bt_hidd_cb,
    .gap_cb             = switch_bt_gap_cb,
    .bt_mode            = ESP_BT_MODE_CLASSIC_BT,
    .appearance         = ESP_HID_APPEARANCE_GAMEPAD,
};

esp_hid_device_config_t switch_hidd_config = {
    .vendor_id  = HID_VEND_NSPRO,
    .product_id = HID_PROD_NSPRO,
    .version    = 0x0000,
    .device_name = "Pro Controller",
    .manufacturer_name = "Nintendo",
    .serial_number = "000000",
    .report_maps    = switch_report_maps,
    .report_maps_len = 1,
};

// Attempt start of Nintendo Switch controller core
int core_bt_switch_start(void)
{
    const char* TAG = "core_bt_switch_start";
    esp_err_t ret;
    int err;

    // Convert calibration data
    switch_analog_calibration_init();

    err = util_bluetooth_init(global_loaded_settings.device_mac);

    bool paired = false;

    for(uint8_t i = 0; i < 6; i++)
    {
        if(global_loaded_settings.paired_host_mac[i] > 0) paired = true;
    }

    // If we are already paired, attempt connection
    if (paired)
    {
        ESP_LOGI(TAG, "NS Paired, attempting to connect...");
        err = util_bluetooth_register_app(&switch_app_params, &switch_hidd_config, false);
        if (err == 1)
        {
            vTaskDelay(1500/portTICK_PERIOD_MS);

            // Set host bluetooth address
            memcpy(&global_loaded_settings.switch_host_mac[0], &global_loaded_settings.paired_host_mac[0], ESP_BD_ADDR_LEN);

            util_bluetooth_connect(global_loaded_settings.paired_host_mac);
        }
        
    }
    else
    {
        // Not paired, await pairing connection
        ESP_LOGI(TAG, "NS not Paired, put into advertise mode...");
        err = util_bluetooth_register_app(&switch_app_params, &switch_hidd_config, true);
    }

    return 1;
}

// Stop Nintendo Switch controller core
void core_bt_switch_stop(void)
{
    const char* TAG = "core_ns_stop";
    //ns_connected = false;
    //ns_controller_input_task_set(NS_REPORT_MODE_IDLE);
    util_bluetooth_deinit();
}

// Save Nintendo Switch bluetooth pairing
void ns_savepairing(uint8_t* host_addr)
{
    const char* TAG = "ns_savepairing";

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

void _switch_bt_task_standard(void * parameters)
{
    ESP_LOGI("_switch_bt_task_standard", "Starting input loop task...");
    for(;;)
    {
        static uint8_t _full_buffer[64] = {0};

        ns_report_clear(_full_buffer, 64);
        ns_report_setinputreport_full(_full_buffer, &_switch_input_data);

        ns_report_settimer(_full_buffer);
        ns_report_setbattconn(_full_buffer);
        //_full_buffer[12] = 0x70;

        esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x30, 47, _full_buffer);

        vTaskDelay(8 / portTICK_PERIOD_MS);
    }
}

// Task used to send short or simple inputs.
// Only sends input when data is changed! SOOPER.
void _switch_bt_task_short(void * parameters)
{
    const char* TAG = "_switch_bt_task_short";
    ESP_LOGI(TAG, "Sending short (0x3F) reports on core %d\n", xPortGetCoreID());

    for(;;)
    {
        static uint8_t _short_buffer[64] = {0};

        ns_report_clear(_short_buffer, 64);

        _ns_report_setinputreport_short(_short_buffer);

        esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x3F, 12, _short_buffer);
        
        vTaskDelay(16 / portTICK_PERIOD_MS); 
    }
}

void _switch_bt_task_empty(void * parameters)
{
    const char* TAG = "ns_report_task_sendempty";
    ESP_LOGI(TAG, "Sending empty reports on core %d\n", xPortGetCoreID());
    uint8_t tmp[2] = {0x00, 0x00};

    for(;;)
    {   
        esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0xA1, 2, tmp);
        vTaskDelay(8 / portTICK_PERIOD_MS);
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

    _switch_input_data.t_l = input->trigger_l;
    _switch_input_data.t_r = input->trigger_r;
    _switch_input_data.t_zl = input->trigger_zl;
    _switch_input_data.t_zr = input->trigger_zr;

    _switch_input_data.sb_left  = input->button_stick_left;
    _switch_input_data.sb_right = input->button_stick_right;
}
