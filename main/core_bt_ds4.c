#include "core_bt_ds4.h"

TaskHandle_t _ds4_bt_task_handle = NULL;

const uint8_t ds4_hid_report_descriptor[] = {
        0x05, 0x01,         /*  Usage Page (Desktop),               */
        0x09, 0x05,         /*  Usage (Gamepad),                    */
        0xA1, 0x01,         /*  Collection (Application),           */
        0x85, 0x01,         /*      Report ID (1),                  */
        0x09, 0x30,         /*      Usage (X),                      */
        0x09, 0x31,         /*      Usage (Y),                      */
        0x09, 0x32,         /*      Usage (Z),                      */
        0x09, 0x35,         /*      Usage (Rz),                     */
        0x15, 0x00,         /*      Logical Minimum (0),            */
        0x26, 0xFF, 0x00,   /*      Logical Maximum (255),          */
        0x75, 0x08,         /*      Report Size (8),                */
        0x95, 0x04,         /*      Report Count (4),               */
        0x81, 0x02,         /*      Input (Variable),               */
        0x09, 0x39,         /*      Usage (Hat Switch),             */
        0x15, 0x00,         /*      Logical Minimum (0),            */
        0x25, 0x07,         /*      Logical Maximum (7),            */
        0x35, 0x00,         /*      Physical Minimum (0),           */
        0x46, 0x3B, 0x01,   /*      Physical Maximum (315),         */
        0x65, 0x14,         /*      Unit (Degrees),                 */
        0x75, 0x04,         /*      Report Size (4),                */
        0x95, 0x01,         /*      Report Count (1),               */
        0x81, 0x42,         /*      Input (Variable, Null State),   */
        0x65, 0x00,         /*      Unit,                           */
        0x05, 0x09,         /*      Usage Page (Button),            */
        0x19, 0x01,         /*      Usage Minimum (01h),            */
        0x29, 0x0E,         /*      Usage Maximum (0Eh),            */
        0x15, 0x00,         /*      Logical Minimum (0),            */
        0x25, 0x01,         /*      Logical Maximum (1),            */
        0x75, 0x01,         /*      Report Size (1),                */
        0x95, 0x0E,         /*      Report Count (14),              */
        0x81, 0x02,         /*      Input (Variable),               */
        0x06, 0x00, 0xFF,   /*      Usage Page (FF00h),             */
        0x09, 0x20,         /*      Usage (20h),                    */
        0x75, 0x06,         /*      Report Size (6),                */
        0x95, 0x01,         /*      Report Count (1),               */
        0x15, 0x00,         /*      Logical Minimum (0),            */
        0x25, 0x3F,         /*      Logical Maximum (63),           */
        0x81, 0x02,         /*      Input (Variable),               */
        0x05, 0x01,         /*      Usage Page (Desktop),           */
        0x09, 0x33,         /*      Usage (Rx),                     */
        0x09, 0x34,         /*      Usage (Ry),                     */
        0x15, 0x00,         /*      Logical Minimum (0),            */
        0x26, 0xFF, 0x00,   /*      Logical Maximum (255),          */
        0x75, 0x08,         /*      Report Size (8),                */
        0x95, 0x02,         /*      Report Count (2),               */
        0x81, 0x02,         /*      Input (Variable),               */
        0x06, 0x00, 0xFF,   /*      Usage Page (FF00h),             */
        0x09, 0x21,         /*      Usage (21h),                    */
        0x95, 0x03,         /*      Report Count (3),               */
        0x81, 0x02,         /*      Input (Variable),               */
        0x05, 0x01,         /*      Usage Page (Desktop),           */
        0x19, 0x40,         /*      Usage Minimum (40h),            */
        0x29, 0x42,         /*      Usage Maximum (42h),            */
        0x16, 0x00, 0x80,   /*      Logical Minimum (-32768),       */
        0x26, 0x00, 0x7F,   /*      Logical Maximum (32767),        */
        0x75, 0x10,         /*      Report Size (16),               */
        0x95, 0x03,         /*      Report Count (3),               */
        0x81, 0x02,         /*      Input (Variable),               */
        0x19, 0x43,         /*      Usage Minimum (43h),            */
        0x29, 0x45,         /*      Usage Maximum (45h),            */
        0x16, 0x00, 0xE0,   /*      Logical Minimum (-8192),        */
        0x26, 0xFF, 0x1F,   /*      Logical Maximum (8191),         */
        0x95, 0x03,         /*      Report Count (3),               */
        0x81, 0x02,         /*      Input (Variable),               */
        0x06, 0x00, 0xFF,   /*      Usage Page (FF00h),             */
        0x09, 0x21,         /*      Usage (21h),                    */
        0x15, 0x00,         /*      Logical Minimum (0),            */
        0x26, 0xFF, 0x00,   /*      Logical Maximum (255),          */
        0x75, 0x08,         /*      Report Size (8),                */
        0x95, 0x27,         /*      Report Count (39),              */
        0x81, 0x02,         /*      Input (Variable),               */
        0x85, 0x05,         /*      Report ID (5),                  */
        0x09, 0x22,         /*      Usage (22h),                    */
        0x95, 0x1F,         /*      Report Count (31),              */
        0x91, 0x02,         /*      Output (Variable),              */
        0xC0                /*  End Collection                      */
};

void core_bt_ds4_stop() {
    return;
}

void _ds4_bt_task(void *parameters);

void ds4_task_start(bool start) {
    if(start) {
        xTaskCreatePinnedToCore(_ds4_bt_task, "_ds4_bt_task", 4096, NULL, 5, &_ds4_bt_task_handle, 1);
    } else {
        vTaskDelete(_ds4_bt_task_handle);
    }
}

// DS4 BTC GAP Event Callback
void ds4_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    const char *TAG = "ds4_bt_gap_cb";
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
        ESP_LOGI(TAG, "ACL Connect Complete.");
        break;

    case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
        ESP_LOGI(TAG, "ACL Disconnect Complete.");
        //ns_controller_input_task_set(NS_REPORT_MODE_IDLE);
        break;
    
    case ESP_BT_GAP_AUTH_CMPL_EVT:
    {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGI(TAG, "authentication success: %s", param->auth_cmpl.device_name);
            //esp_log_buffer_hex(TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
            

            // Set host bluetooth address
            //memcpy(&global_loaded_settings.switch_host_mac[0], &param->auth_cmpl.bda[0], ESP_BD_ADDR_LEN);

            // We set pairing address here
            //if (!app_compare_mac(global_loaded_settings.switch_host_mac, global_loaded_settings.paired_host_mac))
            //{
            //    app_save_host_mac();
            //}

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
        ESP_LOGI(TAG, "power mode change: %d", param->mode_chg.mode);
        if (param->mode_chg.mode == ESP_BT_PM_MD_ACTIVE )
        {
            //_report_interval = 8;
            //ns_controller_sleep_handle(NS_POWER_SLEEP);
        }
        else
        {
            //_report_interval = 30;
            //ns_controller_sleep_handle(NS_POWER_AWAKE);
        }


        break;
    }

    default:
        ESP_LOGI(TAG, "UNKNOWN GAP EVT: %d", event);
        break;
    }
}

// Callbacks for HID report events
void ds4_bt_hidd_cb(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;
    const char *TAG = "ds4_bt_hidd_cb";

    switch (event)
    {
        case ESP_HIDD_START_EVENT:
        {
            if (param->start.status == ESP_OK)
            {
                ESP_LOGI(TAG, "START OK");
                //if(util_bt_get_paired())
                //{
                //    ESP_LOGI(TAG, "Setting to non-connectable, non-discoverable, then attempting connection.");
                //    esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
                //    util_bluetooth_connect();
                //}
                //else
                //{
                //    ESP_LOGI(TAG, "Setting to connectable, discoverable.");
                //    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
                //}
            }
            break;
        }

        case ESP_HIDD_CONNECT_EVENT:
        {
            if (param->connect.status == ESP_OK)
            {
                ESP_LOGI(TAG, "CONNECT OK");
                ESP_LOGI(TAG, "Setting to non-connectable, non-discoverable");
                esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
                app_set_connected_status(1);
                // START INPUT TASK
                ds4_task_start(true);
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
            // HANDLE RUMBLE 0x05 report ID
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
                ESP_LOGI(TAG, "DISCONNECT OK");
                // STOP INPUT TASK
                app_set_connected_status(0);
                ESP_LOGI(TAG, "Setting to connectable, discoverable again");
                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            }
            else
            {
                ESP_LOGE(TAG, "DISCONNECT failed!");
            }
            break;
        }

        case ESP_HIDD_STOP_EVENT:
        {
            ESP_LOGI(TAG, "STOP");
            break;
        }

        default:
            break;
    }
}

// DS4 HID report maps
esp_hid_raw_report_map_t ds4_report_maps[1] = {
    {
        .data = ds4_hid_report_descriptor,
        .len = (uint16_t)162,
    }};

// Bluetooth App setup data
util_bt_app_params_s ds4_app_params = {
    .hidd_cb = ds4_bt_hidd_cb,
    .gap_cb = ds4_bt_gap_cb,
    .bt_mode = ESP_BT_MODE_CLASSIC_BT,
    .appearance = ESP_HID_APPEARANCE_GAMEPAD,
};

esp_hid_device_config_t ds4_hidd_config = {
    .vendor_id = 0x054C,
    .product_id = 0x09cc,
    .version = 0x0100,
    .device_name = "DS4 BT",
    .manufacturer_name = "HHL",
    .serial_number = "000000",
    .report_maps = ds4_report_maps,
    .report_maps_len = 1,
};

int core_bt_ds4_start() {
    const char *TAG = "core_bt_ds4_start";
    esp_err_t ret;
    int err;

    //err = util_bluetooth_init(global_loaded_settings.device_mac);

    bool paired = false;


    // Starting bt mode
    err = util_bluetooth_register_app(&ds4_app_params, &ds4_hidd_config);
    if (err == 1)
    {
        vTaskDelay(1500 / portTICK_PERIOD_MS);

        // Set host bluetooth address
        //memcpy(&global_loaded_settings.switch_host_mac[0], &global_loaded_settings.paired_host_mac[0], ESP_BD_ADDR_LEN);
    }

    return 1;
}

void _ds4_bt_task(void *parameters) {
    const char *TAG = "_ds4_bt_task";
    ESP_LOGI(TAG, "Starting DS4 BT task.");

    while(1) {
        static uint8_t _full_buffer[64] = {0x00, 0x00, 0x00, 0x00, 0x00};

        esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 0x01, 64, _full_buffer);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void ds4_bt_sendinput(i2cinput_input_s *input)
{
    // Define later
}
