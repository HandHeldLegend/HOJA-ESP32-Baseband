#include "core_bt_xinput.h"

#define XINPUT_TICK_DELAY (8/portTICK_PERIOD_MS)
#define XINPUT_US_DELAY (1000 * 4)

TaskHandle_t _xinput_task_handler = NULL;
interval_s _xinput_interval = {0};
static volatile bool        _xinput_connected = false;
static volatile bool        _xinput_gap_started = false;

bool xinput_send_check_nonblocking()
{
    uint32_t timestamp = get_timestamp_us();
    return interval_run(timestamp, XINPUT_US_DELAY, &_xinput_interval);
}

void _xinput_bt_input_task(void * params);

// XINPUT BLE HIDD callback
void xinput_ble_hidd_cb(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;
    static const char *TAG = "HID_DEV_BLE";

    switch (event)
    {
    case ESP_HIDD_START_EVENT:
    {
        ESP_LOGI(TAG, "START"); 
        esp_hid_ble_gap_adv_start();
        break;
    }
    case ESP_HIDD_CONNECT_EVENT:
    {
        _xinput_connected = true;
        ESP_LOGI(TAG, "CONNECT");

        xTaskCreatePinnedToCore(_xinput_bt_input_task, 
                                "XInput Send Task", 4024,
                                NULL, 0, &_xinput_task_handler, 0);
    
        app_set_connected_status(1);
        break;
    }
    case ESP_HIDD_PROTOCOL_MODE_EVENT:
    {
        ESP_LOGI(TAG, "PROTOCOL MODE[%u]: %s", param->protocol_mode.map_index, param->protocol_mode.protocol_mode ? "REPORT" : "BOOT");
        break;
    }
    case ESP_HIDD_CONTROL_EVENT:
    {
        ESP_LOGI(TAG, "CONTROL[%u]: %sSUSPEND", param->control.map_index, param->control.control ? "EXIT_" : "");
        break;
    }
    case ESP_HIDD_OUTPUT_EVENT:
    {
        //ESP_LOGI(TAG, "OUTPUT[%u]: %8s ID: %2u, Len: %d, Data:", param->output.map_index, esp_hid_usage_str(param->output.usage), param->output.report_id, param->output.length);
        if(param->output.report_id == 0x03)
        {
            //ESP_LOGI(TAG, "RUMBLE");
            /*for(uint8_t i = 0; i < param->output.length; i++)
            {
                ESP_LOGI(TAG, "%02X ", param->output.data[i]);
            }*/
            uint8_t _rumble_intensity = (param->output.data[3] >= param->output.data[4]) ? param->output.data[3] : param->output.data[4];
            if(!_rumble_intensity)
            {
                app_set_standard_haptic(0, 0);
            }
            else
            {
                app_set_standard_haptic(param->output.data[3], param->output.data[4]);
            }
            
        }
        break;
    }
    case ESP_HIDD_FEATURE_EVENT:
    {
        ESP_LOGI(TAG, "FEATURE[%u]: %8s ID: %2u, Len: %d, Data:", param->feature.map_index, esp_hid_usage_str(param->feature.usage), param->feature.report_id, param->feature.length);
        break;
    }
    case ESP_HIDD_DISCONNECT_EVENT:
    {
        _xinput_connected = false;
        ESP_LOGI(TAG, "DISCONNECT: %s", esp_hid_disconnect_reason_str(esp_hidd_dev_transport_get(param->disconnect.dev), param->disconnect.reason));

        esp_hid_ble_gap_adv_start();
        app_set_connected_status(0);
        break;
    }
    case ESP_HIDD_STOP_EVENT:
    {
        if (_xinput_task_handler != NULL)
        {
            vTaskDelete(_xinput_task_handler);
            _xinput_task_handler = NULL;
        }

        ESP_LOGI(TAG, "STOP");
        break;
    }
    default:
        break;
    }
    return;
}

// XINPUT GAP BLE event callback
void xinput_ble_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    const char *TAG = "xinput_ble_gap_cb";

    switch (event)
    {
    /*
     * SCAN
     * */
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
    {
        ESP_LOGV(TAG, "BLE GAP EVENT SCAN_PARAM_SET_COMPLETE");
        // SEND_BLE_CB();
        break;
    }
    case ESP_GAP_BLE_SCAN_RESULT_EVT:
    {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt)
        {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
        {
            // handle_ble_device_result(&scan_result->scan_rst);
            break;
        }
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            ESP_LOGV(TAG, "BLE GAP EVENT SCAN DONE: %d", scan_result->scan_rst.num_resps);
            // SEND_BLE_CB();
            break;
        default:
            break;
        }
        break;
    }
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
    {
        ESP_LOGV(TAG, "BLE GAP EVENT SCAN CANCELED");
        break;
    }

    /*
     * ADVERTISEMENT
     * */
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        ESP_LOGV(TAG, "BLE GAP ADV_DATA_SET_COMPLETE");
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        ESP_LOGV(TAG, "BLE GAP ADV_START_COMPLETE");
        break;

    /*
     * AUTHENTICATION
     * */
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        if (!param->ble_security.auth_cmpl.success)
        {
            ESP_LOGE(TAG, "BLE GAP AUTH ERROR: 0x%x", param->ble_security.auth_cmpl.fail_reason);
        }
        else
        {
            _xinput_gap_started = true;

            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));

            conn_params.latency = 0;
            conn_params.max_int = 7;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 1;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);

            ESP_LOGI(TAG, "BLE GAP AUTH SUCCESS");
            
            app_set_connected_status(1);
        }

        break;

    case ESP_GAP_BLE_KEY_EVT: // shows the ble key info share with peer device to the user.
        // ESP_LOGI(TAG, "BLE GAP KEY type = %s", esp_ble_key_type_str(param->ble_security.ble_key.key_type));
        break;

    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT: // ESP_IO_CAP_OUT
        // The app will receive this evt when the IO has Output capability and the peer device IO has Input capability.
        // Show the passkey number to the user to input it in the peer device.
        // ESP_LOGI(TAG, "BLE GAP PASSKEY_NOTIF passkey:%d", param->ble_security.key_notif.passkey);
        break;

    case ESP_GAP_BLE_NC_REQ_EVT: // ESP_IO_CAP_IO
        // The app will receive this event when the IO has DisplayYesNO capability and the peer device IO also has DisplayYesNo capability.
        // show the passkey number to the user to confirm it with the number displayed by peer device.
        ESP_LOGI(TAG, "BLE GAP NC_REQ passkey:%d", (unsigned int)param->ble_security.key_notif.passkey);

        esp_ble_confirm_reply(param->ble_security.key_notif.bd_addr, true);
        break;

    case ESP_GAP_BLE_PASSKEY_REQ_EVT: // ESP_IO_CAP_IN
        // The app will receive this evt when the IO has Input capability and the peer device IO has Output capability.
        // See the passkey number on the peer device and send it back.
        ESP_LOGI(TAG, "BLE GAP PASSKEY_REQ");
        // esp_ble_passkey_reply(param->ble_security.ble_req.bd_addr, true, 1234);
        break;

    case ESP_GAP_BLE_SEC_REQ_EVT:
        _xinput_gap_started = false;
        ESP_LOGI(TAG, "BLE GAP SEC_REQ");
        // Send the positive(true) security response to the peer device to accept the security request.
        // If not accept the security request, should send the security response with negative(false) accept value.
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        
        break;

    default:
        // ESP_LOGV(TAG, "BLE GAP EVENT %s", ble_gap_evt_str(event));
        break;
    }
}

// XInput HID report maps
esp_hid_raw_report_map_t xinput_report_maps[1] = {
    {.data = xinput_hid_report_descriptor,
     .len = (uint16_t)XINPUT_HID_REPORT_MAP_LEN}};

uint8_t xinput_hidd_service_uuid128[] = {
    0xEC,
    0x67,
    0x63,
    0x03,
    0x3D,
    0x9A,
    0x42,
    0x64,
    0xB8,
    0x95,
    0xAC,
    0x1B,
    0xE9,
    0x9D,
    0x63,
    0x79,
};

uint8_t hidd_service_uuid128[] = {
        0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};

// Bluetooth App setup data
util_bt_app_params_s xinput_app_params = {
    .ble_hidd_cb = xinput_ble_hidd_cb,
    .ble_gap_cb = xinput_ble_gap_cb,
    .bt_mode = ESP_BT_MODE_BLE,
    .appearance = ESP_HID_APPEARANCE_GAMEPAD,
    .uuid128 = hidd_service_uuid128,
};

// Takes in both XINPUT structs and returns if they match.
bool xinput_compared_is_different(xi_input_s *one, xi_input_s *two)
{
    bool ret = false;
    ret |= one->buttons_1 != two->buttons_1;
    ret |= one->buttons_2 != two->buttons_2;
    ret |= one->dpad_hat != two->dpad_hat;

    ret |= one->stick_left_x != two->stick_left_x;
    ret |= one->stick_left_y != two->stick_left_y;

    ret |= one->stick_right_x != two->stick_right_x;
    ret |= one->stick_right_y != two->stick_right_y;

    ret |= one->analog_trigger_l != two->analog_trigger_l;
    ret |= one->analog_trigger_r != two->analog_trigger_r;

    return ret;
}

/**
 * @brief Takes in a mode, and dpad inputs, and returns the proper
 * code for D-Pad Hat for HID input.
 * @param mode Type of hat_mode_t
 * @param left_right Sum of 1 - left + right dpad inputs.
 * @param up_down    Sum of 1 - down + up dpad inputs.
 */
uint8_t util_get_dpad_hat(uint8_t left_right, uint8_t up_down)
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

xi_input_s xi_input = {0};
xi_input_s xi_input_last = {0};
uint8_t xi_buffer[XI_HID_LEN];

void _xinput_bt_input_task(void * params)
{
    const char *TAG = "_xinput_bt_input_task";
    ESP_LOGI(TAG, "Starting XInput task loop.");

    static bool outputting = false;

    for(;;)
    {
        if(_xinput_connected && _xinput_gap_started)
        {
            if(xinput_send_check_nonblocking())
            {
                memcpy(xi_buffer, &xi_input, XI_HID_LEN);
                esp_hidd_dev_input_set(xinput_app_params.hid_dev, 0, XI_INPUT_REPORT_ID, xi_buffer, XI_HID_LEN);
            }   
        }
        else 
        {
            vTaskDelay(8/portTICK_PERIOD_MS);
        }
    }
}

void xinput_bt_sendinput(i2cinput_input_s *input)
{
    const char *TAG = "xinput_bt_sendinput";

    xi_input.stick_left_x   = input->lx << 4;
    xi_input.stick_left_y   = 0xFFFF - (input->ly << 4);
    xi_input.stick_right_x  = input->rx << 4;
    xi_input.stick_right_y  = 0xFFFF - (input->ry << 4);

    xi_input.analog_trigger_l = input->trigger_zl ? 1020 : (input->lt>>2);
    xi_input.analog_trigger_r = input->trigger_zr ? 1020 : (input->rt>>2);

    xi_input.bumper_l = input->trigger_l;
    xi_input.bumper_r = input->trigger_r;

    xi_input.button_back = input->button_minus;
    xi_input.button_menu = input->button_plus;

    xi_input.button_guide = input->button_home;

    xi_input.button_stick_l = input->button_stick_left;
    xi_input.button_stick_r = input->button_stick_right;

    xi_input.button_a = input->button_a;
    xi_input.button_b = input->button_b;
    xi_input.button_x = input->button_x;
    xi_input.button_y = input->button_y;

    uint8_t lr = (1 - input->dpad_left) + input->dpad_right;
    uint8_t ud = (1 - input->dpad_down) + input->dpad_up;

    xi_input.dpad_hat = util_get_dpad_hat(lr, ud);
}

esp_hid_device_config_t xinput_hidd_config = {
    .vendor_id = HID_VEND_XINPUT,
    .product_id = HID_PROD_XINPUT,
    //.vendor_id_source = 0x01,
    .version = 0x0000,
    .device_name = "Hoja XInput",
    .manufacturer_name = "HHL",
    .serial_number = "000000",
    .report_maps = xinput_report_maps,
    .report_maps_len = 1
};

// Public Functions
int core_bt_xinput_start(void)
{
    const char *TAG = "core_bt_xinput_start";
    esp_err_t ret;
    int err = 1;

    err = util_bluetooth_init(global_loaded_settings.device_mac_xinput);
    err = util_bluetooth_register_app(&xinput_app_params, &xinput_hidd_config);

    
    return err;
}

void core_bt_xinput_stop(void)
{
    const char *TAG = "core_bt_xinput_stop";

    ESP_LOGI(TAG, "Stopping core...");
    util_bluetooth_deinit();
}