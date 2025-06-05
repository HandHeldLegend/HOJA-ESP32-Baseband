#include "util_bt_hid.h"
#if CONFIG_BT_SDP_COMMON_ENABLED
#include "esp_sdp_api.h"
#endif /* CONFIG_BT_SDP_COMMON_ENABLED */
#include "sdkconfig.h"

esp_bt_controller_config_t bt_cfg = {0};
TaskHandle_t _util_bt_timeout_task = NULL;

typedef struct 
{
    uint16_t    vendor_id;
    uint8_t     vendor_id_source; /*!< HID Vendor Source ID (0x1=BT, 0x2=USB IF) */
    uint16_t    product_id;
    uint16_t    version;
} util_bt_store_s;

uint8_t _util_bt_host_mac[6] = {0};
bool _util_bt_host_paired = false;
util_bt_store_s _util_bt_store = {0};

// BT Classic SDP callback
#if CONFIG_BT_SDP_COMMON_ENABLED
static void util_bt_sdp_cb(esp_sdp_cb_event_t event, esp_sdp_cb_param_t *param)
{
    const char* TAG = "util_bt_sdp_cb";
    
    switch (event) {
    case ESP_SDP_INIT_EVT:
        ESP_LOGI(TAG, "ESP_SDP_INIT_EVT: status:%d", param->init.status);
        if (param->init.status == ESP_SDP_SUCCESS) {
            esp_bluetooth_sdp_dip_record_t dip_record = {
                .hdr =
                    {
                        .type = ESP_SDP_TYPE_DIP_SERVER,
                    },
                .vendor           = _util_bt_store.vendor_id,
                .vendor_id_source = _util_bt_store.vendor_id_source,
                .product          = _util_bt_store.product_id,
                .version          = _util_bt_store.version,
                .primary_record   = true,
            };
            esp_sdp_create_record((esp_bluetooth_sdp_record_t *)&dip_record);
        }
        break;
    case ESP_SDP_DEINIT_EVT:
        ESP_LOGI(TAG, "ESP_SDP_DEINIT_EVT: status:%d", param->deinit.status);
        break;
    case ESP_SDP_SEARCH_COMP_EVT:
        ESP_LOGI(TAG, "ESP_SDP_SEARCH_COMP_EVT: status:%d", param->search.status);
        break;
    case ESP_SDP_CREATE_RECORD_COMP_EVT:
        ESP_LOGI(TAG, "ESP_SDP_CREATE_RECORD_COMP_EVT: status:%d, handle:0x%x", param->create_record.status,
                 param->create_record.record_handle);
        break;
    case ESP_SDP_REMOVE_RECORD_COMP_EVT:
        ESP_LOGI(TAG, "ESP_SDP_REMOVE_RECORD_COMP_EVT: status:%d", param->remove_record.status);
        break;
    default:
        break;
    }
}
#endif /* CONFIG_BT_SDP_COMMON_ENABLED */

// BT Classic HID Callbacks

void util_bt_set_paired(bool paired, uint8_t *host)
{
    _util_bt_host_paired = paired;
    memcpy(_util_bt_host_mac, host, 6);
}

bool util_bt_get_paired(void)
{
    return _util_bt_host_paired;
}

// Public variables

// Status of BT HID Gamepad Utility
util_bt_hid_status_t util_bt_hid_status = UTIL_BT_HID_STATUS_IDLE;
util_bt_hid_mode_t util_bt_hid_mode = UTIL_BT_MODE_CLASSIC;

// Private functions

// Register app with BT Classic
int bt_register_app(util_bt_app_params_s *util_bt_app_params, esp_hid_device_config_t *hidd_device_config)
{
    const char* TAG = "bt_register_app";

    esp_err_t ret;

    util_bt_hid_mode = UTIL_BT_MODE_CLASSIC;

    esp_bt_cod_t hid_cod;

    uint32_t cod = 0x002508;

    memcpy(&hid_cod, &cod, sizeof(uint32_t));
    esp_bt_gap_set_cod(hid_cod, ESP_BT_INIT_COD);

    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

    if ((ret = esp_bt_gap_register_callback(util_bt_app_params->gap_cb)) != ESP_OK) 
    {
        ESP_LOGE(TAG, "GAP callback register failed: %s\n", esp_err_to_name(ret));
        return -1;
    }

    ESP_LOGI(TAG, "Register HID device.");

    //if ((ret = esp_bt_hid_device_register_callback(util_bt_app_params->hidd_cb)) != ESP_OK)
    if((ret = esp_hidd_dev_init(hidd_device_config, ESP_HID_TRANSPORT_BT, util_bt_app_params->hidd_cb, &util_bt_app_params->hid_dev)) != ESP_OK)
    {
        ESP_LOGE(TAG, "HID device failed to start: %s\n", esp_err_to_name(ret));
        return -1;
    }

    esp_bt_dev_set_device_name(hidd_device_config->device_name);
    
    return 1;
}

// Register app with BLE
int ble_register_app(util_bt_app_params_s *util_bt_app_params, esp_hid_device_config_t *hidd_device_config)
{
    esp_err_t ret;
    const char* TAG = "ble_register_app";

    util_bt_hid_mode = UTIL_BT_MODE_BLE;

    // Register GAP callback
    if ((ret = esp_ble_gap_register_callback(util_bt_app_params->ble_gap_cb)) != ESP_OK) {
        ESP_LOGE(TAG, "esp_ble_gap_register_callback failed: %d", ret);
        return -1;
    }

    // Register GATTS callback
    if ((ret = esp_ble_gatts_register_callback(esp_hidd_gatts_event_handler)) != ESP_OK) {
        ESP_LOGE(TAG, "GATTS register callback failed: %d", ret);
        return -1;
    }

    esp_ble_adv_data_t ble_adv_data = {
        .set_scan_rsp = false,
        .include_name = true,
        .include_txpower = true,
        .min_interval = 0x0001, //slave connection min interval, Time = min_interval * 1.25 msec
        .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
        .appearance = util_bt_app_params->appearance,
        .manufacturer_len = 0,
        .p_manufacturer_data =  NULL,
        .service_data_len = 0,
        .p_service_data = NULL,
        .service_uuid_len = 16 * sizeof(uint8_t),
        .p_service_uuid = util_bt_app_params->uuid128,
        .flag = 0x6,
    };

    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;
    //esp_ble_io_cap_t iocap = ESP_IO_CAP_OUT;//you have to enter the key on the host
    //esp_ble_io_cap_t iocap = ESP_IO_CAP_IN;//you have to enter the key on the device
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;//you have to agree that key matches on both
    //esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;//device is not capable of input or output, unsecure
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t key_size = 16; //the key size should be 7~16 bytes
    uint32_t passkey = 1234;//ESP_IO_CAP_OUT

    if ((ret = esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, 1)) != ESP_OK) {
        ESP_LOGE(TAG, "GAP set_security_param AUTHEN_REQ_MODE failed: %d", ret);
        return -1;
    }

    if ((ret = esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, 1)) != ESP_OK) {
        ESP_LOGE(TAG, "GAP set_security_param IOCAP_MODE failed: %d", ret);
        return -1;
    }

    if ((ret = esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, 1)) != ESP_OK) {
        ESP_LOGE(TAG, "GAP set_security_param SET_INIT_KEY failed: %d", ret);
        return -1;
    }

    if ((ret = esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, 1)) != ESP_OK) {
        ESP_LOGE(TAG, "GAP set_security_param SET_RSP_KEY failed: %d", ret);
        return -1;
    }

    if ((ret = esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, 1)) != ESP_OK) {
        ESP_LOGE(TAG, "GAP set_security_param MAX_KEY_SIZE failed: %d", ret);
        return -1;
    }

    if ((ret = esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t))) != ESP_OK) {
        ESP_LOGE(TAG, "GAP set_security_param SET_STATIC_PASSKEY failed: %d", ret);
        return -1;
    }

    if ((ret = esp_ble_gap_set_device_name(hidd_device_config->device_name)) != ESP_OK) {
        ESP_LOGE(TAG, "GAP set_device_name failed: %d", ret);
        return -1;
    }

    if ((ret = esp_ble_gap_config_adv_data(&ble_adv_data)) != ESP_OK) {
        ESP_LOGE(TAG, "GAP config_adv_data failed: %d", ret);
        return -1;
    }

    ESP_ERROR_CHECK(esp_hidd_dev_init(hidd_device_config, ESP_HID_TRANSPORT_BLE, util_bt_app_params->ble_hidd_cb, &util_bt_app_params->hid_dev));
    return 1;
}


//-----------------------------
// Public functions

/**
 * @brief Initialize HID Gamepad Bluetooth app. It will automatically start the BT
 * Controller in the appropriate mode based on SDK Settings.
 * 
 * @param mac_address Pointer to uint8_t array of Mac address (8 long)
*/
int util_bluetooth_init(uint8_t *mac_address)
{
    const char* TAG = "util_bluetooth_init";
    esp_err_t ret;

    if (util_bt_hid_status > UTIL_BT_HID_STATUS_IDLE)
    {
        ESP_LOGE(TAG, "Already initialized or running.");
        return -1;
    }

    if (!mac_address)
    {
        ESP_LOGE(TAG, "Mac address length is not 8 bytes. Using default address.");
    }
    else
    {
        ESP_LOGI(TAG, "Setting mac address...");
        esp_log_buffer_hex(TAG, mac_address, 6);
        esp_base_mac_addr_set(mac_address);
    }

    esp_bt_mode_t mode = ESP_BT_MODE_BTDM;

    #if CONFIG_BTDM_CTRL_MODE_BTDM
        ESP_LOGI(TAG, "BT Dual Mode enabled.");
        mode = ESP_BT_MODE_BTDM;
    #elif (CONFIG_BT_HID_DEVICE_ENABLED && !CONFIG_BT_BLE_ENABLED)
        // Release BT BLE mode memory
        ESP_LOGI(TAG, "BT Classic HID only enabled. Release BLE Memory");
        ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
        mode = ESP_BT_MODE_CLASSIC_BT;
    #elif (CONFIG_BT_BLE_ENABLED && !CONFIG_BT_HID_DEVICE_ENABLED)
        // Release BTC mode memory
        ESP_LOGI(TAG, "BT LE only enabled. Release BTC Memory");
        ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
        mode = ESP_BT_MODE_BLE;
    #endif

    esp_bt_controller_config_t def_config = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    memcpy(&bt_cfg, &def_config, sizeof(esp_bt_controller_config_t));

    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) 
    {
        ESP_LOGE(TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return -1;
    }

    if ((ret = esp_bt_controller_enable(mode)) != ESP_OK) 
    {
        ESP_LOGE(TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return -1;
    }

    //esp_bredr_tx_power_set(ESP_PWR_LVL_N0, ESP_PWR_LVL_P9);

    ESP_LOGI(TAG, "Bluedroid initializing...");
    if ((ret = esp_bluedroid_init()) != ESP_OK) 
    {
        ESP_LOGE(TAG, "Bluedroid failed to initialize: %s\n",  esp_err_to_name(ret));
        return -1;
    }

    ESP_LOGI(TAG, "Bluedroid enabling...");
    if ((ret = esp_bluedroid_enable()) != ESP_OK) 
    {
        ESP_LOGE(TAG, "Bluedroid failed to enable: %s\n",  esp_err_to_name(ret));
        return -1;
    }

    util_bt_hid_status = UTIL_BT_HID_STATUS_INITIALIZED;
    return 1;
}

/**
 * @brief Takes in an app params struct and starts the HID Gamepad
 * 
 * @param util_bt_app_params Pointer to type of util_bt_app_params_s
 * @param hidd_device_config Pointer to type of esp_hid_device_config_t
*/
int util_bluetooth_register_app(util_bt_app_params_s *util_bt_app_params, esp_hid_device_config_t *hidd_device_config)
{
    const char* TAG = "util_bluetooth_register_app";
    esp_err_t ret;
    int err = 1;

    if (util_bt_hid_status < UTIL_BT_HID_STATUS_INITIALIZED)
    {
        ESP_LOGE(TAG, "Register with util_bluetooth_init() first!");
        return -1;
    }

    if (util_bt_hid_status > UTIL_BT_HID_STATUS_INITIALIZED)
    {
        ESP_LOGE(TAG, "App is already registered and running.");
        return -1;
    }

    #if CONFIG_IDF_TARGET_ESP32
    bt_cfg.mode = util_bt_app_params->bt_mode;
    #endif

    switch(util_bt_app_params->bt_mode)
    {
        case ESP_BT_MODE_CLASSIC_BT:
            #if CONFIG_BT_HID_DEVICE_ENABLED

            _util_bt_store.product_id = hidd_device_config->product_id;
            _util_bt_store.vendor_id = hidd_device_config->vendor_id;
            _util_bt_store.vendor_id_source = 0x01;

            #if CONFIG_BT_SDP_COMMON_ENABLED
                ESP_ERROR_CHECK(esp_sdp_register_callback(util_bt_sdp_cb));
                ESP_ERROR_CHECK(esp_sdp_init());
            #endif /* CONFIG_BT_SDP_COMMON_ENABLED */

            bt_cfg.bt_max_acl_conn = 3;
            bt_cfg.bt_max_sync_conn = 3;
            err = bt_register_app(util_bt_app_params, hidd_device_config);
            #else
            ESP_LOGE(TAG, "BT Classic HID disabled. Enable in SDK settings. Also enable BT Dual mode.");
            return -1;
            #endif
        break;

        case ESP_BT_MODE_BLE:
            #if CONFIG_BT_BLE_ENABLED
            err = ble_register_app(util_bt_app_params, hidd_device_config);
            #else
            ESP_LOGE(TAG, "BLE is disabled. Enable in SDK settings. Also enable BT Dual mode.");
            return -1;
            #endif
        break;

        default:
            ESP_LOGE(TAG, "Invalid BT Mode.");
            return -1;
        break;
    }

    if (err == 1)
    {
        util_bt_hid_status = UTIL_BT_HID_STATUS_RUNNING;
    }
    return err;
}

/**
 * @brief Stops the Bluetooth app
*/
void util_bluetooth_deinit(void)
{
    const char* TAG = "util_bluetooth_stop";
    switch (util_bt_hid_mode)
    {   
        default:
        case UTIL_BT_MODE_CLASSIC:
            ESP_LOGI(TAG, "Stopping BT Classic mode...");
            esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            esp_bt_hid_device_disconnect();
            esp_bt_hid_device_unregister_app();
            esp_bt_hid_device_deinit();
            break;

        case UTIL_BT_MODE_BLE:
            ESP_LOGI(TAG, "Stopping BT LE mode...");
            break;
    }
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    
    esp_bt_controller_disable();
    esp_bt_controller_deinit();

    util_bt_hid_status = UTIL_BT_HID_STATUS_IDLE;
}

/**
 * @brief Starts a bluetooth connection attempt.
*/
void util_bluetooth_connect(uint8_t *host_mac)
{
    const char* TAG = "util_bluetooth_connect";

    if(util_bt_hid_status != UTIL_BT_HID_STATUS_RUNNING)
    {
        ESP_LOGE(TAG, "Bluetooth Util needs to be initialized, registered, and disconnected before attempting connection.");
        return;
    }

    switch(util_bt_hid_mode)
    {
        case UTIL_BT_MODE_CLASSIC:
        {
            uint8_t attempts_remaining = 5;
            bool connected = false;
            while (attempts_remaining > 0)
            {
                esp_err_t  err = esp_bt_hid_device_connect(host_mac);
                if (err != ESP_OK)
                {
                    attempts_remaining--;
                    ESP_LOGE(TAG, "Connection attempt failed. Attempts remaining: %d", attempts_remaining);
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
                else
                {
                    attempts_remaining = 0;
                    connected = true;
                }
            }

            //if (!connected)
            //{
            //    // If connection directly failed, set discoverable.
            //    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            //}
        }
        break;

        case UTIL_BT_MODE_BLE:
        {
            ESP_LOGI(TAG, "Bluetooth Connect function not needed for Gamepad as BLE Server.");
        }
        break;
    }
}
