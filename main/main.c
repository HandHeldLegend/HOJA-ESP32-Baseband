/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "hoja_includes.h"

#define I2C_RX_BUFFER_SIZE 32 // Size of the input, plus a byte for ID
#define I2C_TX_BUFFER_SIZE 24

#define I2C_MSG_STATUS_IDX 4
#define I2C_MSG_DATA_START 5
#define I2C_MSG_CMD_IDX 3

#define I2C_SLAVE_SCL_IO 20                    /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO 19                    /*!< gpio number for i2c slave data */
#define I2C_SLAVE_NUM I2C_NUM_0                /*!< I2C port number for slave dev */
#define ESP_SLAVE_ADDR 0x76

#define I2C_TX_CRC_IDX 0
#define I2C_TX_COUNTER_IDX 1
#define I2C_TX_STATUS_IDX 2

// Last processed packet number from Host
uint8_t _current_rx_packet_num = 0;
uint8_t _current_tx_packet_num = 0;

typedef struct
{
    uint8_t data[I2C_RX_BUFFER_SIZE];
} packed_i2c_msg;

// Utilities
uint32_t get_timestamp_us()
{
    int64_t t = esp_timer_get_time();

    if (t > 0xFFFFFFFF)
        t -= 0xFFFFFFFF;
    return (uint32_t)t;
}

uint8_t _i2c_buffer_in[I2C_RX_BUFFER_SIZE];
uint8_t _i2c_buffer_out[I2C_TX_BUFFER_SIZE];

QueueHandle_t main_receive_queue;

// Polynomial for CRC-8 (x^8 + x^2 + x + 1)
#define CRC8_POLYNOMIAL 0x07

uint8_t crc8_compute(uint8_t *data, size_t length)
{
    uint8_t crc = 0x00; // Initial value of CRC
    for (size_t i = 0; i < length; i++)
    {
        crc ^= data[i]; // XOR the next byte into the CRC

        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x80)
            { // If the MSB is set
                crc = (crc << 1) ^ CRC8_POLYNOMIAL;
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    if (!crc)
        crc++; // Must be non-zero

    return crc;
}

bool crc8_verify(uint8_t *data, size_t length, uint8_t received_crc)
{
    uint8_t calculated_crc = crc8_compute(data, length);
    return calculated_crc == received_crc;
}

typedef void (*bluetooth_input_cb_t)(i2cinput_input_s *);

// Input callback pointer
bluetooth_input_cb_t _bluetooth_input_cb = NULL;

// Our loaded configuration data
hoja_settings_s global_loaded_settings = {0};

// Our buffer for outgoing i2cinput messages
#define I2C_STATUS_BUFFER_SIZE 32
uint8_t _i2c_status_buffer[I2C_STATUS_BUFFER_SIZE][I2C_TX_BUFFER_SIZE];

typedef struct
{
    size_t size;
    size_t head;
    size_t tail;
    size_t count;
} RingBuffer;

RingBuffer _status_ringbuffer = {.size = I2C_STATUS_BUFFER_SIZE};

// Add data to the ring buffer
bool ringbuffer_set(RingBuffer *rb, uint8_t *data)
{
    //xSemaphoreTake(mutex_handle, portMAX_DELAY);

    if (rb->count == rb->size)
    {
        // Buffer is full
        // Buffer is full, reset the buffer
        printf("Buffer full - RX Packet num: 0x%x\n", _current_rx_packet_num);
        printf("TX Packet num: 0x%x\n", _current_tx_packet_num);

        rb->head = 0;
        rb->tail = 0;
        rb->count = 0;
    }

    memcpy(&(_i2c_status_buffer[rb->head]), data, I2C_TX_BUFFER_SIZE);
    rb->head = (rb->head + 1) % rb->size;
    rb->count++;
    //xSemaphoreGive(mutex_handle);
    return true;
}

// Get data from the ring buffer
uint8_t *ringbuffer_get(RingBuffer *rb)
{
    //xSemaphoreTake(mutex_handle, portMAX_DELAY);
    if (rb->count == 0)
    {
        // Buffer is empty
        //xSemaphoreGive(mutex_handle);
        //printf("rx empty...\n");
        return NULL;
    }
    //printf("rx OK...\n");
    uint8_t *data = _i2c_status_buffer[rb->tail];
    rb->tail = (rb->tail + 1) % rb->size;
    rb->count--;
    //xSemaphoreGive(mutex_handle);
    return data;
}

// Loads messages into our cross-core buffer
void ringbuffer_load_threadsafe(uint8_t *data)
{

    if(uxQueueSpacesAvailable(main_receive_queue) <= 1)
    {
        printf("Queue full\n");
        xQueueReset(main_receive_queue);
    }
        

    static packed_i2c_msg msg = {0};
    memcpy(&(msg.data[0]), data, I2C_TX_BUFFER_SIZE);
    xQueueSend(main_receive_queue, &msg, 0);
}

volatile bool haptic_buffer_connected = false;
// Obtain messages from our cross-core buffer
void ringbuffer_unload_threadsafe()
{
    static packed_i2c_msg msg = {0};
    while(xQueueReceive(main_receive_queue, &msg, 0) == pdTRUE)
    {
        if(!haptic_buffer_connected)
        {
            // Check if it's haptic data. Do not send if so
            i2cinput_status_s tmp = {0};
            memcpy(&tmp, &msg.data[I2C_TX_STATUS_IDX], sizeof(i2cinput_status_s));

            if(tmp.cmd != I2C_STATUS_HAPTIC_STANDARD)
                ringbuffer_set(&_status_ringbuffer, &(msg.data[0]));
        }
        else ringbuffer_set(&_status_ringbuffer, &(msg.data[0]));
            
    }
}

// Only call from core 1
uint8_t app_core1_get_packet_counter()
{
    static uint16_t counter = 0;
    counter = (counter+1) % 0xFF;
    return (uint8_t) counter;
}

// Only call from core 1
void app_set_connected_status(uint8_t status)
{
    uint8_t conn_buffer[I2C_TX_BUFFER_SIZE] = {0};
    i2cinput_status_s conn_status = {
        .cmd = I2C_STATUS_CONNECTED_STATUS,
    };

    conn_status.data[0] = status;
    conn_status.rand_seed = esp_random();

    memcpy(&(conn_buffer[I2C_TX_STATUS_IDX]), &conn_status, sizeof(i2cinput_status_s));

    uint8_t crc = crc8_compute((uint8_t *) &conn_status, sizeof(i2cinput_status_s));
    conn_buffer[I2C_TX_CRC_IDX] = crc;
    conn_buffer[I2C_TX_COUNTER_IDX] = app_core1_get_packet_counter();

    ringbuffer_load_threadsafe(conn_buffer);
}

bool app_compare_mac(uint8_t *mac_1, uint8_t *mac_2)
{
    ESP_LOGI("app_compare_mac", "Mac 1:");
    esp_log_buffer_hex("Switch HOST: ", mac_1, 6);
    esp_log_buffer_hex("Saved HOST: ", mac_2, 6);

    for (uint8_t i = 0; i < 6; i++)
    {
        if (mac_1[i] != mac_2[i])
        {
            return false;
        }
    }
    return true;
}

// Only call from core 1
void app_set_power_setting(uint8_t power)
{
    uint8_t power_buffer[I2C_TX_BUFFER_SIZE] = {0};
    i2cinput_status_s power_status = {
        .cmd = I2C_STATUS_POWER_CODE,
    };

    power_status.data[0] = power;
    power_status.rand_seed = esp_random();

    memcpy(&(power_buffer[I2C_TX_STATUS_IDX]), &power_status, sizeof(i2cinput_status_s));

    uint8_t crc = crc8_compute((uint8_t *) &power_status, sizeof(i2cinput_status_s));
    power_buffer[I2C_TX_CRC_IDX] = crc;
    power_buffer[I2C_TX_COUNTER_IDX] = app_core1_get_packet_counter();

    ringbuffer_load_threadsafe(power_buffer);
    //ringbuffer_set(&_status_ringbuffer, power_buffer);
}

// Only call from core 1
void app_set_switch_haptic(uint8_t *data)
{
    uint8_t haptic_buffer[I2C_TX_BUFFER_SIZE] = {0};
    i2cinput_status_s haptic_status = {
        .cmd = I2C_STATUS_HAPTIC_SWITCH,
    };

    // Copy haptic data into status
    memcpy(&(haptic_status.data), data, 8);
    // Generate random seed
    haptic_status.rand_seed = esp_random();

    // Copy haptic_status into haptic_buffer
    memcpy(&(haptic_buffer[I2C_TX_STATUS_IDX]), &haptic_status, sizeof(i2cinput_status_s));

    // Set the CRC at byte 0 of our outgoing data
    uint8_t crc = crc8_compute((uint8_t *)&haptic_status, sizeof(i2cinput_status_s));
    haptic_buffer[I2C_TX_CRC_IDX] = crc;
    haptic_buffer[I2C_TX_COUNTER_IDX] = app_core1_get_packet_counter();

    ringbuffer_load_threadsafe(haptic_buffer);
    //ringbuffer_set(&_status_ringbuffer, haptic_buffer);
}

// Only call from core 1
void app_set_standard_haptic(uint8_t left, uint8_t right)
{
    uint8_t haptic_buffer[I2C_TX_BUFFER_SIZE] = {0};
    i2cinput_status_s haptic_status = {
        .cmd = I2C_STATUS_HAPTIC_STANDARD,
    };

    // Copy haptic data into status
    haptic_status.data[0] = left;
    haptic_status.data[1] = right;
    // Generate random seed
    haptic_status.rand_seed = esp_random();

    // Copy haptic_status into haptic_buffer
    memcpy(&(haptic_buffer[I2C_TX_STATUS_IDX]), &haptic_status, sizeof(i2cinput_status_s));

    // Set the CRC at byte 0 of our outgoing data
    uint8_t crc = crc8_compute((uint8_t *)&haptic_status, sizeof(i2cinput_status_s));
    haptic_buffer[I2C_TX_CRC_IDX] = crc;
    haptic_buffer[I2C_TX_COUNTER_IDX] = app_core1_get_packet_counter();

    ringbuffer_load_threadsafe(haptic_buffer);
}

// Function to generate a random MAC address and write it to the buffer
void generate_random_mac(uint8_t *mac) {

    // Generate random values for each byte of the MAC address
    for (int i = 0; i < 6; i++) {
        mac[i] = (uint8_t)(esp_random() % 256);
    }

    // Set the locally administered bit (bit 1 of the first byte) 
    // and clear the multicast bit (bit 0)
    mac[0] = (mac[0] & 0xFE) | 0x02;
}

void settings_default()
{
    memset(&global_loaded_settings, 0, sizeof(hoja_settings_s));
    global_loaded_settings.magic = HOJA_MAGIC_NUM;

    memset(&global_loaded_settings.paired_host_switch_mac, 0, 6);
    generate_random_mac(&global_loaded_settings.device_mac_switch);

    memset(&global_loaded_settings.paired_host_gamecube_mac, 0, 6);
    generate_random_mac(&global_loaded_settings.device_mac_gamecube);

    memset(&global_loaded_settings.paired_host_xinput_mac, 0, 6);
    generate_random_mac(&global_loaded_settings.device_mac_xinput);
}

void app_settings_save()
{
    const char* TAG = "hoja_settings_saveall";
    nvs_handle_t my_handle;
    esp_err_t err;
    // Open
    err = nvs_open(HOJA_SETTINGS_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) 
    {
        ESP_LOGE(TAG, "During HOJA settings save, NVS storage failed to open.");
        return;
    }

    nvs_set_blob(my_handle, "hoja_settings", &global_loaded_settings, sizeof(hoja_settings_s));

    nvs_commit(my_handle);
    nvs_close(my_handle);

    return;
}

// Send updated host MAC address so the host device can save it for later
void app_save_host_mac(input_mode_t mode, uint8_t *address)
{
    ESP_LOGI("app_save_host_mac", "Saving new host mac...");

    uint8_t *write_address = NULL;
    
    switch(mode)
    {
        default:
        case INPUT_MODE_SWPRO:
            write_address = &global_loaded_settings.paired_host_switch_mac;
        break;

        case INPUT_MODE_GAMECUBE:   
            write_address = &global_loaded_settings.paired_host_gamecube_mac;
        break;

        case INPUT_MODE_XINPUT:
            write_address = &global_loaded_settings.paired_host_xinput_mac;
        break;
    }

    if(write_address!=NULL)
    {
        write_address[0] = address[0];
        write_address[1] = address[1];
        write_address[2] = address[2];
        write_address[3] = address[3];
        write_address[4] = address[4];
        write_address[5] = address[5];

        app_settings_save();
    }
}

// Handle incoming input data
// and call our input callback
void bt_device_input(uint8_t* data, bool motion)
{
    static i2cinput_input_s input_data = {0};
    static imu_data_s _new_imu = {0};

    uint8_t crc = data[1];
    bool crc_valid = false;

    crc_valid = crc8_verify(&(data[3]), sizeof(i2cinput_input_s), crc);
    if(!crc_valid) return;

    _current_rx_packet_num = data[2];

    memcpy(&input_data, &(data[3]), sizeof(i2cinput_input_s));

    _new_imu.ax = input_data.ax;
    _new_imu.ay = input_data.ay;
    _new_imu.az = input_data.az;
    _new_imu.gx = input_data.gx;
    _new_imu.gy = input_data.gy;
    _new_imu.gz = input_data.gz;
    imu_fifo_push(&_new_imu);

    if (!_bluetooth_input_cb)
    return;

    _bluetooth_input_cb(&input_data);
}

void i2c_handle_new_tx()
{
    static uint8_t *rb_buffer = NULL;
    static uint8_t rb_buffer_tmp[I2C_TX_BUFFER_SIZE] = {0};
    static bool get_next_packet = true;
    static bool confirmed_packet_sent = false;

    if(get_next_packet)
    {
        // Try to get the next buffer in line
        //printf("get next tx buffer\n");
        rb_buffer = ringbuffer_get(&_status_ringbuffer);

        if(rb_buffer!=NULL)
        {  
            memcpy(rb_buffer_tmp, rb_buffer, I2C_TX_BUFFER_SIZE);
            _current_tx_packet_num = rb_buffer_tmp[I2C_TX_COUNTER_IDX];
            get_next_packet = false;
            confirmed_packet_sent = false;
        }
    }
    else if(!confirmed_packet_sent && !get_next_packet)
    {
        if ( _current_rx_packet_num == rb_buffer_tmp[I2C_TX_COUNTER_IDX] ) // This confirms it's okay to load the next ringbuffer status
        {
            //printf("TX Confirmed received\n");
            confirmed_packet_sent = true;
            get_next_packet = true;
        }
    }

    // Send our curent valid packet as many times as we need until we have a valid response
    if (!get_next_packet && !confirmed_packet_sent)
    {
        memcpy(_i2c_buffer_out, rb_buffer_tmp, I2C_TX_BUFFER_SIZE);
        mi2c_slave_polling_write(_i2c_buffer_out, I2C_TX_BUFFER_SIZE, pdMS_TO_TICKS(32));
    }
}

// Handle startup of bluetooth device
void bt_device_start(uint8_t *data)
{
    esp_log_buffer_hex("DUMP", data, 8);
    const char *TAG = "BT DEVICE START";
    
    uint8_t crc = data[1];

    bool crc_pass = crc8_verify(&(data[2]), 13, crc);
    if(!crc_pass)
    {
        printf("CRC Startup FAIL\n");
        return;
    }

    input_mode_t mode = data[2] & 0x7F;

    // Check if we should clear our addresses
    // to initiate a new pairing sequence
    if(data[2] & 0x80)
    {
        switch(mode)
        {
            default:
            case INPUT_MODE_SWPRO:
                memset(&global_loaded_settings.paired_host_switch_mac, 0, 6);
                generate_random_mac(&global_loaded_settings.device_mac_switch);
            break;

            case INPUT_MODE_GAMECUBE:
                memset(&global_loaded_settings.paired_host_gamecube_mac, 0, 6);
                generate_random_mac(&global_loaded_settings.device_mac_gamecube);
            break;

            case INPUT_MODE_XINPUT:
                memset(&global_loaded_settings.paired_host_xinput_mac, 0, 6);
                generate_random_mac(&global_loaded_settings.device_mac_xinput);
            break;
        }

        app_settings_save();
    }

    switch (mode)
    {
    default:
        break;

    //case INPUT_MODE_DS4:
    //    break; // Disable for now
    //    _bluetooth_input_cb = ds4_bt_sendinput;
    //    ESP_LOGI(TAG, "DS4 BT Mode Init...");
//
    //    core_bt_ds4_start();
    //    break;

    case INPUT_MODE_SWPRO:
        _bluetooth_input_cb = switch_bt_sendinput;
        ESP_LOGI(TAG, "Switch BT Mode Init...");

        core_bt_switch_start();
        break;

    //case INPUT_MODE_XINPUT:
    //    _bluetooth_input_cb = xinput_bt_sendinput;
    //    ESP_LOGI(TAG, "XInput BT Mode Init...");
    //    core_bt_xinput_start();
    //    break;
    }
}

// Return current FW version
void bt_device_return_fw_version()
{
    uint8_t tmp_out[I2C_TX_BUFFER_SIZE] = {0};
    tmp_out[0] = I2C_STATUS_FIRMWARE_VERSION;
    tmp_out[1] = HOJA_BASEBAND_VERSION >> 8;
    tmp_out[2] = HOJA_BASEBAND_VERSION & 0xFF;

    // Generate random seed
    //firmware_status.rand_seed = 0;
    //firmware_status.crc = 0;

    // Instant transmit
    mi2c_slave_polling_write(tmp_out, I2C_TX_BUFFER_SIZE, pdMS_TO_TICKS(1000));
}

volatile uint32_t _timer = 0;
volatile bool _timer_owned = false;
uint32_t get_timer_value()
{
    static uint32_t safe_value = 0;
    if (_timer_owned)
        return safe_value;
    else
    {
        _timer_owned = true;
        safe_value = _timer;
        _timer_owned = false;
    }
    return safe_value;
}

void update_timer_value()
{
    if (_timer_owned)
        return;
    else
    {
        _timer_owned = true;
        _timer = get_timestamp_us();
        _timer_owned = false;
    }
}

/**
int legacy_i2c_setup()
{
    //!< ESP32 slave address, you can set any 7bit value

    int i2c_slave_port = I2C_SLAVE_NUM;
    i2c_config_t conf_slave = {
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .mode = I2C_MODE_SLAVE,
        .slave.maximum_speed = 400 * 1000,
        .slave.addr_10bit_en = 0,
        .slave.slave_addr = ESP_SLAVE_ADDR,
    };
    esp_err_t err = i2c_param_config(i2c_slave_port, &conf_slave);
    if (err != ESP_OK)
    {
        return err;
    }
    return i2c_driver_install(i2c_slave_port, conf_slave.mode, I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0);
}**/

void app_main(void)
{
    esp_task_wdt_deinit();

    const char *TAG = "app_main";
    esp_err_t ret;

    ESP_LOGI(TAG, "Bluetooth FW starting...");

    // Create mutex
    main_receive_queue = xQueueCreate(32, sizeof(packed_i2c_msg));

    // NVS Storage Load
    {
        // Load settings or create
        nvs_handle_t my_handle;

        // Initialize flash storage
        ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);

        ret = nvs_open(HOJA_SETTINGS_NAMESPACE, NVS_READWRITE, &my_handle);
        if(ret != ESP_OK)
        {
            ESP_LOGE(TAG, "During HOJA load settings, NVS Open failed.");
            return;
        }

        size_t required_size = 0;
        ret = nvs_get_blob(my_handle, "hoja_settings", NULL, &required_size);
        if(required_size>0)
        {
            ret = nvs_get_blob(my_handle, "hoja_settings", &global_loaded_settings, &required_size);
            if(global_loaded_settings.magic != HOJA_MAGIC_NUM)
            {
                settings_default();
                nvs_set_blob(my_handle, "hoja_settings", &global_loaded_settings, sizeof(hoja_settings_s));
                nvs_commit(my_handle);
                nvs_close(my_handle);
                ESP_LOGI(TAG, "HOJA config set to default OK.");
            }
            else
            {
                ESP_LOGI(TAG, "HOJA config loaded OK.");
                nvs_close(my_handle);
            }
        }
        else
        {
            settings_default();
            nvs_set_blob(my_handle, "hoja_settings", &global_loaded_settings, sizeof(hoja_settings_s));
            nvs_commit(my_handle);
            nvs_close(my_handle);
            ESP_LOGI(TAG, "HOJA config set to default OK.");
        }
    }

    // Set up I2C slave device with custom driver
    mi2c_slave_setup();

    // Main I2C loop
    for (;;)
    {   
        // Unload any and all pending messages
        ringbuffer_unload_threadsafe();

        //printf("i2c RX read attempt\n");
        mi2c_status_t read_status = mi2c_slave_polling_read(_i2c_buffer_in, I2C_RX_BUFFER_SIZE, 8);

        if (read_status == MI2C_OK)
        {
            //printf("I2C RX OK\n");
            // Check command type
            switch (_i2c_buffer_in[0])
            {
            default:
                ESP_LOGI(TAG, "Unknown RX");
                break;

            case I2C_CMD_STANDARD:
                //printf("I2C standard OK\n");
                // Say we're OK to send haptic data
                haptic_buffer_connected = true;
                // ESP_LOGI(TAG, "Input RX");
                bt_device_input(_i2c_buffer_in, false);
                // Transmit
                i2c_handle_new_tx();
                break;

            case I2C_CMD_START:
                ESP_LOGI(TAG, "Start System RX");
                bt_device_start(_i2c_buffer_in);
                // We do not transmit anything with this command
                break;

            case I2C_CMD_FIRMWARE_VERSION:
                ESP_LOGI(TAG, "Firmware Version Request RX");
                bt_device_return_fw_version();
                break;
            }   
        }
        else
        {

        }
        vTaskDelay(1/portTICK_PERIOD_MS);
    }
}
