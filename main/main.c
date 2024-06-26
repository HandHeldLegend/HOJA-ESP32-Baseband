/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "hoja_includes.h"

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 64 /*!< Data buffer length of test buffer */
#define HOJA_I2C_MSG_SIZE_IN 32
#define HOJA_I2C_MSG_SIZE_OUT 11+1

#define I2C_SLAVE_SCL_IO 20                    /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO 19                    /*!< gpio number for i2c slave data */
#define I2C_SLAVE_NUM I2C_NUMBER(0)            /*!< I2C port number for slave dev */
#define I2C_SLAVE_TX_BUF_LEN (2 * DATA_LENGTH) /*!< I2C slave tx buffer size */
#define I2C_SLAVE_RX_BUF_LEN (2 * DATA_LENGTH) /*!< I2C slave rx buffer size */
#define ESP_SLAVE_ADDR 0x76                    /*!< ESP32 slave address, you can set any 7bit value */

typedef void (*bluetooth_input_cb_t)(i2cinput_input_s *);

i2cinput_status_s _bluetooth_status = {0};

bluetooth_input_cb_t _bluetooth_input_cb = NULL;

hoja_settings_s global_loaded_settings = {0};

bool     _msg_override = false;
uint8_t  _msg_override_data[HOJA_I2C_MSG_SIZE_OUT]  = {0};

bool _i2c_read_msg(uint8_t *buffer)
{
    uint8_t idx = 0;
    static bool d = false;
    static bool e = false;
    static bool f = false;

    uint8_t pattern = 0x00;
    bool got = false;

    while(idx<HOJA_I2C_MSG_SIZE_IN)
    {
        if(i2c_slave_read_buffer(I2C_SLAVE_NUM, &buffer[idx], 1, portMAX_DELAY)>0)
        {
            if(buffer[idx] == 0xDD)
            {
                pattern += 1;
            }
            else if( (buffer[idx] == 0xEE) && (pattern == 1) )
            {
                pattern += 1;
            }
            else if( (buffer[idx] == 0xAA) && (pattern == 2) ) // If we get here we reset our index to re-align
            {
                idx = 2;
                pattern = 0;
                got = true;
            }
        }
        idx++;
    }

    i2c_reset_rx_fifo(I2C_SLAVE_NUM);

    return got;
}

void _i2c_write_status_msg()
{
    if(_msg_override)
    {
        ESP_LOGI("_i2c_write_status_msg", "Sending override message.");
        //i2c_reset_tx_fifo(I2C_SLAVE_NUM);
        if(i2c_slave_write_buffer(I2C_SLAVE_NUM, _msg_override_data, HOJA_I2C_MSG_SIZE_OUT, portMAX_DELAY))
        _msg_override = false;
    }
    else 
    {
        uint8_t _msg_out[HOJA_I2C_MSG_SIZE_OUT] = {0};
        _msg_out[0] = I2CINPUT_ID_STATUS;

        memcpy(&(_msg_out[1]), &(_bluetooth_status), 11);

        //i2c_reset_tx_fifo(I2C_SLAVE_NUM);
        i2c_slave_write_buffer(I2C_SLAVE_NUM, _msg_out, HOJA_I2C_MSG_SIZE_OUT, portMAX_DELAY);
    }
}

static esp_err_t i2c_slave_init(void)
{
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
}

bool app_compare_mac(uint8_t *mac_1, uint8_t *mac_2)
{
    ESP_LOGI("app_compare_mac", "Mac 1:");
    esp_log_buffer_hex("Switch HOST: ", mac_1, 6);
    esp_log_buffer_hex("Saved HOST: ", mac_2, 6);
    
    for(uint8_t i = 0; i < 6; i++)
    {
        if (mac_1[i] != mac_2[i])
        {
            return false;
        }
    }
    return true;
}

void app_set_rumble(float frequency_hi, uint8_t amplitude_hi, float frequency_lo, uint8_t amplitude_lo)
{
    _bluetooth_status.rumble_frequency_hi = frequency_hi;
    _bluetooth_status.rumble_amplitude_hi = amplitude_hi;
    _bluetooth_status.rumble_frequency_lo = frequency_lo;
    _bluetooth_status.rumble_amplitude_lo = amplitude_lo;
}

void app_set_connected(uint8_t connected)
{
    _bluetooth_status.connected_status = connected;
}

void app_send_command(uint8_t cmd, uint8_t msg)
{
    _msg_override_data[0] = cmd;
    _msg_override_data[1] = msg;
    _msg_override = true;
}

void app_save_host_mac()
{
    ESP_LOGI("app_save_host_mac", "Saving new host MAC.");

    _msg_override_data[0] = I2CINPUT_ID_SAVEMAC;
    _msg_override_data[1] = global_loaded_settings.switch_host_mac[0];
    _msg_override_data[2] = global_loaded_settings.switch_host_mac[1];
    _msg_override_data[3] = global_loaded_settings.switch_host_mac[2];
    _msg_override_data[4] = global_loaded_settings.switch_host_mac[3];
    _msg_override_data[5] = global_loaded_settings.switch_host_mac[4];
    _msg_override_data[6] = global_loaded_settings.switch_host_mac[5];
    _msg_override = true;

    memcpy(global_loaded_settings.paired_host_mac, global_loaded_settings.switch_host_mac, 6);
}

void app_input(i2cinput_input_s *input)
{
    static imu_data_s _new_imu = {0};

    _new_imu.ax = input->ax;
    _new_imu.ay = input->ay;
    _new_imu.az = input->az;

    _new_imu.gx = input->gx;
    _new_imu.gy = input->gy;
    _new_imu.gz = input->gz;

    imu_fifo_push(&_new_imu);

    if (!_bluetooth_input_cb)
        return;

    _bluetooth_input_cb(input);
}

i2cinput_input_s input = {0};

void app_main(void)
{
    const char *TAG = "app_main";
    esp_err_t ret;

    ESP_LOGI(TAG, "Bluetooth FW starting...");

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(i2c_slave_init());

    static uint8_t data[HOJA_I2C_MSG_SIZE_IN]      = {0xFF, 0xFF, 0xFF};
    

    for (;;)
    {
        
        vTaskDelay(1 / portTICK_PERIOD_MS);
        // size_t d_size = i2c_slave_write_buffer(I2C_SLAVE_NUM, data, 3, 1000 / portTICK_PERIOD_MS);

        if (_i2c_read_msg(data))
        {
            switch (data[3])
            {
                default:
                    ESP_LOGI(TAG, "WRONG CODE");
                    memset(data, 0, HOJA_I2C_MSG_SIZE_IN);
                    break;

                // Initialize bluetooth
                case I2CINPUT_ID_INIT:
                {

                    input_mode_t mode = data[4];

                    global_loaded_settings.device_mac[0] = data[5];
                    global_loaded_settings.device_mac[1] = data[6];
                    global_loaded_settings.device_mac[2] = data[7];

                    global_loaded_settings.device_mac[3] = data[8];
                    global_loaded_settings.device_mac[4] = data[9];
                    global_loaded_settings.device_mac[5] = data[10];

                    // Load paired mac
                    global_loaded_settings.paired_host_mac[0] = data[11];
                    global_loaded_settings.paired_host_mac[1] = data[12];
                    global_loaded_settings.paired_host_mac[2] = data[13];

                    global_loaded_settings.paired_host_mac[3] = data[14];
                    global_loaded_settings.paired_host_mac[4] = data[15];
                    global_loaded_settings.paired_host_mac[5] = data[16];

                    esp_log_buffer_hex(TAG, global_loaded_settings.paired_host_mac, 6);

                    switch (mode)
                    {
                        default:
                            break;

                        case INPUT_MODE_DS4:
                            _bluetooth_input_cb = ds4_bt_sendinput;
                            ESP_LOGI(TAG, "DS4 BT Mode Init...");
                            memset(data, 0, HOJA_I2C_MSG_SIZE_IN);
                            core_bt_ds4_start();
                            break;

                        case INPUT_MODE_SWPRO:
                            _bluetooth_input_cb = switch_bt_sendinput;
                            ESP_LOGI(TAG, "Switch BT Mode Init...");
                            memset(data, 0, HOJA_I2C_MSG_SIZE_IN);
                            core_bt_switch_start();
                            break;

                        case INPUT_MODE_XINPUT:
                            _bluetooth_input_cb = xinput_bt_sendinput;
                            ESP_LOGI(TAG, "XInput BT Mode Init...");
                            memset(data, 0, HOJA_I2C_MSG_SIZE_IN);
                            core_bt_xinput_start();
                            break;
                    }
                }
                break;

                case I2CINPUT_ID_INPUT:
                {
                    memcpy(&input, &(data[4]), sizeof(i2cinput_input_s));
                    app_input(&input);
                    memset(data, 0, HOJA_I2C_MSG_SIZE_IN);
                    // Write our response data
                    _i2c_write_status_msg();
                }
                break;

                case I2CINPUT_ID_GETVERSION:
                {
                    _msg_override_data[0] = I2CINPUT_ID_GETVERSION;
                    _msg_override_data[1] = HOJA_BASEBAND_VERSION >> 8;
                    _msg_override_data[2] = HOJA_BASEBAND_VERSION & 0xFF;
                    _msg_override = true;

                    // Write our response data
                    _i2c_write_status_msg();
                }
                break;

                
            }

            
        }
    }
}
