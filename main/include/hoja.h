#ifndef HOJA_H
#define HOJA_H

#define SDP_MAX_PAD_LEN 500

#include "hoja_includes.h"

uint64_t get_timestamp_ms();
uint64_t get_timestamp_us();

imu_data_s* imu_fifo_last();
void imu_fifo_push(imu_data_s *imu_data);

void app_set_power_setting(i2c_power_code_t power);
void app_set_connected_status(uint8_t status);

void app_set_sinput_haptic(uint8_t *data, uint8_t len);
void app_set_standard_haptic(uint8_t left, uint8_t right);
void app_set_switch_haptic(uint8_t *data);

void app_save_host_mac(input_mode_t mode, uint8_t *address);

bool app_compare_mac(uint8_t *mac_1, uint8_t *mac_2);
uint32_t get_timer_value();


#endif