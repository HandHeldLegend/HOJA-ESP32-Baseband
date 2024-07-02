#ifndef HOJA_H
#define HOJA_H

#include "hoja_includes.h"

// reboot the gamepad Bluetooth with the same mode
void app_running_reboot();

imu_data_s* imu_fifo_last();
void imu_fifo_push(imu_data_s *imu_data);
void app_set_connected(uint8_t connected);
void app_send_command(uint8_t cmd, uint8_t msg);
void app_set_rumble(float frequency_hi, uint8_t amplitude_hi, float frequency_lo, uint8_t amplitude_lo);
bool app_compare_mac(uint8_t *mac_1, uint8_t *mac_2);

void app_save_host_mac();

#endif