#ifndef CORE_BT_DS4_H
#define CORE_BT_DS4_H

#include "hoja_includes.h"

void core_bt_ds4_stop();
void ds4_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
void ds4_bt_hidd_cb(void *handler_args, esp_event_base_t base, int32_t id, void *event_data);
int core_bt_ds4_start();
void ds4_bt_sendinput(i2cinput_input_s *input);

#endif // CORE_BT_DS4_H