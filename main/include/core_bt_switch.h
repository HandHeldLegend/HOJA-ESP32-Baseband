#ifndef CORE_BT_SWITCH_H
#define CORE_BT_SWITCH_H

#define SWITCH_BT_REPORT_SIZE 48

// Include any necessary includes from HOJA backend
#include "hoja_includes.h"
#include "hoja.h"

extern sw_input_s _switch_input_data;

void switch_bt_set_cmd_data(uint8_t *data, uint16_t len);

void ns_controller_setinputreportmode(uint8_t report_mode);

// Start the Nintendo Switch controller core
int core_bt_switch_start();

// Stop the Nintendo Switch controller core
void core_bt_switch_stop(void);

void switch_bt_sendinput(i2cinput_input_s *input);

#endif