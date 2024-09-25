#ifndef CORE_BT_GAMECUBE_H
#define CORE_BT_GAMECUBE_H

#define GAMECUBE_BT_REPORT_SIZE 9

// Include any necessary includes from HOJA backend
#include "hoja_includes.h"
#include "hoja.h"

extern gc_input_s _gamecube_input_data;

void gc_bt_end_task();
void gc_reset_report_spacer();

// Start the controller core
int core_bt_gamecube_start();

// Stop the controller core
void core_bt_gamecube_stop(void);

void gamecube_bt_sendinput(i2cinput_input_s *input);

#endif