#ifndef CORE_BT_XINPUT_H
#define CORE_BT_XINPUT_H

// Include any necessary includes from HOJA backend
#include "hoja_includes.h"
#include "hoja.h"

void xinput_bt_sendinput(i2cinput_input_s *input);
int core_bt_xinput_start(void);

#endif