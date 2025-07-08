#ifndef INTERVAL_H
#define INTERVAL_H

#include <stdint.h>
#include "hoja_types.h"

bool interval_run(uint64_t timestamp, uint32_t interval, interval_s *state);
bool interval_resettable_run(uint64_t timestamp, uint32_t interval, bool reset, interval_s *state);

#endif