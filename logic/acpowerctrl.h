#ifndef ACPOWERCTRL_H
#define ACPOWERCTRL_H

#include "inttypes.h"
#include "stdbool.h"

void setRelay(uint8_t num, bool state);

void acInit();
void setTriac(uint16_t val);







#endif
