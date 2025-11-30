#ifndef ESP32_PCNT_H
#define ESP32_PCNT_H

#include "driver/pcnt.h"


void qei_setup_x4(pcnt_unit_t pcnt_unit, int gpioA, int gpioB);
void qei_setup_x1(pcnt_unit_t pcnt_unit, int gpioA, int gpioB);


#endif
