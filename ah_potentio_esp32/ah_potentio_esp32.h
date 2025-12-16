#ifndef AH_POTENTIO_ESP32_H
#define AH_POTENTIO_ESP32_H

#include <driver/adc.h>

void init_potentio(int motor_id);
int read_potentio(int motor_id);

#endif
