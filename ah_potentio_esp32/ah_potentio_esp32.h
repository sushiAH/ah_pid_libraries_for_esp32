#ifndef AH_POTENTIO_ESP32_H
#define AH_POTENTIO_ESP32_H

#include <driver/adc.h>

void potentio_init(int motor_id);
float read_potentio(int motor_id);

#endif
