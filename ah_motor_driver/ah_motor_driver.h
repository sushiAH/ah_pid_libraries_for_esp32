#ifndef AH_MOTOR_DRIVER_H
#define AH_MOTOR_DRIVER_H

#include <Arduino.h>

void init_motor(const int PINNUM_POWER, const int CHANNEL_NUM, const int PINNUM_DIR);
void write_to_motor(const int pwm, const int CHANNEL_NUM, const int PINNUM_DIR);

#endif
