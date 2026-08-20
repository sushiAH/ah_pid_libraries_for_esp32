#ifndef AH_MOTOR_DRIVER_H
#define AH_MOTOR_DRIVER_H

#include <Arduino.h>

struct motor_driver {
    int channel_num;
    int pinnum_dir;
    int pinnum_power;

    int motor_rot_dir;
};

void init_motor(const int PINNUM_POWER, const int CHANNEL_NUM, const int PINNUM_DIR, struct motor_driver *driver);
void write_to_motor(const int pwm, struct motor_driver *driver);

#endif
