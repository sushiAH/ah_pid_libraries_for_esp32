#ifndef OPERATING_MODE_H
#define OPERATING_MODE_H

enum operating_mode {
    stop_mode = 0,
    encoder_position_mode = 1,
    potentio_position_mode = 2,
    velocity_mode = 3,
    pwm_mode = 4,
    air_mode = 5,
};

#endif
