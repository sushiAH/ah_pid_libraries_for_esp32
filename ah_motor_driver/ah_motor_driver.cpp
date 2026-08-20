#include <ah_motor_driver.h>

void init_motor(const int PINNUM_POWER, const int CHANNEL_NUM, const int PINNUM_DIR, struct motor_driver *driver)
{

    pinMode(PINNUM_POWER, OUTPUT);
    pinMode(PINNUM_DIR, OUTPUT);

    ledcSetup(CHANNEL_NUM, 20000, 10);
    ledcAttachPin(PINNUM_POWER, CHANNEL_NUM);

    driver->channel_num = CHANNEL_NUM;
    driver->pinnum_dir = PINNUM_DIR;
    driver->pinnum_power = PINNUM_POWER;
    driver->motor_rot_dir = 0;
}

void write_to_motor(int pwm, struct motor_driver *driver)
{
    int dir = 0;

    if (driver->motor_rot_dir == 0) {
        if (pwm > 0) {
            dir = 0;

        } else if (pwm < 0) {
            dir = 1;
            pwm = -pwm;
        }
    }

    else if (driver->motor_rot_dir == 1) {
        if (pwm > 0) {
            dir = 1;

        } else if (pwm < 0) {
            dir = 0;
            pwm = -pwm;
        }
    }

    ledcWrite(driver->channel_num, pwm);
    digitalWrite(driver->pinnum_dir, dir);
}
