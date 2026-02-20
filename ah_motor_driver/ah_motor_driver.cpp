#include <ah_motor_driver.h>

void init_motor(const int PINNUM_POWER, const int CHANNEL_NUM, const int PINNUM_DIR)
{
    pinMode(PINNUM_POWER, OUTPUT);
    pinMode(PINNUM_DIR, OUTPUT);

    ledcSetup(CHANNEL_NUM, 20000, 10);
    ledcAttachPin(PINNUM_POWER, CHANNEL_NUM);
}

void write_to_motor(int pwm, const int CHANNEL_NUM, const int PINNUM_DIR)
{
    int dir = 0;

    if (pwm > 0) {
        dir = 0;
    }
    if (pwm < 0) {
        dir = 1;
        pwm = -pwm;
    }

    ledcWrite(CHANNEL_NUM, pwm);
    digitalWrite(PINNUM_DIR, dir);
}
