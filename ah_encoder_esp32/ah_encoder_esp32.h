#ifndef AH_ENCODER_ESP32_H
#define AH_ENCODER_ESP32_H

#include <Arduino.h>
#include <esp32_pcnt.h>
#include <stdint.h>

struct encoder {
    // private
    int enc_resolution; // 分解能
    int enc_pinnum_a;
    int enc_pinnum_b;
    pcnt_unit_t PCNT_UNIT;

    // public
    int pre_count;
    int16_t now_count;
    unsigned int pre_time;
    float vel;
    float pos;
};

void init_enc(const int pcnt_unit_num, const int enc_pinnum_a, const int enc_pinnnum_b, const int enc_resolution,
              encoder *p);

float update_vel(encoder *p);
float update_pos(encoder *p);

#endif
