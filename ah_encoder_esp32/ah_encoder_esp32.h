#ifndef AH_ENCODER_ESP32_H
#define AH_ENCODER_ESP32_H

#include <Arduino.h>
#include <stdint.h>
#include <esp32_pcnt.h>


struct encoder{
  //private
  int enc_resolution; //分解能
  int enc_pinnum_a;
  int enc_pinnum_b;
  pcnt_unit_t PCNT_UNIT;

  //public
  int pre_count;
  int16_t now_count;
  unsigned int pre_time;
  float vel;
  float pos;
};


void enc_init(const int enc_resolution,const int motor_id,encoder* p);

float update_vel(encoder* p);
float update_pos(encoder* p);



#endif
