#include "ah_encoder_esp32.h"


// -- Config --
const int ENC_PINNUM_A[4] = {22,21,18,17};   //motor_id 0,1,2,3
const int ENC_PINNUM_B[4] = {23,19,5,16};

void enc_init(const int enc_resolution,const int motor_id,encoder* p){

  p->enc_resolution = enc_resolution;
  p->enc_pinnum_a = ENC_PINNUM_A[motor_id];
  p->enc_pinnum_b = ENC_PINNUM_B[motor_id];


  if(motor_id == 0){
    p->PCNT_UNIT = PCNT_UNIT_0;
  }else if(motor_id == 1){
    p->PCNT_UNIT = PCNT_UNIT_1;
  }else if(motor_id == 2){
    p->PCNT_UNIT = PCNT_UNIT_2;
  }else if(motor_id == 3){
    p->PCNT_UNIT = PCNT_UNIT_3;
  }
  qei_setup_x1(p->PCNT_UNIT,ENC_PINNUM_A[motor_id],ENC_PINNUM_B[motor_id]);

  pinMode(ENC_PINNUM_A[motor_id],INPUT_PULLDOWN);
  pinMode(ENC_PINNUM_B[motor_id],INPUT_PULLDOWN);
}

float update_vel(encoder* p){ 
  unsigned int now_time = millis();
  float dt = (now_time - p->pre_time)/1000.000f;
  if(dt == 0.0f || p->pre_time == 0){
    dt = 0.010f;   //10ms
  }

  pcnt_get_counter_value(p->PCNT_UNIT,&p->now_count);

  
  int delta_count = p->now_count - p->pre_count;


  float delta_count_f = (float)delta_count;

  float diff_count_sec = (delta_count_f/dt);
  float vel = (diff_count_sec/(float)p->enc_resolution);

  p->pre_time = now_time;
  p->pre_count = 0;
  pcnt_counter_clear(p->PCNT_UNIT);
  p->vel = vel;



  return vel;
}

float update_pos(encoder* p){

  pcnt_get_counter_value(p->PCNT_UNIT,&p->now_count);
  float pos = ((double)p->now_count/p->enc_resolution)*360;  //degree
  p->pos = pos;

  return pos;
}
