#ifndef AH_PID_CONTROLLER_H
#define AH_PID_CONTROLLER_H

#include <stdint.h>


//positional_pid
struct pos_pid_controller{
  //private
  float kp,ki,kd;    //pid_gain
  int max_output_pwm; 
  int max_i_value;
  
  //public
  //保持する必要があるもののみ
  unsigned int pre_time;
  float pre_error;
  float pre_i_value;

//for debug
  int pre_pos_pid;
 
  };



void pos_pid_init(const float kp,const float ki,const float kd,
                  const int max_output_pwm,
                  const int max_i_value,
                  pos_pid_controller* p);


float calc_pos_p(float target,float current,
             const float kp
             );

float calc_pos_i(float target,float current,
             float dt,
             float pre_i_value,
             const float ki,
             const int max_i_value
             );

float calc_pos_d(float target,float current,
             float dt,
             float pre_error,
             const float kd
             );

int calc_pos_pid(float target,float current,
                 pos_pid_controller* p);



//velocity_pid
struct vel_pid_controller{
  //private
  float kp,ki,kd;
  int max_output_pwm;

  //public
  unsigned int pre_time;
  float pre_error;
  float pre_pre_error;
  int pre_vel_pid; 
  };

void vel_pid_init(const float kp,const float ki,const float kd,
                  const int max_output_pwm,
                  vel_pid_controller* p);

float calc_vel_p(float error,float pre_error,
                 const float kp);

float calc_vel_i(float error,
                 float dt,
                 const float ki);

float calc_vel_d(float error,float pre_error,float pre_pre_error,
                 const float kd);

int calc_vel_pid(float target,float current,
                 vel_pid_controller* p);




#endif
