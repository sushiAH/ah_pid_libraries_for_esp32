#ifndef AH_PID_ESP_H
#define AH_PID_ESP_H

#include <Arduino.h>
#include <ah_encoder_esp32.h>
#include <ah_motor_driver.h>
#include <ah_pid_controller.h>
#include <ah_potentio_esp32.h>
#include <operating_mode.h>
#include <stdint.h>

struct motor_controller {
    encoder ENC;
    pos_pid_controller POS_PID;
    vel_pid_controller VEL_PID;
    motor_driver DRIVER;

    SemaphoreHandle_t mutex;

    int pid_period;
    int motor_id;
    uint8_t init_flag;

    // 共有変数

    uint8_t operating_mode;
    float goal_pos;
    float goal_vel;
    float current_pos;
    float current_vel;
    int32_t goal_pwm;
    uint8_t air_val;
};

void init_motor_controller(const int max_output_pwm, const int max_i_value, const int enc_resolution,
                           const unsigned int pid_period, const int motor_id, struct motor_controller *ctrl);

void set_mode(uint8_t operating_mode, struct motor_controller *ctrl);
void set_pos(float goal_pos, struct motor_controller *ctrl);
void set_vel(float goal_vel, struct motor_controller *ctrl);
void set_pwm(int32_t goal_pwm, struct motor_controller *ctrl);
void set_air(uint8_t air_val, struct motor_controller *ctrl);

void set_pos_p_gain(float pos_p_gain, struct motor_controller *ctrl);
void set_pos_i_gain(float pos_i_gain, struct motor_controller *ctrl);
void set_pos_d_gain(float pos_d_gain, struct motor_controller *ctrl);

void set_vel_p_gain(float vel_p_gain, struct motor_controller *ctrl);
void set_vel_i_gain(float vel_i_gain, struct motor_controller *ctrl);
void set_vel_d_gain(float vel_d_gain, struct motor_controller *ctrl);

void set_motor_rot_dir(uint8_t motor_rot_dir, struct motor_controller *ctrl);

void set_profile_vel(float profile_vel, struct motor_controller *ctrl);
void set_profile_accel(float profile_accel, struct motor_controller *ctrl);

float get_current_pos(struct motor_controller *ctrl);
float get_current_vel(struct motor_controller *ctrl);

#endif
