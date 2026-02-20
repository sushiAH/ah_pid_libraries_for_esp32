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

    SemaphoreHandle_t mutex;

    int pid_period;
    int motor_id;

    // 共有変数
    int32_t operating_mode;  // 0
    int32_t goal_pos_int;    // 1
    int32_t goal_vel_int;    // 2
    int32_t current_pos_int; // 3
    int32_t current_vel_int; // 4
    int32_t goal_pwm;        // 5
    int32_t air_val;         // 6
};

void init_motor_controller(const int max_output_pwm, const int max_i_value, const int enc_resolution,
                           const unsigned int pid_period, const int motor_id, struct motor_controller *ctrl);

void write_to_motor(int pwm, const int CHANNEL_NUM, const int PINNUM_DIR);

float run_pid_pos(float target, int motor_id, pos_pid_controller *pid_pos, ENC *enc);
float run_pid_pos_with_potentio(float target, int motor_id, pos_pid_controller *pid_pos, ENC *enc);
float run_pid_vel(float target, int motor_id, vel_pid_controller *pid_vel, ENC *enc);

void run_pid(void *pvParameters);

int read_target_by_operating(motor_controller *ctrl, int operating_mode, float *target);

int run_pid_by_operating(motor_controller *ctrl, int operating_mode, float target);

void set_mode(int operating_mode, motor_controller *ctrl);
void set_pos(float goal_pos, motor_controller *ctrl);
void set_vel(float goal_vel, motor_controller *ctrl);
void set_pwm(int goal_pwm, motor_controller *ctrl);

float read_current_pos(motor_controller *ctrl);
float read_current_vel(motor_controller *ctrl);

#endif
