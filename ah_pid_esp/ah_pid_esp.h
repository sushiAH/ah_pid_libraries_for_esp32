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

#endif
