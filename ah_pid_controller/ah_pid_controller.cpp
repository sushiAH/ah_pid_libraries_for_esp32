/**
 * @file ah_pid_controller.cpp
 * @brief pid計算ライブラリ,位置型、速度型
 */

#include "ah_pid_controller.h"

#include <Arduino.h>

/**
 * @brief positional_pidの初期化
 *
 * @param kp 比例ゲイン
 * @param ki 積分ゲイン
 * @param kd 微分ゲイン
 * @param max_output_pwm 出力pid限界
 * @param max_i_value i値限界
 * @param p pos_pid_controller pointer
 */
void init_pos_pid(const float kp, const float ki, const float kd, const int max_output_pwm, const int max_i_value,
                  pos_pid_controller *p)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->max_output_pwm = max_output_pwm;
    p->max_i_value = max_i_value;

    p->pre_time = 0;
    p->pre_error = 0;
    p->pre_i_value = 0;
}

/**
 * @brief positional_pid_p計算
 *
 * @param target 目標値
 * @param current 現在値
 * @param kp 比例ゲイン
 * @return positional_pid_p
 */
float calc_pos_p(float target, float current, const float kp)
{
    float error = target - current;
    float pos_p = kp * error;

    return pos_p;
}

/**
 * @brief positional_pid_i計算
 *
 * @param target　目標値
 * @param current 現在値
 * @param dt 微小時間
 * @param pre_i_value 前回I値
 * @param ki Iゲイン
 * @param max_i_value I値限界
 * @return I値
 */
float calc_pos_i(float target, float current, float dt, float pre_i_value, const float ki, const int max_i_value)
{
    float error = target - current;

    float pos_i = pre_i_value + ki * error * dt;

    // anti wind up
    if (pos_i > max_i_value) { // if i_value > 0
        pos_i = max_i_value;
    }

    if (pos_i < -max_i_value) { // if i_value < 0
        pos_i = -max_i_value;
    }

    return pos_i;
}

/**
 * @brief positional_pid_d値計算
 *
 * @param target 目標値
 * @param current 現在値
 * @param dt 微小時間
 * @param pre_error 前回偏差
 * @param kd 微分ゲイン
 * @return d値
 */
float calc_pos_d(float target, float current, float dt, float pre_error, const float kd)
{
    float error = target - current;
    float pos_d = kd * ((error - pre_error) / dt);

    return pos_d;
}

/**
 * @brief calclate positional_pid
 *
 * @param target 目標値
 * @param current 現在値
 * @param p pos_pid_controller pointer
 * @return positional_pid_value
 */
int calc_pos_pid(float target, float current, // 位置型pid
                 pos_pid_controller *p)
{
    unsigned int now_time = millis();
    float dt = (now_time - p->pre_time) / 1000.00; // seconds

    if (p->pre_time == 0 || dt == 0) {
        dt = 0.01; // 10ms
    }

    float pos_p = calc_pos_p(target, current, p->kp);

    float pos_i = calc_pos_i(target, current, dt, p->pre_i_value, p->ki, p->max_i_value);

    float pos_d = calc_pos_d(target, current, dt, p->pre_error, p->kd);

    int pos_pid = int(pos_p + pos_i + pos_d);

    // anti wind up
    if (pos_pid > p->max_output_pwm) {
        pos_pid = p->max_output_pwm;
    }
    if (pos_pid < -p->max_output_pwm) {
        pos_pid = -p->max_output_pwm;
    }

    // update data
    p->pre_time = now_time;
    p->pre_error = target - current;
    p->pre_i_value = pos_i;
    p->pre_pos_pid = pos_pid;

    return pos_pid;
}

void init_vel_pid(const float kp, const float ki, const float kd, const int max_output_pwm, vel_pid_controller *p)
{
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
    p->max_output_pwm = max_output_pwm;

    p->pre_time = 0;
    p->pre_error = 0;
    p->pre_pre_error = 0;
    p->pre_vel_pid = 0;
}

float calc_vel_p(float error, float pre_error, const float kp)
{
    float vel_p = kp * (error - pre_error);
    return vel_p;
}

float calc_vel_i(float error, float dt, const float ki)
{
    float vel_i = ki * (error)*dt;
    return vel_i;
}

float calc_vel_d(float error, float pre_error, float pre_pre_error, const float kd)
{
    float vel_d = kd * (error - 2 * pre_error + pre_pre_error);
    return vel_d;
}

int calc_vel_pid(float target, float current, // 速度型pid
                 vel_pid_controller *p)
{
    unsigned int now_time = millis();
    float dt = (now_time - p->pre_time) / 1000.00; // seconds
    if (p->pre_time == 0 || dt == 0) {
        dt = 0.01; // 10ms
    }

    float error = target - current;

    float vel_p = calc_vel_p(error, p->pre_error, p->kp);

    float vel_i = calc_vel_i(error, dt, p->ki);

    float vel_d = calc_vel_d(error, p->pre_error, p->pre_pre_error, p->kd);

    int vel_pid = p->pre_vel_pid + int(vel_p + vel_i + vel_d);

    if (vel_pid > p->max_output_pwm) {
        vel_pid = p->max_output_pwm;
    }
    if (vel_pid < -p->max_output_pwm) {
        vel_pid = -p->max_output_pwm;
    }

    p->pre_pre_error = p->pre_error;
    p->pre_error = error;
    p->pre_time = now_time;
    p->pre_vel_pid = vel_pid;

    return vel_pid;
}
