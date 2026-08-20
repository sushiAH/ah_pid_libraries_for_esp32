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

    reset_pos_pid(p);
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
int calc_pos_pid(float target, float current, float dt, // 位置型pid
                 pos_pid_controller *p)
{

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

    reset_vel_pid(p);
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

int calc_vel_pid(float target, float current, float dt, // 速度型pid
                 vel_pid_controller *p)
{

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
    p->pre_vel_pid = vel_pid;

    return vel_pid;
}

void reset_pos_pid(pos_pid_controller *pos_pid)
{
    pos_pid->pre_time = 0;
    pos_pid->pre_error = 0.00;
    pos_pid->pre_i_value = 0.00;
    pos_pid->pre_pos_pid = 0;
    pos_pid->current_smooth_target = 0.00;
    pos_pid->v_state = 0.00;
    pos_pid->max_vel = 0.00;
    pos_pid->max_acc = 0.00;
}

void reset_vel_pid(vel_pid_controller *vel_pid)
{
    vel_pid->pre_time = 0;
    vel_pid->pre_error = 0.00;
    vel_pid->pre_pre_error = 0.00;
    vel_pid->pre_vel_pid = 0;
}

float calc_profile_vel(float pc_goal, float current_smooth_target, float *v_state,
                       float max_vel, float max_acc, float dt)
{

    // 1. ゴールまでの残りの距離を計算
    float distance_to_go = pc_goal - current_smooth_target;

    // 5. 目的地に到着したかどうかの判定（微振動を防ぐ）
    // 距離が近く、かつ速度が十分に落ちていたら、ピタッと止める
    if (fabsf(distance_to_go) < 20.0f) {
        *v_state = 0;
        return pc_goal; // 目的地そのものを返す
    }

    // 2. 「今からブレーキをかけて止まれる距離（制動距離）」を計算
    // 公式: stop_dist = v^2 / (2 * a)
    float stop_distance = (*v_state * *v_state) / (2.0f * max_acc);

    // 3. 次の周期の速度（v_state）をどうすべきか判断する
    if (distance_to_go > 0) {
        // --- 正方向へ進むべきとき ---
        if (distance_to_go > stop_distance) {
            *v_state += max_acc * dt; // まだ余裕があるので加速
        } else {
            *v_state -= max_acc * dt; // ブレーキをかけないと行き過ぎる。減速
        }
    } else if (distance_to_go < 0) {
        // --- 負方向へ進むべきとき ---
        if (fabsf(distance_to_go) > stop_distance) {
            *v_state -= max_acc * dt; // 負の方向に加速
        } else {
            *v_state += max_acc * dt; // 負の方向からブレーキ（速度を0に近づける）
        }
    }

    // 4. 速度が最高速度制限(max_vel)を超えないようにガード
    if (*v_state > max_vel)
        *v_state = max_vel;
    if (*v_state < -max_vel)
        *v_state = -max_vel;

    // 6. 「今の位置」に「決まった速度 × 時間」を足して、次の位置を算出
    float next_smooth_target = current_smooth_target + (*v_state * dt);

    return next_smooth_target;
}

void update_profile_vel(float pc_goal, float dt, pos_pid_controller *pos_pid)
{
    pos_pid->current_smooth_target = calc_profile_vel(pc_goal, pos_pid->current_smooth_target, &pos_pid->v_state, pos_pid->max_vel, pos_pid->max_acc, dt);
}
