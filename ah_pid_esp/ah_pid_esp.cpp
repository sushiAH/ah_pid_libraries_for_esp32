//*@file ah_pid_esp.cpp
/**
 * @brief pid実行ライブラリ
 */

#include <ah_pid_esp.h>

// ---- Config ----
// ピンの並びは使用する基盤に合わせて要変更
// pinnum array for pwm
const int PINNUM_POWER[4] = {32, 26, 27, 12};
const int PINNUM_DIR[4] = {33, 25, 14, 13};

// pinnum array for air
const int PINNUM_AIR[4] = {33, 25, 14, 13};

// pinnum array for encoder
// 使用する基板及び機構によってピンを変更する
const int ENC_PINNUM_A[4] = {23, 18, 16, 2};
const int ENC_PINNUM_B[4] = {19, 17, 21, 15};

// pid位置制御実行
static float run_pid_pos(float target, int motor_id, struct pos_pid_controller *pid_pos, struct encoder *enc, struct motor_driver *driver)
{
    update_enc(enc);

    unsigned int now_time = millis();
    float dt = (now_time - pid_pos->pre_time) / 1000.00; // seconds
    //
    if (pid_pos->pre_time == 0 || dt == 0) {
        dt = 0.01; // 10ms
        pid_pos->current_smooth_target = enc->pos;
    }

    update_profile_vel(target, dt, pid_pos);
    int pid_pos_value = calc_pos_pid(pid_pos->current_smooth_target, enc->pos, dt, pid_pos);
    write_to_motor(pid_pos_value, driver);

    pid_pos->pre_time = now_time;
    return pid_pos_value;
}

// pid ポテンショメーター位置制御実行
static float run_pid_pos_with_potentio(float target, int motor_id, struct pos_pid_controller *pid_pos,
                                       struct encoder *enc, struct motor_driver *driver)
{

    float pos = read_potentio(motor_id);
    enc->pos = pos;

    unsigned int now_time = millis();
    float dt = (now_time - pid_pos->pre_time) / 1000.00; // seconds

    // 初期化直後
    if (pid_pos->pre_time == 0 || dt == 0) {
        dt = 0.01; // 10ms
        pid_pos->current_smooth_target = enc->pos;
    }
    update_profile_vel(target, dt, pid_pos);
    int pid_pos_value = calc_pos_pid(pid_pos->current_smooth_target, enc->pos, dt, pid_pos);
    write_to_motor(pid_pos_value, driver);

    pid_pos->pre_time = now_time;
    return pid_pos_value;
}

// pid速度制御実行
static float run_pid_vel(float target, int motor_id, struct vel_pid_controller *pid_vel, struct encoder *enc, struct motor_driver *driver)
{
    unsigned int now_time = millis();
    float dt = (now_time - pid_vel->pre_time) / 1000.00; // seconds

    if (pid_vel->pre_time == 0 || dt == 0) {
        dt = 0.01; // 10ms
    }

    update_enc(enc);
    int pid_vel_value = calc_vel_pid(target, enc->vel, dt, pid_vel);

    write_to_motor(pid_vel_value, driver);

    pid_vel->pre_time = now_time;
    return pid_vel_value;
}

// pid cascade 位置制御実行
static float run_pid_cascade_pos(float target, int motor_id, struct vel_pid_controller *pid_vel,
                                 struct pos_pid_controller *pid_pos, struct encoder *enc, struct motor_driver *driver)
{
    unsigned int now_time = millis();
    float dt = (now_time - pid_vel->pre_time) / 1000.00; // seconds

    if (pid_vel->pre_time == 0 || dt == 0) {
        dt = 0.01; // 10ms
    }

    update_enc(enc);
    int pid_pos_value = calc_pos_pid(target, enc->pos, dt, pid_pos);
    int pid_vel_value = calc_vel_pid(pid_pos_value, enc->vel, dt, pid_vel);

    write_to_motor(pid_vel_value, driver);

    pid_vel->pre_time = now_time;
    return pid_vel_value;
}

// modeを設定
void set_mode(uint8_t operating_mode, struct motor_controller *ctrl)
{
    if (xSemaphoreTake(ctrl->mutex, portMAX_DELAY) == pdTRUE) {
        ctrl->operating_mode = operating_mode;
        xSemaphoreGive(ctrl->mutex);
    }
}

// goal_posに書き込み
void set_pos(float goal_pos, struct motor_controller *ctrl)
{
    if (xSemaphoreTake(ctrl->mutex, portMAX_DELAY) == pdTRUE) {
        ctrl->goal_pos = goal_pos;
        xSemaphoreGive(ctrl->mutex);
    }
}

// goal_velに書き込み
void set_vel(float goal_vel, struct motor_controller *ctrl)
{
    if (xSemaphoreTake(ctrl->mutex, portMAX_DELAY) == pdTRUE) {
        ctrl->goal_vel = goal_vel;
        xSemaphoreGive(ctrl->mutex);
    }
}

// goal_pwmに書き込み
void set_pwm(int32_t goal_pwm, struct motor_controller *ctrl)
{
    if (xSemaphoreTake(ctrl->mutex, portMAX_DELAY) == pdTRUE) {
        ctrl->goal_pwm = goal_pwm;
        xSemaphoreGive(ctrl->mutex);
    }
}

void set_air(uint8_t air_val, struct motor_controller *ctrl)
{
    if (xSemaphoreTake(ctrl->mutex, portMAX_DELAY) == pdTRUE) {
        ctrl->air_val = air_val;
        xSemaphoreGive(ctrl->mutex);
    }
}

void set_pos_p_gain(float pos_p_gain, struct motor_controller *ctrl)
{
    if (xSemaphoreTake(ctrl->mutex, portMAX_DELAY) == pdTRUE) {
        ctrl->POS_PID.kp = pos_p_gain;
        xSemaphoreGive(ctrl->mutex);
    }
}

void set_pos_i_gain(float pos_i_gain, struct motor_controller *ctrl)
{
    if (xSemaphoreTake(ctrl->mutex, portMAX_DELAY) == pdTRUE) {
        ctrl->POS_PID.ki = pos_i_gain;
        xSemaphoreGive(ctrl->mutex);
    }
}

void set_pos_d_gain(float pos_d_gain, struct motor_controller *ctrl)
{
    if (xSemaphoreTake(ctrl->mutex, portMAX_DELAY) == pdTRUE) {
        ctrl->POS_PID.kd = pos_d_gain;
        xSemaphoreGive(ctrl->mutex);
    }
}

void set_vel_p_gain(float vel_p_gain, struct motor_controller *ctrl)
{
    if (xSemaphoreTake(ctrl->mutex, portMAX_DELAY) == pdTRUE) {
        ctrl->VEL_PID.kp = vel_p_gain;
        xSemaphoreGive(ctrl->mutex);
    }
}

void set_vel_i_gain(float vel_i_gain, struct motor_controller *ctrl)
{
    if (xSemaphoreTake(ctrl->mutex, portMAX_DELAY) == pdTRUE) {
        ctrl->VEL_PID.ki = vel_i_gain;
        xSemaphoreGive(ctrl->mutex);
    }
}

void set_vel_d_gain(float vel_d_gain, struct motor_controller *ctrl)
{
    if (xSemaphoreTake(ctrl->mutex, portMAX_DELAY) == pdTRUE) {
        ctrl->VEL_PID.kd = vel_d_gain;
        xSemaphoreGive(ctrl->mutex);
    }
}

void set_motor_rot_dir(uint8_t motor_rot_dir, struct motor_controller *ctrl)
{
    if (xSemaphoreTake(ctrl->mutex, portMAX_DELAY) == pdTRUE) {
        ctrl->DRIVER.motor_rot_dir = motor_rot_dir;
        xSemaphoreGive(ctrl->mutex);
    }
}

void set_profile_vel(float profile_vel, struct motor_controller *ctrl)
{
    if (xSemaphoreTake(ctrl->mutex, portMAX_DELAY) == pdTRUE) {
        ctrl->POS_PID.max_vel = profile_vel;
        xSemaphoreGive(ctrl->mutex);
    }
}

void set_profile_accel(float profile_accel, struct motor_controller *ctrl)
{
    if (xSemaphoreTake(ctrl->mutex, portMAX_DELAY) == pdTRUE) {
        ctrl->POS_PID.max_acc = profile_accel;
        xSemaphoreGive(ctrl->mutex);
    }
}

// 現在のposを返す
float get_current_pos(struct motor_controller *ctrl)
{
    float current_pos = 0;
    if (xSemaphoreTake(ctrl->mutex, portMAX_DELAY) == pdTRUE) {
        current_pos = ctrl->ENC.pos;
        xSemaphoreGive(ctrl->mutex);
    }
    return current_pos;
}

// 現在のvelを返す
float get_current_vel(struct motor_controller *ctrl)
{
    float current_vel = 0;
    if (xSemaphoreTake(ctrl->mutex, portMAX_DELAY) == pdTRUE) {
        current_vel = ctrl->ENC.vel;
        xSemaphoreGive(ctrl->mutex);
    }
    return current_vel;
}

// operating modeに応じてpidを実行
static int run_pid_by_operating(int operating_mode, struct motor_controller *ctrl)
{
    if (operating_mode == encoder_position_mode) {
        run_pid_pos(ctrl->goal_pos, ctrl->motor_id, &ctrl->POS_PID, &ctrl->ENC, &ctrl->DRIVER);
        return 0;
    } else if (operating_mode == potentio_position_mode) {
        run_pid_pos_with_potentio(ctrl->goal_pos, ctrl->motor_id, &ctrl->POS_PID, &ctrl->ENC, &ctrl->DRIVER);
        return 0;
    } else if (operating_mode == velocity_mode) {
        run_pid_vel(ctrl->goal_vel, ctrl->motor_id, &ctrl->VEL_PID, &ctrl->ENC, &ctrl->DRIVER);
        return 0;
    } else if (operating_mode == pwm_mode) {
        update_enc(&ctrl->ENC);
        write_to_motor(ctrl->goal_pwm, &ctrl->DRIVER);
        return 0;
    } else if (operating_mode == air_mode) {
        digitalWrite(PINNUM_AIR[ctrl->motor_id], ctrl->air_val);
        return 0;
    } else if (operating_mode == stop_mode) {
        write_to_motor(0, &ctrl->DRIVER);
        // reset
        reset_enc(&ctrl->ENC);
        reset_pos_pid(&ctrl->POS_PID);
        reset_vel_pid(&ctrl->VEL_PID);
        ctrl->goal_pos = 0.00;
        ctrl->goal_vel = 0.00;
        return 0;
    } else {
        return operating_mode;
    }
}

// pid実行
static void run_pid(void *pvParameters)
{
    float target = 0;
    int operating_mode = 0;
    motor_controller *ctrl = (motor_controller *)pvParameters;
    // 周期管理
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(ctrl->pid_period);

    while (1) {
        operating_mode = ctrl->operating_mode;
        // pid実行
        run_pid_by_operating(operating_mode, ctrl);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// free rtos pid task 作成
void init_motor_controller(const int max_output_pwm, const int max_i_value, const int enc_resolution,
                           const unsigned int pid_period, const int motor_id, struct motor_controller *ctrl)
{
    ctrl->pid_period = pid_period;
    ctrl->motor_id = motor_id;
    // 共有用変数の初期化
    ctrl->operating_mode = 0;
    ctrl->goal_pos = 0;
    ctrl->goal_vel = 0;
    ctrl->goal_pwm = 0;
    ctrl->current_pos = 0;
    ctrl->current_vel = 0;

    init_pos_pid(0, 0, 0, max_output_pwm, max_i_value, &ctrl->POS_PID);
    init_vel_pid(0, 0, 0, max_output_pwm, &ctrl->VEL_PID);
    init_enc(motor_id, ENC_PINNUM_A[motor_id], ENC_PINNUM_B[motor_id], enc_resolution, &ctrl->ENC);
    init_potentio(motor_id);
    init_motor(PINNUM_POWER[motor_id], motor_id, PINNUM_DIR[motor_id], &ctrl->DRIVER);

    ctrl->mutex = xSemaphoreCreateMutex();

    // タスクの作成
    xTaskCreate(run_pid,      // タスク関数
                "pid",        // タスク名
                4096,         // スタックサイズ
                (void *)ctrl, // タスクに渡す引数
                5,            // 優先度
                NULL          // タスクハンドル
    );
}
