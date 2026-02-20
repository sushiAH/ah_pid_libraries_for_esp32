/**
 * @file ah_pid_esp.cpp
 * @brief pid実行ライブラリ
 */

#include <ah_pid_esp.h>

// ---- Config ----
// ピンの並びは使用する基盤に合わせて要変更
// pinnum array for pwm
const int PINNUM_POWER[4] = {32, 26, 27, 12};
const int PINNUM_DIR[4] = {33, 25, 14, 13};

// pinnum array for air
const int PINNUM_AIR[4] = {0, 25, 0, 0};

// pinnum array for encoder
const int ENC_PINNUM_A[4] = {23, 17, 21, 15};
const int ENC_PINNUM_B[4] = {19, 18, 16, 2};

/**
 * @brief pid位置制御実行
 *
 * @param target 目標値
 * @param motor_id
 * @param p
 * @return 現在のposition
 */
static float run_pid_pos(float target, int motor_id, struct pos_pid_controller *pid_pos, struct encoder *enc)
{
    update_enc(enc);
    int pid_pos_value = calc_pos_pid(target, enc->pos, pid_pos);
    write_to_motor(pid_pos_value, motor_id, PINNUM_DIR[motor_id]);

    return pid_pos_value;
}

/**
 * @brief pid位置制御実行 with potentio meter
 *
 * @param target 目標値
 * @param motor_id
 * @return 現在のabsolute position
 * @param pid_pos
 */
static float run_pid_pos_with_potentio(float target, int motor_id, struct pos_pid_controller *pid_pos,
                                       struct encoder *enc)
{
    float pos = read_potentio(motor_id);
    enc->pos = pos;
    int pid_pos_value = calc_pos_pid(target, pos, pid_pos);
    write_to_motor(pid_pos_value, motor_id, PINNUM_DIR[motor_id]);

    return pid_pos_value;
}

/**
 * @brief pid速度制御実行
 *
 * @param target 目標値
 * @param motor_id
 * @param p
 * @return 現在のvelocity
 */
static float run_pid_vel(float target, int motor_id, struct vel_pid_controller *pid_vel, struct encoder *enc)
{
    update_enc(enc);
    int pid_vel_value = calc_vel_pid(target, enc->vel, pid_vel);
    write_to_motor(pid_vel_value, motor_id, PINNUM_DIR[motor_id]);

    return pid_vel_value;
}

static float run_pid_cascade_pos(float target, int motor_id, struct vel_pid_controller *pid_vel,
                                 struct pos_pid_controller *pid_pos, struct encoder *enc)
{
    update_enc(enc);
    int pid_pos_value = calc_pos_pid(target, enc->pos, pid_pos);
    int pid_vel_value = calc_vel_pid(pid_pos_value, enc->vel, pid_vel);

    write_to_motor(pid_vel_value, motor_id, PINNUM_DIR[motor_id]);

    return pid_vel_value;
}

/**
 * @brief 共有変数,operating_modeに書き込み
 * mode
 * 1 position
 * 2 velocity
 * 3 pwm
 * @param operating_mode
 * @param ctrl
 */
static void set_mode(int operating_mode, struct motor_controller *ctrl)
{
    if (xSemaphoreTake(ctrl->mutex, portMAX_DELAY) == pdTRUE) {
        ctrl->operating_mode = operating_mode;
        xSemaphoreGive(ctrl->mutex);
    }
}

/**
 * @brief 共有変数、goal_pos_intに書き込み
 *
 * @param goal_pos
 * @param ctrl
 */
static void set_pos(float goal_pos, struct motor_controller *ctrl)
{
    if (xSemaphoreTake(ctrl->mutex, portMAX_DELAY) == pdTRUE) {
        ctrl->goal_pos_int = int32_t(goal_pos * 1000);
        xSemaphoreGive(ctrl->mutex);
    }
}

/**
 * @brief 共有変数、goal_velに書き込み
 *
 * @param goal_vel
 * @param ctrl
 */
static void set_vel(float goal_vel, struct motor_controller *ctrl)
{
    if (xSemaphoreTake(ctrl->mutex, portMAX_DELAY) == pdTRUE) {
        ctrl->goal_vel_int = int32_t(goal_vel * 1000);
        xSemaphoreGive(ctrl->mutex);
    }
}

static void set_pwm(int goal_pwm, struct motor_controller *ctrl)
{
    if (xSemaphoreTake(ctrl->mutex, portMAX_DELAY) == pdTRUE) {
        ctrl->goal_pwm = goal_pwm * 1000;
        xSemaphoreGive(ctrl->mutex);
    }
}

/**
 * @brief 現在のposを返す
 *
 * @param ctrl
 * @return 現在のpos [float]
 */
static float read_current_pos(struct motor_controller *ctrl)
{
    float current_pos = 0;
    if (xSemaphoreTake(ctrl->mutex, portMAX_DELAY) == pdTRUE) {
        current_pos = float(ctrl->current_pos_int / 1000.0);
        xSemaphoreGive(ctrl->mutex);
    }
    return current_pos;
}

/**
 * @brief 現在のvelを返す
 *
 * @param ctrl
 * @return 現在のvel [float]
 */
static float read_current_vel(struct motor_controller *ctrl)
{
    float current_vel = 0;
    if (xSemaphoreTake(ctrl->mutex, portMAX_DELAY) == pdTRUE) {
        current_vel = float(ctrl->current_vel_int / 1000.0);
        xSemaphoreGive(ctrl->mutex);
    }
    return current_vel;
}

/**
 * @brief operating_modeに応じて、目標値を返す
 *
 * @param p
 * @param target 目標値のポインタ
 * @return err
 */
static int read_target_by_operating(int operating_mode, float *target, struct motor_controller *ctrl)
{
    if (operating_mode == encoder_position_mode) {
        *target = (float(ctrl->goal_pos_int) / 1000.0);
        return 0;

    } else if (operating_mode == potentio_position_mode) {
        *target = (float(ctrl->goal_pos_int) / 1000.0);
        return 0;
    }

    else if (operating_mode == velocity_mode) {
        *target = (float(ctrl->goal_vel_int) / 1000.0);
        return 0;
    }

    else if (operating_mode == pwm_mode) {
        *target = (float(ctrl->goal_pwm) / 1000.0);
        return 0;
    }

    else if (operating_mode == air_mode) {
        *target = ctrl->air_val;
        return 0;
    }

    else {
        return operating_mode;
    }
}

/**
 * @brief operating_mode に応じてpidを実行する
 *
 * @param p
 * @param target 目標値
 */
static int run_pid_by_operating(int operating_mode, float target, struct motor_controller *ctrl)
{
    if (operating_mode == encoder_position_mode) {
        run_pid_pos(target, ctrl->motor_id, &ctrl->POS_PID, &ctrl->ENC);
        return 0;

    } else if (operating_mode == potentio_position_mode) {
        run_pid_pos_with_potentio(target, ctrl->motor_id, &ctrl->POS_PID, &ctrl->ENC);
        return 0;

    } else if (operating_mode == velocity_mode) {
        run_pid_vel(target, ctrl->motor_id, &ctrl->VEL_PID, &ctrl->ENC);
        return 0;

    } else if (operating_mode == pwm_mode) {
        update_enc(&ctrl->ENC);
        write_to_motor(int(target), ctrl->motor_id, PINNUM_DIR[ctrl->motor_id]);
        return 0;

    } else if (operating_mode == air_mode) {
        digitalWrite(PINNUM_AIR[ctrl->motor_id], target);
        return 0;

    } else if (operating_mode == stop_mode) {
        write_to_motor(0, ctrl->motor_id, PINNUM_DIR[ctrl->motor_id]);
        reset_enc(&ctrl->ENC);
        reset_pos_pid(&ctrl->POS_PID);
        reset_vel_pid(&ctrl->VEL_PID);
        return 0;
    }

    else {
        return operating_mode;
    }
}

/**
 * @brief
 * motor_controllerの共有変数へ、目標値の書き込み,現在値の読み出しを行う。
 * 目標値に追従するようにpidを実行する
 *
 * @param pvParameters free_rtos_task pointer
 */
static void run_pid(void *pvParameters)
{
    float target = 0;
    int operating_mode = 0;

    motor_controller *ctrl = (motor_controller *)pvParameters;

    // 周期管理
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(ctrl->pid_period);

    while (1) {
        // 共有変数から目標値の読み出し
        if (xSemaphoreTake(ctrl->mutex, portMAX_DELAY) == pdTRUE) {
            operating_mode = ctrl->operating_mode;
            read_target_by_operating(operating_mode, &target, ctrl);
            xSemaphoreGive(ctrl->mutex);
        }

        // pid実行
        run_pid_by_operating(operating_mode, target, ctrl);

        // 共有データの書き込み
        if (xSemaphoreTake(ctrl->mutex, portMAX_DELAY) == pdTRUE) {
            ctrl->current_pos_int = int32_t(ctrl->ENC.pos * 1000);
            ctrl->current_vel_int = int32_t(ctrl->ENC.vel * 1000);
            xSemaphoreGive(ctrl->mutex);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
 * @brief motor_controller構造体の初期化
 *
 * @param max_output_pwm pwm値制限
 * @param max_i_value i値制限
 * @param enc_resolution エンコーダー分解能
 * @param pid_period pid周期 [ms]
 * @param motor_id モーターID
 * @param p motor_controller pointer
 */
void init_motor_controller(const int max_output_pwm, const int max_i_value, const int enc_resolution,
                           const unsigned int pid_period, const int motor_id, struct motor_controller *ctrl)
{
    ctrl->pid_period = pid_period;
    ctrl->motor_id = motor_id;

    // 共有用変数の初期化
    ctrl->goal_pos_int = 0;
    ctrl->goal_vel_int = 0;
    ctrl->goal_pwm = 0;
    ctrl->current_pos_int = 0;
    ctrl->current_vel_int = 0;
    ctrl->operating_mode = 0;

    init_pos_pid(0, 0, 0, max_output_pwm, max_i_value, &ctrl->POS_PID);
    init_vel_pid(0, 0, 0, max_output_pwm, &ctrl->VEL_PID);
    init_enc(motor_id, ENC_PINNUM_A[motor_id], ENC_PINNUM_B[motor_id], enc_resolution, &ctrl->ENC);
    init_potentio(motor_id);

    ctrl->mutex = xSemaphoreCreateMutex();

    // タスクの作成
    xTaskCreate(run_pid,      // タスク関数
                "pid",        // タスク名
                4096,         // スタックサイズ
                (void *)ctrl, // タスクに渡す引数
                5,            // 優先度
                NULL          // タスクハンドル
    );

    init_motor(PINNUM_POWER[motor_id], motor_id, PINNUM_DIR[motor_id]);
}
