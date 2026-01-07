/**
 * @file ah_pid_esp.cpp
 * @brief pid実行ライブラリ
 */

#include <ah_pid_esp.h>

// ---- Config ----
// ピンの並びは使用する基盤に合わせて要変更
const int PINNUM_POWER[4] = {32, 26, 14, 12};
const int PINNUM_DIR[4] = {33, 25, 27, 13};

const int PINNUM_AIR[4] = {0, 25, 0, 0};

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
void init_motor_controller(const int max_output_pwm, const int max_i_value,
                           const int enc_resolution,
                           const unsigned int pid_period, const int motor_id,
                           motor_controller* ctrl) {
  ctrl->pid_period = pid_period;
  ctrl->motor_id = motor_id;

  // 共有用変数の初期化
  ctrl->goal_pos_int = 0;
  ctrl->goal_vel_int = 0;
  ctrl->goal_pwm = 0;
  ctrl->current_pos_int = 0;
  ctrl->current_vel_int = 0;
  ctrl->operating_mode = 0;

  init_pid_pos_esp(0, 0, 0, max_output_pwm, max_i_value, enc_resolution,
                   motor_id, &ctrl->POS);

  init_pid_vel_esp(0, 0, 0, max_output_pwm, enc_resolution, motor_id,
                   &ctrl->VEL);

  ctrl->mutex = xSemaphoreCreateMutex();

  // タスクの作成
  xTaskCreate(run_pid,      // タスク関数
              "pid",        // タスク名
              4096,         // スタックサイズ
              (void*)ctrl,  // タスクに渡す引数
              5,            // 優先度
              NULL          // タスクハンドル
  );

  pinMode(PINNUM_POWER[motor_id], OUTPUT);
  pinMode(PINNUM_DIR[motor_id], OUTPUT);

  ledcSetup(motor_id, 20000, 10);  // 10bit
  ledcAttachPin(PINNUM_POWER[motor_id], motor_id);
}

/**
 * @brief operating_modeに応じて、目標値を返す
 *
 * @param p
 * @param target 目標値のポインタ
 * @return err
 */
int read_target_by_operating(motor_controller* ctrl, int operating_mode,
                             float* target) {
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
int run_pid_by_operating(motor_controller* ctrl, int operating_mode,
                         float target) {
  if (operating_mode == encoder_position_mode) {
    run_pid_pos(target, ctrl->motor_id, &ctrl->POS);
    return 0;

  } else if (operating_mode == potentio_position_mode) {
    run_pid_pos_with_potentio(target, ctrl->motor_id, &ctrl->POS);
    return 0;

  } else if (operating_mode == velocity_mode) {
    run_pid_vel(target, ctrl->motor_id, &ctrl->VEL);
    return 0;

  } else if (operating_mode == pwm_mode) {
    update_vel(&ctrl->VEL.ENC);
    write_to_motor(int(target), ctrl->motor_id, PINNUM_DIR[ctrl->motor_id]);
    return 0;

  } else if (operating_mode == air_mode) {
    digitalWrite(PINNUM_AIR[ctrl->motor_id], target);
    return 0;

  } else if (operating_mode == stop_mode){
    write_to_motor(0, ctrl->motor_id, PINNUM_DIR[ctrl->motor_id]);
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
void run_pid(void* pvParameters) {
  float target = 0;
  int operating_mode = 0;

  motor_controller* ctrl = (motor_controller*)pvParameters;

  // 周期管理
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(ctrl->pid_period);

  while (1) {
    // 共有変数から目標値の読み出し
    if (xSemaphoreTake(ctrl->mutex, portMAX_DELAY) == pdTRUE) {
      operating_mode = ctrl->operating_mode;
      read_target_by_operating(ctrl, operating_mode, &target);
      xSemaphoreGive(ctrl->mutex);
    }

    // pid実行
    run_pid_by_operating(ctrl, operating_mode, target);

    // 共有データの書き込み
    if (xSemaphoreTake(ctrl->mutex, portMAX_DELAY) == pdTRUE) {
      ctrl->current_pos_int = int32_t(ctrl->POS.ENC.pos * 1000);
      ctrl->current_vel_int = int32_t(ctrl->VEL.ENC.vel * 1000);
      xSemaphoreGive(ctrl->mutex);
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void init_pid_pos_esp(const float kp, const float ki, const float kd,
                      const int max_output_pwm, const int max_i_value,
                      const int enc_resolution, const int motor_id,
                      pid_pos_esp* pid_pos) {
  pos_pid_init(kp, ki, kd, max_output_pwm, max_i_value, &pid_pos->PID);
  enc_init(motor_id, enc_resolution, &pid_pos->ENC);
  potentio_init(motor_id);
}

void init_pid_vel_esp(const float kp, const float ki, const float kd,
                      const int max_output_pwm, const int enc_resolution,
                      const int motor_id, pid_vel_esp* pid_vel) {
  vel_pid_init(kp, ki, kd, max_output_pwm, &pid_vel->PID);
  enc_init(motor_id, enc_resolution, &pid_vel->ENC);
}

/**
 * @brief モーターへpwmを書き込む
 *
 * @param pwm
 * @param CHANNEL_NUM esp32 pwm channel
 * @param PINNUM_DIR pin number motor dir
 */
void write_to_motor(int pwm, const int CHANNEL_NUM, const int PINNUM_DIR) {
  int dir = 0;

  if (pwm > 0) {
    dir = 0;
  }
  if (pwm < 0) {
    dir = 1;
    pwm = -pwm;
  }

  ledcWrite(CHANNEL_NUM, pwm);
  digitalWrite(PINNUM_DIR, dir);
}

/**
 * @brief pid位置制御実行
 *
 * @param target 目標値
 * @param motor_id
 * @param p
 * @return 現在のposition
 */
float run_pid_pos(float target, int motor_id, pid_pos_esp* pid_pos) {
  float pos = update_pos(&pid_pos->ENC);
  int pos_pid = calc_pos_pid(target, pos, &pid_pos->PID);
  write_to_motor(pos_pid, motor_id, PINNUM_DIR[motor_id]);

  return pos;
}

/**
 * @brief pid位置制御実行 with potentio meter
 *
 * @param target 目標値
 * @param motor_id
 * @param pid_pos
 * @return 現在のabsolute position
 */
float run_pid_pos_with_potentio(float target, int motor_id,
                                pid_pos_esp* pid_pos) {
  float pos = read_potentio(motor_id);
  pid_pos->ENC.pos = pos;
  int pos_pid = calc_pos_pid(target, pos, &pid_pos->PID);
  write_to_motor(pos_pid, motor_id, PINNUM_DIR[motor_id]);

  return pos;
}

/**
 * @brief pid速度制御実行
 *
 * @param target 目標値
 * @param motor_id
 * @param p
 * @return 現在のvelocity
 */
float run_pid_vel(float target, int motor_id, pid_vel_esp* pid_vel) {
  float vel = update_vel(&pid_vel->ENC);
  int vel_pid = calc_vel_pid(target, vel, &pid_vel->PID);
  write_to_motor(vel_pid, motor_id, PINNUM_DIR[motor_id]);

  return vel;
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
void set_mode(int operating_mode, motor_controller* ctrl) {
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
void set_pos(float goal_pos, motor_controller* ctrl) {
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
void set_vel(float goal_vel, motor_controller* ctrl) {
  if (xSemaphoreTake(ctrl->mutex, portMAX_DELAY) == pdTRUE) {
    ctrl->goal_vel_int = int32_t(goal_vel * 1000);
    xSemaphoreGive(ctrl->mutex);
  }
}

void set_pwm(int goal_pwm, motor_controller* ctrl) {
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
float read_current_pos(motor_controller* ctrl) {
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
float read_current_vel(motor_controller* ctrl) {
  float current_vel = 0;
  if (xSemaphoreTake(ctrl->mutex, portMAX_DELAY) == pdTRUE) {
    current_vel = float(ctrl->current_vel_int / 1000.0);
    xSemaphoreGive(ctrl->mutex);
  }
  return current_vel;
}
