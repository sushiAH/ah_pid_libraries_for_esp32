/**
 * @file ah_pid_esp.cpp
 * @brief pid実行ライブラリ
 */

#include <ah_pid_esp.h>

// ---- Config ----
const int PINNUM_POWER[4] = {32, 25, 27, 12};
const int PINNUM_DIR[4] = {33, 26, 14, 13};

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
                           motor_controller* p) {
  p->pid_period = pid_period;
  p->motor_id = motor_id;

  // 共有用変数の初期化
  p->goal_pos_int = 0;
  p->goal_vel_int = 0;
  p->goal_pwm = 0;
  p->current_pos_int = 0;
  p->current_vel_int = 0;
  p->operating_mode = 0;

  init_pid_pos_esp(0, 0, 0, max_output_pwm, max_i_value, enc_resolution,
                   motor_id, &p->POS);
  init_pid_vel_esp(0, 0, 0, max_output_pwm, enc_resolution, motor_id, &p->VEL);

  p->mutex = xSemaphoreCreateMutex();

  // タスクの作成
  xTaskCreate(run_pid,   // タスク関数
              "pid",     // タスク名
              4096,      // スタックサイズ
              (void*)p,  // タスクに渡す引数
              3,         // 優先度
              NULL       // タスクハンドル
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
int read_target_by_operating(motor_controller* p, int operating_mode,
                             float* target) {
  if (operating_mode == 1) {
    *target = (float(p->goal_pos_int) / 1000.0);
    return 0;
  }

  else if (operating_mode == 2) {
    *target = (float(p->goal_vel_int) / 1000.0);
    return 0;
  }

  else if (operating_mode == 3) {
    *target = (float(p->goal_pwm) / 1000.0);
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
int run_pid_by_operating(motor_controller* p, int operating_mode,
                         float target) {
  if (operating_mode == 1) {
    run_pid_pos(target, p->motor_id, &p->POS);
    return 0;

  } else if (operating_mode == 2) {
    run_pid_vel(target, p->motor_id, &p->VEL);
    return 0;

  } else if (operating_mode == 3) {
    p->VEL.ENC.vel = update_vel(&p->VEL.ENC);
    write_to_motor(target, p->motor_id, PINNUM_DIR[p->motor_id]);
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

  motor_controller* p = (motor_controller*)pvParameters;

  // 周期管理
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(p->pid_period);

  while (1) {
    // 共有変数から目標値の読み出し
    if (xSemaphoreTake(p->mutex, portMAX_DELAY) == pdTRUE) {
      operating_mode = p->operating_mode;
      read_target_by_operating(p, operating_mode, &target);
      xSemaphoreGive(p->mutex);
    }

    // pid実行
    run_pid_by_operating(p, operating_mode, target);

    // 共有データの書き込み
    if (xSemaphoreTake(p->mutex, portMAX_DELAY) == pdTRUE) {
      p->current_pos_int = int32_t(p->POS.ENC.pos * 1000);
      p->current_vel_int = int32_t(p->VEL.ENC.vel * 1000);
      xSemaphoreGive(p->mutex);
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void init_pid_pos_esp(const float kp, const float ki, const float kd,
                      const int max_output_pwm, const int max_i_value,
                      const int enc_resolution, const int motor_id,
                      pid_pos_esp* p) {
  pos_pid_init(kp, ki, kd, max_output_pwm, max_i_value, &p->PID);
  enc_init(motor_id, enc_resolution, &p->ENC);
}

void init_pid_vel_esp(const float kp, const float ki, const float kd,
                      const int max_output_pwm, const int enc_resolution,
                      const int motor_id, pid_vel_esp* p) {
  vel_pid_init(kp, ki, kd, max_output_pwm, &p->PID);
  enc_init(motor_id, enc_resolution, &p->ENC);
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
float run_pid_pos(float target, int motor_id, pid_pos_esp* p) {
  float pos = update_pos(&p->ENC);
  int pos_pid = calc_pos_pid(target, pos, &p->PID);
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
float run_pid_vel(float target, int motor_id, pid_vel_esp* p) {
  float vel = update_vel(&p->ENC);
  int vel_pid = calc_vel_pid(target, vel, &p->PID);
  write_to_motor(vel_pid, motor_id, PINNUM_DIR[motor_id]);

  return vel;
}
