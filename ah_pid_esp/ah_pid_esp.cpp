#include <ah_pid_esp.h>

// ---- Config ----
const int PINNUM_POWER[4] = {32, 25, 27, 12};
const int PINNUM_DIR[4] = {33, 26, 14, 13};

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

void run_pid(void* pvParameters) {
  float goal_pos = 0;
  float goal_vel = 0;
  float goal_pwm = 0;

  int operating_mode = 0;

  motor_controller* p = (motor_controller*)pvParameters;

  // 周期管理準備
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(p->pid_period);

  while (1) {
    // 共有変数読み出し
    if (xSemaphoreTake(p->mutex, portMAX_DELAY) == pdTRUE) {
      operating_mode = p->operating_mode;
      goal_pos = float(p->goal_pos_int / 1000.0);
      goal_vel = float(p->goal_vel_int / 1000.0);
      goal_pwm = int(p->goal_pwm / 1000);
      xSemaphoreGive(p->mutex);
    }

    // run_pid
    if (operating_mode == 1) {
      run_pid_pos(goal_pos, p->motor_id, &p->POS);
    } else if (operating_mode == 2) {
      run_pid_vel(goal_vel, p->motor_id, &p->VEL);
    }

    else if (operating_mode == 3) {
      p->VEL.ENC.vel = update_vel(&p->VEL.ENC);
      write_to_motor(goal_pwm, p->motor_id, PINNUM_DIR[p->motor_id]);
    }

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
  enc_init(enc_resolution, motor_id, &p->ENC);
}

void init_pid_vel_esp(const float kp, const float ki, const float kd,
                      const int max_output_pwm, const int enc_resolution,
                      const int motor_id, pid_vel_esp* p) {
  vel_pid_init(kp, ki, kd, max_output_pwm, &p->PID);
  enc_init(enc_resolution, motor_id, &p->ENC);
}

// if you use esp
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

float run_pid_pos(float target, int motor_id, pid_pos_esp* p) {
  float pos = update_pos(&p->ENC);
  int pos_pid = calc_pos_pid(target, pos, &p->PID);
  write_to_motor(pos_pid, motor_id, PINNUM_DIR[motor_id]);

  return pos;
}

float run_pid_vel(float target, int motor_id, pid_vel_esp* p) {
  float vel = update_vel(&p->ENC);
  int vel_pid = calc_vel_pid(target, vel, &p->PID);
  write_to_motor(vel_pid, motor_id, PINNUM_DIR[motor_id]);

  // update_data
  return vel;
}
