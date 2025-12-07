pid制御　設計


通信ライブラリ(uart or can) でpcからのデータを受信。受信したデータをcontrol_tableに基づいてmotor_controller構造体の共有変数に書き込む。
operating_modeに基づいてpidを実行する。
通信タスクとpidタスクは非同期で動いている。
pidタスクは、目標値の読み込み、pidの実行、現在値の書き込みを行う
通信タスクは、受信したデータを共有変数に書き込み、送信の命令があれば現在値を読み出して送信を行う






データ構造
  motor_controller        

    pid_pos_esp
      encoder
      pos_pid_controller
  
    pid_vel_esp
      encoder
      pos_pid_controller








