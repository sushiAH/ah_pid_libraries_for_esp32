/**
 * @file ah_encoder_esp32.cpp
 * @brief esp32でエンコーダーから値を取得する
 * 注意 必ず50ms以内のループにて使用すること
 */

#include "ah_encoder_esp32.h"

/**
 * @brief エンコーダーの立ち上げ
 *
 * @param enc_resolution エンコーダー分解能
 * @param motor_id モーターID
 * @param p
 */
void enc_init(const int pcnt_unit_num, const int enc_pinnum_a,
              const int enc_pinnum_b, const int enc_resolution, encoder *p)
{
    p->enc_resolution = enc_resolution;
    p->enc_pinnum_a = enc_pinnum_a;
    p->enc_pinnum_b = enc_pinnum_b;

    if (pcnt_unit_num == 0) {
        p->PCNT_UNIT = PCNT_UNIT_0;
    } else if (pcnt_unit_num == 1) {
        p->PCNT_UNIT = PCNT_UNIT_1;
    } else if (pcnt_unit_num == 2) {
        p->PCNT_UNIT = PCNT_UNIT_2;
    } else if (pcnt_unit_num == 3) {
        p->PCNT_UNIT = PCNT_UNIT_3;
    }

    pinMode(enc_pinnum_a, INPUT_PULLUP);
    pinMode(enc_pinnum_b, INPUT_PULLUP);

    qei_setup_x1(p->PCNT_UNIT, enc_pinnum_a, enc_pinnum_b);
}

/**
 * @brief rpmの計算
 *
 * @param now_count 現在のカウント
 * @param pre_count 前回のカウント
 * @param enc_resolution エンコーダー分解能
 * @param dt 微小時間
 * @return rpm
 */
float calc_rpm_from_countdiff(int16_t now_count, int16_t pre_count,
                              int enc_resolution, float dt)
{
    int delta_count = now_count - pre_count;
    float delta_count_f = (float)delta_count;
    float diff_count_sec = (delta_count_f / dt);
    float vel = (diff_count_sec / (float)enc_resolution);

    return vel;
}

/**
 * @brief degreeの計算
 *
 * @param now_count 現在のカウント
 * @param enc_resolution エンコーダー分解能
 * @return motor_position [degree]
 */
float calc_degree(int16_t now_count, int enc_resolution)
{
    float pos_degree = ((double)now_count / enc_resolution) * 360; // degree
    //
    return pos_degree;
}

/**
 * @brief 微小時間の計算
 *
 * @param now_time 現在時間
 * @param pre_time 前回時間
 * @return 微小時間
 */
float calc_dt(unsigned int now_time, unsigned int pre_time)
{
    float dt = (now_time - pre_time) / 1000.000f;

    if (dt == 0.0f || pre_time == 0) {
        return 0.010f; // 10ms
    } else {
        return dt;
    }
}

/**
 * @brief rpmの更新
 *
 * @param
 * @return rpm
 */
float update_vel(encoder *p)
{
    unsigned int now_time = millis();

    float dt = calc_dt(now_time, p->pre_time);

    pcnt_get_counter_value(p->PCNT_UNIT, &p->now_count);
    float vel = calc_rpm_from_countdiff(p->now_count, p->pre_count,
                                        p->enc_resolution, dt);

    // update
    p->pre_time = now_time;
    p->pre_count = 0;
    pcnt_counter_clear(p->PCNT_UNIT);
    p->vel = vel;

    return vel;
}

/**
 * @brief motor_pos [degree]の更新
 *
 * @param p encoder pointer
 * @return motor_pos [degree]
 */
float update_pos(encoder *p)
{
    pcnt_get_counter_value(p->PCNT_UNIT, &p->now_count);
    float pos = calc_degree(p->now_count, p->enc_resolution);

    // update
    p->pos = pos;

    return pos;
}
