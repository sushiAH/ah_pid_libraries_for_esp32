#include <ah_potentio_esp32.h>

//                                       35,34,36
adc1_channel_t POTENTIO_PINNUM = {ADC1_CHANNEL_7, ADC1_CHANNEL_6,
                                  ADC1_CHANNEL_0};

void init_potentio(int motor_id) {
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(POTENTIO_PINNUM[motor_id], ADC_ATTEN_DB_11);
}

int read_potentio(int motor_id) {
  long sum = 0;
  int min_val = 4096;
  int max_val = 0;
  int potentio_value = 0;

  for (int i = 0; i < 64; i++) {
    potentio_value = adc1_get_raw(POTENTIO_PINNUM[motor_id]);
    sum += potentio_value;

    if (potentio_value < min_val) {
      min_val = potentio_value;
    }

    if (potentio_value > max_val) {
      max_val = potentio_value;
    }
  }

  sum -= min_val;
  sum -= max_val;

  int filtered_potentio_value = sum / (64 - 2);

  return filtered_potentio_value;
}
