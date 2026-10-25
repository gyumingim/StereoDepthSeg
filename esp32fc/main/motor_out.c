/* motor_out.c — DShot via the ESP32-S3 RMT peripheral (4 TX channels = 4 motors), PWM fallback via LEDC.
 * NOT COMPILED / NOT HARDWARE-TESTED (ESP-IDF not installed on the build PC). Written against ESP-IDF v5.2+ APIs. */
#include "motor_out.h"

#include <string.h>

#include "driver/gpio.h"
#include "fc_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const int k_pins[FC_N_MOTORS] = FC_PIN_MOTOR;
static SemaphoreHandle_t s_mtx;
static volatile bool s_estop;

#if FC_ESC_PROTOCOL == FC_ESC_DSHOT600 || FC_ESC_PROTOCOL == FC_ESC_DSHOT300
#include "driver/rmt_tx.h"

#define RMT_RES_HZ (40 * 1000 * 1000)
#if FC_ESC_PROTOCOL == FC_ESC_DSHOT600
#define DSHOT_BIT_TICKS 67 /* 1.67 us */
#else
#define DSHOT_BIT_TICKS 133 /* 3.33 us */
#endif
#define DSHOT_T1H (DSHOT_BIT_TICKS * 3 / 4)
#define DSHOT_T0H (DSHOT_BIT_TICKS * 3 / 8)

static rmt_channel_handle_t s_ch[FC_N_MOTORS];
static rmt_encoder_handle_t s_enc[FC_N_MOTORS];
static uint8_t s_buf[FC_N_MOTORS][2][2]; /* ping-pong frame buffers (RMT reads them asynchronously) */
static uint8_t s_pp;

static inline uint16_t dshot_frame(uint16_t value, bool telem) {
  uint16_t v = (uint16_t)((value << 1) | (telem ? 1 : 0));
  uint16_t crc = (uint16_t)((v ^ (v >> 4) ^ (v >> 8)) & 0x0F);
  return (uint16_t)((v << 4) | crc);
}

static void send_values(const uint16_t *val) {
  const rmt_transmit_config_t tx = {.loop_count = 0};
  s_pp ^= 1;
  for (int i = 0; i < FC_N_MOTORS; i++) {
    uint16_t f = dshot_frame(val[i], false);
    s_buf[i][s_pp][0] = (uint8_t)(f >> 8); /* MSB first on the wire */
    s_buf[i][s_pp][1] = (uint8_t)(f & 0xff);
    rmt_transmit(s_ch[i], s_enc[i], s_buf[i][s_pp], 2, &tx);
  }
}

esp_err_t motor_out_init(void) {
  s_mtx = xSemaphoreCreateMutex();
  for (int i = 0; i < FC_N_MOTORS; i++) {
    rmt_tx_channel_config_t cc = {
        .gpio_num = k_pins[i],
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_RES_HZ,
        .mem_block_symbols = 48,
        .trans_queue_depth = 2,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&cc, &s_ch[i]));
    rmt_bytes_encoder_config_t bc = {
        .bit0 = {.level0 = 1, .duration0 = DSHOT_T0H, .level1 = 0, .duration1 = DSHOT_BIT_TICKS - DSHOT_T0H},
        .bit1 = {.level0 = 1, .duration0 = DSHOT_T1H, .level1 = 0, .duration1 = DSHOT_BIT_TICKS - DSHOT_T1H},
        .flags.msb_first = 1,
    };
    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&bc, &s_enc[i]));
    ESP_ERROR_CHECK(rmt_enable(s_ch[i]));
  }
  motor_out_disarm();
  return ESP_OK;
}

void motor_out_write(const float *thr) {
  if (s_estop) { motor_out_disarm(); return; }
  uint16_t v[FC_N_MOTORS];
  for (int i = 0; i < FC_N_MOTORS; i++) {
    float t = thr[i];
    if (!(t >= 0.0f)) t = 0.0f; /* NaN -> 0 (fc_safety already kills on NaN; belt and braces) */
    if (t > 1.0f) t = 1.0f;
    v[i] = (uint16_t)(48 + t * (2047 - 48)); /* 48..2047 = throttle; 1..47 = commands; 0 = stop */
  }
  if (xSemaphoreTake(s_mtx, 0) != pdTRUE) return; /* supervisor owns the outputs */
  send_values(v);
  xSemaphoreGive(s_mtx);
}

void motor_out_disarm(void) {
  static const uint16_t z[FC_N_MOTORS] = {0};
  if (xSemaphoreTake(s_mtx, pdMS_TO_TICKS(1)) != pdTRUE) return;
  send_values(z);
  xSemaphoreGive(s_mtx);
}

void motor_out_emergency_stop(void) {
  s_estop = true;
  static const uint16_t z[FC_N_MOTORS] = {0};
  if (xSemaphoreTake(s_mtx, pdMS_TO_TICKS(1)) == pdTRUE) {
    send_values(z);
    xSemaphoreGive(s_mtx);
  } else { /* control task stuck while holding the outputs: take the pins away -> ESC signal loss */
    for (int i = 0; i < FC_N_MOTORS; i++) rmt_disable(s_ch[i]);
    for (int i = 0; i < FC_N_MOTORS; i++) {
      gpio_reset_pin(k_pins[i]);
      gpio_set_direction(k_pins[i], GPIO_MODE_OUTPUT);
      gpio_set_level(k_pins[i], 0);
    }
  }
}

#else /* ---------------- PWM 400 Hz via LEDC ---------------- */
#include "driver/ledc.h"

#define PWM_HZ 400
#define PWM_RES LEDC_TIMER_14_BIT
static inline uint32_t us_to_duty(float us) { return (uint32_t)(us * PWM_HZ * 16383.0f / 1e6f); }

esp_err_t motor_out_init(void) {
  s_mtx = xSemaphoreCreateMutex();
  ledc_timer_config_t tc = {.speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = PWM_RES, .timer_num = LEDC_TIMER_0,
                            .freq_hz = PWM_HZ, .clk_cfg = LEDC_AUTO_CLK};
  ESP_ERROR_CHECK(ledc_timer_config(&tc));
  for (int i = 0; i < FC_N_MOTORS; i++) {
    ledc_channel_config_t ch = {.gpio_num = k_pins[i], .speed_mode = LEDC_LOW_SPEED_MODE, .channel = (ledc_channel_t)i,
                                .timer_sel = LEDC_TIMER_0, .duty = us_to_duty(1000), .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ch));
  }
  return ESP_OK;
}
static void set_us(const float *us) {
  for (int i = 0; i < FC_N_MOTORS; i++) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i, us_to_duty(us[i]));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i);
  }
}
void motor_out_write(const float *thr) {
  if (s_estop) { motor_out_disarm(); return; }
  float us[FC_N_MOTORS];
  for (int i = 0; i < FC_N_MOTORS; i++) {
    float t = thr[i];
    if (!(t >= 0.0f)) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    us[i] = 1000.0f + 1000.0f * t;
  }
  if (xSemaphoreTake(s_mtx, 0) != pdTRUE) return;
  set_us(us);
  xSemaphoreGive(s_mtx);
}
void motor_out_disarm(void) {
  float us[FC_N_MOTORS];
  for (int i = 0; i < FC_N_MOTORS; i++) us[i] = 1000.0f;
  if (xSemaphoreTake(s_mtx, pdMS_TO_TICKS(1)) != pdTRUE) return;
  set_us(us);
  xSemaphoreGive(s_mtx);
}
void motor_out_emergency_stop(void) {
  s_estop = true;
  motor_out_disarm();
}
#endif

void motor_out_clear_estop(void) { s_estop = false; }
bool motor_out_estopped(void) { return s_estop; }
