/* board.c — NOT COMPILED / NOT HARDWARE-TESTED (ESP-IDF v5.2+ APIs). */
#include "board.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_attr.h"
#include "fc_config.h"
#include "fc_shared.h"
#include "nvs.h"

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_cali;
static bool s_cali_ok;

static void IRAM_ATTR kill_isr(void *arg) {
  (void)arg;
  g_kill_switch_isr = true; /* latched; the control task turns it into fs KILL, supervisor also stops motors */
}

void board_init(void) {
  adc_oneshot_unit_init_cfg_t uc = {.unit_id = ADC_UNIT_1};
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&uc, &s_adc));
  adc_oneshot_chan_cfg_t cc = {.atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT};
  ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, FC_BATT_ADC_CHANNEL, &cc));
  adc_cali_curve_fitting_config_t cal = {.unit_id = ADC_UNIT_1, .chan = FC_BATT_ADC_CHANNEL, .atten = ADC_ATTEN_DB_12,
                                         .bitwidth = ADC_BITWIDTH_DEFAULT};
  s_cali_ok = adc_cali_create_scheme_curve_fitting(&cal, &s_cali) == ESP_OK;

  gpio_config_t ks = {.pin_bit_mask = 1ULL << FC_PIN_KILL_SWITCH, .mode = GPIO_MODE_INPUT,
                      .pull_up_en = FC_KILL_SWITCH_ACTIVE_LEVEL == 0 ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
                      .pull_down_en = FC_KILL_SWITCH_ACTIVE_LEVEL == 1 ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
                      .intr_type = FC_KILL_SWITCH_ACTIVE_LEVEL == 0 ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE};
  gpio_config(&ks);
  gpio_install_isr_service(ESP_INTR_FLAG_IRAM); /* may already be installed: ESP_ERR_INVALID_STATE is fine */
  gpio_isr_handler_add(FC_PIN_KILL_SWITCH, kill_isr, NULL);

  gpio_config_t led = {.pin_bit_mask = 1ULL << FC_PIN_LED, .mode = GPIO_MODE_OUTPUT};
  gpio_config(&led);
}

float board_battery_v(void) {
  int raw = 0, mv = 0;
  if (adc_oneshot_read(s_adc, FC_BATT_ADC_CHANNEL, &raw) != ESP_OK) return -1.0f;
  if (s_cali_ok && adc_cali_raw_to_voltage(s_cali, raw, &mv) == ESP_OK) return (float)mv * 1e-3f * FC_BATT_DIVIDER;
  return (float)raw / 4095.0f * 3.1f * FC_BATT_DIVIDER; /* uncalibrated fallback */
}

bool board_kill_switch(void) { return gpio_get_level(FC_PIN_KILL_SWITCH) == FC_KILL_SWITCH_ACTIVE_LEVEL; }
void board_led(bool on) { gpio_set_level(FC_PIN_LED, on ? 1 : 0); }

/* ---------------- hover record persistence ---------------- */
#define HOVER_MAGIC 0x484F5652u /* "HOVR" */
RTC_NOINIT_ATTR static fs_hover_rec_t s_rtc_rec;
RTC_NOINIT_ATTR static uint32_t s_rtc_magic, s_rtc_check;

static uint32_t rec_check(const fs_hover_rec_t *r) {
  const uint8_t *p = (const uint8_t *)r;
  uint32_t h = 2166136261u;
  for (size_t i = 0; i < sizeof(*r); i++) h = (h ^ p[i]) * 16777619u;
  return h;
}

void hover_store_load(fs_hover_rec_t *prev) {
  memset(prev, 0, sizeof(*prev));
  if (s_rtc_magic == HOVER_MAGIC && s_rtc_check == rec_check(&s_rtc_rec) && s_rtc_rec.valid) { *prev = s_rtc_rec; return; }
  nvs_handle_t h;
  if (nvs_open("fc", NVS_READONLY, &h) == ESP_OK) {
    size_t n = sizeof(*prev);
    if (nvs_get_blob(h, "hover", prev, &n) != ESP_OK || n != sizeof(*prev)) memset(prev, 0, sizeof(*prev));
    nvs_close(h);
  }
}

void hover_store_save_rtc(const fs_hover_rec_t *r) {
  s_rtc_rec = *r;
  s_rtc_check = rec_check(r);
  s_rtc_magic = HOVER_MAGIC;
}

void hover_store_flush_nvs(void) {
  static uint32_t s_saved_check;
  if (s_rtc_magic != HOVER_MAGIC || s_rtc_check != rec_check(&s_rtc_rec) || !s_rtc_rec.valid) return;
  if (s_saved_check == s_rtc_check) return;
  fs_hover_rec_t r = s_rtc_rec;
  hover_store_save_nvs(&r);
  s_saved_check = s_rtc_check;
}

void hover_store_save_nvs(const fs_hover_rec_t *r) {
  nvs_handle_t h;
  if (nvs_open("fc", NVS_READWRITE, &h) != ESP_OK) return;
  nvs_set_blob(h, "hover", r, sizeof(*r));
  nvs_commit(h);
  nvs_close(h);
}
