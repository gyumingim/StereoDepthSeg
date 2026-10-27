/* imu_icm42688.c — OPTIONAL local IMU (TDK ICM-42688-P over SPI); compiled only with FC_IMU_SOURCE = FC_IMU_LOCAL_ICM42688. Compiled with ESP-IDF v5.5.6 in the local-IMU build (fcarm 2026-10-27); NOT HARDWARE-TESTED.
 * Register values from the ICM-42688-P datasheet (rev 1.x): bank 0 only. Gyro 2000 dps, accel 16 g, ODR FC_IMU_ODR_HZ,
 * UI data-ready on INT1 (push-pull, active high, pulsed). Reads 12 data bytes + temperature with one polling transaction
 * (for 14-byte transfers polling is faster than queued DMA transactions on the ESP32). */
#include "fc_config.h"
#if FC_IMU_SOURCE == FC_IMU_LOCAL_ICM42688 /* optional module (default: phone IMU) */
#include <math.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "fc_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu.h"

#define REG_DEVICE_CONFIG 0x11
#define REG_INT_CONFIG 0x14
#define REG_TEMP_DATA1 0x1D
#define REG_INT_STATUS 0x2D
#define REG_PWR_MGMT0 0x4E
#define REG_GYRO_CONFIG0 0x4F
#define REG_ACCEL_CONFIG0 0x50
#define REG_INT_CONFIG1 0x64
#define REG_INT_SOURCE0 0x65
#define REG_WHO_AM_I 0x75
#define REG_BANK_SEL 0x76
#define WHO_AM_I_42688 0x47

#define G0 9.80665f
#define GYRO_SCALE (2000.0f / 32768.0f * 0.0174532925f)
#define ACC_SCALE (16.0f / 32768.0f * G0)

static spi_device_handle_t s_dev;
static TaskHandle_t s_notify;
static const float k_rot[9] = FC_IMU_ROT;
static uint8_t s_last_raw[12];
static int s_same_count;

static esp_err_t wr(uint8_t reg, uint8_t val) {
  spi_transaction_t t = {.flags = SPI_TRANS_USE_TXDATA, .length = 16, .tx_data = {reg & 0x7F, val}};
  return spi_device_polling_transmit(s_dev, &t);
}
static esp_err_t rd(uint8_t reg, uint8_t *buf, int n) {
  uint8_t tx[16] = {0}, rx[16] = {0};
  if (n > 15) return ESP_ERR_INVALID_ARG;
  tx[0] = reg | 0x80;
  spi_transaction_t t = {.length = (size_t)(8 * (n + 1)), .tx_buffer = tx, .rx_buffer = rx};
  esp_err_t e = spi_device_polling_transmit(s_dev, &t);
  memcpy(buf, rx + 1, (size_t)n);
  return e;
}

static void IRAM_ATTR drdy_isr(void *arg) {
  (void)arg;
  BaseType_t hp = pdFALSE;
  if (s_notify) vTaskNotifyGiveFromISR(s_notify, &hp);
  portYIELD_FROM_ISR(hp);
}

static uint8_t odr_code(int hz) {
  switch (hz) {
    case 8000: return 0x03;
    case 4000: return 0x04;
    case 2000: return 0x05;
    case 1000: return 0x06;
    case 500: return 0x0F;
    default: return 0x06;
  }
}

esp_err_t imu_init(void *drdy_task) {
  spi_bus_config_t bus = {.mosi_io_num = FC_PIN_IMU_MOSI, .miso_io_num = FC_PIN_IMU_MISO, .sclk_io_num = FC_PIN_IMU_SCLK,
                          .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 64};
  ESP_ERROR_CHECK(spi_bus_initialize(FC_IMU_SPI_HOST, &bus, SPI_DMA_CH_AUTO));
  spi_device_interface_config_t dc = {.clock_speed_hz = FC_IMU_SPI_HZ, .mode = 3, .spics_io_num = FC_PIN_IMU_CS,
                                      .queue_size = 2};
  ESP_ERROR_CHECK(spi_bus_add_device(FC_IMU_SPI_HOST, &dc, &s_dev));
  /* the device needs 1 MHz for the first accesses only if in I2C mode; SPI full speed is fine after power-up */
  wr(REG_BANK_SEL, 0);
  wr(REG_DEVICE_CONFIG, 0x01); /* soft reset */
  vTaskDelay(pdMS_TO_TICKS(2));
  uint8_t who = 0;
  rd(REG_WHO_AM_I, &who, 1);
  if (who != WHO_AM_I_42688) return ESP_ERR_NOT_FOUND;
  wr(REG_PWR_MGMT0, 0x0F);                        /* gyro + accel low-noise mode */
  vTaskDelay(pdMS_TO_TICKS(1));
  wr(REG_GYRO_CONFIG0, (0 << 5) | odr_code(FC_IMU_ODR_HZ));  /* 2000 dps */
  wr(REG_ACCEL_CONFIG0, (0 << 5) | odr_code(FC_IMU_ODR_HZ)); /* 16 g */
  wr(REG_INT_CONFIG, 0x03);                       /* INT1 push-pull, active high, pulsed */
  wr(REG_INT_CONFIG1, 0x00);                      /* INT_ASYNC_RESET = 0 (datasheet: required) */
  wr(REG_INT_SOURCE0, 0x08);                      /* UI data ready -> INT1 */
  vTaskDelay(pdMS_TO_TICKS(50));                  /* gyro start-up */
  s_notify = (TaskHandle_t)drdy_task;
  if (s_notify) {
    gpio_config_t io = {.pin_bit_mask = 1ULL << FC_PIN_IMU_INT1, .mode = GPIO_MODE_INPUT, .intr_type = GPIO_INTR_POSEDGE};
    gpio_config(&io);
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(FC_PIN_IMU_INT1, drdy_isr, NULL);
  }
  return ESP_OK;
}

bool imu_read(imu_sample_t *s) {
  uint8_t b[14];
  if (rd(REG_TEMP_DATA1, b, 14) != ESP_OK) return false;
  s->t_us = esp_timer_get_time();
  const uint8_t *d = b + 2;
  bool all0 = true, allf = true;
  for (int i = 0; i < 12; i++) { all0 &= d[i] == 0; allf &= d[i] == 0xFF; }
  if (all0 || allf) return false; /* bus dead / chip in reset */
  /* a real sensor never returns 12 identical bytes many times in a row (noise): stuck-data detector */
  if (memcmp(d, s_last_raw, 12) == 0) { if (++s_same_count > 20) return false; }
  else { s_same_count = 0; memcpy(s_last_raw, d, 12); }
  float a[3], g[3];
  for (int i = 0; i < 3; i++) {
    int16_t ra = (int16_t)((d[2 * i] << 8) | d[2 * i + 1]);
    int16_t rg = (int16_t)((d[6 + 2 * i] << 8) | d[6 + 2 * i + 1]);
    /* the accelerometer measures specific force in the chip frame: at rest, chip z up (out of the package) = +1 g */
    a[i] = (float)ra * ACC_SCALE;
    g[i] = (float)rg * GYRO_SCALE;
  }
  int16_t rt = (int16_t)((b[0] << 8) | b[1]);
  s->temp_c = (float)rt / 132.48f + 25.0f;
  /* chip frame -> body FRD: pure rotation FC_IMU_ROT (no sign tricks); at rest level acc_body = (0, 0, -g) */
  for (int r = 0; r < 3; r++) {
    s->acc[r] = k_rot[3 * r] * a[0] + k_rot[3 * r + 1] * a[1] + k_rot[3 * r + 2] * a[2];
    s->gyro[r] = k_rot[3 * r] * g[0] + k_rot[3 * r + 1] * g[1] + k_rot[3 * r + 2] * g[2];
  }
  /* NOTE (assumption to verify on the bench): the default FC_IMU_ROT = diag(1,-1,-1) assumes chip x forward, chip z
     up (component side up). Check: level at rest -> acc = (0, 0, -9.8); nose down -> pitch negative; roll right -> roll
     positive. Fix FC_IMU_ROT until those three static checks pass BEFORE any flight. */
  s->acc_ok = s->gyro_ok = true;
  for (int i = 0; i < 3; i++) { /* saturated: not trustworthy */
    if (!(fabsf(s->acc[i]) <= 15.9f * G0)) s->acc_ok = false;
    if (!(fabsf(s->gyro[i]) <= 34.8f)) s->gyro_ok = false;
  }
  return true;
}
#endif /* FC_IMU_SOURCE == FC_IMU_LOCAL_ICM42688 */
