/* imu.h — IMU driver interface (default implementation: TDK ICM-42688-P on SPI, imu_icm42688.c). */
#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
  float gyro[3];   /* rad/s, body FRD */
  float acc[3];    /* m/s^2 specific force, body FRD (at rest: 0, 0, -g) */
  float temp_c;
  int64_t t_us;
  bool acc_ok, gyro_ok; /* per-sensor plausibility (not saturated) */
} imu_sample_t;

/* drdy_task: task notified from the data-ready ISR (NULL = no interrupt, poll) */
esp_err_t imu_init(void *drdy_task);
/* read one sample (polling SPI transaction, ~15-25 us at 20 MHz, 추정). Returns false on SPI error / dead bus /
 * stuck frames; otherwise true with acc_ok / gyro_ok telling which sensor is plausible (not saturated). */
bool imu_read(imu_sample_t *s);

#endif
