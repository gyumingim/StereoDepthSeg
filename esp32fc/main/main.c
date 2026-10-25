/* main.c — esp32fc entry point: init order, task layout, shared data. NOT COMPILED (ESP-IDF not installed). */
#include <string.h>

#include "board.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "fc_config.h"
#include "fc_shared.h"
#include "motor_out.h"
#include "nvs_flash.h"

QueueHandle_t g_fs_inbox, g_ev_queue, g_imu_queue;
TaskHandle_t g_ctrl_task, g_pos_task;
volatile uint32_t g_ctrl_alive;
volatile bool g_kill_switch_isr;
volatile float g_batt_v;
volatile uint32_t g_kill_req;
static volatile uint32_t s_inbox_drops;

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static fc_pose_local_t s_pose;
static fc_att_sp_t s_att_sp;
static fc_ctrl_state_t s_ctrl;
static fc_setpoint_t s_sp;

#define SHARED_COPY(dst, src)       \
  do {                              \
    portENTER_CRITICAL(&s_mux);     \
    memcpy((dst), (src), sizeof(*(dst))); \
    portEXIT_CRITICAL(&s_mux);      \
  } while (0)

void shared_put_pose(const fc_pose_local_t *p) { SHARED_COPY(&s_pose, p); }
void shared_get_pose(fc_pose_local_t *p) { SHARED_COPY(p, &s_pose); }
void shared_put_att_sp(const fc_att_sp_t *s) { SHARED_COPY(&s_att_sp, s); }
void shared_get_att_sp(fc_att_sp_t *s) { SHARED_COPY(s, &s_att_sp); }
void shared_put_ctrl(const fc_ctrl_state_t *s) { SHARED_COPY(&s_ctrl, s); }
void shared_get_ctrl(fc_ctrl_state_t *s) { SHARED_COPY(s, &s_ctrl); }
void shared_put_setpoint(const fc_setpoint_t *s) { SHARED_COPY(&s_sp, s); }
void shared_get_setpoint(fc_setpoint_t *s) { SHARED_COPY(s, &s_sp); }

void fc_post(const fc_msg_t *m) {
  if (xQueueSend(g_fs_inbox, m, 0) != pdTRUE) s_inbox_drops++;
}
uint32_t fc_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

void app_main(void) {
  /* 1) motors to a defined stop state FIRST (ESCs see DShot 0 / 1000 us from the first milliseconds) */
  ESP_ERROR_CHECK(motor_out_init());
  esp_err_t e = nvs_flash_init();
  if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) { nvs_flash_erase(); nvs_flash_init(); }
  board_init();
  esp_log_level_set("*", ESP_LOG_WARN); /* no logging from the loops anyway; keep the console quiet */

  g_fs_inbox = xQueueCreate(64, sizeof(fc_msg_t));
  g_ev_queue = xQueueCreate(64, sizeof(fs_event_t));
  g_imu_queue = xQueueCreate(32, sizeof(fc_imu_msg_t));

  /* 2) control task on core 1 (highest priority), woken by each phone IMU frame (or the optional local IMU DRDY) */
  xTaskCreatePinnedToCore(ctrl_task, "ctrl", 6144, NULL, FC_PRIO_CTRL, &g_ctrl_task, FC_CORE_CTRL);
  xTaskCreatePinnedToCore(pos_task, "pos", 4096, NULL, FC_PRIO_POS, &g_pos_task, FC_CORE_CTRL);

  /* 3) core 0: radio + USB link, supervisor */
  link_init();
  xTaskCreatePinnedToCore(link_task, "link", 6144, NULL, FC_PRIO_LINK, NULL, FC_CORE_LINK);
  xTaskCreatePinnedToCore(supervisor_task, "superv", 3072, NULL, FC_PRIO_SUPERVISOR, NULL, FC_CORE_LINK);
}
