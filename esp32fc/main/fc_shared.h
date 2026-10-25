/* fc_shared.h — data shared between the firmware tasks.
 * Rule: fc_safety (fs_t) is owned by the control task only. Other tasks send fc_msg_t to g_fs_inbox (non-blocking);
 * the control task drains it at the start of every cycle. Small shared structs are copied under a spinlock
 * (portMUX, a few hundred ns) — no mutexes, no malloc in any loop. */
#ifndef FC_SHARED_H
#define FC_SHARED_H

#include <stdbool.h>
#include <stdint.h>

#include "fc_safety.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

typedef enum {
  FC_MSG_POSE = 1,
  FC_MSG_HB,
  FC_MSG_LINK,
  FC_MSG_CMD,
  FC_MSG_PHONE_CONN,
  FC_MSG_KILL_REQ,
  FC_MSG_LAND_REQ,
} fc_msg_type_t;

typedef struct {
  uint8_t type;
  uint32_t t_ms;
  union {
    fs_pose_info_t pose;
    struct { uint16_t seq; uint8_t vio_state, app_flags; } hb;
    uint8_t cmd;
    bool connected;
    uint32_t reason;
  } u;
} fc_msg_t;

/* newest phone pose converted to the local NED frame (link task -> position / control task) */
typedef struct {
  uint32_t seq;          /* +1 per update (0 = never) */
  int64_t t_rx_us;       /* esp_timer time at reception */
  uint32_t age_us;       /* phone-side age at send (send_delay_us) */
  float p[3], v[3];      /* NED, m, m/s */
  float yaw;             /* rad */
  bool good;             /* passes the same checks as fs_on_pose */
} fc_pose_local_t;

/* position task -> control task */
typedef struct {
  float roll, pitch, yaw, thr;
  uint32_t t_ms;
  bool valid;
} fc_att_sp_t;

/* control task -> other tasks (telemetry, position loop) */
typedef struct {
  fs_output_t out;
  float roll, pitch, yaw;
  float vz_imu;
  bool vz_imu_valid;
  uint16_t loop_max_us;
  uint32_t overruns;
  float batt_v;
  uint32_t land_reason;   /* fs land_reason (EMERG_LAND) or kill_reason (KILLED) */
  uint16_t hb_age_ms;
} fc_ctrl_state_t;

/* PC setpoint (link task -> position task) */
typedef struct {
  float p[3];
  float yaw;
  bool valid;
} fc_setpoint_t;

extern QueueHandle_t g_fs_inbox;  /* fc_msg_t, depth 64 */
extern QueueHandle_t g_ev_queue;  /* fs_event_t, depth 64 */
extern TaskHandle_t g_ctrl_task, g_pos_task;
extern volatile uint32_t g_ctrl_alive;      /* +1 per control cycle (supervisor watches it) */
extern volatile bool g_kill_switch_isr;     /* set by the kill-switch GPIO ISR */
extern volatile float g_batt_v;             /* battery volts (supervisor, 50 Hz), 0 = unknown */

void shared_put_pose(const fc_pose_local_t *p);
void shared_get_pose(fc_pose_local_t *p);
void shared_put_att_sp(const fc_att_sp_t *s);
void shared_get_att_sp(fc_att_sp_t *s);
void shared_put_ctrl(const fc_ctrl_state_t *s);
void shared_get_ctrl(fc_ctrl_state_t *s);
void shared_put_setpoint(const fc_setpoint_t *s);
void shared_get_setpoint(fc_setpoint_t *s);

/* non-blocking post to the control task (drops + counts when full) */
void fc_post(const fc_msg_t *m);
uint32_t fc_ms(void);

/* tasks */
void ctrl_task(void *arg);
void pos_task(void *arg);
void link_task(void *arg);
void supervisor_task(void *arg);
void link_init(void);

#endif /* FC_SHARED_H */
