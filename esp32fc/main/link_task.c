/* link_task.c — core 0: USB CDC from the phone (TinyUSB device; the phone is the USB host via OTG) and ESP-NOW to the
 * PC-side ESP32. Parses VIOP frames, forwards safety inputs to the control task (queue), publishes the newest pose,
 * sends safety events + 20 Hz status to the PC. Compiled with ESP-IDF v5.5.6 (fcarm 2026-10-27, 0 warnings); NOT HARDWARE-TESTED. */
#include <math.h>
#include <string.h>

#include "board.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "fc_config.h"
#include "fc_shared.h"
#include "fc_core.h"
#include "fs_viop.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"

typedef struct {
  uint8_t src[6];
  uint8_t len;
  uint8_t data[VIOP_MAX_FRAME];
} espnow_rx_t;

static QueueHandle_t s_rxq;
static viop_stream_t s_usb_stream; /* touched only from the TinyUSB task (rx callback) */
static const uint8_t k_pc_mac[6] = FC_PC_PEER_MAC;
static uint16_t s_tx_seq_ev, s_tx_seq_st;
static uint32_t s_pose_seq_local;
#if FC_IMU_SOURCE == FC_IMU_PHONE /* fcarm: only the phone-IMU build uses it (IDF -Wunused-variable otherwise) */
static uint32_t s_imu_drops; /* IMU queue full (control task not keeping up) */
#endif

/* ---------------- phone (USB CDC) ---------------- */

static void handle_phone_frame(const uint8_t *f, int n) {
  (void)n;
  uint32_t now = fc_ms();
  uint8_t type = f[3];
  uint16_t seq = (uint16_t)(f[6] | (f[7] << 8));
  if (type == VIOP_T_POSE && f[4] == sizeof(viop_pose_t)) {
    viop_pose_t p;
    memcpy(&p, f + VIOP_HDR, sizeof(p));
    fc_msg_t m = {.type = FC_MSG_POSE, .t_ms = now};
    fs_pose_info_from_viop(&p, seq, &m.u.pose);
    fc_post(&m);
    /* local NED pose for the position / control loops */
    float pl[3], vl[3], R[9], wb[3];
    viop_pose_to_local(&p, FC_PHONE_MOUNT, pl, vl, R, wb);
    fc_pose_local_t lp = {.seq = ++s_pose_seq_local, .t_rx_us = esp_timer_get_time(), .age_us = p.send_delay_us};
    memcpy(lp.p, pl, sizeof(pl));
    memcpy(lp.v, vl, sizeof(vl));
    lp.yaw = atan2f(R[3], R[0]);
    const fs_pose_info_t *pi = &m.u.pose;
    lp.good = (pi->flags & VIOP_F_VALID) && (pi->flags & VIOP_F_TRACKING) && pi->quality > 0 && pi->pos_std_max_m <= 0.15f;
    shared_put_pose(&lp);
    if (g_pos_task) xTaskNotifyGive(g_pos_task);
  } else if (type == VIOP_T_HEARTBEAT && f[4] == sizeof(viop_heartbeat_t)) {
    viop_heartbeat_t hb;
    memcpy(&hb, f + VIOP_HDR, sizeof(hb));
    fc_msg_t m = {.type = FC_MSG_HB, .t_ms = now};
    m.u.hb.seq = seq;
    m.u.hb.vio_state = hb.state;
    m.u.hb.app_flags = hb.flags;
    fc_post(&m);
  }
  else if (type == VIOP_T_IMU) {
#if FC_IMU_SOURCE == FC_IMU_PHONE
    viop_imu_dec_t d[VIOP_IMU_MAX_N];
    int ns = viop_imu_decode(f + VIOP_HDR, f[4], d);
    int64_t rx = esp_timer_get_time();
    for (int i = 0; i < ns; i++) {
      fc_imu_msg_t m = {.t_phone_us = d[i].t_us, .rx_us = rx, .seq = d[i].seq};
      fc_rot3(FC_PHONE_MOUNT, d[i].gyro, m.gyro); /* phone IMU axes -> drone body FRD */
      fc_rot3(FC_PHONE_MOUNT, d[i].acc, m.acc);
      if (xQueueSend(g_imu_queue, &m, 0) != pdTRUE) s_imu_drops++;
    }
    if (ns > 0 && g_ctrl_task) xTaskNotifyGive(g_ctrl_task); /* inner loop runs on arrival */
#endif
  }
  /* VIOP_T_TIME_RESP (clock offset for latency statistics): TODO, not needed by the safety logic */
}

static void usb_rx_cb(int itf, cdcacm_event_t *event) {
  (void)event;
  uint8_t buf[128];
  size_t got = 0;
  while (tinyusb_cdcacm_read(itf, buf, sizeof(buf), &got) == ESP_OK && got > 0) {
    viop_stream_push(&s_usb_stream, buf, (int)got);
    int fs_n;
    while ((fs_n = viop_stream_next(&s_usb_stream)) > 0) handle_phone_frame(s_usb_stream.buf, fs_n);
  }
}

static void usb_line_cb(int itf, cdcacm_event_t *event) {
  (void)itf;
  /* DTR 1 -> 0 after it had been 1 = the phone app closed the port (faster than the heartbeat timeout). If the app never
     sets DTR nothing is reported and the heartbeat timeout covers it. A pulled cable may give no callback at all. */
  static bool s_dtr_seen;
  bool dtr = event->line_state_changed_data.dtr;
  if (dtr) s_dtr_seen = true;
  else if (!s_dtr_seen) return;
  fc_msg_t m = {.type = FC_MSG_PHONE_CONN, .t_ms = fc_ms()};
  m.u.connected = dtr;
  fc_post(&m);
}

/* ---------------- PC (ESP-NOW) ---------------- */

static void espnow_rx_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len <= 0 || len > VIOP_MAX_FRAME) return;
  espnow_rx_t r;
  memcpy(r.src, info->src_addr, 6);
  r.len = (uint8_t)len;
  memcpy(r.data, data, (size_t)len);
  xQueueSend(s_rxq, &r, 0); /* runs in the Wi-Fi task: never block */
}

static bool pc_mac_ok(const uint8_t *src) {
  static const uint8_t bc[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  if (memcmp(k_pc_mac, bc, 6) == 0) return true; /* not paired yet: accept any (bench only!) */
  return memcmp(src, k_pc_mac, 6) == 0;
}

static void handle_pc_frame(const espnow_rx_t *r) {
  if (viop_check(r->data, r->len) != 0 || !pc_mac_ok(r->src)) return;
  uint32_t now = fc_ms();
  fc_msg_t link = {.type = FC_MSG_LINK, .t_ms = now};
  fc_post(&link);
  uint8_t type = r->data[3];
  if (type == VIOP_T_CMD && r->data[4] == sizeof(viop_cmd_t)) {
    viop_cmd_t c;
    memcpy(&c, r->data + VIOP_HDR, sizeof(c));
    if (c.confirm != VIOP_CMD_CONFIRM(c.cmd)) return;
    fc_msg_t m = {.type = FC_MSG_CMD, .t_ms = now};
    m.u.cmd = c.cmd;
    fc_post(&m);
  } else if (type == VIOP_T_SETPOINT && r->data[4] == sizeof(viop_setpoint_t)) {
    viop_setpoint_t s;
    memcpy(&s, r->data + VIOP_HDR, sizeof(s));
    fc_setpoint_t sp = {.p = {s.x, s.y, s.z}, .yaw = s.yaw, .valid = isfinite(s.x) && isfinite(s.y) && isfinite(s.z)};
    shared_put_setpoint(&sp);
  }
}

static void send_pc(uint8_t type, uint16_t seq, const void *payload, uint8_t len) {
  uint8_t f[VIOP_MAX_FRAME];
  int n = viop_build(f, type, VIOP_SRC_DRONE, seq, payload, len);
  esp_now_send(k_pc_mac, f, (size_t)n);
}

void link_init(void) {
  s_rxq = xQueueCreate(16, sizeof(espnow_rx_t));
  /* USB device (TinyUSB CDC-ACM) — the phone is the USB host */
  const tinyusb_config_t tc = {.device_descriptor = NULL, .string_descriptor = NULL, .external_phy = false};
  ESP_ERROR_CHECK(tinyusb_driver_install(&tc));
  tinyusb_config_cdcacm_t acm = {.usb_dev = TINYUSB_USBDEV_0, .cdc_port = TINYUSB_CDC_ACM_0, .rx_unread_buf_sz = 256,
                                 .callback_rx = usb_rx_cb, .callback_rx_wanted_char = NULL,
                                 .callback_line_state_changed = usb_line_cb, .callback_line_coding_changed = NULL};
  ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm));
  /* Wi-Fi STA (no AP) + ESP-NOW, fixed channel, no power save (latency) */
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  wifi_init_config_t wc = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&wc));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());
  ESP_ERROR_CHECK(esp_wifi_set_channel(FC_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
  esp_wifi_set_ps(WIFI_PS_NONE);
  ESP_ERROR_CHECK(esp_now_init());
  ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_rx_cb));
  esp_now_set_pmk((const uint8_t *)FC_ESPNOW_PMK);
  esp_now_peer_info_t peer = {.channel = FC_ESPNOW_CHANNEL, .ifidx = WIFI_IF_STA, .encrypt = false};
  memcpy(peer.peer_addr, k_pc_mac, 6);
  esp_now_add_peer(&peer);
}

void link_task(void *arg) {
  (void)arg;
  int64_t t_status = 0;
  for (;;) {
    espnow_rx_t r;
    if (xQueueReceive(s_rxq, &r, pdMS_TO_TICKS(5)) == pdTRUE) handle_pc_frame(&r);
    /* safety events -> PC (every transition / tier change / rejection) */
    fs_event_t ev;
    bool disarmed_now = false;
    while (xQueueReceive(g_ev_queue, &ev, 0) == pdTRUE) {
      viop_fc_event_t ve;
      fs_event_to_viop(&ev, &ve);
      send_pc(VIOP_T_FC_EVENT, s_tx_seq_ev++, &ve, sizeof(ve));
      if (ev.code == FS_EV_TRANSITION && ev.to == FS_DISARMED) disarmed_now = true;
    }
    if (disarmed_now) hover_store_flush_nvs(); /* flash write only on the ground */
    int64_t t = esp_timer_get_time();
    if (t - t_status >= 1000000 / FC_STATUS_HZ) {
      t_status = t;
      fc_ctrl_state_t cs;
      fc_pose_local_t pl;
      shared_get_ctrl(&cs);
      shared_get_pose(&pl);
      viop_fc_status_t st = {0};
      st.t_ms = (uint32_t)(t / 1000);
      st.state = (uint8_t)cs.out.state;
      st.land_tier = (uint8_t)cs.out.tier;
      st.batt_mv = (uint16_t)(cs.batt_v * 1000.0f);
      st.roll_cdeg = (int16_t)(cs.roll * 5729.58f);
      st.pitch_cdeg = (int16_t)(cs.pitch * 5729.58f);
      st.yaw_cdeg = (int16_t)(cs.yaw * 5729.58f);
      st.thr_permille = (int16_t)(cs.out.thr * 1000.0f);
      memcpy(st.p_l, pl.p, sizeof(st.p_l));
      st.loop_max_us = cs.loop_max_us;
      st.overruns = (uint16_t)(cs.overruns > 65535 ? 65535 : cs.overruns);
      int64_t pa = (t - pl.t_rx_us) / 1000;
      st.pose_age_ms = (uint16_t)(pl.seq == 0 || pa > 65535 ? 65535 : pa);
      st.hb_age_ms = cs.hb_age_ms;
      st.land_reason = cs.land_reason;
      send_pc(VIOP_T_FC_STATUS, s_tx_seq_st++, &st, sizeof(st));
    }
  }
}
