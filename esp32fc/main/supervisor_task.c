/* supervisor_task.c — core 0, highest priority there. Independent of the control task:
 *  - fast software watchdog: control loop counter not advancing for FC_CTRL_STALL_MS -> cut motors directly
 *    (motor_out_emergency_stop) and post a KILL request (fc_safety latches KILLED when the loop runs again);
 *  - second path for the physical kill switch (motors stop even if the control task is stuck);
 *  - battery voltage at 50 Hz (ADC is too slow for the 1 kHz loop).
 * The ESP-IDF task watchdog (1 s, panic -> reboot, outputs then stop) is the last line behind this.
 * Compiled with ESP-IDF v5.5.6 (fcarm 2026-10-27, 0 warnings); NOT HARDWARE-TESTED. */
#include "board.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "fc_config.h"
#include "fc_shared.h"
#include "motor_out.h"

void supervisor_task(void *arg) {
  (void)arg;
  esp_task_wdt_add(NULL);
  uint32_t last_alive = g_ctrl_alive;
  int64_t t_alive = esp_timer_get_time(), t_batt = 0;
  bool stall_reported = false, switch_reported = false;
  float vf = 0;
  int batt_fail = 0;
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000 / FC_SUPERVISOR_HZ) ? pdMS_TO_TICKS(1000 / FC_SUPERVISOR_HZ) : 1);
    int64_t t = esp_timer_get_time();
    uint32_t a = g_ctrl_alive;
    if (a != last_alive) { last_alive = a; t_alive = t; stall_reported = false; }
    if (t - t_alive > (int64_t)FC_CTRL_STALL_MS * 1000) {
      motor_out_emergency_stop();
      if (!stall_reported) { g_kill_req |= FS_R_LOOP_STALL; stall_reported = true; }
    }
    bool ks = board_kill_switch() || g_kill_switch_isr;
    if (ks) {
      motor_out_emergency_stop();
      if (!switch_reported) { g_kill_req |= FS_R_SWITCH; switch_reported = true; }
    } else switch_reported = false;
    if (t - t_batt >= 20000) {
      t_batt = t;
      float v = board_battery_v();
      if (v > 1.0f && v < 60.0f) { vf = vf <= 0 ? v : vf + 0.1f * (v - vf); g_batt_v = vf; batt_fail = 0; }
      else if (++batt_fail >= 5) { g_batt_v = 0; vf = 0; } /* unknown: arming refused (no stale "good" value) */
    }
    esp_task_wdt_reset();
  }
}
