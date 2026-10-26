/* arm_replay.c — gyrocheck 2026-10-27: feed a recorded pose / health / phone-IMU stream into the UNCHANGED fc_safety.c
 * and probe, every control cycle, whether an ARM command would be accepted (on a copy of the state, so the replay itself
 * stays DISARMED). Minimal reproduction harness, not part of the firmware or test/test_safety.c.
 *
 * input (stdin, time-sorted text lines, t in ms from the recording start):
 *   I t gyro_norm_rps acc_norm_mps2 tilt_rad   -> control cycle: fs_on_link + fs_step + ARM probe
 *   P t flags quality pos_std_max_m            -> fs_on_pose (seq auto-increment, reset_counter 0, send_delay 0)
 *   H t vio_state                              -> fs_on_heartbeat (seq auto-increment, app flags 0)
 * output (stdout): one CSV row per cycle: t_ms,blocks,armable,gyro,acc_var,acc_mean,tilt_deg
 */
#include <stdio.h>
#include <string.h>
#include "fc_safety.h"

int main(void) {
  fs_config_t cfg;
  fs_config_default(&cfg);
  fs_t fs;
  fs_init(&fs, &cfg, 0);
  char line[256];
  uint16_t pseq = 0, hseq = 0;
  uint32_t last_t = 0;
  printf("t_ms,blocks,armable,gyro,acc_var,acc_mean,tilt_deg\n");
  while (fgets(line, sizeof line, stdin)) {
    char k;
    unsigned t;
    if (sscanf(line, " %c %u", &k, &t) != 2) continue;
    if (k == 'P') {
      unsigned fl, q;
      float sd;
      if (sscanf(line, " %c %u %u %u %f", &k, &t, &fl, &q, &sd) != 5) continue;
      fs_pose_info_t p;
      memset(&p, 0, sizeof p);
      p.seq = ++pseq;
      p.flags = (uint16_t)fl;
      p.quality = (uint8_t)q;
      p.pos_std_max_m = sd;
      fs_on_pose(&fs, t, &p);
    } else if (k == 'H') {
      unsigned st;
      if (sscanf(line, " %c %u %u", &k, &t, &st) != 3) continue;
      fs_on_heartbeat(&fs, t, ++hseq, (uint8_t)st, 0);
    } else if (k == 'I') {
      float g, a, tilt;
      if (sscanf(line, " %c %u %f %f %f", &k, &t, &g, &a, &tilt) != 5) continue;
      fs_on_link(&fs, t);
      fs_input_t in;
      memset(&in, 0, sizeof in);
      in.now_ms = t;
      in.dt_s = last_t ? (float)(t - last_t) * 1e-3f : 0.01f;
      last_t = t;
      in.imu_ok = true;
      in.acc_ok = true;
      in.roll_rad = tilt;
      in.pitch_rad = 0.0f;
      in.gyro_norm_rps = g;
      in.acc_norm = a;
      in.thr_cmd = 0.0f;
      in.batt_valid = true;
      in.batt_v = 16.0f;
      in.kill_switch = false;
      fs_output_t out;
      fs_step(&fs, &in, &out);
      fs_t probe = fs; /* copy: ARM on the copy only */
      bool ok = fs_command(&probe, t, FS_CMD_ARM);
      uint32_t b = 0;
      fs_event_t e;
      while (fs_pop_event(&probe, &e))
        if (e.code == FS_EV_ARM_REJECT) b = e.reason;
      printf("%u,%u,%d,%.4f,%.4f,%.4f,%.2f\n", t, (unsigned)b, ok ? 1 : 0, g, fs.acc_var, fs.acc_mean, tilt * 57.29578f);
    }
  }
  return 0;
}
