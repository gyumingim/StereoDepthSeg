/* fcarm_replay.c — fcarm 2026-10-27: replay one recorded pose / heartbeat / phone-IMU event stream through fc_safety.c
 * (this branch, new arm options) for MANY arm configurations, probing every control cycle whether ARM would be accepted
 * (on a copy of the state, the replay itself stays DISARMED). Same event mapping as gyrocheck arm_replay.c.
 *
 * usage: fcarm_replay <events.txt> <configs.txt> <h0_ms> <h1_ms> <e_ms> <e3_ms>
 *   events (time-sorted lines, t ms):  I t gyro_norm acc_norm tilt_rad | P t flags quality pos_std_max_m speed_mps | H t vio_state
 *   configs: one per line: still_ms gyro_max acc_var_max speed_max speed_hold_ms pose_std_max
 *   segments: H = [h0, h1] (hand-held), E = (e, end] (end still), E3 = (e3, end]
 * output: one line per config:
 *   idx nH armH nE armE nE3 armE3 base_okE3 base_ok_new_rejE3 first_arm_after_e_ms rejbits_E3
 *   (config line 0 must be the default config = baseline for base_ok*) */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fc_safety.h"

typedef struct { char k; unsigned t; float a, b, c, d; unsigned u1, u2; } ev_t;

int main(int argc, char **argv) {
  if (argc < 7) { fprintf(stderr, "usage\n"); return 2; }
  FILE *fe = fopen(argv[1], "r"), *fc = fopen(argv[2], "r");
  if (!fe || !fc) { fprintf(stderr, "open\n"); return 2; }
  unsigned h0 = (unsigned)atoi(argv[3]), h1 = (unsigned)atoi(argv[4]), e0 = (unsigned)atoi(argv[5]), e3 = (unsigned)atoi(argv[6]);
  size_t n = 0, cap = 1 << 16;
  ev_t *ev = malloc(cap * sizeof *ev);
  char line[256];
  while (fgets(line, sizeof line, fe)) {
    ev_t e;
    memset(&e, 0, sizeof e);
    if (sscanf(line, " %c %u", &e.k, &e.t) != 2) continue;
    int ok = 0;
    if (e.k == 'I') ok = sscanf(line, " %c %u %f %f %f", &e.k, &e.t, &e.a, &e.b, &e.c) == 5;
    else if (e.k == 'P') ok = sscanf(line, " %c %u %u %u %f %f", &e.k, &e.t, &e.u1, &e.u2, &e.a, &e.b) == 6;
    else if (e.k == 'H') ok = sscanf(line, " %c %u %u", &e.k, &e.t, &e.u1) == 3;
    if (!ok) continue;
    if (n == cap) { cap *= 2; ev = realloc(ev, cap * sizeof *ev); }
    ev[n++] = e;
  }
  fclose(fe);
  /* baseline armable per I-cycle (config 0) */
  size_t nI = 0;
  for (size_t i = 0; i < n; i++) nI += ev[i].k == 'I';
  unsigned char *base = calloc(nI ? nI : 1, 1);
  int idx = 0;
  while (fgets(line, sizeof line, fc)) {
    float still, gmax, avar, smax, shold, sdmax;
    if (sscanf(line, "%f %f %f %f %f %f", &still, &gmax, &avar, &smax, &shold, &sdmax) != 6) continue;
    fs_config_t cfg;
    fs_config_default(&cfg);
    if (idx > 0) { /* config 0 = untouched defaults */
      cfg.arm_still_ms = (uint32_t)still;
      cfg.arm_gyro_max_rps = gmax;
      cfg.arm_acc_var_max = avar;
      cfg.arm_vio_speed_max = smax;
      cfg.arm_vio_speed_hold_ms = (uint32_t)shold;
      cfg.pose_std_max_m = sdmax;
    }
    fs_t fs;
    fs_init(&fs, &cfg, 0);
    uint16_t pseq = 0, hseq = 0;
    uint32_t last_t = 0;
    long nH = 0, aH = 0, nE = 0, aE = 0, nE3 = 0, aE3 = 0, bE3 = 0, brE3 = 0;
    long first = -1;
    uint32_t rej = 0;
    size_t ii = 0;
    for (size_t i = 0; i < n; i++) {
      const ev_t *e = &ev[i];
      if (e->k == 'P') {
        fs_pose_info_t p;
        memset(&p, 0, sizeof p);
        p.seq = ++pseq;
        p.flags = (uint16_t)e->u1;
        p.quality = (uint8_t)e->u2;
        p.pos_std_max_m = e->a;
        p.speed_mps = e->b;
        fs_on_pose(&fs, e->t, &p);
      } else if (e->k == 'H') {
        fs_on_heartbeat(&fs, e->t, ++hseq, (uint8_t)e->u1, 0);
      } else {
        fs_on_link(&fs, e->t);
        fs_input_t in;
        memset(&in, 0, sizeof in);
        in.now_ms = e->t;
        in.dt_s = last_t ? (float)(e->t - last_t) * 1e-3f : 0.01f;
        last_t = e->t;
        in.imu_ok = true;
        in.acc_ok = true;
        in.roll_rad = e->c;
        in.gyro_norm_rps = e->a;
        in.acc_norm = e->b;
        in.batt_valid = true;
        in.batt_v = 16.0f;
        fs_output_t out;
        fs_step(&fs, &in, &out);
        fs_t probe = fs;
        bool ok = fs_command(&probe, e->t, FS_CMD_ARM);
        uint32_t b = 0;
        fs_event_t pe;
        while (fs_pop_event(&probe, &pe))
          if (pe.code == FS_EV_ARM_REJECT) b = pe.reason;
        if (idx == 0) base[ii] = ok;
        unsigned t = e->t;
        if (t >= h0 && t <= h1) { nH++; aH += ok; }
        if (t > e0) {
          nE++; aE += ok;
          if (ok && first < 0) first = (long)(t - e0);
        }
        if (t > e3) {
          nE3++; aE3 += ok;
          if (base[ii]) { bE3++; if (!ok) { brE3++; rej |= b; } }
        }
        ii++;
      }
    }
    printf("%d %ld %ld %ld %ld %ld %ld %ld %ld %ld %u\n", idx, nH, aH, nE, aE, nE3, aE3, bE3, brE3, first, (unsigned)rej);
    idx++;
  }
  fclose(fc);
  free(base);
  free(ev);
  return 0;
}
