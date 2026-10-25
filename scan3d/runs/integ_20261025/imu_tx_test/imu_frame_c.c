/* integ-20261025: reads "t_us flags seq sample_seq g0 g1 g2 a0 a1 a2 hex" lines (ImuFrameTest output, float bits),
 * builds the same VIOP_T_IMU frame with viop_proto.h (viop_imu_t + viop_build), decodes it with viop_imu_decode and the
 * stream parser, and compares with the Java bytes. Rounding = lround (Java Math.round differs only at exact .5 ties of
 * negative values: counted separately). */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "viop_proto.h"

static int16_t sat(double v) { long r = lround(v); return (int16_t)(r > 32767 ? 32767 : r < -32768 ? -32768 : r); }

int main(void) {
  char hex[200];
  unsigned long long t;
  unsigned flags, seq, sseq;
  unsigned gb[3], ab[3];
  int n = 0, same = 0, decode_ok = 0, stream_ok = 0;
  viop_stream_t st;
  memset(&st, 0, sizeof(st));
  while (scanf("%llu %u %u %u %u %u %u %u %u %u %199s", &t, &flags, &seq, &sseq, &gb[0], &gb[1], &gb[2], &ab[0], &ab[1],
               &ab[2], hex) == 11) {
    float g[3], a[3];
    memcpy(g, gb, 12);
    memcpy(a, ab, 12);
    viop_imu_t m;
    memset(&m, 0, sizeof(m));
    m.t0_us = t;
    m.n = 1;
    m.flags = (uint8_t)flags;
    m.sample_seq0 = (uint16_t)sseq;
    for (int k = 0; k < 3; k++) {
      m.s[0].gyro[k] = sat((double)g[k] / (double)1e-3);
      m.s[0].acc[k] = sat((double)a[k] / (double)5e-3);
    }
    uint8_t f[VIOP_MAX_FRAME];
    int fs = viop_build(f, VIOP_T_IMU, VIOP_SRC_PHONE, (uint16_t)seq, &m, viop_imu_payload_len(1));
    char h2[200];
    for (int i = 0; i < fs; i++) sprintf(h2 + 2 * i, "%02x", f[i]);
    n++;
    if (strcmp(h2, hex) == 0) same++;
    else if (n - same <= 5) printf("DIFF line %d\n java %s\n c    %s\n", n, hex, h2);
    /* decode the JAVA bytes */
    uint8_t jf[64];
    int jn = (int)strlen(hex) / 2;
    for (int i = 0; i < jn; i++) sscanf(hex + 2 * i, "%2hhx", &jf[i]);
    viop_imu_dec_t d[VIOP_IMU_MAX_N];
    if (viop_check(jf, jn) == 0 && jf[3] == VIOP_T_IMU && viop_imu_decode(jf + VIOP_HDR, jf[4], d) == 1 &&
        d[0].t_us == (int64_t)t && d[0].seq == (uint16_t)sseq) {
      int okv = 1;
      for (int k = 0; k < 3; k++) {
        if (fabs(g[k]) < 32.7 && fabs(d[0].gyro[k] - g[k]) > 0.51e-3) okv = 0;
        if (fabs(a[k]) < 163.8 && fabs(d[0].acc[k] - a[k]) > 2.6e-3) okv = 0;
      }
      decode_ok += okv;
    }
    if (viop_stream_push(&st, jf, jn) == jn) {
      int fsz = viop_stream_next(&st);
      if (fsz == jn && memcmp(st.buf, jf, (size_t)jn) == 0) stream_ok++;
    }
  }
  printf("frames %d | java == C bytes %d | decoded within 1/2 LSB %d | stream parser %d\n", n, same, decode_ok, stream_ok);
  return (n > 0 && same == n && decode_ok == n && stream_ok == n) ? 0 : 1;
}
