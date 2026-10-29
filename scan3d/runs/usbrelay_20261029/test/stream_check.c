/* usbrelay 2026-10-29: feed the relay's USB OUT byte stream (RelayTest usb_out_*.bin) to the FC's own parser
 * (esp32fc viop_proto.h viop_stream_push / viop_stream_next, 128-byte reads as in link_task.c usb_rx_cb) and decode the
 * IMU frames with viop_imu_decode; check the forwarded 0x23 frames (reset_fwd.hex) the way ov_bridge mav_out.h does.
 * usage: stream_check <file.bin>... [--reset reset_fwd.hex]   exit 0 = all ok */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "viop_proto.h"

static int check_bin(const char *path) {
  FILE *f = fopen(path, "rb");
  if (!f) { printf("%s: cannot open\n", path); return 1; }
  static viop_stream_t st;
  memset(&st, 0, sizeof st);
  unsigned long n_type[256] = {0}, n_imu_samples = 0, bytes = 0;
  uint8_t buf[128];
  size_t got;
  int bad_imu = 0;
  while ((got = fread(buf, 1, sizeof buf, f)) > 0) {
    bytes += got;
    viop_stream_push(&st, buf, (int)got);
    int fs;
    while ((fs = viop_stream_next(&st)) > 0) {
      uint8_t t = st.buf[3];
      n_type[t]++;
      if (t == VIOP_T_IMU) {
        viop_imu_dec_t d[VIOP_IMU_MAX_N];
        int ns = viop_imu_decode(st.buf + VIOP_HDR, st.buf[4], d);
        if (ns != 1) bad_imu++;
        else n_imu_samples++;
      }
    }
  }
  fclose(f);
  int fail = st.bad != 0 || st.dropped_bytes != 0 || bad_imu != 0 || st.n != 0;
  printf("%s: %lu bytes, FC parser ok %u bad %u dropped %u left %d | POSE %lu HB %lu IMU %lu (samples %lu, bad %d) -> %s\n",
         path, bytes, st.ok, st.bad, st.dropped_bytes, st.n, n_type[VIOP_T_POSE], n_type[VIOP_T_HEARTBEAT], n_type[VIOP_T_IMU],
         n_imu_samples, bad_imu, fail ? "FAIL" : "ok");
  return fail;
}

static int check_reset(const char *path) {
  FILE *f = fopen(path, "r");
  if (!f) { printf("%s: cannot open\n", path); return 1; }
  char line[512];
  int n = 0, ok = 0;
  while (fgets(line, sizeof line, f)) {
    uint8_t rx[200];
    int len = 0;
    for (char *p = line; p[0] && p[1] && p[0] != '\n'; p += 2) { unsigned v; sscanf(p, "%2x", &v); rx[len++] = (uint8_t)v; }
    n++;
    /* = ov_bridge mav_out.h loop(): viop_check, type 0x23, len sizeof(viop_cmd_t), cmd + confirm */
    if (viop_check(rx, len) == 0 && rx[3] == VIOP_T_VIO_RESET_REQ && rx[4] == sizeof(viop_cmd_t)) {
      viop_cmd_t c;
      memcpy(&c, rx + VIOP_HDR, sizeof c);
      if (c.cmd == VIOP_CMD_VIO_RESET && c.confirm == VIOP_CMD_CONFIRM(VIOP_CMD_VIO_RESET)) ok++;
    }
  }
  fclose(f);
  printf("%s: %d frames, %d accepted by the ov_bridge reset check -> %s\n", path, n, ok, n == 2 && ok == 2 ? "ok" : "FAIL");
  return !(n == 2 && ok == 2);
}

int main(int argc, char **argv) {
  int fail = 0;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--reset") && i + 1 < argc) fail |= check_reset(argv[++i]);
    else fail |= check_bin(argv[i]);
  }
  printf("%s\n", fail ? "STREAM CHECK FAILED" : "STREAM CHECK ALL OK");
  return fail;
}
