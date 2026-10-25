/* viop_c_test.c — checks viop_proto.h (v2) the way the ESP32 firmware would use it, on a viop_rx.py UDP capture:
 *  1) every datagram with viop_check + pose decode + viop_pose_to_local (cam_fwd mount);
 *  2) all frames concatenated into ONE byte stream with junk between them (false syncs A5 5A with plausible headers,
 *     stray A5), fed in random-sized chunks through viop_stream_push / viop_stream_next (USB CDC / UART / ESP-NOW case):
 *     every real frame must come out, none may be swallowed.
 *   gcc -std=c99 -O2 -Wall -Wextra -I<ov_bridge dir> viop_c_test.c -o viop_c_test -lm && ./viop_c_test DIR/raw.bin */
#include <stdio.h>
#include <stdlib.h>
#include "viop_proto.h"

static uint8_t stream[1 << 24];

int main(int argc, char **argv) {
  if (argc < 2) return 2;
  FILE *f = fopen(argv[1], "rb");
  if (!f) return 1;
  int sn = 0, ok = 0, bad = 0, other = 0, poses = 0;
  float maxpos = 0;
  uint32_t rng = 12345;
  for (;;) {
    int64_t pc;
    uint16_t len;
    if (fread(&pc, 8, 1, f) != 1 || fread(&len, 2, 1, f) != 1) break;
    uint8_t d[2048];
    if (fread(d, 1, len, f) != len) break;
    if (viop_check(d, len) != 0) { bad++; continue; }
    ok++;
    if (d[3] == VIOP_T_POSE) {
      viop_pose_t p;
      memcpy(&p, d + VIOP_HDR, sizeof(p));
      float pl[3], vl[3], R[9], wb[3];
      viop_pose_to_local(&p, viop_mount_cam_fwd, pl, vl, R, wb);
      for (int i = 0; i < 3; i++) if (pl[i] * pl[i] > maxpos * maxpos) maxpos = pl[i] < 0 ? -pl[i] : pl[i];
      poses++;
    } else other++;
    /* junk before every frame: false sync with a header claiming 20 bytes, a stray A5, then the frame */
    const uint8_t junk[8] = {0xA5, 0x5A, 0x02, 0x01, 20, 0x01, 0x00, 0xA5};
    if (sn + (int)sizeof(junk) + len < (int)sizeof(stream)) {
      memcpy(stream + sn, junk, sizeof(junk)); sn += sizeof(junk);
      memcpy(stream + sn, d, len); sn += len;
    }
  }
  static viop_stream_t st;
  int got = 0, pos = 0;
  while (pos < sn) {
    rng = rng * 1103515245u + 12345u;
    int c = 1 + (int)((rng >> 16) % 64);
    if (pos + c > sn) c = sn - pos;
    viop_stream_push(&st, stream + pos, c);
    pos += c;
    while (viop_stream_next(&st) > 0) got++;
  }
  printf("datagrams: frames ok %d (pose %d, other %d) bad %d | max |pos| %.3f m | byte stream with junk: recovered %d of %d "
         "frames, rejected false starts %u, dropped bytes %u\n", ok, poses, other, bad, maxpos, got, ok, st.bad, st.dropped_bytes);
  return got == ok ? 0 : 3;
}
