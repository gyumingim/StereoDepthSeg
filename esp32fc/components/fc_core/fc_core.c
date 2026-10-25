/* fc_core.c — see fc_core.h */
#include "fc_core.h"

#include <math.h>
#include <string.h>

static inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static inline float wrap_pi(float a) {
  while (a > 3.14159265f) a -= 6.28318531f;
  while (a < -3.14159265f) a += 6.28318531f;
  return a;
}

/* ---------------- attitude estimator ---------------- */

static void quat_norm(float q[4]) {
  float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (n > 1e-9f) for (int i = 0; i < 4; i++) q[i] /= n;
  else { q[0] = 1; q[1] = q[2] = q[3] = 0; }
}

void att_est_init(att_est_t *e, float kp, float ki, float k_yaw) {
  memset(e, 0, sizeof(*e));
  e->q[0] = 1;
  e->kp = kp;
  e->ki = ki;
  e->k_yaw = k_yaw;
}

void att_est_rot(const att_est_t *e, float R[9]) {
  float w = e->q[0], x = e->q[1], y = e->q[2], z = e->q[3];
  R[0] = 1 - 2 * (y * y + z * z); R[1] = 2 * (x * y - w * z);     R[2] = 2 * (x * z + w * y);
  R[3] = 2 * (x * y + w * z);     R[4] = 1 - 2 * (x * x + z * z); R[5] = 2 * (y * z - w * x);
  R[6] = 2 * (x * z - w * y);     R[7] = 2 * (y * z + w * x);     R[8] = 1 - 2 * (x * x + y * y);
}

void att_est_euler(const att_est_t *e, float *roll, float *pitch, float *yaw) {
  float R[9];
  att_est_rot(e, R);
  *roll = atan2f(R[7], R[8]);
  *pitch = -asinf(clampf(R[6], -1.0f, 1.0f));
  *yaw = atan2f(R[3], R[0]);
}

void att_est_update(att_est_t *e, const float gyro[3], const float acc[3], float dt) {
  float an = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] + acc[2] * acc[2]);
  if (!e->init && an > 1.0f) { /* initial roll / pitch from the accelerometer */
    float roll = atan2f(-acc[1], -acc[2]);
    float pitch = atan2f(acc[0], sqrtf(acc[1] * acc[1] + acc[2] * acc[2]));
    float cr = cosf(roll / 2), sr = sinf(roll / 2), cp = cosf(pitch / 2), sp = sinf(pitch / 2);
    e->q[0] = cr * cp; e->q[1] = sr * cp; e->q[2] = cr * sp; e->q[3] = -sr * sp;
    quat_norm(e->q);
    e->init = true;
  }
  float w[3] = {gyro[0] - e->bias[0], gyro[1] - e->bias[1], gyro[2] - e->bias[2]};
  /* accel correction only near 1 g (not during hard manoeuvres / impacts) */
  if (an > 0.5f * FC_G && an < 1.5f * FC_G) {
    float m[3] = {-acc[0] / an, -acc[1] / an, -acc[2] / an}; /* measured "down" in body */
    float R[9];
    att_est_rot(e, R);
    float v[3] = {R[6], R[7], R[8]}; /* estimated "down" in body = R^T (0,0,1) */
    float err[3] = {m[1] * v[2] - m[2] * v[1], m[2] * v[0] - m[0] * v[2], m[0] * v[1] - m[1] * v[0]};
    for (int i = 0; i < 3; i++) {
      w[i] += e->kp * err[i];
      e->bias[i] -= e->ki * err[i] * dt;
    }
  }
  float q0 = e->q[0], q1 = e->q[1], q2 = e->q[2], q3 = e->q[3];
  float h = 0.5f * dt;
  e->q[0] += h * (-q1 * w[0] - q2 * w[1] - q3 * w[2]);
  e->q[1] += h * (q0 * w[0] + q2 * w[2] - q3 * w[1]);
  e->q[2] += h * (q0 * w[1] - q1 * w[2] + q3 * w[0]);
  e->q[3] += h * (q0 * w[2] + q1 * w[1] - q2 * w[0]);
  quat_norm(e->q);
}

void att_est_fuse_yaw(att_est_t *e, float yaw_meas, float dt) {
  float r, p, y;
  att_est_euler(e, &r, &p, &y);
  float a = e->k_yaw * wrap_pi(yaw_meas - y) * dt; /* rotate about local z (left multiply) */
  float c = cosf(a / 2), s = sinf(a / 2);
  float q0 = e->q[0], q1 = e->q[1], q2 = e->q[2], q3 = e->q[3];
  e->q[0] = c * q0 - s * q3;
  e->q[1] = c * q1 - s * q2;
  e->q[2] = c * q2 + s * q1;
  e->q[3] = c * q3 + s * q0;
  quat_norm(e->q);
}

/* ---------------- vertical speed estimator ---------------- */

void vz_est_init(vz_est_t *v, float k_v, float k_b, float leak_tau_s, float horizon_s) {
  memset(v, 0, sizeof(*v));
  v->k_v = k_v;
  v->k_b = k_b;
  v->leak_tau_s = leak_tau_s;
  v->horizon_s = horizon_s;
  v->since_vio_s = 1e9f;
}

void vz_est_predict(vz_est_t *v, const float R[9], const float acc[3], float dt) {
  float az_ned = R[6] * acc[0] + R[7] * acc[1] + R[8] * acc[2] + FC_G; /* kinematic accel, NED z */
  float a_up = -az_ned;
  v->vz += (a_up - v->bias) * dt;
  v->since_vio_s += dt;
  if (v->since_vio_s > 0.1f && v->leak_tau_s > 0) v->vz -= v->vz * dt / v->leak_tau_s;
}

void vz_est_correct(vz_est_t *v, float vz_vio_up, float dt) {
  float e = vz_vio_up - v->vz;
  v->vz += v->k_v * e * dt;
  v->bias -= v->k_b * e * dt;
  v->since_vio_s = 0;
  v->have_vio = true;
}

bool vz_est_valid(const vz_est_t *v) { return v->have_vio && v->since_vio_s < v->horizon_s; }

/* ---------------- PID ---------------- */

void pid_init(pid_t_ *p, float kp, float ki, float kd, float i_lim, float out_lim, float d_lpf_a) {
  memset(p, 0, sizeof(*p));
  p->kp = kp; p->ki = ki; p->kd = kd; p->i_lim = i_lim; p->out_lim = out_lim; p->d_lpf_a = d_lpf_a;
}
void pid_reset(pid_t_ *p) { p->i = 0; p->d = 0; p->init = false; }
float pid_step(pid_t_ *p, float sp, float meas, float dt, bool integrate) {
  float e = sp - meas;
  if (!p->init) { p->prev_meas = meas; p->init = true; }
  if (dt > 0) {
    float d_raw = -(meas - p->prev_meas) / dt; /* derivative on measurement: no kick on setpoint steps */
    p->d += p->d_lpf_a * (d_raw - p->d);
    if (integrate) p->i = clampf(p->i + p->ki * e * dt, -p->i_lim, p->i_lim);
  }
  p->prev_meas = meas;
  return clampf(p->kp * e + p->i + p->kd * p->d, -p->out_lim, p->out_lim);
}

/* ---------------- attitude controller ---------------- */

void att_ctrl_init(att_ctrl_t *c) {
  /* placeholder gains (추정, 450 class) */
  pid_init(&c->rate[0], 0.10f, 0.20f, 0.0020f, 0.2f, 0.5f, 0.3f);
  pid_init(&c->rate[1], 0.10f, 0.20f, 0.0020f, 0.2f, 0.5f, 0.3f);
  pid_init(&c->rate[2], 0.20f, 0.10f, 0.0f, 0.2f, 0.3f, 1.0f);
  c->att_kp[0] = c->att_kp[1] = 6.0f;
  c->att_kp[2] = 3.0f;
  c->rate_max[0] = c->rate_max[1] = 3.5f;
  c->rate_max[2] = 2.0f;
}
void att_ctrl_reset(att_ctrl_t *c) { for (int i = 0; i < 3; i++) pid_reset(&c->rate[i]); }
void att_ctrl_step(att_ctrl_t *c, const float sp[3], float roll, float pitch, float yaw, const float gyro[3], float dt,
                   bool integrate, float torque[3]) {
  float err[3] = {sp[0] - roll, sp[1] - pitch, wrap_pi(sp[2] - yaw)};
  for (int i = 0; i < 3; i++) {
    float rsp = clampf(c->att_kp[i] * err[i], -c->rate_max[i], c->rate_max[i]);
    torque[i] = pid_step(&c->rate[i], rsp, gyro[i], dt, integrate);
  }
}

/* ---------------- position controller ---------------- */

void pos_ctrl_init(pos_ctrl_t *c, float hover_thr) {
  c->kp_pos[0] = c->kp_pos[1] = 1.0f;
  c->kp_pos[2] = 1.2f;
  for (int i = 0; i < 2; i++) pid_init(&c->vel[i], 2.0f, 0.4f, 0.0f, 2.0f, 4.0f, 1.0f);
  pid_init(&c->vel[2], 3.0f, 1.0f, 0.0f, 3.0f, 5.0f, 1.0f);
  c->vel_max_xy = 1.0f;
  c->vel_max_up = 0.8f;
  c->vel_max_down = 0.5f;
  c->tilt_max = 0.35f;
  c->hover_thr = hover_thr;
}
void pos_ctrl_reset(pos_ctrl_t *c) { for (int i = 0; i < 3; i++) pid_reset(&c->vel[i]); }
void pos_ctrl_step(pos_ctrl_t *c, const float p[3], const float v[3], const float p_sp[3], float yaw, float vz_sp_override,
                   float dt, bool integrate, float *roll_sp, float *pitch_sp, float *thr) {
  float vsp[3];
  for (int i = 0; i < 3; i++) vsp[i] = c->kp_pos[i] * (p_sp[i] - p[i]);
  float vxy = sqrtf(vsp[0] * vsp[0] + vsp[1] * vsp[1]);
  if (vxy > c->vel_max_xy) { vsp[0] *= c->vel_max_xy / vxy; vsp[1] *= c->vel_max_xy / vxy; }
  if (vz_sp_override == vz_sp_override) vsp[2] = vz_sp_override; /* not NaN */
  vsp[2] = clampf(vsp[2], -c->vel_max_up, c->vel_max_down);   /* NED: negative = up */
  float a[3];
  for (int i = 0; i < 3; i++) a[i] = pid_step(&c->vel[i], vsp[i], v[i], dt, integrate);
  float cy = cosf(yaw), sy = sinf(yaw);
  float ax_h = cy * a[0] + sy * a[1], ay_h = -sy * a[0] + cy * a[1];
  float up = FC_G - a[2];
  if (up < 0.3f * FC_G) up = 0.3f * FC_G;
  float pitch = clampf(-atan2f(ax_h, up), -c->tilt_max, c->tilt_max);
  float roll = clampf(atan2f(ay_h * cosf(pitch), up), -c->tilt_max, c->tilt_max);
  float t = c->hover_thr * (up / FC_G) / (cosf(roll) * cosf(pitch));
  *roll_sp = roll;
  *pitch_sp = pitch;
  *thr = clampf(t, 0.05f, 0.95f);
}

/* ---------------- mixer ---------------- */

/* rows: motors 1..4 (RR, FR, RL, FL); columns: roll, pitch, yaw */
static const float k_mix[4][3] = {
    {-1.0f, -1.0f, -1.0f}, /* 1 rear-right, CW  */
    {-1.0f, +1.0f, +1.0f}, /* 2 front-right, CCW */
    {+1.0f, -1.0f, +1.0f}, /* 3 rear-left, CCW  */
    {+1.0f, +1.0f, -1.0f}, /* 4 front-left, CW   */
};

void mixer_quad_x(float thr, const float torque[3], float idle, float out[4]) {
  float rp[4], y[4], lo = 1e9f, hi = -1e9f;
  for (int i = 0; i < 4; i++) {
    rp[i] = k_mix[i][0] * torque[0] + k_mix[i][1] * torque[1];
    y[i] = k_mix[i][2] * torque[2];
  }
  float range = 1.0f - idle;
  /* yaw is scaled down first if roll/pitch + yaw do not fit */
  for (int i = 0; i < 4; i++) { float s = rp[i] + y[i]; lo = s < lo ? s : lo; hi = s > hi ? s : hi; }
  if (hi - lo > range) {
    float rlo = 1e9f, rhi = -1e9f;
    for (int i = 0; i < 4; i++) { rlo = rp[i] < rlo ? rp[i] : rlo; rhi = rp[i] > rhi ? rp[i] : rhi; }
    if (rhi - rlo >= range) { /* roll / pitch alone saturate: drop yaw, scale roll / pitch */
      float s = range / (rhi - rlo);
      for (int i = 0; i < 4; i++) { rp[i] *= s; y[i] = 0; }
    } else {
      float ys = 1.0f, step = 0.5f;
      for (int it = 0; it < 12; it++) { /* bisection on the yaw scale */
        float l2 = 1e9f, h2 = -1e9f;
        for (int i = 0; i < 4; i++) { float s = rp[i] + ys * y[i]; l2 = s < l2 ? s : l2; h2 = s > h2 ? s : h2; }
        ys += (h2 - l2 > range) ? -step : step;
        if (ys > 1.0f) ys = 1.0f;
        step *= 0.5f;
      }
      ys -= 0.001f;
      if (ys < 0) ys = 0;
      for (int i = 0; i < 4; i++) y[i] *= ys;
    }
  }
  lo = 1e9f; hi = -1e9f;
  for (int i = 0; i < 4; i++) { out[i] = thr + rp[i] + y[i]; lo = out[i] < lo ? out[i] : lo; hi = out[i] > hi ? out[i] : hi; }
  float shift = 0;
  if (hi > 1.0f) shift = 1.0f - hi;
  if (lo + shift < idle) shift = idle - lo;
  for (int i = 0; i < 4; i++) out[i] = clampf(out[i] + shift, idle, 1.0f);
}

/* ---------------- phone IMU stream ---------------- */

void imu_stream_init(imu_stream_t *s, int64_t max_gap_us, float nominal_dt) {
  memset(s, 0, sizeof(*s));
  s->max_gap_us = max_gap_us;
  s->nominal_dt = nominal_dt;
}

bool imu_stream_accept(imu_stream_t *s, int64_t t_phone_us, uint16_t seq, int64_t rx_local_us, float *dt) {
  *dt = 0;
  if (!s->have) {
    s->have = true;
    s->last_t_us = t_phone_us;
    s->last_rx_us = rx_local_us;
    s->last_seq = seq;
    s->gap_flag = false;
    s->n_ok++;
    *dt = s->nominal_dt;
    return true;
  }
  int64_t d = t_phone_us - s->last_t_us;
  if (d == 0) { s->n_dup++; return false; }
  if (d < 0) {
    if (d < -1000000) { /* phone clock / app restart: resync, treated as a gap */
      s->last_t_us = t_phone_us; s->last_rx_us = rx_local_us; s->last_seq = seq; s->gap_flag = true; s->n_gap++;
      *dt = s->nominal_dt;
      return true;
    }
    s->n_old++;
    return false;
  }
  if ((uint16_t)(seq - s->last_seq) != 1) s->n_seq_skip++;
  s->gap_flag = d > s->max_gap_us;
  if (s->gap_flag) s->n_gap++;
  float fdt = (float)d * 1e-6f;
  if (fdt > 2.0f * s->nominal_dt) fdt = 2.0f * s->nominal_dt; /* never integrate a long hole in one step */
  *dt = fdt;
  s->last_t_us = t_phone_us;
  s->last_rx_us = rx_local_us;
  s->last_seq = seq;
  s->n_ok++;
  return true;
}

bool imu_stream_ok(const imu_stream_t *s, int64_t now_local_us) {
  return s->have && !s->gap_flag && now_local_us - s->last_rx_us <= s->max_gap_us;
}

void fc_rot3(const float R[9], const float v[3], float out[3]) {
  for (int i = 0; i < 3; i++) out[i] = R[3 * i] * v[0] + R[3 * i + 1] * v[1] + R[3 * i + 2] * v[2];
}

/* ---------------- ESC pulses ---------------- */

float fc_pwm_pulse_us(float thr, float min_us, float max_us) {
  if (!(thr > 0.0f)) return min_us; /* NaN, 0, negative -> stop */
  if (thr > 1.0f) thr = 1.0f;
  return min_us + thr * (max_us - min_us);
}

uint32_t fc_pwm_duty(float pulse_us, float freq_hz, int bits) {
  float period_us = 1e6f / freq_hz;
  if (!(pulse_us >= 0.0f)) pulse_us = 0.0f;
  if (pulse_us > period_us - 20.0f) pulse_us = period_us - 20.0f; /* keep >= 20 us low time */
  float full = (float)((1u << bits) - 1u);
  return (uint32_t)(pulse_us / period_us * full + 0.5f);
}

bool fc_pwm_config_ok(float freq_hz, float max_us) { return freq_hz > 0.0f && max_us + 20.0f <= 1e6f / freq_hz; }
