/* fc_core.h — hardware-independent flight-control math (pure C99, float only, no malloc): attitude estimator
 * (Mahony complementary filter), IMU vertical-speed estimator (for LAND_IMU), PID, attitude / position controllers,
 * quad-X mixer. Used by the ESP-IDF firmware and by the PC tests. Frames: body FRD, local NED (z down).
 * NOTE: gains are placeholders (450-class quad guess, 추정) — must be tuned on the real vehicle. */
#ifndef FC_CORE_H
#define FC_CORE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FC_G 9.80665f

/* ---------- attitude estimator (Mahony) ---------- */
typedef struct {
  float q[4];          /* w x y z, body FRD -> local NED */
  float bias[3];       /* gyro bias, rad/s */
  float kp, ki;        /* accel correction gains */
  float k_yaw;         /* external yaw (VIO) correction gain, 1/s */
  bool init;
} att_est_t;
void att_est_init(att_est_t *e, float kp, float ki, float k_yaw);
/* gyro rad/s, acc m/s^2 (specific force, body FRD: +z down => at rest acc = (0,0,-g)) */
void att_est_update(att_est_t *e, const float gyro[3], const float acc[3], float dt);
void att_est_fuse_yaw(att_est_t *e, float yaw_meas, float dt);
void att_est_euler(const att_est_t *e, float *roll, float *pitch, float *yaw);
void att_est_rot(const att_est_t *e, float R[9]); /* R_LB, row-major */

/* ---------- vertical speed from the IMU (LAND_IMU tier) ----------
 * 2-state complementary filter (vz_up, accel bias). While VIO vz is fresh: corrected toward it and the bias is learnt.
 * Without VIO: pure (leaky) integration of the vertical specific force minus the learnt bias, valid for horizon_s. */
typedef struct {
  float vz;            /* up positive, m/s */
  float bias;          /* vertical accel bias, m/s^2 */
  float k_v, k_b;      /* correction gains (1/s, 1/s^2) */
  float leak_tau_s;    /* integration leak without VIO (0 = none) */
  float horizon_s;     /* valid this long after the last VIO correction */
  float since_vio_s;
  bool have_vio;
} vz_est_t;
void vz_est_init(vz_est_t *v, float k_v, float k_b, float leak_tau_s, float horizon_s);
/* R_LB row-major (from att_est_rot), acc body specific force */
void vz_est_predict(vz_est_t *v, const float R_LB[9], const float acc[3], float dt);
void vz_est_correct(vz_est_t *v, float vz_vio_up, float dt);
bool vz_est_valid(const vz_est_t *v);

/* ---------- PID ---------- */
typedef struct {
  float kp, ki, kd, i_lim, out_lim;
  float d_lpf_a;       /* 0..1 low pass factor on the derivative (per call) */
  float i, prev_meas, d;
  bool init;
} pid_t_;
void pid_init(pid_t_ *p, float kp, float ki, float kd, float i_lim, float out_lim, float d_lpf_a);
float pid_step(pid_t_ *p, float sp, float meas, float dt, bool integrate);
void pid_reset(pid_t_ *p);

/* ---------- rate + attitude controller ---------- */
typedef struct {
  pid_t_ rate[3];      /* roll, pitch, yaw rate -> torque command (normalised) */
  float att_kp[3];     /* angle error -> rate setpoint, 1/s */
  float rate_max[3];   /* rad/s */
} att_ctrl_t;
void att_ctrl_init(att_ctrl_t *c);
/* setpoint roll/pitch/yaw (rad), current attitude, gyro (body). Output torque[3] normalised -1..1 */
void att_ctrl_step(att_ctrl_t *c, const float sp_rpy[3], float roll, float pitch, float yaw, const float gyro[3], float dt,
                   bool integrate, float torque[3]);
void att_ctrl_reset(att_ctrl_t *c);

/* ---------- position controller (local NED) -> roll/pitch/collective ---------- */
typedef struct {
  float kp_pos[3];     /* m -> m/s */
  pid_t_ vel[3];       /* m/s -> m/s^2 */
  float vel_max_xy, vel_max_up, vel_max_down;
  float tilt_max;      /* rad */
  float hover_thr;     /* collective that holds 1 g (estimate) */
} pos_ctrl_t;
void pos_ctrl_init(pos_ctrl_t *c, float hover_thr);
/* p, v current (NED), p_sp setpoint, yaw current. vz_sp_override: if not NaN, z uses this NED vertical speed setpoint
 * instead of the z position loop (emergency land in place). out: roll, pitch (rad), collective 0..1 */
void pos_ctrl_step(pos_ctrl_t *c, const float p[3], const float v[3], const float p_sp[3], float yaw, float vz_sp_override,
                   float dt, bool integrate, float *roll_sp, float *pitch_sp, float *thr);
void pos_ctrl_reset(pos_ctrl_t *c);

/* ---------- quad X mixer ----------
 * Motor order = Betaflight "quad X" numbering and default props-in spin: 1 rear-right (CW), 2 front-right (CCW),
 * 3 rear-left (CCW), 4 front-left (CW). Change k_mix in fc_core.c if the wiring / spin differs.
 * Sign conventions: roll + = right side down, pitch + = nose up, yaw + = nose right (FRD).
 * Desaturation: roll/pitch keep priority, yaw is scaled down first, then the collective is shifted. */
void mixer_quad_x(float thr, const float torque[3], float idle, float out[4]);

#ifdef __cplusplus
}
#endif
#endif /* FC_CORE_H */
