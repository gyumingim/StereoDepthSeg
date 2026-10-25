/* fc_safety.h — flight safety state machine (KILL, tiered EMERGENCY LAND, arming rules, watchdog inputs).
 *
 * Pure C99, no hardware, no OS, no malloc, no floats outside plain arithmetic: compiled unchanged into the ESP-IDF
 * firmware (components/fc_safety) and into the PC unit tests (test/test_safety.c).
 *
 * Threading: ONE owner (the control task). Other tasks never call these functions directly; they post messages
 * (pose / heartbeat / link / command / kill request) to a queue that the control task drains at the start of every
 * cycle, then calls fs_step(). Every function that changes state does it immediately, so a KILL command or a fault
 * found in this cycle disables the motors in the SAME cycle (the control task writes outputs after fs_step /
 * fs_check_outputs).
 *
 * STATES and allowed transitions (anything else is refused and logged as FS_EV_ILLEGAL):
 *   DISARMED   -> ARMED_IDLE (ARM cmd + all arm checks)        | KILLED (kill cmd / kill switch)
 *   ARMED_IDLE -> FLYING (collective > thr_takeoff)            | DISARMED (DISARM cmd, idle timeout, land trigger on
 *                                                                the ground)               | KILLED
 *   FLYING     -> ARMED_IDLE (normal touchdown detected)       | EMERG_LAND (any land trigger) | KILLED
 *   EMERG_LAND -> DISARMED (touchdown / impact / open-loop timeout) | KILLED  (never back to FLYING: no auto-resume)
 *   KILLED     -> DISARMED (CLEAR_KILL cmd, only on the ground, level, switch released, >= kill_clear_min_ms)
 *
 * EMERGENCY LAND tiers (chosen at the trigger, re-evaluated every cycle, can only step DOWN):
 *   FS_TIER_VIO      phone pose valid + ESP32 IMU ok: hold x,y of the land start (position loop), descend at
 *                    land_rate_mps using the VIO vertical speed, touchdown -> DISARMED
 *   FS_TIER_IMU      no usable VIO, IMU ok: level attitude, vertical speed from the IMU estimator, horizontal drift
 *                    accepted, touchdown (low accel variance + low collective or impact spike) -> DISARMED
 *   FS_TIER_OPENLOOP no usable IMU (= phone IMU stream lost: with the phone as the only IMU, VIO and IMU tiers are then
 *                    both impossible): NO ATTITUDE STABILISATION. Per-motor commands BACK-CALCULATED from this flight's
 *                    takeoff hover record: thrust ~ cmd^k (k = openloop_thrust_exp) =>
 *                      cmd_i = hover_i * ratio^(1/k) * (V_record / V_now)
 *                    with ratio = 1 - acc_down/g for vz_down/acc_down s (start descending), then cruise_ratio (just
 *                    under hover). Cut at the estimated touchdown time from the last known height * margin
 *                    (or openloop_max_ms), or at an impact if the accelerometer still works.
 *                    Last resort; the vehicle may tip and drift. Hover source: this flight's record > previous
 *                    flight's record (set with fs_set_prev_hover) > cfg.openloop_default_thr.
 * KILL overrides every state and tier.
 */
#ifndef FC_SAFETY_H
#define FC_SAFETY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FS_MAX_MOTORS 8
#define FS_EV_RING 64

typedef enum { FS_DISARMED = 0, FS_ARMED_IDLE = 1, FS_FLYING = 2, FS_EMERG_LAND = 3, FS_KILLED = 4, FS_NUM_STATES = 5 } fs_state_t;
typedef enum { FS_TIER_NONE = 0, FS_TIER_VIO = 1, FS_TIER_IMU = 2, FS_TIER_OPENLOOP = 3 } fs_tier_t;
/* attitude source the controller must use this cycle */
typedef enum { FS_CTRL_OFF = 0, FS_CTRL_NORMAL = 1, FS_CTRL_IDLE = 2, FS_CTRL_LAND_HOLD_XY = 3, FS_CTRL_LAND_LEVEL = 4, FS_CTRL_OPENLOOP = 5 } fs_ctrl_mode_t;
/* where the tier-3 hover values came from */
typedef enum { FS_HOVER_NONE = 0, FS_HOVER_THIS_FLIGHT = 1, FS_HOVER_PREV_FLIGHT = 2, FS_HOVER_DEFAULT = 3 } fs_hover_src_t;

/* reason bits (triggers of transitions / tier changes) */
#define FS_R_CMD (1u << 0)
#define FS_R_SWITCH (1u << 1)
#define FS_R_IMU (1u << 2)
#define FS_R_TILT (1u << 3)
#define FS_R_NAN (1u << 4)
#define FS_R_RATE (1u << 5)
#define FS_R_POSE_STALE (1u << 6)
#define FS_R_POSE_BAD (1u << 7)
#define FS_R_POSE_RESET (1u << 8)
#define FS_R_HB_STALE (1u << 9)
#define FS_R_HB_LOST (1u << 10)
#define FS_R_HB_SEQ (1u << 11)
#define FS_R_PHONE_DISC (1u << 12)
#define FS_R_LINK (1u << 13)
#define FS_R_BATT (1u << 14)
#define FS_R_OVERRUN (1u << 15)
#define FS_R_LOOP_STALL (1u << 16)
#define FS_R_TOUCHDOWN (1u << 17)
#define FS_R_IDLE (1u << 18)
#define FS_R_LAND_TIMEOUT (1u << 19)
#define FS_R_TAKEOFF (1u << 20)
#define FS_R_CLEAR (1u << 21)
#define FS_R_IMPACT (1u << 22)
#define FS_R_ARM (1u << 23)
#define FS_R_DISARM (1u << 24)

/* arm / clear-kill block bits (FS_EV_ARM_REJECT.reason) */
#define FS_B_STATE (1u << 0)
#define FS_B_NO_INPUT (1u << 1)
#define FS_B_SWITCH (1u << 2)
#define FS_B_IMU (1u << 3)
#define FS_B_TILT (1u << 4)
#define FS_B_THR (1u << 5)
#define FS_B_MOVING (1u << 6)
#define FS_B_POSE (1u << 7)
#define FS_B_HB (1u << 8)
#define FS_B_LINK (1u << 9)
#define FS_B_BATT (1u << 10)
#define FS_B_OVERRUN (1u << 11)
#define FS_B_PHONE_DISC (1u << 12)
#define FS_B_TOO_SOON (1u << 13)

/* event codes */
enum {
  FS_EV_TRANSITION = 1, /* from/to = states, reason = FS_R_* */
  FS_EV_ARM_REJECT = 2, /* reason = FS_B_* */
  FS_EV_CMD_REJECT = 3, /* aux = command, from = state */
  FS_EV_TIER = 4,       /* from/to = tiers (from = NONE at the land start), aux = fs_hover_src_t for OPENLOOP */
  FS_EV_TRIGGER = 5,    /* extra land trigger bits seen while already landing (reason = new bits) */
  FS_EV_ILLEGAL = 6,    /* refused transition from/to */
  FS_EV_HOVER_REC = 7,  /* this flight's takeoff hover record became valid; aux = samples / 100 */
  FS_EV_CLEAR_REJECT = 8,
};

typedef enum { FS_CMD_ARM = 1, FS_CMD_DISARM = 2, FS_CMD_KILL = 3, FS_CMD_LAND = 4, FS_CMD_CLEAR_KILL = 5 } fs_cmd_t;

typedef struct {
  uint32_t t_ms;
  uint16_t seq;
  uint8_t code, from, to, aux;
  uint32_t reason;
} fs_event_t;

typedef struct {
  int n_motors;                 /* 4 for a quad */
  /* KILL */
  float tilt_kill_deg;          /* |tilt| beyond this while armed -> KILL */
  float land_tilt_kill_deg;     /* stricter limit during EMERG_LAND (vehicle should be level) */
  float rate_kill_dps;          /* |gyro| beyond this while armed -> KILL (spin-out) */
  bool imu_fail_kill;           /* true: IMU failure in flight -> KILL; false (default): -> EMERG_LAND tier OPENLOOP */
  /* land triggers */
  uint32_t pose_timeout_ms;     /* no GOOD pose for this long -> land (tier drops below VIO) */
  float pose_std_max_m;         /* largest axis 1-sigma accepted as good */
  uint32_t pose_max_delay_us;   /* send_delay_us above this = not good */
  bool require_pose_in_flight;  /* false = pose loss only lowers the tier if already landing, never triggers */
  bool land_on_pose_reset;      /* VIO re-initialised (new world frame) while armed -> land */
  uint32_t hb_timeout_ms;       /* phone heartbeat */
  bool require_hb_in_flight;
  uint32_t link_timeout_ms;     /* PC ESP-NOW heartbeat */
  bool require_link_in_flight;
  int batt_cells;
  float batt_land_v_cell;       /* below this (debounced) -> land */
  float batt_arm_min_v_cell;    /* arm refused below this */
  uint32_t batt_debounce_ms;
  bool require_batt_to_arm;     /* battery voltage must be known (valid, finite) to arm */
  int32_t overrun_add, overrun_land_level; /* leaky bucket: +add per overrun, -1 per good cycle */
  /* arming */
  float arm_tilt_max_deg;
  float thr_arm_max;            /* collective request must be below this to arm / clear kill */
  bool require_pose_to_arm, require_hb_to_arm, require_link_to_arm;
  uint32_t idle_disarm_ms;      /* ARMED_IDLE without takeoff for this long -> DISARMED */
  uint32_t kill_clear_min_ms;
  float thr_takeoff;            /* collective above this in ARMED_IDLE -> FLYING */
  /* touchdown detector */
  float touch_acc_var;          /* (m/s^2)^2, EMA variance of |specific force| */
  float touch_acc_dev;          /* | |f| - g | below this (rules out free fall) */
  float touch_thr_frac;         /* collective <= frac * hover */
  float touch_vz_max;           /* |vz| (VIO) below this, when VIO vz is valid */
  uint32_t touch_hold_ms;
  float impact_acc;             /* |f| above this = impact spike (m/s^2) */
  float acc_var_tau_s;
  /* emergency land, tiers VIO / IMU */
  float land_rate_mps;          /* descent speed */
  float land_kp, land_ki;       /* vz -> collective */
  float hover_thr_default;      /* collective used before a hover estimate exists */
  float land_thr_max_frac;      /* collective clamp, * hover */
  uint32_t land_max_ms;         /* EMERG_LAND in tier VIO/IMU longer than this -> step down to OPENLOOP */
  /* tier 3 open loop */
  float openloop_start_frac;    /* first command = frac * recorded hover (per motor) */
  float openloop_ramp_per_s;    /* frac decreases this much per second */
  float openloop_min_frac;
  uint32_t openloop_max_ms;     /* then motors off */
  float openloop_default_thr;   /* per-motor command if no hover record at all (conservative) */
  float openloop_thrust_exp;    /* thrust ~ cmd^k (props: ~2 for PWM throttle) */
  float openloop_acc_down;      /* m/s^2 downward acceleration of the first phase */
  float openloop_vz_down;       /* m/s descent speed reached by the first phase */
  float openloop_cruise_ratio;  /* thrust / weight after the first phase (< 1: keeps descending slowly) */
  float openloop_cut_margin;    /* cut time = estimated touchdown time * margin (+0.3 s) */
  bool openloop_volt_comp;      /* scale commands by V_record / V_now (PWM ESCs: thrust depends on voltage) */
  /* takeoff hover recorder (this flight) */
  uint32_t rec_skip_ms;         /* ignore the first part after takeoff (spin-up, ground effect) */
  uint32_t rec_min_ms;          /* record valid after this much stable data */
  uint32_t rec_window_ms;       /* record frozen after this much stable data */
  float rec_tilt_max_deg, rec_rate_max_dps, rec_acc_dev_max, rec_vz_max;
  float rec_climb_vz;            /* vz (up) that proves lift-off; recording starts rec_skip_ms after it */
} fs_config_t;

/* one decoded phone pose (from viop_pose_t, see fs_viop.h) */
typedef struct {
  uint16_t seq;
  uint16_t flags;               /* VIOP_F_* */
  uint8_t quality;
  uint8_t reset_counter;
  float pos_std_max_m;
  uint32_t send_delay_us;
} fs_pose_info_t;

/* control-loop snapshot, every cycle */
typedef struct {
  uint32_t now_ms;
  float dt_s;
  bool imu_ok;                  /* gyro + accel usable for attitude: PHONE IMU stream fresh (no gap > max), plausible */
  bool acc_ok;                  /* accelerometer alone usable (impact detection in tier 3) */
  float roll_rad, pitch_rad;
  float gyro_norm_rps;
  float acc_norm;               /* |specific force| m/s^2 (~9.81 at rest) */
  float thr_cmd;                /* collective requested by the normal controller 0..1 */
  bool vz_vio_valid; float vz_vio;  /* up positive, m/s, from fresh VIO */
  bool vz_imu_valid; float vz_imu;  /* up positive, from the IMU vertical-speed estimator */
  bool batt_valid; float batt_v;
  bool height_valid; float height_m; /* height above the takeoff point (VIO), m, up */
  bool kill_switch;             /* physical switch active */
  bool loop_overrun;
  float motor_prev[FS_MAX_MOTORS]; /* motor commands written last cycle 0..1 (hover recorder) */
} fs_input_t;

typedef struct {
  fs_state_t state;
  fs_tier_t tier;
  fs_ctrl_mode_t mode;
  bool motors_enabled;          /* false: write DShot 0 (disarmed) this cycle */
  float thr;                    /* collective for LAND_HOLD_XY / LAND_LEVEL */
  float vz_target;              /* informative */
  float motor[FS_MAX_MOTORS];   /* per-motor commands for FS_CTRL_OPENLOOP */
} fs_output_t;

typedef struct {
  bool valid;
  int n;
  float m[FS_MAX_MOTORS];
  float collective;             /* mean of m */
  float batt_v;                 /* mean battery voltage while recording (0 = unknown) */
  uint32_t samples;
} fs_hover_rec_t;

typedef struct {
  fs_config_t cfg;
  fs_state_t state;
  fs_tier_t tier;
  uint32_t t_state_ms, t_tier_ms, now_ms;
  bool have_input;
  fs_input_t last;
  /* pose */
  bool pose_seen, pose_seq_seen, pose_lost_flag, pose_reset_flag;
  uint32_t t_pose_ok_ms;
  uint16_t pose_seq;
  uint8_t pose_reset_counter;
  /* phone heartbeat */
  bool hb_seen, hb_lost_flag, hb_seq_stalled, phone_disconnected;
  uint32_t t_hb_ok_ms;
  uint16_t hb_seq;
  /* PC link */
  bool link_seen;
  uint32_t t_link_ms;
  /* battery */
  bool batt_low_active, batt_low_timing;
  uint32_t t_batt_low_ms;
  /* overrun */
  int32_t overrun_bucket;
  uint32_t overruns;
  /* accel statistics */
  float acc_mean, acc_var;
  uint32_t t_impact_ms;
  bool impact_seen;
  /* land */
  uint32_t land_reason, kill_reason;
  float hover_ema, land_i, land_thr;
  uint32_t t_touch_ms;
  bool touch_timing;
  fs_hover_src_t ol_src;
  float ol_base[FS_MAX_MOTORS];
  float ol_vcomp;
  uint32_t ol_cut_ms;
  float last_height;
  bool last_height_valid;
  float rec_batt_sum;
  uint32_t rec_batt_n;
  /* takeoff hover recorder */
  uint32_t t_takeoff_ms;
  float rec_sum[FS_MAX_MOTORS];  /* float on purpose: ESP32-S3 FPU is single precision */
  uint32_t rec_n;
  float rec_ms;
  bool rec_frozen;
  bool rc_seen;                  /* a pose reset_counter has been seen */
  bool takeoff_enabled;          /* ARMED_IDLE: collective request has been low since entering */
  bool airborne;                 /* FLYING: climb seen (hover recorder / hover estimate only after this) */
  uint32_t t_airborne_ms;
  fs_hover_rec_t rec, prev;
  /* pending requests (from other contexts via the control task queue) */
  uint32_t pend_land;
  /* events */
  fs_event_t ev[FS_EV_RING];
  uint16_t ev_head, ev_count, ev_seq;
  uint32_t ev_dropped;
} fs_t;

void fs_config_default(fs_config_t *c);
void fs_init(fs_t *fs, const fs_config_t *cfg, uint32_t now_ms);

/* inputs from the link task (via the control task queue) */
void fs_on_pose(fs_t *fs, uint32_t now_ms, const fs_pose_info_t *p);
/* seq = VIOP frame seq; vio_state / app_flags = viop_heartbeat_t.state / .flags */
void fs_on_heartbeat(fs_t *fs, uint32_t now_ms, uint16_t seq, uint8_t vio_state, uint8_t app_flags);
void fs_on_phone_connection(fs_t *fs, uint32_t now_ms, bool connected); /* USB CDC port closed (DTR 1->0) / reopened */
void fs_on_link(fs_t *fs, uint32_t now_ms);                             /* valid PC frame (heartbeat / cmd) */
/* commands: returns true if accepted (rejections are logged as events) */
bool fs_command(fs_t *fs, uint32_t now_ms, fs_cmd_t cmd);
void fs_request_kill(fs_t *fs, uint32_t now_ms, uint32_t reason);       /* supervisor: loop stall, ... */
void fs_request_land(fs_t *fs, uint32_t now_ms, uint32_t reason);

/* once per control cycle, after the drained messages */
void fs_step(fs_t *fs, const fs_input_t *in, fs_output_t *out);
/* after the mixer: NaN / inf / out of range -> KILL (returns false; caller writes zero outputs) */
bool fs_check_outputs(fs_t *fs, uint32_t now_ms, const float *m, int n);

/* hover record persistence: get this flight's record (save to RTC / NVS after landing), set last flight's at boot */
void fs_get_hover_record(const fs_t *fs, fs_hover_rec_t *out);
void fs_set_prev_hover(fs_t *fs, const fs_hover_rec_t *prev);

bool fs_pop_event(fs_t *fs, fs_event_t *ev);
bool fs_transition_allowed(fs_state_t from, fs_state_t to);
const char *fs_state_name(fs_state_t s);
const char *fs_tier_name(fs_tier_t t);

#ifdef __cplusplus
}
#endif
#endif /* FC_SAFETY_H */
