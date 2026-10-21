// ov_bridge_diag (2026-10-20, collapse-detection study): copy of scan3d/ov_bridge/main.cpp (commit e301140) + --diag flag
// (pose lines add sig15 = IMU error-state std, n_track). Linked to ~/scanner/src/ovbuild_diag (OpenVINS copy with [CHI2]/[DIAGT] log lines).
// Kept separate so the shared ov_bridge source/binary is untouched. Estimates are identical to ov_bridge for the same input.
// ov_bridge — OpenVINS (ROS 없음) VioManager 를 바이너리 레코드로 구동하는 작은 실행 파일.
// 프로토콜: PROTOCOL.md. 입력 = stdin 또는 TCP, 출력 = 같은 채널로 JSON 한 줄씩.
//
//   ov_bridge <estimator_config.yaml> [--stdio | --tcp HOST:PORT] [--max-cam-queue N]
//             [--max-speed M_S] [--lost-frames N] [--startup-lost-frames N] [--auto-reset] [--emit-features N]
//             [--imu-hold S] [--reinit-state] [--reinit-max-gap S] [--reinit-sigma ORI_DEG,POS_M,VEL_M_S] [--remap FILE]
//             [--init-gate MEAN_RAD_S,MAX_RAD_S,DISP_DEG] [--diag]
//
// --init-gate (2026-10-18, 10-18 오전 "그림자" 방식으로 바꿈): 기본 VioManager (원래 설정, 시차 기준 그대로) 옆에
// init_max_disparity 를 DISP_DEG 각도 (cam0 fx 로 화소 환산) 로 넓힌 "그림자" VioManager 를 하나 더 돌린다 (초기화 전에만).
//  - 그림자는 카메라 시작 뒤 0.8 s 동안의 프레임은 안 받는다 (시작 직후 흔들림·AE 로 곧바로 초기화되는 것 막기).
//  - 그림자의 정지 초기화가 끝나면 그 창 init_window_time(+0.05 s) 의 자이로가 조용했는지 (평균 각속도 벡터 크기 <= MEAN,
//    순간 최대 <= MAX) 확인해 아니면 그림자만 새로 만든다.
//  - 기본 쪽이 정지 초기화를 하면 그림자는 버린다 (= 예전과 완전히 같은 결과, 참조 녹화 그대로).
//  - 기본 쪽이 초기화 못 한 채로 폰이 움직이기 시작하면 (최근 0.1 s 자이로 > MAX, 또는 그림자가 이미 갱신 시작) 확인된 그림자로
//    바꿔 쓴다 ("init_gate":"shadow" 포즈 줄 + stderr 로그).
// 예전 방식 (기본 쪽 기준을 바로 넓히고 버리기) 은 175446 의 초기화 순간을 1 프레임 바꿔 궤적이 크게 달라졌다 (키프레임 142 -> 111):
// 이 녹화는 초기화 순간에 매우 민감하다. 회전은 자이로가 직접 막으므로 (바이어스 오차 <= MEAN) 영상 쪽 기준은 흐린 장면 KLT 잡음·가까운 물체 시차만큼
// 여유를 줄 수 있다. 근거: rec_20261018_002349 는 자이로 평균 0.001 rad/s 로 멈춰 있었는데 흐린 근접 장면이라 시차
// 3.95~6.6 px (> 3.09) 로 초기화가 늦어짐. 예전 문제 (천천히 도는 중 2.6~14.8 px 에서 초기화 → 바이어스 0.016~0.035 rad/s) 는
// 평균 각속도 기준으로 막힌다. 그림자 초기화를 버릴 때 ("moving"|"short") 는 stderr 로그만 남는다.
//
// --reinit-state (2026-10-17): lost(특징 끊김) auto-reset 때 정지 초기화 대신 옛 상태를 새 VioManager 에 넣어 같은 월드로 이어 감
// ({"type":"reset","auto":true,"reason":"lost","reinit":"state",...}). PROTOCOL.md 참고.
//
// 방어: OpenVINS 호출에서 난 예외(cv::Exception 포함)는 잡아서 VioManager 를 새로 만들고
// {"type":"reset","auto":true,"reason":"exception"} 를 보낸다 (프로세스는 계속 산다).
// use_stereo=true 설정이면 2장짜리 STEREO 레코드만 받는다 (mono 섞임 = TrackKLT levels2 단언 원인).
//
// stdio 모드에서는 OpenVINS 로그(printf -> fd 1)를 stderr 로 돌리고, 결과만 원래 stdout 으로 쓴다.

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "update/UpdaterZeroVelocity.h"
#include "track/TrackBase.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/IMU.h"
#include "types/PoseJPL.h"
#include "types/Vec.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

namespace {

// --diag (2026-10-20, 초기화 붕괴 감지 조사): VioManager 의 protected 추적기에 닿으려고 파생 클래스로 만든다 (동작은 같음).
struct VioDiag : public ov_msckf::VioManager {
  using ov_msckf::VioManager::VioManager;
  // G1.0 (2026-10-22, docs/g10_plan.md): pointer-to-member of the protected ZUPT updater (OpenVINS library unchanged).
  static std::shared_ptr<ov_msckf::UpdaterZeroVelocity> ov_msckf::VioManager::*zupt_member() { return &VioDiag::updaterZUPT; }
  size_t n_track() {  // 마지막 영상에서 cam0 가 추적 중인 특징 수
    if (!trackFEATS) return 0;
    auto lo = trackFEATS->get_last_obs();
    auto it = lo.find(0);
    return it == lo.end() ? 0 : it->second.size();
  }
};

// G1.0: pointer-to-member of UpdaterZeroVelocity::_zupt_max_disparity (protected) -> per-frame threshold setter.
struct ZuptAcc : public ov_msckf::UpdaterZeroVelocity {
  static double ov_msckf::UpdaterZeroVelocity::*disp_member() { return &ZuptAcc::_zupt_max_disparity; }
};

constexpr uint32_t kMagic = 0x4F564252u;  // "OVBR"
enum MsgType : uint32_t { kImu = 1, kStereo = 2, kReset = 3, kQuit = 4 };
constexpr uint32_t kMaxPayload = 64u * 1024u * 1024u;

struct Args {
  std::string config;
  bool tcp = false;
  std::string host = "127.0.0.1";
  int port = 7600;
  size_t max_cam_queue = 3;
  double max_speed = 5.0;   // m/s 넘으면 lost (손에 든 폰 기준)
  int lost_frames = 15;     // 특징점 0 이 연속 N 프레임이면 lost
  int startup_lost_frames = 45;  // 초기화 뒤 첫 특징 갱신 전에는 이만큼 (15 fps 에서 3 s) 기다린 뒤 lost
  bool auto_reset = false;
  bool diag = false;        // --diag: pose 줄에 IMU 상태 표준편차 sig15 (자세 rad, 위치 m, 속도 m/s, bg, ba) 와 n_track
  int emit_features = 0;    // JSON 에 SLAM 특징점 최대 N 개
  double imu_hold_s = 0.25; // 카메라를 기다리며 IMU 를 붙잡아 두는 최대 시간 (IMU·카메라 시간 순서 유지)
  // 정지 없이 다시 시작 (2026-10-17): 특징점이 끊겨 lost 가 되면 VioManager 를 새로 만들되, 정지 초기화를 기다리지 않고
  // 옛 상태(자세·위치·속도·바이어스, 마지막 특징 갱신 뒤 IMU 만으로 전파한 값)와 추정된 외부 파라미터·시간 오프셋을
  // initialize_with_gt 로 넣는다 → 같은 월드 좌표로 바로 이어서 추적. 마지막 특징 갱신이 reinit_max_gap 초보다 오래됐거나
  // 상태가 망가졌으면(속도 초과·NaN) 예전처럼 정지 초기화 (reason lost, reinit 없음).
  // 근거: rec_20261016_175446/162204/165645 는 lost 뒤 계속 움직여 정지 초기화를 한 번도 못 함 (재생 로그 "failed static init").
  bool reinit_state = false;
  double reinit_max_gap = 1.5;
  double reinit_sig_ori_deg = 3.0;  // 넣은 상태의 공분산 (IMU 만 전파한 1 s 동안 늘어난 불확실성, 추정)
  double reinit_sig_pos = 0.10;
  double reinit_sig_vel = 0.30;
  double gate_mean = 0.0;   // --init-gate (0 = 끔)
  double gate_max = 0.0;
  double gate_disp_deg = 0.0;
  // --remap FILE (2026-10-18, 폰 실시간): cam0 영상을 받은 뒤 이 맵으로 cv::remap (렌즈 2 원본 → lens0u 왜곡 편 영상).
  // 파일 = int32 w, h + float32 mapx[w*h] + float32 mapy[w*h] (scan3d/ov_bridge/android/make_phone_ov.py). 크기가 다른 영상은 그대로.
  std::string remap;
  double zupt_alt_disp = -1.0;  // --zupt-alt-disp X (G1.0): frame flag byte 17 = 1 -> zupt_max_disparity X, else config value
};

// JSON 문자열 이스케이프 (예외 메시지에 따옴표·줄바꿈이 들어 있음)
std::string jstr(const std::string &s) {
  std::string o = "\"";
  for (char ch : s) {
    unsigned char c = static_cast<unsigned char>(ch);
    if (c == '"' || c == '\\') {
      o += '\\';
      o += ch;
    } else if (c < 0x20) {
      char b[8];
      std::snprintf(b, sizeof(b), "\\u%04x", c);
      o += b;
    } else {
      o += ch;
    }
  }
  return o + "\"";
}

// ---------- I/O ----------
bool read_full(int fd, void *buf, size_t n) {
  auto *p = static_cast<uint8_t *>(buf);
  while (n > 0) {
    ssize_t r = ::read(fd, p, n);
    if (r == 0) return false;
    if (r < 0) {
      if (errno == EINTR) continue;
      return false;
    }
    p += r;
    n -= static_cast<size_t>(r);
  }
  return true;
}

bool write_full(int fd, const std::string &s, bool is_socket) {
  const char *p = s.data();
  size_t n = s.size();
  while (n > 0) {
    ssize_t r = is_socket ? ::send(fd, p, n, MSG_NOSIGNAL) : ::write(fd, p, n);
    if (r < 0) {
      if (errno == EINTR) continue;
      return false;
    }
    p += r;
    n -= static_cast<size_t>(r);
  }
  return true;
}

template <typename T> T rd(const uint8_t *p) {  // little-endian (x86)
  T v;
  std::memcpy(&v, p, sizeof(T));
  return v;
}

// ---------- JSON helpers ----------
void jnum(std::ostringstream &o, double v) {
  if (std::isfinite(v))
    o << v;
  else
    o << "null";
}
template <typename M> void jarr(std::ostringstream &o, const M &m) {  // row-major
  o << "[";
  bool first = true;
  for (int r = 0; r < m.rows(); r++)
    for (int c = 0; c < m.cols(); c++) {
      if (!first) o << ",";
      first = false;
      jnum(o, m(r, c));
    }
  o << "]";
}

struct CamMsg {
  int64_t t_ns;
  double t;
  uint8_t g10 = 0;  // G1.0 gate flag (STEREO payload byte 17, was padding = 0)
  ov_core::CameraData data;
};

class Bridge {
public:
  Bridge(const Args &a) : args_(a) {
    parser_ = std::make_shared<ov_core::YamlParser>(a.config);
    std::string verbosity = "INFO";
    parser_->parse_config("verbosity", verbosity);
    ov_core::Printer::setPrintLevel(verbosity);
    params_.print_and_load(parser_);
    params_.use_multi_threading_subs = false;  // 순서대로 처리 (카메라 feed 는 이 스레드에서만)
    if (!a.remap.empty()) load_remap(a.remap);
    if (a.gate_mean > 0.0 && a.gate_disp_deg > 0.0 && params_.camera_intrinsics.count(0)) {
      double fx = params_.camera_intrinsics.at(0)->get_value()(0);
      double px = fx * std::tan(a.gate_disp_deg * M_PI / 180.0);
      std::fprintf(stderr, "[ov_bridge] init gate: gyro mean <= %.4f max <= %.3f rad/s, init_max_disparity %.2f -> %.2f px (%.2f deg, fx %.1f)\n",
                   a.gate_mean, a.gate_max, params_.init_options.init_max_disparity, px, a.gate_disp_deg, fx);
      shadow_params_ = params_;
      shadow_params_.init_options.init_max_disparity = px;  // 기본 params_ 는 그대로 (그림자만 넓힘)
      use_shadow_ = true;
    }
    if (!parser_->successful()) {
      PRINT_ERROR("[ov_bridge] unable to parse all parameters in %s\n", a.config.c_str());
      ok_ = false;
      return;
    }
    make_system();
  }

  bool ok() const { return ok_; }

  // --init-gate 그림자 VioManager (넓힌 시차 기준) 새로 만들기. refeed = 지금까지 넣은 최근 IMU 를 다시 넣음.
  void make_shadow(bool refeed) {
    shadow_.reset();
    shadow_checked_ = shadow_ok_ = false;
    if (!use_shadow_) return;
    shadow_ = std::make_shared<VioDiag>(shadow_params_);
    if (refeed)
      for (const auto &m : imu_recent_) shadow_->feed_measurement_imu(m);
  }

  // VioManager 새로 만들기. 붙잡아 둔 IMU(imu_hold_) 는 새 시스템에 그대로 넣을 수 있으므로 남긴다.
  void make_system() {
    sys_ = std::make_shared<VioDiag>(params_);
    make_shadow(false);
    init_checked_ = false;
    cams_.clear();
    last_imu_t_ = -1e300;
    zero_feat_frames_ = 0;
    feat_seen_ = false;
    lost_ = false;
  }

  // 입력 하나 처리. 출력 JSON 줄들을 out 에 덧붙인다. false = QUIT.
  bool handle(uint32_t type, const std::vector<uint8_t> &buf, std::string &out) {
    switch (type) {
    case kImu:
      if (buf.size() != 56) {
        out += err_line("IMU payload must be 56 bytes");
        return true;
      }
      on_imu(buf.data(), out);
      return true;
    case kStereo:
      on_stereo(buf, out);
      return true;
    case kReset:
      make_system();
      imu_hold_.clear();
      imu_recent_.clear();
      last_upd_t_ = -1e300;
      newest_imu_t_ = -1e300;
      first_cam_t_ = -1e300;
      out += "{\"type\":\"reset\"}\n";
      return true;
    case kQuit:
      return false;
    default:
      out += err_line("unknown message type " + std::to_string(type));
      return true;
    }
  }

  std::string ready_line() const {
    std::ostringstream o;
    o << "{\"type\":\"ready\",\"num_cameras\":" << params_.state_options.num_cameras
      << ",\"use_stereo\":" << (params_.use_stereo ? "true" : "false") << ",\"imu_hold_s\":" << args_.imu_hold_s
      << "}\n";
    return o.str();
  }

private:
  double to_sec(int64_t t_ns) {
    if (!have_t0_) {
      t0_ns_ = t_ns;
      have_t0_ = true;
    }
    return static_cast<double>(t_ns - t0_ns_) * 1e-9;
  }

  static std::string err_line(const std::string &m) {
    return "{\"type\":\"error\",\"msg\":" + jstr(m) + "}\n";
  }

  // OpenVINS 안에서 난 예외 (예: TrackKLT 의 cv::Exception) → 프로세스를 죽이지 않고 VioManager 를 새로 만든다.
  // 클라이언트에는 auto-reset 과 같은 줄을 보내 맵·궤적 초기화 규칙(on-vio-reset)을 그대로 타게 한다.
  void on_exception(const char *where, const char *what, std::string &out) {
    n_exceptions_++;
    std::fprintf(stderr, "[ov_bridge] exception in %s: %s -> VioManager reset (#%llu)\n", where, what,
                 static_cast<unsigned long long>(n_exceptions_));
    try {
      make_system();
    } catch (const std::exception &e) {
      std::fprintf(stderr, "[ov_bridge] make_system failed too: %s\n", e.what());
      throw;
    }
    out += "{\"type\":\"reset\",\"auto\":true,\"reason\":\"exception\",\"where\":" + jstr(where) +
           ",\"msg\":" + jstr(std::string(what).substr(0, 300)) + "}\n";
  }

  void on_imu(const uint8_t *p, std::string &out) {
    int64_t t_ns = rd<int64_t>(p);
    double t = to_sec(t_ns);
    if (t <= newest_imu_t_) {  // 역행/중복은 버림
      n_imu_dropped_++;
      return;
    }
    ov_core::ImuData m;
    m.timestamp = t;
    m.wm << rd<double>(p + 8), rd<double>(p + 16), rd<double>(p + 24);
    m.am << rd<double>(p + 32), rd<double>(p + 40), rd<double>(p + 48);
    if (!m.wm.allFinite() || !m.am.allFinite()) {
      n_imu_dropped_++;
      return;
    }
    imu_hold_.push_back(m);
    newest_imu_t_ = t;
    pump(out);
  }

  void on_stereo(const std::vector<uint8_t> &buf, std::string &out) {
    if (buf.size() < 20) {
      out += err_line("STEREO payload too short");
      return;
    }
    const uint8_t *p = buf.data();
    int64_t t_ns = rd<int64_t>(p);
    uint32_t w = rd<uint32_t>(p + 8), h = rd<uint32_t>(p + 12);
    uint8_t n = p[16];
    size_t img = static_cast<size_t>(w) * h;
    if (n < 1 || n > 2 || w == 0 || h == 0 || buf.size() != 20 + n * img) {
      out += err_line("STEREO size mismatch");
      return;
    }
    if (n > params_.state_options.num_cameras) n = static_cast<uint8_t>(params_.state_options.num_cameras);
    // 스테레오 설정에서 mono/stereo 를 섞으면 TrackKLT 가 깨진다: mono 프레임은 cam1 의 이전 피라미드를 갱신하지 않으므로
    // 처음 몇 프레임이 mono 이고 그다음 stereo 가 오면 img_pyramid_last[1] 이 빈 벡터인 채로
    // calcOpticalFlowPyrLK(…, img1pyr=빈 벡터) → "levels2 >= 0" 단언 (cv::Exception) → 프로세스 종료.
    // 그래서 use_stereo=true 면 2장짜리만 받는다 (클라이언트도 섞어 보내지 않음).
    if (params_.use_stereo && params_.state_options.num_cameras >= 2 && n != 2) {
      n_cam_rejected_++;
      return;
    }
    // 크기가 바뀌어도 이전 피라미드와 맞지 않아 KLT 가 깨진다 → 처음 크기만 받는다.
    if (img_w_ == 0) {
      img_w_ = w;
      img_h_ = h;
    } else if (w != img_w_ || h != img_h_) {
      n_cam_rejected_++;
      return;
    }
    CamMsg c;
    c.t_ns = t_ns;
    c.t = to_sec(t_ns);
    c.data.timestamp = c.t;
    c.g10 = p[17];
    for (int i = 0; i < n; i++) {
      cv::Mat g(static_cast<int>(h), static_cast<int>(w), CV_8UC1);
      std::memcpy(g.data, p + 20 + i * img, img);
      if (i == 0 && !map_x_.empty() && map_x_.cols == static_cast<int>(w) && map_x_.rows == static_cast<int>(h)) {
        cv::Mat u;
        cv::remap(g, u, map_x_, map_y_, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0));
        g = u;
      }
      c.data.sensor_ids.push_back(i);
      c.data.images.push_back(g);
      if (params_.use_mask && params_.masks.count(i))
        c.data.masks.push_back(params_.masks.at(i));
      else
        c.data.masks.push_back(cv::Mat::zeros(static_cast<int>(h), static_cast<int>(w), CV_8UC1));
    }
    if (!cams_.empty() && c.t <= cams_.back().t) {
      n_cam_dropped_++;
      return;
    }
    if (first_cam_t_ < -1e299) first_cam_t_ = c.t;
    cams_.push_back(std::move(c));
    while (cams_.size() > args_.max_cam_queue) {  // 밀리면 오래된 카메라만 버린다 (IMU 는 안 버림)
      cams_.pop_front();
      n_cam_dropped_++;
    }
    pump(out);
  }

  // IMU 와 카메라를 시간 순서대로 VioManager 에 넣는다 (ROS1Visualizer::callback_inertial 과 같은 규칙 + IMU 붙잡기).
  //  1) 맨 앞 카메라보다 늦은 IMU 가 이미 들어갔으면 (t_cam + dt_cam_imu < last_imu) 그 카메라를 넣는다.
  //  2) 기다리는 카메라가 있으면 그 카메라를 넘을 때까지 IMU 를 넣는다.
  //  3) 기다리는 카메라가 없으면 IMU 를 imu_hold_s 동안 붙잡아 둔다 (늦게 오는 카메라보다 IMU 가 한참 앞서 들어가면
  //     초기화 직후 ZUPT 의 IMU 버퍼가 그 카메라 구간을 못 덮어 "No IMU measurements to propagate with (0 of 2)" 경고가 난다:
  //     ZUPT 는 초기화 뒤에 들어온 IMU 만 갖고 있기 때문).
  void pump(std::string &out) {
    while (true) {
      double dt = sys_->get_state()->_calib_dt_CAMtoIMU->value()(0);
      if (!cams_.empty() && cams_.front().t < last_imu_t_ - dt) {
        CamMsg c = std::move(cams_.front());
        cams_.pop_front();
        feed_camera(c, out);
        continue;
      }
      if (!imu_hold_.empty() && (!cams_.empty() || imu_hold_.front().timestamp < newest_imu_t_ - args_.imu_hold_s)) {
        ov_core::ImuData m = imu_hold_.front();
        imu_hold_.pop_front();
        try {
          sys_->feed_measurement_imu(m);
        } catch (const std::exception &e) {
          on_exception("feed_measurement_imu", e.what(), out);
          continue;
        }
        if (shadow_) {
          try {
            shadow_->feed_measurement_imu(m);
          } catch (const std::exception &e) {
            std::fprintf(stderr, "[ov_bridge] init gate: shadow imu exception %s -> new shadow\n", e.what());
            make_shadow(false);
          }
        }
        last_imu_t_ = m.timestamp;
        imu_recent_.push_back(m);  // 상태 주입 재시작 때 새 VioManager 에 다시 넣을 최근 IMU (2 s)
        while (!imu_recent_.empty() && imu_recent_.front().timestamp < m.timestamp - 2.0) imu_recent_.pop_front();
        continue;
      }
      break;
    }
  }

  // --init-gate: 이 카메라 시각까지 init 창 동안 자이로가 조용했나 (넣은 IMU 기준). why 에 이유.
  bool init_still(double t, const char *&why) {
    double W = params_.init_options.init_window_time + 0.05;
    // 카메라 시작 직후 프레임은 그림자에 아예 안 넣으므로 (feed_shadow) 여기서는 따로 안 본다.
    if (imu_recent_.empty() || imu_recent_.front().timestamp > t - kGateMinImuS) {  // OpenVINS 도 창이 덜 차도 초기화함
      why = "short";
      return false;
    }
    Eigen::Vector3d s = Eigen::Vector3d::Zero();
    double mx = 0.0;
    int n = 0;
    for (const auto &m : imu_recent_) {
      if (m.timestamp < t - W || m.timestamp > t + 0.05) continue;
      s += m.wm;
      mx = std::max(mx, m.wm.norm());
      n++;
    }
    if (n < 10) {
      why = "short";
      return false;
    }
    gate_last_mean_ = (s / n).norm();
    gate_last_max_ = mx;
    if (gate_last_mean_ > args_.gate_mean || mx > args_.gate_max) {
      why = "moving";
      return false;
    }
    return true;
  }

  void feed_camera(const CamMsg &c, std::string &out) {
    auto t0 = std::chrono::steady_clock::now();
    if (args_.zupt_alt_disp > 0 && sys_) {  // G1.0: set this frame's ZUPT disparity threshold before its update
      auto &up = (*sys_).*VioDiag::zupt_member();
      if (up) {
        double want = c.g10 ? args_.zupt_alt_disp : params_.zupt_max_disparity;
        double &cur = (*up).*ZuptAcc::disp_member();
        if (cur != want) std::fprintf(stderr, "[G10] %.6f thr %.3f\n", c.t, want);
        cur = want;
      }
    }
    try {
      sys_->feed_measurement_camera(c.data);
    } catch (const std::exception &e) {  // cv::Exception 도 std::exception
      on_exception("feed_measurement_camera", e.what(), out);
      return;
    }
    double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    // --init-gate: 그림자 VioManager (기본 쪽이 아직 정지 초기화 전일 때만)
    const char *gate_tag = shadow_ ? feed_shadow(c) : nullptr;
    try {
      std::string pl = pose_line(c, ms);
      if (gate_tag && !pl.empty() && pl.back() == '\n' && pl.size() > 2 && pl[pl.size() - 2] == '}')
        pl = pl.substr(0, pl.size() - 2) + ",\"init_gate\":\"" + gate_tag + "\"}\n";
      out += pl;
    } catch (const std::exception &e) {
      on_exception("pose_line", e.what(), out);
      return;
    }
    if (lost_ && args_.auto_reset) {
      std::string extra;
      if (!(args_.reinit_state && reinit_from_state(extra))) make_system();
      out += "{\"type\":\"reset\",\"auto\":true,\"reason\":\"lost\"" + extra + "}\n";
    }
  }

  // --init-gate 그림자 처리 (기본 sys_ 에 카메라를 넣은 직후). 반환: 포즈 줄 표시 ("shadow" = 이번에 그림자로 바꿈) 또는 nullptr.
  const char *feed_shadow(const CamMsg &c) {
    if (sys_->get_state()->_timestamp >= 0.0) {  // 기본 쪽이 스스로 정지 초기화함 → 예전과 같음, 그림자 버림
      std::fprintf(stderr, "[ov_bridge] init gate: default init at t=%.3f -> shadow dropped\n", sys_->get_state()->_timestamp);
      shadow_.reset();
      return nullptr;
    }
    if (first_cam_t_ > -1e299 && c.t - first_cam_t_ < kGateSkipCamS) return nullptr;  // 카메라 시작 직후는 그림자에 안 넣음
    try {
      shadow_->feed_measurement_camera(c.data);
    } catch (const std::exception &e) {
      std::fprintf(stderr, "[ov_bridge] init gate: shadow camera exception %s -> new shadow\n", e.what());
      make_shadow(true);
      return nullptr;
    }
    if (!shadow_checked_ && shadow_->get_state()->_timestamp >= 0.0) {
      shadow_checked_ = true;
      const char *why = "";
      double ti = shadow_->get_state()->_timestamp;
      if (!init_still(ti, why)) {
        n_init_vetoed_++;
        std::fprintf(stderr, "[ov_bridge] init gate: shadow init at t=%.3f vetoed (%s, gyro mean %.4f max %.3f rad/s) #%llu\n",
                     ti, why, gate_last_mean_, gate_last_max_, static_cast<unsigned long long>(n_init_vetoed_));
        make_shadow(true);
        return nullptr;
      }
      shadow_ok_ = true;
      std::fprintf(stderr, "[ov_bridge] init gate: shadow init at t=%.3f ok (gyro mean %.4f max %.3f rad/s)\n", ti,
                   gate_last_mean_, gate_last_max_);
    }
    if (!shadow_ok_) return nullptr;
    // 움직이기 시작했나: 최근 0.1 s 자이로 최대 > MAX, 또는 그림자가 이미 특징 갱신을 시작함
    double mx = 0.0;
    for (auto it = imu_recent_.rbegin(); it != imu_recent_.rend() && it->timestamp > c.t - 0.1; ++it)
      mx = std::max(mx, it->wm.norm());
    if (mx <= args_.gate_max && !shadow_->initialized()) return nullptr;
    n_shadow_used_++;
    std::fprintf(stderr, "[ov_bridge] init gate: default not initialized at motion start (t=%.3f, gyro %.3f rad/s) -> "
                 "using gated shadow init (t=%.3f) #%llu\n", c.t, mx, shadow_->get_state()->_timestamp,
                 static_cast<unsigned long long>(n_shadow_used_));
    sys_ = shadow_;
    shadow_.reset();
    init_checked_ = true;
    zero_feat_frames_ = 0;
    feat_seen_ = false;
    lost_ = false;
    return "shadow";
  }

  // lost → 새 VioManager 에 옛 상태를 넣어 정지 없이 이어 가기. 성공하면 true, extra 에 JSON 필드 (",\"reinit\":\"state\",...").
  bool reinit_from_state(std::string &extra) {
    if (!sys_->initialized() || lost_bad_) return false;
    auto st = sys_->get_state();
    double t = st->_timestamp;
    double gap = t - last_upd_t_;
    std::ostringstream o;
    o.precision(6);
    if (!(gap >= 0.0 && gap <= args_.reinit_max_gap)) {
      o << ",\"reinit\":\"static\",\"gap\":";
      jnum(o, gap);
      extra = o.str();
      return false;
    }
    Eigen::Matrix<double, 16, 1> imu = st->_imu->value();
    Eigen::Matrix<double, 7, 1> ext = st->_calib_IMUtoCAM.at(0)->value();
    Eigen::VectorXd dt = st->_calib_dt_CAMtoIMU->value();
    if (!imu.allFinite() || !ext.allFinite() || !dt.allFinite()) return false;
    make_system();
    auto ns = sys_->get_state();
    ns->_calib_IMUtoCAM.at(0)->set_value(ext);
    ns->_calib_IMUtoCAM.at(0)->set_fej(ext);
    ns->_calib_dt_CAMtoIMU->set_value(dt);
    ns->_calib_dt_CAMtoIMU->set_fej(dt);
    Eigen::Matrix<double, 17, 1> s;
    s(0) = t;
    s.block(1, 0, 16, 1) = imu;
    sys_->initialize_with_gt(s);
    shadow_.reset();       // 상태 주입은 정지 초기화가 아님 (--init-gate 그림자 필요 없음)
    init_checked_ = true;
    // initialize_with_gt 의 공분산(속도 0.01 m/s 등)은 정지 가정이라 너무 작다 → IMU 만 전파한 뒤의 불확실성으로 키움
    std::vector<std::shared_ptr<ov_type::Type>> order = {ns->_imu};
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(15, 15);
    C.block(0, 0, 3, 3) = std::pow(args_.reinit_sig_ori_deg * M_PI / 180.0, 2) * Eigen::Matrix3d::Identity();
    C.block(3, 3, 3, 3) = std::pow(args_.reinit_sig_pos, 2) * Eigen::Matrix3d::Identity();
    C.block(6, 6, 3, 3) = std::pow(args_.reinit_sig_vel, 2) * Eigen::Matrix3d::Identity();
    C.block(9, 9, 3, 3) = std::pow(0.01, 2) * Eigen::Matrix3d::Identity();
    C.block(12, 12, 3, 3) = std::pow(0.1, 2) * Eigen::Matrix3d::Identity();
    ov_msckf::StateHelper::set_initial_covariance(ns, C, order);
    // 옛 시스템에 이미 넣은 최근 IMU 를 다시 넣는다 (다음 카메라까지 전파할 IMU 가 새 시스템에 없으므로)
    for (const auto &m : imu_recent_) {
      if (m.timestamp < t - 0.5) continue;
      sys_->feed_measurement_imu(m);
      last_imu_t_ = m.timestamp;
    }
    n_reinit_state_++;
    o << ",\"reinit\":\"state\",\"gap\":";
    jnum(o, gap);
    o << ",\"t_state\":";
    jnum(o, t);
    o << ",\"v_norm\":";
    jnum(o, imu.block(7, 0, 3, 1).norm());  // IMU 값 = [q(4) p(3) v(3) bg(3) ba(3)]
    o << ",\"n_reinit_state\":" << n_reinit_state_;
    extra = o.str();
    std::fprintf(stderr, "[ov_bridge] lost -> reinit from state t=%.3f gap=%.2f s |v|=%.2f (#%llu)\n", t, gap,
                 imu.block(7, 0, 3, 1).norm(), static_cast<unsigned long long>(n_reinit_state_));
    return true;
  }

  std::string pose_line(const CamMsg &c, double ms) {
    auto st = sys_->get_state();
    bool init = sys_->initialized();
    size_t n_slam = 0, n_msckf = 0;
    std::vector<Eigen::Vector3d> slam;
    if (init) {
      slam = sys_->get_features_SLAM();
      n_slam = slam.size();
      n_msckf = sys_->get_good_features_MSCKF().size();
    }
    std::ostringstream o;
    o.precision(9);
    o << "{\"type\":\"pose\",\"t_ns\":" << c.t_ns << ",\"t\":";
    jnum(o, c.t);
    o << ",\"mono\":" << (c.data.sensor_ids.size() == 1 ? "true" : "false");
    o << ",\"t_state\":";  // OpenVINS state time (s, same base as t) -> client flags stale poses (REVIEW 9)
    jnum(o, st->_timestamp);
    if (!init) {
      o << ",\"status\":\"init\",\"init\":false,\"T_world_cam0\":null";
    } else {
      Eigen::Vector4d q_GtoI = st->_imu->quat();  // JPL
      Eigen::Matrix3d R_GtoI = ov_core::quat_2_Rot(q_GtoI);
      Eigen::Matrix3d R_ItoG = R_GtoI.transpose();
      Eigen::Vector3d p_IinG = st->_imu->pos(), v = st->_imu->vel();
      Eigen::Quaterniond qh(R_ItoG);  // Hamilton, world <- IMU
      auto calib0 = st->_calib_IMUtoCAM.at(0);
      Eigen::Matrix3d R_ItoC = calib0->Rot();
      Eigen::Vector3d p_IinC = calib0->pos();
      Eigen::Matrix3d R_CtoG = R_ItoG * R_ItoC.transpose();
      Eigen::Vector3d p_CinG = p_IinG - R_CtoG * p_IinC;
      Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
      T.block<3, 3>(0, 0) = R_CtoG;
      T.block<3, 1>(0, 3) = p_CinG;

      bool bad = !p_IinG.allFinite() || !v.allFinite() || !R_CtoG.allFinite() || !T.allFinite() ||
                 v.norm() > args_.max_speed;
      zero_feat_frames_ = (n_slam + n_msckf == 0) ? zero_feat_frames_ + 1 : 0;
      if (n_slam + n_msckf > 0) {
        feat_seen_ = true;
        last_upd_t_ = st->_timestamp;  // 마지막으로 영상이 상태를 고친 시각 (상태 주입 재시작의 gap 기준)
      }
      lost_bad_ = bad;
      // 초기화 직후(정지 ZUPT 가 끝나고 처음 움직일 때)는 clone 창이 차고 시차가 생길 때까지 특징 갱신이 0 인 게 정상이다
      // (rec_20261016_143702: 10~14 프레임, 천천히 움직이기 시작하면 더 길다). 첫 특징 갱신 전에는 startup_lost_frames 까지 기다린다.
      int lim = feat_seen_ ? args_.lost_frames : std::max(args_.lost_frames, args_.startup_lost_frames);
      lost_ = bad || zero_feat_frames_ >= lim;

      o << ",\"status\":\"" << (lost_ ? "lost" : "tracking") << "\",\"init\":true";
      o << ",\"p\":";
      jarr(o, p_IinG);
      o << ",\"q\":[";
      jnum(o, qh.x()); o << ","; jnum(o, qh.y()); o << ","; jnum(o, qh.z()); o << ","; jnum(o, qh.w());
      o << "],\"v\":";
      jarr(o, v);
      o << ",\"T_world_cam0\":";
      jarr(o, T);
      o << ",\"q_GtoI_jpl\":";
      jarr(o, q_GtoI);
      o << ",\"bg\":";
      jarr(o, st->_imu->bias_g());
      o << ",\"ba\":";
      jarr(o, st->_imu->bias_a());
      o << ",\"R_ItoC0\":";
      jarr(o, R_ItoC);
      o << ",\"p_IinC0\":";
      jarr(o, p_IinC);
      if (args_.diag) {  // 순서 = OpenVINS IMU 오차 상태 (자세 3, 위치 3, 속도 3, bg 3, ba 3)
        std::vector<std::shared_ptr<ov_type::Type>> vars = {st->_imu};
        Eigen::MatrixXd C = ov_msckf::StateHelper::get_marginal_covariance(st, vars);
        Eigen::VectorXd sg = C.diagonal().cwiseMax(0.0).cwiseSqrt();
        o << ",\"sig15\":";
        jarr(o, sg);
        o << ",\"n_track\":" << static_cast<VioDiag *>(sys_.get())->n_track();
      }
    }
    o << ",\"dt_cam_imu\":";
    jnum(o, st->_calib_dt_CAMtoIMU->value()(0));
    o << ",\"n_slam\":" << n_slam << ",\"n_msckf\":" << n_msckf << ",\"ms\":";
    jnum(o, ms);
    o << ",\"cam_dropped\":" << n_cam_dropped_ << ",\"imu_dropped\":" << n_imu_dropped_
      << ",\"cam_rejected\":" << n_cam_rejected_ << ",\"exceptions\":" << n_exceptions_
      << ",\"imu_held\":" << imu_hold_.size() << ",\"n_reinit_state\":" << n_reinit_state_;
    if (args_.emit_features > 0 && init) {
      o << ",\"slam_pts\":[";
      size_t k = std::min(slam.size(), static_cast<size_t>(args_.emit_features));
      for (size_t i = 0; i < k; i++) {
        if (i) o << ",";
        jarr(o, slam[i]);
      }
      o << "]";
    }
    o << "}\n";
    return o.str();
  }

  Args args_;
  cv::Mat map_x_, map_y_;  // --remap (CV_16SC2 + CV_16UC1 고정소수 맵, 빠름)

  void load_remap(const std::string &path) {
    FILE *f = std::fopen(path.c_str(), "rb");
    int32_t wh[2] = {0, 0};
    if (!f || std::fread(wh, 4, 2, f) != 2 || wh[0] <= 0 || wh[1] <= 0) {
      std::fprintf(stderr, "[ov_bridge] remap: cannot read %s\n", path.c_str());
      if (f) std::fclose(f);
      return;
    }
    cv::Mat mx(wh[1], wh[0], CV_32FC1), my(wh[1], wh[0], CV_32FC1);
    size_t n = static_cast<size_t>(wh[0]) * wh[1];
    bool ok = std::fread(mx.data, 4, n, f) == n && std::fread(my.data, 4, n, f) == n;
    std::fclose(f);
    if (!ok) {
      std::fprintf(stderr, "[ov_bridge] remap: short file %s\n", path.c_str());
      return;
    }
    cv::convertMaps(mx, my, map_x_, map_y_, CV_16SC2);
    std::fprintf(stderr, "[ov_bridge] remap %dx%d from %s\n", wh[0], wh[1], path.c_str());
  }

  bool ok_ = true;
  std::shared_ptr<ov_core::YamlParser> parser_;
  ov_msckf::VioManagerOptions params_;
  std::shared_ptr<ov_msckf::VioManager> sys_;
  std::deque<CamMsg> cams_;
  bool have_t0_ = false;
  int64_t t0_ns_ = 0;
  double last_imu_t_ = -1e300;    // VioManager 에 마지막으로 넣은 IMU 시각
  double newest_imu_t_ = -1e300;  // 받은 IMU 중 가장 새 시각 (붙잡아 둔 것 포함)
  std::deque<ov_core::ImuData> imu_hold_;
  uint32_t img_w_ = 0, img_h_ = 0;
  int zero_feat_frames_ = 0;
  bool feat_seen_ = false;  // 이 VioManager 가 특징 갱신(SLAM/MSCKF > 0)을 한 번이라도 했는지
  bool lost_ = false;
  bool lost_bad_ = false;           // lost 원인이 속도 초과·NaN (상태 주입 재시작 안 함)
  double last_upd_t_ = -1e300;      // 마지막 특징 갱신 시각 (make_system 이 지우지 않음: 주입 뒤 또 끊기면 gap 이 커져 정지 초기화로)
  std::deque<ov_core::ImuData> imu_recent_;
  uint64_t n_reinit_state_ = 0;
  uint64_t n_init_vetoed_ = 0;
  bool init_checked_ = false;  // 이 VioManager 의 초기화를 --init-gate 로 확인했나
  bool use_shadow_ = false;    // --init-gate 켜짐
  ov_msckf::VioManagerOptions shadow_params_;           // 그림자 설정 (넓힌 시차 기준)
  std::shared_ptr<ov_msckf::VioManager> shadow_;        // 초기화 전에만 있음
  bool shadow_checked_ = false, shadow_ok_ = false;     // 그림자 초기화를 자이로로 확인했나 / 통과했나
  uint64_t n_shadow_used_ = 0;
  double gate_last_mean_ = 0.0, gate_last_max_ = 0.0;
  double first_cam_t_ = -1e300;                   // 받은 첫 카메라 시각 (--init-gate). RESET 때 지움
  static constexpr double kGateSkipCamS = 0.8;   // --init-gate: 카메라 시작 뒤 이 시간 동안의 프레임은 그림자에 안 넣음
  static constexpr double kGateMinImuS = 0.8;    // --init-gate: 확인에 쓸 IMU 최소 길이 (s)
  uint64_t n_cam_dropped_ = 0, n_imu_dropped_ = 0, n_cam_rejected_ = 0, n_exceptions_ = 0;
};

// 한 연결(또는 stdin) 을 끝까지 처리. 반환 false = QUIT 받음.
bool serve(Bridge &b, int in_fd, int out_fd, bool is_socket) {
  if (!write_full(out_fd, b.ready_line(), is_socket)) return true;
  std::vector<uint8_t> buf;
  std::string out;
  uint32_t hdr[3];
  while (read_full(in_fd, hdr, sizeof(hdr))) {
    if (hdr[0] != kMagic || hdr[2] > kMaxPayload) {
      write_full(out_fd, "{\"type\":\"error\",\"msg\":\"bad header, closing\"}\n", is_socket);
      return true;
    }
    buf.resize(hdr[2]);
    if (hdr[2] && !read_full(in_fd, buf.data(), hdr[2])) break;
    out.clear();
    bool cont = true;
    try {
      cont = b.handle(hdr[1], buf, out);
    } catch (const std::exception &e) {  // 마지막 방어선: 어떤 예외로도 프로세스가 죽지 않게
      std::fprintf(stderr, "[ov_bridge] unhandled exception: %s\n", e.what());
      out += "{\"type\":\"error\",\"msg\":" + jstr(std::string("exception: ") + e.what()) + "}\n";
    }
    if (!out.empty() && !write_full(out_fd, out, is_socket)) return true;
    if (!cont) return false;
  }
  return true;
}

void usage() {
  std::fprintf(stderr,
               "usage: ov_bridge <estimator_config.yaml> [--stdio | --tcp HOST:PORT] [--max-cam-queue N]\n"
               "                 [--max-speed M_S] [--lost-frames N] [--startup-lost-frames N] [--auto-reset] [--emit-features N]\n"
               "                 [--imu-hold S] [--reinit-state] [--reinit-max-gap S] [--reinit-sigma ORI_DEG,POS_M,VEL_M_S] [--remap FILE]\n"
               "                 [--init-gate MEAN_RAD_S,MAX_RAD_S,DISP_DEG]\n");
}

}  // namespace

int main(int argc, char **argv) {
  signal(SIGPIPE, SIG_IGN);
  if (argc < 2) {
    usage();
    return 2;
  }
  Args a;
  a.config = argv[1];
  for (int i = 2; i < argc; i++) {
    std::string s = argv[i];
    auto next = [&]() -> std::string {
      if (i + 1 >= argc) {
        usage();
        std::exit(2);
      }
      return argv[++i];
    };
    if (s == "--stdio") a.tcp = false;
    else if (s == "--tcp") {
      a.tcp = true;
      std::string hp = next();
      auto c = hp.rfind(':');
      if (c == std::string::npos) a.port = std::stoi(hp);
      else {
        a.host = hp.substr(0, c);
        a.port = std::stoi(hp.substr(c + 1));
      }
    } else if (s == "--max-cam-queue") a.max_cam_queue = static_cast<size_t>(std::max(1, std::stoi(next())));
    else if (s == "--max-speed") a.max_speed = std::stod(next());
    else if (s == "--lost-frames") a.lost_frames = std::stoi(next());
    else if (s == "--startup-lost-frames") a.startup_lost_frames = std::stoi(next());
    else if (s == "--auto-reset") a.auto_reset = true;
    else if (s == "--emit-features") a.emit_features = std::stoi(next());
    else if (s == "--diag") a.diag = true;
    else if (s == "--zupt-alt-disp") a.zupt_alt_disp = std::stod(next());
    else if (s == "--imu-hold") a.imu_hold_s = std::max(0.0, std::stod(next()));
    else if (s == "--reinit-state") a.reinit_state = true;
    else if (s == "--remap") a.remap = next();
    else if (s == "--reinit-max-gap") a.reinit_max_gap = std::stod(next());
    else if (s == "--reinit-sigma") {  // ori_deg,pos_m,vel_m_s
      std::string v = next();
      if (std::sscanf(v.c_str(), "%lf,%lf,%lf", &a.reinit_sig_ori_deg, &a.reinit_sig_pos, &a.reinit_sig_vel) != 3) {
        usage();
        return 2;
      }
    } else if (s == "--init-gate") {  // mean,max rad/s, disparity deg
      std::string v = next();
      if (std::sscanf(v.c_str(), "%lf,%lf,%lf", &a.gate_mean, &a.gate_max, &a.gate_disp_deg) != 3) {
        usage();
        return 2;
      }
    } else {
      usage();
      return 2;
    }
  }

  // OpenVINS 는 printf 로 stdout 에 로그를 쓴다 -> 결과 채널과 섞이지 않게 fd1 을 stderr 로.
  int out_fd = ::dup(STDOUT_FILENO);
  std::fflush(stdout);
  ::dup2(STDERR_FILENO, STDOUT_FILENO);

  Bridge b(a);
  if (!b.ok()) return 1;

  if (!a.tcp) {
    serve(b, STDIN_FILENO, out_fd, false);
    return 0;
  }

  int srv = ::socket(AF_INET, SOCK_STREAM, 0);
  int one = 1;
  ::setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(a.port));
  if (::inet_pton(AF_INET, a.host.c_str(), &addr.sin_addr) != 1) {
    std::fprintf(stderr, "[ov_bridge] bad host %s\n", a.host.c_str());
    return 2;
  }
  if (::bind(srv, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0 || ::listen(srv, 1) < 0) {
    std::perror("[ov_bridge] bind/listen");
    return 1;
  }
  std::fprintf(stderr, "[ov_bridge] listening on %s:%d\n", a.host.c_str(), a.port);
  // 클라이언트 하나씩. 연결이 끊겨도 VIO 상태는 유지 (새 세션이면 클라이언트가 RESET 을 보낸다).
  while (true) {
    int c = ::accept(srv, nullptr, nullptr);
    if (c < 0) {
      if (errno == EINTR) continue;
      std::perror("[ov_bridge] accept");
      break;
    }
    ::setsockopt(c, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    bool cont = serve(b, c, c, true);
    ::close(c);
    if (!cont) break;
  }
  ::close(srv);
  return 0;
}
