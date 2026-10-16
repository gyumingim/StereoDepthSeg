// ov_bridge — OpenVINS (ROS 없음) VioManager 를 바이너리 레코드로 구동하는 작은 실행 파일.
// 프로토콜: PROTOCOL.md. 입력 = stdin 또는 TCP, 출력 = 같은 채널로 JSON 한 줄씩.
//
//   ov_bridge <estimator_config.yaml> [--stdio | --tcp HOST:PORT] [--max-cam-queue N]
//             [--max-speed M_S] [--lost-frames N] [--auto-reset] [--emit-features N]
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

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "state/State.h"
#include "types/IMU.h"
#include "types/PoseJPL.h"
#include "types/Vec.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/print.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

namespace {

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
  bool auto_reset = false;
  int emit_features = 0;    // JSON 에 SLAM 특징점 최대 N 개
};

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
    if (!parser_->successful()) {
      PRINT_ERROR("[ov_bridge] unable to parse all parameters in %s\n", a.config.c_str());
      ok_ = false;
      return;
    }
    make_system();
  }

  bool ok() const { return ok_; }

  void make_system() {
    sys_ = std::make_shared<ov_msckf::VioManager>(params_);
    cams_.clear();
    last_imu_t_ = -1e300;
    zero_feat_frames_ = 0;
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
      << ",\"use_stereo\":" << (params_.use_stereo ? "true" : "false") << "}\n";
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
    std::ostringstream o;
    o << "{\"type\":\"error\",\"msg\":\"" << m << "\"}\n";
    return o.str();
  }

  void on_imu(const uint8_t *p, std::string &out) {
    int64_t t_ns = rd<int64_t>(p);
    double t = to_sec(t_ns);
    if (t <= last_imu_t_) {  // 역행/중복은 버림
      n_imu_dropped_++;
      return;
    }
    ov_core::ImuData m;
    m.timestamp = t;
    m.wm << rd<double>(p + 8), rd<double>(p + 16), rd<double>(p + 24);
    m.am << rd<double>(p + 32), rd<double>(p + 40), rd<double>(p + 48);
    sys_->feed_measurement_imu(m);
    last_imu_t_ = t;
    process_cams(out);
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
    CamMsg c;
    c.t_ns = t_ns;
    c.t = to_sec(t_ns);
    c.data.timestamp = c.t;
    for (int i = 0; i < n; i++) {
      cv::Mat g(static_cast<int>(h), static_cast<int>(w), CV_8UC1);
      std::memcpy(g.data, p + 20 + i * img, img);
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
    cams_.push_back(std::move(c));
    while (cams_.size() > args_.max_cam_queue) {  // 밀리면 오래된 카메라만 버린다 (IMU 는 안 버림)
      cams_.pop_front();
      n_cam_dropped_++;
    }
    process_cams(out);
  }

  // ROS1Visualizer::callback_inertial 과 같은 규칙: 카메라 시각(IMU 시계로) 보다 늦은 IMU 가 있어야 feed.
  void process_cams(std::string &out) {
    while (!cams_.empty()) {
      double dt = sys_->get_state()->_calib_dt_CAMtoIMU->value()(0);
      if (!(cams_.front().t < last_imu_t_ - dt)) break;
      CamMsg c = std::move(cams_.front());
      cams_.pop_front();
      auto t0 = std::chrono::steady_clock::now();
      sys_->feed_measurement_camera(c.data);
      double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
      out += pose_line(c, ms);
      if (lost_ && args_.auto_reset) {
        make_system();
        out += "{\"type\":\"reset\",\"auto\":true}\n";
        break;
      }
    }
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
      lost_ = bad || zero_feat_frames_ >= args_.lost_frames;

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
    }
    o << ",\"dt_cam_imu\":";
    jnum(o, st->_calib_dt_CAMtoIMU->value()(0));
    o << ",\"n_slam\":" << n_slam << ",\"n_msckf\":" << n_msckf << ",\"ms\":";
    jnum(o, ms);
    o << ",\"cam_dropped\":" << n_cam_dropped_ << ",\"imu_dropped\":" << n_imu_dropped_;
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
  bool ok_ = true;
  std::shared_ptr<ov_core::YamlParser> parser_;
  ov_msckf::VioManagerOptions params_;
  std::shared_ptr<ov_msckf::VioManager> sys_;
  std::deque<CamMsg> cams_;
  bool have_t0_ = false;
  int64_t t0_ns_ = 0;
  double last_imu_t_ = -1e300;
  int zero_feat_frames_ = 0;
  bool lost_ = false;
  uint64_t n_cam_dropped_ = 0, n_imu_dropped_ = 0;
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
    bool cont = b.handle(hdr[1], buf, out);
    if (!out.empty() && !write_full(out_fd, out, is_socket)) return true;
    if (!cont) return false;
  }
  return true;
}

void usage() {
  std::fprintf(stderr,
               "usage: ov_bridge <estimator_config.yaml> [--stdio | --tcp HOST:PORT] [--max-cam-queue N]\n"
               "                 [--max-speed M_S] [--lost-frames N] [--auto-reset] [--emit-features N]\n");
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
    else if (s == "--auto-reset") a.auto_reset = true;
    else if (s == "--emit-features") a.emit_features = std::stoi(next());
    else {
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
