// ovopt.cpp — option storage + parser + profiler slots for ovpatch/ovopt.h (2026-10-25).
#include "ovopt.h"

#include <cstdlib>
#include <sstream>

namespace ovopt {

Flags g;
const float *rs_mapy = nullptr;
int rs_map_w = 0, rs_map_h = 0, rs_src_h = 0;
RsStats rs_stats;
Health h;  // 2026-10-25 ovrobust health counters
double prof_ms[N_SLOTS] = {0};
long prof_cnt[N_COUNTS] = {0};

namespace {
struct Field {
  const char *name;
  int Flags::*ptr;
  bool exact;
};
const Field kFields[] = {
    {"skip_retri", &Flags::skip_retri, true}, {"par_tri", &Flags::par_tri, true},
    {"par_jac", &Flags::par_jac, true},       {"par_ma", &Flags::par_ma, true},
    {"par_s", &Flags::par_s, true},
    {"cov_move", &Flags::cov_move, true},     {"frame_nocopy", &Flags::frame_nocopy, true},
    {"givens_rm", &Flags::givens_rm, true},
    {"ekf_gemm", &Flags::ekf_gemm, false},
    {"qr_compress", &Flags::qr_compress, false}, {"ekf_chol", &Flags::ekf_chol, false},
    {"par_cov", &Flags::par_cov, false},      {"prof", &Flags::prof, false},
    {"rs_derot", &Flags::rs_derot, false},    {"rs_row_ppm", &Flags::rs_row_ppm, false},
    {"rs_postinit", &Flags::rs_postinit, false},
};
} // namespace

bool parse(const std::string &list, std::string &err) {
  std::stringstream ss(list);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    if (tok.empty() || tok == "none" || tok == "0")
      continue;
    std::string name = tok;
    int val = 1;
    auto eq = tok.find('=');
    if (eq != std::string::npos) {
      name = tok.substr(0, eq);
      val = std::atoi(tok.c_str() + eq + 1);
    }
    if (name == "all_exact") {
      for (const auto &f : kFields)
        if (f.exact)
          g.*(f.ptr) = val;
      continue;
    }
    bool found = false;
    for (const auto &f : kFields) {
      if (name == f.name) {
        g.*(f.ptr) = val;
        found = true;
        break;
      }
    }
    if (!found) {
      err = "unknown --ovopt option '" + name + "'";
      return false;
    }
  }
  return true;
}

std::string describe() {
  std::ostringstream o;
  bool first = true;
  for (const auto &f : kFields) {
    o << (first ? "" : " ") << f.name << "=" << g.*(f.ptr);
    first = false;
  }
  return o.str();
}

const char *slot_name(int s) {
  static const char *n[N_SLOTS] = {"track",     "prop",      "select",     "msckf_clean", "msckf_tri", "msckf_jac",
                                   "msckf_comp", "msckf_ekf", "slam_marg",  "slam_jac",    "slam_ekf",  "delay_tri",
                                   "delay_init", "retri",     "db_clean",   "anchor",      "marg_clone", "ekf_ma",
                                   "ekf_s",      "ekf_k",     "ekf_cov",    "ekf_sym",     "ekf_dx",    "rs"};
  return (s >= 0 && s < N_SLOTS) ? n[s] : "?";
}
const char *count_name(int c) {
  static const char *n[N_COUNTS] = {"ekf_calls", "ekf_rows", "cov_n", "msckf_feats", "msckf_rows", "msckf_cols", "slam_up", "slam_delay"};
  return (c >= 0 && c < N_COUNTS) ? n[c] : "?";
}
void prof_reset() {
  for (int i = 0; i < N_SLOTS; i++)
    prof_ms[i] = 0.0;
  for (int i = 0; i < N_COUNTS; i++)
    prof_cnt[i] = 0;
}

} // namespace ovopt

// ---- ovpipe (2026-10-26, branch ovpipe-20261026): see ovpipe.h ----
#include "ovpipe.h"

#include <cstdint>
#include <cstdio>
#include <cstring>

#include <opencv2/imgproc.hpp>

#include <sched.h>

namespace ovpipe {

Pipe P;
thread_local bool tl_worker = false;
thread_local std::shared_ptr<Res> tl_base;
thread_local std::shared_ptr<Res> tl_out;
thread_local uint64_t tl_seq = 0;
thread_local uint64_t tl_gen = 0;
std::shared_ptr<ov_core::TrackBase> last_tracker;

namespace {
int64_t mono_ns_now() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
double ms_since(std::chrono::steady_clock::time_point t0) {
  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}
} // namespace

void Pipe::start() {
  {
    std::lock_guard<std::mutex> lk(m_);
    stop_ = false;
  }
  th_ = std::thread([this] { run(); });
}

void Pipe::stop() {
  {
    std::lock_guard<std::mutex> lk(m_);
    stop_ = true;
  }
  cv_.notify_all();
  if (th_.joinable())
    th_.join();
}

void Pipe::set_tracker(std::shared_ptr<void> keep, std::function<void(const cv::Mat &, const cv::Mat &)> track,
                       const void *tgt) {
  std::lock_guard<std::mutex> lk(m_);
  gen_++;
  trk_ = std::move(keep);
  track_ = std::move(track);
  com_.reset();
  com_wait_ = false;
  target.store(tgt);
  rechain_after(SIZE_MAX);
}

void Pipe::rechain_after(size_t idx) {
  s_.rechains++;
  for (size_t i = (idx == SIZE_MAX ? 0 : idx + 1); i < jobs_.size(); i++) {
    Job &j = *jobs_[i];
    if (j.st == J_DONE || j.st == J_RUNNING) {
      j.st = J_PENDING;
      j.res.reset();
    }
  }
  cv_.notify_all();
}

void Pipe::submit(uint64_t seq, const uint8_t *src, size_t len) {
  auto j = std::make_shared<Job>();
  j->seq = seq;
  j->src = src;
  j->len = len;
  j->t_sub = mono_ns_now();
  std::lock_guard<std::mutex> lk(m_);
  jobs_.push_back(std::move(j));
  s_.submitted++;
  cv_.notify_all();
}

void Pipe::remap_job(Job &j) {
  // same checks and the same image as Bridge::on_stereo (cam0 only)
  const uint8_t *p = j.src;
  if (!p || j.len < 20)
    return;
  uint32_t w, h;
  std::memcpy(&j.t_ns, p, 8);
  std::memcpy(&w, p + 8, 4);
  std::memcpy(&h, p + 12, 4);
  const uint8_t n = p[16];
  const size_t img = static_cast<size_t>(w) * h;
  if (n < 1 || n > 2 || w == 0 || h == 0 || j.len != 20 + n * img)
    return;
  const bool do_remap =
      !map_x.empty() && (remap_src_w > 0 ? (remap_src_w == static_cast<int>(w) && remap_src_h == static_cast<int>(h))
                                         : (map_x.cols == static_cast<int>(w) && map_x.rows == static_cast<int>(h)));
  cv::Mat g;
  if (do_remap && frame_nocopy) {
    g = cv::Mat(static_cast<int>(h), static_cast<int>(w), CV_8UC1, const_cast<uint8_t *>(p + 20));
  } else {
    g.create(static_cast<int>(h), static_cast<int>(w), CV_8UC1);
    std::memcpy(g.data, p + 20, img);
  }
  if (do_remap) {
    auto r0 = std::chrono::steady_clock::now();
    cv::Mat u;
    cv::remap(g, u, map_x, map_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0));
    g = u;
    j.remap_ms = ms_since(r0);
  }
  j.img = g;
}

bool Pipe::get_image(uint64_t seq, cv::Mat &img, double &remap_ms) {
  std::unique_lock<std::mutex> lk(m_);
  std::shared_ptr<Job> j;
  for (auto &x : jobs_)
    if (x->seq == seq) {
      j = x;
      break;
    }
  if (!j)
    return false;
  if (!j->img_ok && !j->img_bad && !j->remapping && j->src) {
    // not started yet: the main thread does it now itself (the worker may be busy tracking)
    j->remapping = true;
    j->remap_by = 2;
    j->t_rs = mono_ns_now();
    lk.unlock();
    remap_job(*j);
    lk.lock();
    j->t_re = mono_ns_now();
    j->remapping = false;
    j->img_ok = !j->img.empty();
    j->img_bad = !j->img_ok;
    cv_.notify_all();
  } else if (j->remapping) {
    auto t0 = std::chrono::steady_clock::now();
    while (j->remapping)
      cv_.wait(lk);
    s_.img_waits++;
    s_.img_wait_ms += ms_since(t0);
  }
  if (!j->img_ok)
    return false;
  img = j->img;
  remap_ms = j->remap_ms;
  return true;
}

void Pipe::drop(uint64_t seq) {
  std::lock_guard<std::mutex> lk(m_);
  for (size_t i = 0; i < jobs_.size(); i++) {
    Job &j = *jobs_[i];
    if (j.seq != seq)
      continue;
    if (j.st != J_DROPPED) {
      const int was = j.st;
      j.st = J_DROPPED;
      j.res.reset();
      s_.drops++;
      if (was == J_DONE || was == J_RUNNING)
        rechain_after(i);
    }
    cv_.notify_all();
    return;
  }
}

void Pipe::release(uint64_t seq) {
  std::unique_lock<std::mutex> lk(m_);
  for (auto &x : jobs_) {
    if (x->seq != seq)
      continue;
    while (x->remapping)
      cv_.wait(lk);
    x->src = nullptr;
    if (!x->img_ok)
      x->img_bad = true;
    cv_.notify_all();
    return;
  }
}

std::shared_ptr<Res> Pipe::claim(uint64_t seq, int nf) {
  std::unique_lock<std::mutex> lk(m_);
  if (seq == 0) {
    s_.nojob++;
    com_wait_ = true;
    return nullptr;
  }
  // records before this one were never fed: if one of them was part of the worker's chain, re-chain
  bool broken = false;
  while (!jobs_.empty() && jobs_.front()->seq < seq) {
    Job &j = *jobs_.front();
    if (j.st == J_DONE || j.st == J_RUNNING)
      broken = true;
    j.st = J_DROPPED;
    jobs_.pop_front();
  }
  if (broken)
    rechain_after(SIZE_MAX);
  if (jobs_.empty() || jobs_.front()->seq != seq || jobs_.front()->st == J_DROPPED) {
    s_.nojob++;
    com_wait_ = true;
    return nullptr;
  }
  std::shared_ptr<Job> j = jobs_.front();
  const int64_t tc0 = mono_ns_now();
  const int st0 = j->st;
  if (j->st == J_PENDING || j->st == J_RUNNING) {
    auto t0 = std::chrono::steady_clock::now();
    while (j->st == J_PENDING || j->st == J_RUNNING) {
      if (j->st == J_PENDING && (!j->img_ok || !track_)) {  // nothing the worker can do for it -> synchronous
        jobs_.pop_front();
        s_.nojob++;
        com_wait_ = true;
        return nullptr;
      }
      cv_.wait(lk);
    }
    const double w = ms_since(t0);
    s_.claim_waited++;
    s_.claim_wait_ms += w;
    if (w > s_.max_claim_wait_ms)
      s_.max_claim_wait_ms = w;
  }
  jobs_.pop_front();
  std::shared_ptr<Res> r = j->res;
  const uint64_t cid = com_ ? com_->id : 0;
  if (log) {  // --pipe-log: seq,state at claim,claim wait ms,submit->claim ms,remap_by,remap start/len ms (from submit),
              // track start/len ms (from submit),tracks,hit
    const int64_t tc1 = mono_ns_now();
    auto rel = [&](int64_t t) { return t ? (t - j->t_sub) / 1e6 : -1.0; };
    std::fprintf(log, "%llu,%d,%.3f,%.3f,%d,%.3f,%.3f,%.3f,%.3f,%d,%d\n", (unsigned long long)seq, st0, (tc1 - tc0) / 1e6,
                 rel(tc0), j->remap_by, rel(j->t_rs), j->t_re ? (j->t_re - j->t_rs) / 1e6 : -1.0, rel(j->t_ts),
                 j->t_te ? (j->t_te - j->t_ts) / 1e6 : -1.0, j->tracks,
                 (r && r->gen == gen_ && r->base == cid && r->nf == nf) ? 1 : 0);
  }
  // the commit follows right away: from now on the worker chains on r (hit) or waits for the synchronous result (miss)
  com_wait_ = true;
  if (!r) {
    s_.nores++;
    return nullptr;
  }
  if (r->gen != gen_) {
    s_.miss_gen++;
    return nullptr;
  }
  if (r->base != cid) {
    s_.miss_base++;
    return nullptr;
  }
  if (r->nf != nf) {
    s_.miss_nf++;
    return nullptr;
  }
  s_.hits++;
  com_ = r;
  com_wait_ = false;
  return r;
}

void Pipe::committed(const std::shared_ptr<Res> &r, bool fresh) {
  std::lock_guard<std::mutex> lk(m_);
  com_ = r;
  com_wait_ = false;
  if (fresh)
    rechain_after(SIZE_MAX);  // results computed on another base than r
  cv_.notify_all();
}

void Pipe::run() {
  if (!cpus.empty()) {  // --pipe-cpus
    cpu_set_t set;
    CPU_ZERO(&set);
    for (int c : cpus)
      CPU_SET(c, &set);
    const int rc = sched_setaffinity(0, sizeof(set), &set);
    std::fprintf(stderr, "[ov_bridge] pipe: worker affinity %zu cpus rc %d\n", cpus.size(), rc);
  }
  while (true) {
    std::shared_ptr<Job> j;
    bool do_remap = false;
    std::shared_ptr<Res> base;
    uint64_t g = 0;
    std::function<void(const cv::Mat &, const cv::Mat &)> fn;
    std::shared_ptr<void> keep;
    {
      std::unique_lock<std::mutex> lk(m_);
      while (true) {
        if (stop_)
          return;
        for (auto &x : jobs_)  // 1) remap of new records first (cheap, the main thread needs the image early)
          if (x->st != J_DROPPED && x->src && !x->img_ok && !x->img_bad && !x->remapping) {
            j = x;
            do_remap = true;
            break;
          }
        if (j) {
          j->remapping = true;
          j->remap_by = 1;
          j->t_rs = mono_ns_now();
          break;
        }
        if (track_) {  // 2) tracking of the first pending frame, chained on the last done one (or the committed state)
          std::shared_ptr<Res> prev;
          for (auto &x : jobs_) {
            if (x->st == J_DROPPED)
              continue;
            if (x->st == J_DONE) {
              if (!x->res)
                break;  // failed precompute: the main thread recomputes it first
              prev = x->res;
              continue;
            }
            if (x->st == J_PENDING) {
              if (x->img_bad) {
                x->st = J_DROPPED;  // no image: the bridge rejects / never feeds it
                continue;
              }
              const int64_t dl = drop_late_ns.load();
              if (dl > 0 && x->img_ok && x->t_ns && sys_init.load()) {  // the bridge will drop it as late: skip
                timespec ts;
                clock_gettime(CLOCK_BOOTTIME, &ts);
                const int64_t lag = (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec - x->t_ns;
                if (lag - lag_min_ns.load() > dl) {
                  x->st = J_DROPPED;
                  s_.late_skips++;
                  continue;
                }
              }
              if (x->img_ok)
                j = x;
            }
            break;  // RUNNING (cannot happen: one worker) or image not there yet
          }
          if (j && !prev && com_wait_)
            j.reset();  // its base is the frame being committed right now: wait for committed()
          if (j) {
            base = prev ? prev : com_;
            j->st = J_RUNNING;
            j->t_ts = mono_ns_now();
            j->tracks++;
            g = gen_;
            fn = track_;
            keep = trk_;
            break;
          }
        }
        cv_.wait(lk);
      }
    }
    busy.store(true);
    if (do_remap) {
      remap_job(*j);
      {
        std::lock_guard<std::mutex> lk(m_);
        j->t_re = mono_ns_now();
        j->remapping = false;
        j->img_ok = !j->img.empty();
        j->img_bad = !j->img_ok;
        s_.worker_remap_ms += j->remap_ms;
        busy.store(false);
      }
      cv_.notify_all();
      continue;
    }
    tl_worker = true;
    tl_base = base;
    tl_gen = g;
    tl_out.reset();
    bool exc = false;
    auto t0 = std::chrono::steady_clock::now();
    try {
      fn(j->img, mask.empty() ? cv::Mat(cv::Mat::zeros(j->img.rows, j->img.cols, CV_8UC1)) : mask);
    } catch (const std::exception &e) {
      exc = true;
      std::fprintf(stderr, "[ov_bridge] pipe: worker exception %s (frame recomputed on the main thread)\n", e.what());
    }
    const double tms = ms_since(t0);
    tl_worker = false;
    tl_base.reset();
    std::shared_ptr<Res> out = exc ? nullptr : tl_out;
    tl_out.reset();
    {
      std::lock_guard<std::mutex> lk(m_);
      j->t_te = mono_ns_now();
      if (j->st == J_RUNNING && g == gen_) {
        j->res = out;
        if (out)
          out->gen = g;
        j->track_ms = tms;
        j->st = J_DONE;
        s_.worker_tracks++;
        s_.worker_track_ms += tms;
      } else {
        s_.worker_wasted++;
      }
      if (exc)
        s_.worker_exc++;
      busy.store(false);
    }
    cv_.notify_all();
  }
}

} // namespace ovpipe
