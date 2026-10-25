// ovpipe.h — track frame N+1 while frame N is updated (2026-10-26, branch ovpipe-20261026; ov_bridge --pipe 1).
//
// OFF by default (P.on == false): the patched TrackKLT / VioManager / ov_bridge run their previous code path unchanged.
//
// Idea: OpenVINS feed_measurement_camera(N) = TrackKLT (histogram, pyramid, FAST top-off on N-1, KLT N-1 -> N, RANSAC)
// + propagate + MSCKF/SLAM update + marginalisation. The tracking result of a frame depends ONLY on the tracker's own
// state after the previous frame (pts/ids/pyramid/mask of N-1, the id counter currid, num_features) and the new image;
// it never reads the filter state or the feature database (it reads the camera intrinsics for undistort: they must be
// fixed -> --pipe is refused with calib_cam_intrinsics). Only the LAST step of TrackKLT writes shared data: the
// feature database (update_feature of every good track) and the tracker members (*_last).
// So a worker thread computes "tracking of N+1" (pure, into a Res) while the main thread is still inside the update of
// N, and the main thread later COMMITS that Res at exactly the place where the sequential code ran the tracker (start of
// track_image_and_update of N+1): same database writes, same member values, same order -> identical poses.
// A Res is committed only when it was computed from exactly the committed state (base id == committed id, same tracker
// generation, same num_features); anything else (frame dropped by the bridge, tracker reset, num_features changed at
// initialisation, worker exception) -> the main thread recomputes that frame synchronously from the committed state
// (the old sequential path) and the worker re-chains the later frames.
// The worker also does the --remap of each new frame (bit-identical cv::remap) as soon as the reader thread has it.
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <climits>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>

namespace ov_core {
class TrackBase;
}

namespace ovpipe {

// tracker state after one frame (= TrackKLT members img_last/img_mask_last/img_pyramid_last/pts_last/ids_last + currid)
struct TrkSt {
  cv::Mat img, mask;
  std::vector<cv::Mat> pyr;
  std::vector<cv::KeyPoint> pts;
  std::vector<size_t> ids;
  size_t currid = 0;
};
struct Obs {  // one database->update_feature(id, t, cam, u, v, nu, nv)
  size_t id;
  float u, v, nu, nv;
};
struct Res {
  uint64_t id = 0, base = 0, gen = 0;  // base = id of the state it was computed from (0 = tracker initial state)
  int nf = -1;                         // num_features used (-1 = it changed during the compute -> never valid)
  bool rsc_fail = false;               // upstream "Failed to get enough points to do RANSAC, resetting"
  TrkSt st;
  std::vector<Obs> obs;
};
enum JobSt : int { J_PENDING = 0, J_RUNNING, J_DONE, J_DROPPED };
struct Job {
  uint64_t seq = 0;
  const uint8_t *src = nullptr;  // record payload (owned by the reader queue until release())
  size_t len = 0;
  bool remapping = false, img_ok = false, img_bad = false;
  cv::Mat img;
  double remap_ms = 0.0, track_ms = 0.0;
  // --pipe-log timestamps (steady clock ns; 0 = not happened): submit, remap start/end, last track start/end
  int64_t t_sub = 0, t_rs = 0, t_re = 0, t_ts = 0, t_te = 0;
  int64_t t_ns = 0;  // frame time from the record header (drop-late mirror)
  int remap_by = 0;  // 1 = worker, 2 = main thread (get_image)
  int tracks = 0;    // worker computes of this job (re-chains)
  int st = J_PENDING;
  std::shared_ptr<Res> res;
};

struct Stats {
  uint64_t submitted = 0, hits = 0, miss_base = 0, miss_nf = 0, miss_gen = 0, nojob = 0, nores = 0, sync = 0;
  uint64_t rechains = 0, drops = 0, worker_tracks = 0, worker_wasted = 0, worker_exc = 0, img_waits = 0;
  double claim_wait_ms = 0.0, max_claim_wait_ms = 0.0, worker_track_ms = 0.0, worker_remap_ms = 0.0, img_wait_ms = 0.0;
  uint64_t claim_waited = 0;
  uint64_t late_skips = 0;  // frames the worker did not track because the bridge will drop them as late
};

class Pipe {
public:
  std::atomic<bool> on{false};
  std::atomic<const void *> target{nullptr};  // the TrackKLT that uses the pipe (sys_ tracker); others: upstream path
  std::atomic<bool> busy{false};              // worker is computing (idle spin uses it)
  std::mutex compute_m;                       // serialises tracker computes (they use the member currid)
  std::atomic<uint64_t> next_id{1};

  // ---- bridge setup (before start) ----
  cv::Mat map_x, map_y;       // --remap tables (empty = no remap)
  int remap_src_w = 0, remap_src_h = 0;
  bool frame_nocopy = false;  // remap straight from the record (else copy first, as the bridge does)
  cv::Mat mask;               // cam0 mask the bridge gives every frame (zeros or the config mask)
  int ncam = 1;
  std::vector<int> cpus;      // --pipe-cpus: worker thread affinity (empty = not pinned)
  double spin_ms = 0.0;       // --pipe-spin MS (03:30): worker busy-waits up to MS ms for new work before blocking (0 = off)
  // --drop-late mirror (2026-10-26 03:00, after phone P_2/P_4 cascade): the worker does not track a frame the bridge will
  // certainly drop: lag only grows and the minimum lag only shrinks, so "late now" (with the bridge's current minimum)
  // implies "late when the bridge checks it" while the VIO stays initialised. A wrong guess only costs a recompute.
  std::atomic<int64_t> drop_late_ns{0};             // 0 = off
  std::atomic<int64_t> lag_min_ns{INT64_MAX};       // bridge: smallest feed lag seen (CLOCK_BOOTTIME - frame t_ns)
  std::atomic<bool> sys_init{false};                // bridge: VIO initialised after the last fed frame

  void start();
  void stop();
  // new tracker (VioManager created): previous results void, frames re-chained on it
  // track(img, mask) = trk->feed_new_camera of a one-image message (the worker sets tl_worker around it); target = the
  // TrackKLT object (P.target); keep = keeps it alive while the worker may still use it. nullptr target = no tracker.
  void set_tracker(std::shared_ptr<void> keep, std::function<void(const cv::Mat &, const cv::Mat &)> track,
                   const void *target);
  // reader thread: a new image record (payload stays valid until release(seq))
  void submit(uint64_t seq, const uint8_t *src, size_t len);
  // main thread, on_stereo: the (remapped) cam0 image of record seq; false = no job / bad -> bridge does it itself
  bool get_image(uint64_t seq, cv::Mat &img, double &remap_ms);
  // main thread: record seq will not be fed (rejected / queue overflow / late / reset)
  void drop(uint64_t seq);
  // main thread, after handle(): the record buffer goes back to the reader
  void release(uint64_t seq);

  // ---- TrackKLT (main thread) ----
  // valid precomputed result for record seq (waits for the worker), or nullptr = compute synchronously
  std::shared_ptr<Res> claim(uint64_t seq, int nf);
  // after the commit: r is the new committed state; fresh = computed synchronously (later results re-chained)
  void committed(const std::shared_ptr<Res> &r, bool fresh);
  // base state for a synchronous compute (committed, else the tracker's initial state), id in *base_id
  std::shared_ptr<Res> com_res() {
    std::lock_guard<std::mutex> lk(m_);
    return com_;
  }
  uint64_t gen() {
    std::lock_guard<std::mutex> lk(m_);
    return gen_;
  }
  // initial tracker state (captured by TrackKLT at its first pipe compute, under compute_m)
  std::shared_ptr<Res> init_state;   // guarded by compute_m
  const void *init_trk = nullptr;    // guarded by compute_m: tracker + generation the initial state belongs to
  uint64_t init_gen = 0;             // guarded by compute_m

  FILE *log = nullptr;  // --pipe-log FILE (main thread writes one row per claim)
  Stats stats() {
    std::lock_guard<std::mutex> lk(m_);
    return s_;
  }
  void add_sync() {
    std::lock_guard<std::mutex> lk(m_);
    s_.sync++;
  }

private:
  void run();
  void rechain_after(size_t idx);  // m_ held: jobs after index idx (all if idx == SIZE_MAX) -> PENDING
  void remap_job(Job &j);           // worker, no lock
  void notify_() {                  // every state change: wakes waiters and the --pipe-spin poll
    ev_.fetch_add(1, std::memory_order_release);
    cv_.notify_all();
  }
  std::atomic<uint64_t> ev_{0};

  std::mutex m_;
  std::condition_variable cv_;
  std::deque<std::shared_ptr<Job>> jobs_;
  std::shared_ptr<Res> com_;
  bool com_wait_ = false;  // claim() returned, committed() not yet: com_ may be stale for chaining
  std::shared_ptr<void> trk_;
  std::function<void(const cv::Mat &, const cv::Mat &)> track_;
  uint64_t gen_ = 0;
  bool stop_ = false;
  std::thread th_;
  Stats s_;
};

extern Pipe P;
// thread-local hand-over between the worker / bridge and the patched TrackKLT::feed_new_camera
extern thread_local bool tl_worker;              // this call is the worker's precompute
extern thread_local std::shared_ptr<Res> tl_base;  // worker: state to compute from (nullptr = tracker initial)
extern thread_local std::shared_ptr<Res> tl_out;   // worker: result
extern thread_local uint64_t tl_seq;             // main: record seq of the frame being fed (0 = none)
extern thread_local uint64_t tl_gen;             // worker: tracker generation of this precompute
extern std::shared_ptr<ov_core::TrackBase> last_tracker;  // set by the patched VioManager constructor

} // namespace ovpipe
