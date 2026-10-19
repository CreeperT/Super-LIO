

#ifndef SUPER_LIO_H_
#define SUPER_LIO_H_

#include <queue>
#include <vector>
#include <iostream>
#include <cassert>
#include <filesystem>
#include <atomic>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include "basic/alias.h"
#include "common/ds.h"
#include "common/timer.h"
#include "params.h"
#include "ESKF.h"
#include "OctVoxMap/OctVoxMap.hpp"
#include "OctVoxMap/VoxelGridFilter.h"
#include "ros/ROSWrapper.h"

namespace LI2Sup{

class SuperLIO{
public:
  SuperLIO(){};
  ~SuperLIO(){};

  void setROSWrapper(const ROSWrapper::Ptr& wrapper){
    data_wrapper_ = wrapper;
  }
  virtual void init();
  void process();
  void saveMap();
  void printTimeRecord();

protected:
  void stateWaitKFInit();
  void stateWaitMapInit();
  void stateProcess();
  virtual bool kf_init();
  virtual bool map_init();
  void Propagation_Undistort();
  void DownSample();
  void Observe();
  virtual void UpdateMap();
  virtual void Output();
  void caceData();
  void ProcessCaceMap();

  /// Robust plane fit of the ground from the CURRENT scan, in the body (IMU) frame.
  /// The map's own ground planes cannot be used for the structural prior: they were
  /// built by the drifting estimate and therefore carry the very tilt we measure.
  /// Returns the unit normal (pointing up) and the inlier count.
  bool fit_scan_ground(BASIC::V3& n_body, int& n_inliers) const;

  using StateFn = void (SuperLIO::*)();
  using OctVoxMapType = OctVoxMap<BASIC::V3, BASIC::scalar>;
  using KNNHeapType = KNNHeap<5, BASIC::V3>;
  StateFn state_fn_;
  ESKF::Ptr kf_;
  OctVoxMapType::Ptr ivox_;
  VoxelGridClosest<BASIC::PointType> voxel_grid_fliter_;
  ROSWrapper::Ptr data_wrapper_;
  MeasureGroup measures_;
  
  /// per-point motion info for time-offset Jacobian (Phase 2: spatiotemporal calibration)
  struct PointMotionInfo {
    double query_time = 0.0;      // measurement time (scan start + offset_time)
    BASIC::V3 v_body = BASIC::V3::Zero();   // body-frame point velocity at measurement time (m/s)
  };

  bool flg_init_ = false;
  bool flg_first_scan_ = true;
  std::vector<DynamicState> propagate_states_;
  BASIC::CloudPtr scan_undistort_full_;
  BASIC::CloudPtr scan_undistort_full_2_;   // second LiDAR deskewed cloud
  BASIC::CloudPtr ds_undistort_;
  BASIC::CloudPtr point_map_, world_pc_, ds_world_;
  std::vector<PointMotionInfo> full_motion_info_;   // parallel to scan_undistort_full_
  std::vector<PointMotionInfo> ds_motion_info_;     // parallel to ds_undistort_
  int frame_num_ = 0;
  BASIC::SE3 sys_init_pose_;
  BASIC::SE3 last_pose_;

  std::size_t effect_knn_num_ = 0;
  BASIC::VV3 points_world_v3_, points_body_v3_;
  std::vector<char> effect_mask_;       // dynamic: resized per frame
  std::vector<char> effect_knn_mask_;   // dynamic: resized per frame
  std::vector<int> effect_knn_idxs_;
  std::vector<std::pair<BASIC::M6, BASIC::V6>> H_R_;
  std::vector<std::array<double, 4>> abcd_vec_;
  std::vector<char> edge_mask_;         // dynamic: resized per frame
  std::vector<BASIC::V3> line_dir_vec_;
  std::vector<BASIC::V3> line_center_vec_;
  int pcd_index_ = -1;
  int skip_observe_frames_ = 0;  // frames to skip Observe after map reset
  bool allow_map_reset_ = true;  // relocation mode sets this to false

  // ── Structural regularity ──────────────────────────────────────────────
  // The ground is NOT necessarily horizontal (this site has a real cross-slope of
  // up to ±5°), so floor planes must never be snapped to world-vertical. What is
  // captured here is the terrain's own normal during the initial stationary
  // segment; at runtime only its component ALONG THE DIRECTION OF TRAVEL (the
  // longitudinal grade) is enforced, leaving the cross-slope untouched.
  BASIC::V3 ground_ref_w_ = BASIC::V3::Zero();  // world-frame terrain normal at startup
  int  ground_ref_count_ = 0;                   // plane samples accumulated
  int  ground_ref_frames_ = 0;                  // frames spent capturing
  bool ground_ref_done_ = false;                // capture finished (success or gave up)
  BASIC::V3 travel_dir_w_ = BASIC::V3::Zero();  // last valid horizontal travel direction
  bool travel_dir_valid_ = false;
  // Route direction. Only the manual grade reference (ground_grade_deg, a scalar) needs
  // it: that scalar has no world-frame axis of its own, so it is defined w.r.t. this
  // direction and gets negated once the platform turns around. The captured normal plays
  // the same role for the auto reference, which is why the auto branch needs no latch.
  // Taken from the platform's DISPLACEMENT after it exceeds kRouteLatchDist, not from the
  // instantaneous velocity: right after init the 0.15 m/s gate alone can be passed by
  // jitter, and a reversed latch mirrors the reference for the whole run.
  BASIC::V3 travel_dir0_w_ = BASIC::V3::Zero();    // horizontal route direction (unit)
  BASIC::V3 travel_dir0_p0_ = BASIC::V3::Zero();   // position when the first valid direction appeared
  bool travel_dir0_pending_ = false;               // p0 taken, waiting for the displacement
  bool travel_dir0_valid_ = false;

  // Per-frame ground plane fitted from the current scan (body frame), used by the
  // scan-based grade residual. Unlike the map-normal snap, this has full attitude
  // leverage: its Jacobian is O(1) rather than O(|p_w − c|).
  BASIC::V3 scan_ground_n_b_ = BASIC::V3::Zero();
  int  scan_ground_inliers_ = 0;
  bool scan_ground_valid_ = false;

  // ── Deskew-fallback accounting (Propagation_Undistort) ───────────────────
  // A point whose timestamp exceeds the last propagated IMU state is copied RAW with
  // v_body = 0 — the sweep is then not motion-compensated at all, which biases the
  // scan match toward "not moving" and can collapse the ESKF velocity. A spike here
  // means the IMU bundle of that frame ended before the scan did (timing-dependent).
  // Written inside a tbb::parallel_for ⇒ atomics; per-frame, reset at the top of the
  // undistort and reported next to [FeatDiag].
  // The three-way bookkeeping over the bracket search (per frame, reset at the top
  // of the undistort and reported by [PU]):
  //   above  = query_time > back().time  -> copied RAW, no motion compensation
  //   below  = query_time < front().time -> bracket search finds nothing, silently
  //            keeps match_iter = begin()  (the FRONT miss)
  //   inside = window-touching frames that are neither of the above
  // `total` is the cumulative count of the `below` class ONLY (never reset) and is
  // what verifies that the `below` branch is the only one incrementing it.
  std::atomic<long> stale_n1_{0}, stale_n2_{0};   // above  (LiDAR1 / LiDAR2)
  std::atomic<long> stale_lo_n1_{0};              // below, per frame
  std::atomic<long> stale_lo_total_{0};           // below, cumulative, NEVER reset
  std::atomic<long> stale_in_n1_{0};              // inside the window
  std::atomic<long> stale_dt1_{0}, stale_dt2_{0}; // Σ(query_time − last IMU time), µs

  Timer time_record_;
};

} // namespace END.

#endif


