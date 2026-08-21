
#include "lio/super_lio.h"

#include <sys/resource.h>
#include <fstream>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>

#include <pcl/kdtree/kdtree_flann.h>  // Phase 2: KD-tree for point motion matching


using namespace BASIC;

namespace LI2Sup{

inline bool calc_plane_coeff(const int N, const std::array<V3, 5>& points, std::array<double, 4>& abcd)
{
  Eigen::Vector3d normvec;
  if (N == 5) {
    Eigen::Matrix<double, 5, 3> A;
    Eigen::Matrix<double, 5, 1> b;
    for (int j = 0; j < 5; j++) {
      A.row(j) = points[j].cast<double>();
      b(j) = -1.0;
    }
    normvec = A.colPivHouseholderQr().solve(b);
  }
  else {
    Eigen::Matrix<double, 4, 3> A;
    Eigen::Matrix<double, 4, 1> b;

    for (int j = 0; j < N; j++) {
      A.row(j) = points[j].cast<double>();
      b(j) = -1.0;
    }
    normvec = A.colPivHouseholderQr().solve(b);
  }

  double n = normvec.norm();
  if (n < 1e-6f) return false;

  abcd[3] = 1.0 / n;
  normvec *= abcd[3];
  abcd[0] = normvec[0];
  abcd[1] = normvec[1];
  abcd[2] = normvec[2];
  
  for (int i = 0; i < N; ++i) {
    const V3& p = points[i];
    auto dist = abcd[0] * p(0) + abcd[1] * p(1) + abcd[2] * p(2) + abcd[3];
    if (std::abs(dist) > 0.1) return false;
  }
  return true;
}


inline bool compute_error(
  const std::array<double, 4>& abcd, const V3& point,
  const float length, scalar& error)
{
  error = abcd[0] * point[0] + abcd[1] * point[1] + abcd[2] * point[2] + abcd[3];
  return length > 81 * error * error;
}


/// Fit a line from 4-5 neighbor points using PCA.
/// Checks linearity (λ_max / λ_2nd > linearity_ratio) and distance of all points to line.
/// Returns line direction (unit vector) and centroid.
inline bool fit_line_from_points(
    const int N, const std::array<V3, 5>& points,
    V3& direction, V3& centroid)
{
  // compute centroid
  centroid = V3::Zero();
  for (int i = 0; i < N; i++) {
    centroid += points[i];
  }
  centroid /= static_cast<scalar>(N);

  // build covariance matrix
  M3 cov = M3::Zero();
  for (int i = 0; i < N; i++) {
    V3 d = points[i] - centroid;
    cov += d * d.transpose();
  }

  // PCA: eigenvalues in increasing order (λ0 ≤ λ1 ≤ λ2)
  Eigen::SelfAdjointEigenSolver<M3> eig(cov);
  V3 ev = eig.eigenvalues();

  // check linearity: λ2 (largest) >> λ1 (second largest)
  if (ev(1) < 1e-10f || ev(2) / ev(1) < g_edge_linearity_ratio) {
    return false;
  }

  // line direction = eigenvector of largest eigenvalue
  direction = eig.eigenvectors().col(2);

  // check all neighbors are close to the fitted line
  for (int i = 0; i < N; i++) {
    V3 diff = points[i] - centroid;
    scalar dist = (diff - direction * direction.dot(diff)).norm();
    if (dist > g_edge_line_dist_thresh) {
      return false;
    }
  }

  return true;
}


void SuperLIO::init(){
  ivox_.reset(new OctVoxMapType(OctVoxMapType::Options{g_ivox_resolution, g_ivox_capacity}));
  kf_.reset(new ESKF());

  // set degeneracy options
  {
    ESKF::DegeneracyOptions degen_opts;
    degen_opts.enabled = g_degeneracy_enabled;
    degen_opts.max_trans_stddev = g_degeneracy_max_stddev;
    degen_opts.htvh_cond_thresh = g_degeneracy_htvh_cond_thresh;
    kf_->setDegeneracyOptions(degen_opts);
  }

  kf_->setCalibEnabled(g_enable_online_calib);

  data_wrapper_->setESKF(kf_);
  
  scan_undistort_full_.reset(new PointCloudType());
  scan_undistort_full_2_.reset(new PointCloudType());
  ds_undistort_.reset(new PointCloudType());
  world_pc_.reset(new PointCloudType());
  ds_world_.reset(new PointCloudType());

  if(g_save_map){
    point_map_.reset(new PointCloudType());
  }
  
  points_world_v3_.reserve(21000);
  voxel_grid_fliter_.setLeafSize(g_voxel_fliter_size);

  state_fn_ = &SuperLIO::stateWaitKFInit;

  LOG(INFO) << GREEN << " ---> [SuperLIO]: initialized." << RESET;
}


void SuperLIO::stateWaitKFInit()
{
  if (kf_init()) {
    state_fn_ = &SuperLIO::stateWaitMapInit;
    LOG(INFO) << GREEN << " ---> [SuperLIO]: KF init done" << RESET;
  }
}

void SuperLIO::stateWaitMapInit()
{
  if (map_init()) {
    // kf_->init_ = true;
    state_fn_ = &SuperLIO::stateProcess;
    LOG(INFO) << GREEN << " ---> [SuperLIO]: Map init done" << RESET;
  }
}

void SuperLIO::process(){
  if(!data_wrapper_->sync_measure(measures_)){
    return;
  }
  (this->*state_fn_)();
}


bool SuperLIO::kf_init(){
  static int imu_cout = 0;
  static V3 mean_gyro = V3::Zero();
  static V3 mean_acce = V3::Zero();

  for(auto& imu: measures_.imu){
    imu_cout ++;
    mean_gyro += (imu.gyr - mean_gyro) / imu_cout;
    mean_acce += (imu.acc - mean_acce) / imu_cout;
  }

  /// 100 Hz for 1 second.
  if(imu_cout < 50){
    return false;
  }

  V3 gravity = - mean_acce * g_gravity_norm / mean_acce.norm();
  V3 ref_gravity(0, 0, - g_gravity_norm);
  M3 init_rot = Quat::FromTwoVectors(gravity, ref_gravity).toRotationMatrix();
  V3 n = init_rot.col(0);
  double yaw = atan2(n(1), n(0));

  M3 R_yaw_inv = Eigen::AngleAxis<scalar>(-yaw, V3::UnitZ()).toRotationMatrix(); 

  // init_rot represents the IMU orientation after gravity alignment (level orientation).
  // Perform LiDAR leveling correction, then transform the orientation into the robot frame.
  M3 rot = g_lidar_robo_yaw * R_yaw_inv * init_rot;  

  ESKF::Options options;
  options.gyro_var_ = g_imu_ng;
  options.acce_var_ = g_imu_na;
  options.bias_gyro_var_ = g_imu_nbg;
  options.bias_acce_var_ = g_imu_nba;
  options.num_iterations_ = g_kf_max_iterations;
  options.quit_eps_ = g_kf_quit_eps;

  float imu_scale = g_gravity_norm / mean_acce.norm();

  // Estimate accelerometer bias from stationary init data.
  // At rest: measured = -gravity_imu + bias.  mean_acce ≈ -gravity_imu + bias.
  // Expected static acc = mean_acce.normalized() * g ≈ -gravity_imu (unbiased).
  // Therefore bias ≈ mean_acce - expected = mean_acce * (1 - g/|mean|).
  V3 estimated_ba = mean_acce * (1.0f - imu_scale);

  kf_->SetInitialConditions(options, mean_gyro, estimated_ba, imu_scale, ref_gravity);
  auto state = kf_->GetSysState();
  state.R = SO3(rot);
  state.p = g_odom_robo.t_;        // By default, the robot frame is used as the reference origin.
  state.bg = mean_gyro;
  state.ba = estimated_ba;
  state.timestamp = measures_.imu.back().secs;
  kf_->SetX(state);
  sys_init_pose_ = kf_->GetSE3();
  return true;
}


bool SuperLIO::map_init(){
  frame_num_++;

  std::size_t ptsize = measures_.lidar.pc->size();
  points_world_v3_.resize(ptsize);

  const SE3 transform = sys_init_pose_ * g_lidar_imu;

  tbb::parallel_for(
    tbb::blocked_range<size_t>(0, ptsize),
    [&](const tbb::blocked_range<size_t>& r) {
      for (size_t idx = r.begin(); idx < r.end(); ++idx) {
        auto& point_pcl = measures_.lidar.pc->points[idx];
        V3 point_body(point_pcl.x, point_pcl.y, point_pcl.z);
        points_world_v3_[idx] = transform * point_body;
      }
    }
  );

  ivox_->insert(points_world_v3_);
  kf_->SetLastObsTime(measures_.lidar.end_time);

  // 20 Hz for 1.0 seconds. Integral coverage area > 70%
  if(frame_num_ > 3){
    g_flg_map_init = false;
    return true;
  }
  return false;
}


void SuperLIO::stateProcess(){
  frame_num_++;
  if(g_time_eva){
    time_record_.Evaluate([this](){Propagation_Undistort();}, "[Undistort]");
    time_record_.Evaluate([this]() { DownSample(); }, "[DownSample]");
    time_record_.Evaluate([this]() { Observe(); }, "[Observe]");
    time_record_.Evaluate([this]() { UpdateMap(); }, "[UpdateMap]");
  }else{
    Propagation_Undistort();
    DownSample();
    Observe();
    UpdateMap();
  }
  Output();
  caceData();

  // ── Map reset on persistent degeneracy ──
  // In relocation mode, the prior map is the only reference — never clear it.
  if (allow_map_reset_ && kf_->shouldResetMap()) {
    LOG(WARNING) << YELLOW << " ---> [LIO] Resetting map (degeneracy)."
                 << RESET;
    ivox_->clear();
    kf_->clearResetMapFlag();
    skip_observe_frames_ = 5;
  }

  kf_->init_ = true;
}


void SuperLIO::caceData(){
  if(!g_save_map) return;
  auto state = kf_->GetNavState();
  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  transformation.block<3, 3>(0, 0) = state.R.R_.cast<float>();
  transformation.block<3, 1>(0, 3) = state.p.cast<float>();

  if(g_if_filter){
    pcl::transformPointCloud(*ds_undistort_, *world_pc_, transformation);
  }else{
    pcl::transformPointCloud(*scan_undistort_full_, *world_pc_, transformation);
  }

  static int scan_wait_num = 0;
  if(!world_pc_->empty()){
    *point_map_ += *world_pc_;
    scan_wait_num++;
  }

  if(g_pcd_save_interval < 0) {
    scan_wait_num = 0;
    return;
  }

  static bool rm_PCD_dir = false;
  if(!rm_PCD_dir){
    rm_PCD_dir = true;
    std::string cmd = "rm -rf " + g_save_map_dir + "/PCD";
    [[maybe_unused]] int res;
    res = system(cmd.c_str());
    cmd = "mkdir -p " + g_save_map_dir + "/PCD";
    res = system(cmd.c_str());
  }

  if (point_map_->size() > 0 && scan_wait_num >= g_pcd_save_interval) {
    pcd_index_++;
    std::string map_name(std::string(g_save_map_dir + "/PCD/scans_") + std::to_string(pcd_index_) +
                               std::string(".pcd"));
    LOG(INFO) << GREEN << " ---> current scan saved to /PCD/scans_" << pcd_index_ << "  size:  " << point_map_->size() << RESET;
    pcl::io::savePCDFileBinary(map_name, *point_map_);
    point_map_->clear();
    scan_wait_num = 0;
  }
}


void SuperLIO::ProcessCaceMap(){
  namespace fs = std::filesystem;

  std::string pcd_folder = g_save_map_dir + "/PCD";
  std::string output_map_name = g_save_map_dir + "/" + g_map_name;

  LOG(INFO) << YELLOW << " ---> Merging PCD fragments in: " << pcd_folder << RESET;

  PointCloudType::Ptr merged_map(new PointCloudType());

  int count = 0;
  for (const auto& entry : fs::directory_iterator(pcd_folder)) {
    if (entry.path().extension() == ".pcd" &&
      entry.path().filename().string().find("scans_") != std::string::npos) {
      PointCloudType::Ptr tmp_cloud(new PointCloudType());
      if (pcl::io::loadPCDFile<PointType>(entry.path().string(), *tmp_cloud) == 0) {
        *merged_map += *tmp_cloud;
        count++;
        // LOG(INFO) << GREEN << " ---> Merged: " << entry.path().filename().string() 
        //           << "   size: " << tmp_cloud->size() << RESET;
      } else {
        LOG(WARNING) << RED << " ---> Failed to load: " << entry.path().string() << RESET;
      }
    }
  }

  LOG(INFO) << YELLOW << " ---> Total merged fragments: " << count << RESET;

  PointCloudType filtered_map;

  if(g_if_filter){
    LOG(INFO) << YELLOW << " ---> Downsampling merged map before final save..." << RESET;
    pcl::VoxelGrid<PointType> voxel_filter;
    voxel_filter.setLeafSize(g_map_ds_size, g_map_ds_size, g_map_ds_size);
    
    voxel_filter.setInputCloud(merged_map);
    voxel_filter.filter(filtered_map);
  }else{
    LOG(INFO) << YELLOW << " ---> Not Downsampling merged map before final save..." << RESET;
    filtered_map = *merged_map;
  }
  
  if (filtered_map.size() > 0) {
    filtered_map.width = filtered_map.size();
    filtered_map.height = 1;
    filtered_map.is_dense = false;
  }

  pcl::io::savePCDFileBinary(output_map_name, filtered_map);

  LOG(INFO) << GREEN << " ---> Final map saved to: " << output_map_name << RESET;
  LOG(INFO) << GREEN << " ---> Final map size: " << filtered_map.size() << RESET;
}


void SuperLIO::saveMap(){
  if(!g_save_map) return;
  if(g_pcd_save_interval > 0){
    LOG(INFO) << YELLOW << " ---> Saving last cace ... " << RESET;
    if (point_map_->size() > 0) {
      pcd_index_++;
      std::string map_name(std::string(g_save_map_dir + "/PCD/scans_") + std::to_string(pcd_index_) +
                                 std::string(".pcd"));
      LOG(INFO) << GREEN << " ---> current scan saved to /PCD/scans_" << pcd_index_ << "  size:  " << point_map_->size() << RESET;
      pcl::io::savePCDFileBinary(map_name, *point_map_);
      point_map_->clear();
    }
    LOG(INFO) << GREEN << " ---> Save last cace success. " << RESET;
    LOG(INFO) << YELLOW << " ---> Process cace map ... " << RESET;
    ProcessCaceMap();
    LOG(INFO) << GREEN << " ---> Process cace map success. " << RESET;
    return;
  }

  LOG(INFO) << YELLOW << " ---> Saving map..... " << RESET;
  if(!point_map_->empty()){
    std::string map_name = g_save_map_dir + "/" + g_map_name;
    LOG(INFO) << YELLOW << " ---> Save map to: " << map_name << RESET;
    pcl::VoxelGrid<PointType> voxel_fliter;
    PointCloudType latst_map;
    voxel_fliter.setInputCloud(point_map_);
    voxel_fliter.setLeafSize(g_map_ds_size, g_map_ds_size, g_map_ds_size);
    voxel_fliter.filter(latst_map);
    if(latst_map.size() > 0){
      latst_map.width = latst_map.size();
      latst_map.height = 1;
      latst_map.is_dense = false;
    }
    pcl::io::savePCDFileBinary(map_name, latst_map);
    LOG(INFO) << GREEN << " ---> Save map success. File: " << map_name << RESET;
    LOG(INFO) << GREEN << " ---> Map size: " << latst_map.size() << RESET;
  }
}


inline double get_cpu_time_seconds() {
  struct rusage usage;
  getrusage(RUSAGE_SELF, &usage);
  return usage.ru_utime.tv_sec + usage.ru_utime.tv_usec / 1e6 +
         usage.ru_stime.tv_sec + usage.ru_stime.tv_usec / 1e6;
}


void SuperLIO::Propagation_Undistort(){
  propagate_states_.clear();
  propagate_states_.emplace_back(kf_->GetDynamicState());
  kf_->SetObsTime(measures_.lidar.end_time);
  for (auto &imu : measures_.imu) {
    kf_->Predict(imu);
    propagate_states_.emplace_back(kf_->GetDynamicState());
  }

  const SE3 T_end = kf_->GetSE3();
  const M3  R_inv = T_end.R_.transpose();
  const V3  T_end_t = T_end.t_;

  // ── LiDAR 1 deskew ──────────────────────────────────────────
  {
    // Phase 1: use current calib estimate (not fixed nominal) for self-consistency
    const SE3 T_ext1 = kf_->getExtrinsic1();
    const M3 TLI_R = T_ext1.R_;
    const V3 TLI_t = T_ext1.t_;
    const double start_time = measures_.lidar.start_time;
    auto& raw_pc = measures_.lidar.pc;

    std::size_t ptsize = raw_pc->points.size();
    scan_undistort_full_->resize(ptsize);
    full_motion_info_.resize(ptsize);  // Phase 2: per-point velocity for time-offset Jacobian

    tbb::parallel_for(
    tbb::blocked_range<size_t>(0, ptsize),
    [&](const tbb::blocked_range<size_t>& r) {
      M3 R_h, R_t; V3 p_h, v_h, acc_t, w_t;
      for (size_t idx = r.begin(); idx < r.end(); ++idx) {
        auto& pt_full = scan_undistort_full_->points[idx];
        const auto& pt = raw_pc->points[idx];
        pt_full.intensity = pt.intensity;
        double query_time = start_time + pt.offset_time + kf_->getDeltaT1();
        if (query_time > propagate_states_.back().time) {
          V3 raw(pt.x, pt.y, pt.z);
          V3 eigen_point = TLI_R * raw + TLI_t;
          pt_full.x = eigen_point[0];
          pt_full.y = eigen_point[1];
          pt_full.z = eigen_point[2];
          full_motion_info_[idx].query_time = query_time;
          full_motion_info_[idx].v_body = V3::Zero();
          continue;
        }
        auto match_iter = propagate_states_.begin();
        for (auto iter = propagate_states_.begin(); iter != propagate_states_.end(); ++iter) {
          auto next_iter = std::next(iter);
          if (iter->time < query_time && next_iter->time >= query_time) {
            match_iter = iter;
            break;
          }
        }
        auto match_iter_n = std::next(match_iter);
        double dt = match_iter_n->time - match_iter->time;
        double s = (query_time - match_iter->time) / dt;
        R_h = match_iter->R;
        R_t = match_iter_n->R;
        p_h = match_iter->p;
        v_h = match_iter->v;
        acc_t = match_iter_n->a;
        w_t = match_iter_n->w;
        M3 R_i = Quat(R_h).slerp(s, Quat(R_t)).toRotationMatrix();
        V3 t_ei(p_h + v_h * dt + 0.5 * acc_t * dt * dt - T_end_t);
        V3 raw(pt.x, pt.y, pt.z);
        V3 eigen_point = R_inv * (R_i * (TLI_R * raw + TLI_t) + t_ei);
        pt_full.x = eigen_point[0];
        pt_full.y = eigen_point[1];
        pt_full.z = eigen_point[2];

        // Phase 2: body-frame point velocity for time-offset Jacobian
        {
          V3 p_lidar_imu = TLI_R * raw + TLI_t;
          V3 v_inst_w = v_h + acc_t * s * dt;                        // world-frame linear velocity
          V3 w_inst_w = w_t;                                          // world-frame angular velocity
          V3 v_point_w = v_inst_w + w_inst_w.cross(R_i * p_lidar_imu); // point velocity (world)
          V3 v_body = R_inv.cast<scalar>() * v_point_w.cast<scalar>(); // rotate to scan-end body frame
          full_motion_info_[idx].query_time = query_time;
          full_motion_info_[idx].v_body = v_body;
        }
      }
    });
  }

  // ── LiDAR 2 deskew (dual-LiDAR) ─────────────────────────────
  if (measures_.has_lidar2 && measures_.lidar2.pc && !measures_.lidar2.pc->empty()) {
    const SE3 T_ext2 = kf_->getExtrinsic2();
    const M3 TLI2_R = T_ext2.R_;
    const V3 TLI2_t = T_ext2.t_;
    const double start_time2 = measures_.lidar2.start_time;
    auto& raw_pc2 = measures_.lidar2.pc;

    std::size_t ptsize2 = raw_pc2->points.size();
    scan_undistort_full_2_->resize(ptsize2);

    // Phase 2: pre-allocate motion info for lidar1 + lidar2
    size_t lidar1_sz = scan_undistort_full_->size();
    full_motion_info_.resize(lidar1_sz + ptsize2);

    tbb::parallel_for(
    tbb::blocked_range<size_t>(0, ptsize2),
    [&](const tbb::blocked_range<size_t>& r) {
      M3 R_h, R_t; V3 p_h, v_h, acc_t, w_t;
      for (size_t idx = r.begin(); idx < r.end(); ++idx) {
        auto& pt_full = scan_undistort_full_2_->points[idx];
        const auto& pt = raw_pc2->points[idx];
        // intensity normalization (Mid-360 reflectivity mapped to same scale)
        pt_full.intensity = pt.intensity;
        double query_time = start_time2 + pt.offset_time + kf_->getDeltaT2();
        if (query_time > propagate_states_.back().time) {
          V3 raw(pt.x, pt.y, pt.z);
          V3 eigen_point = TLI2_R * raw + TLI2_t;
          pt_full.x = eigen_point[0];
          pt_full.y = eigen_point[1];
          pt_full.z = eigen_point[2];
          full_motion_info_[lidar1_sz + idx].query_time = query_time;
          full_motion_info_[lidar1_sz + idx].v_body = V3::Zero();  // no valid IMU state
          continue;
        }
        auto match_iter = propagate_states_.begin();
        for (auto iter = propagate_states_.begin(); iter != propagate_states_.end(); ++iter) {
          auto next_iter = std::next(iter);
          if (iter->time < query_time && next_iter->time >= query_time) {
            match_iter = iter;
            break;
          }
        }
        auto match_iter_n = std::next(match_iter);
        double dt = match_iter_n->time - match_iter->time;
        double s = (query_time - match_iter->time) / dt;
        R_h = match_iter->R;
        R_t = match_iter_n->R;
        p_h = match_iter->p;
        v_h = match_iter->v;
        acc_t = match_iter_n->a;
        w_t = match_iter_n->w;
        M3 R_i = Quat(R_h).slerp(s, Quat(R_t)).toRotationMatrix();
        V3 t_ei(p_h + v_h * dt + 0.5 * acc_t * dt * dt - T_end_t);
        V3 raw(pt.x, pt.y, pt.z);
        V3 eigen_point = R_inv * (R_i * (TLI2_R * raw + TLI2_t) + t_ei);
        pt_full.x = eigen_point[0];
        pt_full.y = eigen_point[1];
        pt_full.z = eigen_point[2];

        // Phase 2: body-frame point velocity for time-offset Jacobian (LiDAR2)
        {
          V3 p_lidar_imu = TLI2_R * raw + TLI2_t;
          V3 v_inst_w = v_h + acc_t * s * dt;
          V3 w_inst_w = w_t;
          V3 v_point_w = v_inst_w + w_inst_w.cross(R_i * p_lidar_imu);
          V3 v_body = R_inv.cast<scalar>() * v_point_w.cast<scalar>();
          full_motion_info_[lidar1_sz + idx].query_time = query_time;
          full_motion_info_[lidar1_sz + idx].v_body = v_body;
        }
      }
    });

    // merge lidar2 into lidar1 cloud, mark lidar2 pts with negative intensity
    static bool merge_logged = false;
    if (!merge_logged) {
      LOG(INFO) << GREEN << " ---> [Dual-LiDAR] Merged lidar2 ("
                << scan_undistort_full_2_->size() << " pts) into lidar1 ("
                << scan_undistort_full_->size() << " pts before merge)"
                << " weight=" << g_lidar2_weight << RESET;
      merge_logged = true;
    }
    lidar1_sz = scan_undistort_full_->size();
    *scan_undistort_full_ += *scan_undistort_full_2_;
    for (size_t i = lidar1_sz; i < scan_undistort_full_->size(); ++i) {
      scan_undistort_full_->points[i].intensity =
          -std::abs(scan_undistort_full_->points[i].intensity);
    }
  } else {
    static bool nolidar2_logged = false;
    if (g_dual_lidar_enable && !nolidar2_logged) {
      LOG(WARNING) << YELLOW << " ---> [Dual-LiDAR] lidar2 NOT available: has_lidar2="
                   << measures_.has_lidar2
                   << " pc=" << (measures_.lidar2.pc ? "ok" : "null")
                   << " empty=" << (measures_.lidar2.pc ? measures_.lidar2.pc->empty() : true)
                   << RESET;
      nolidar2_logged = true;
    }
  }
}


void SuperLIO::DownSample(){
  voxel_grid_fliter_.setInputCloud(scan_undistort_full_);
  voxel_grid_fliter_.filter(ds_undistort_);
}


struct ThreadACC{
  M6d HTVH = M6d::Zero();
  V6d HTVr = V6d::Zero();
  // Phase 1: calibration accumulation (20D obs: pose 6 + ext1 6 + ext2 6 + dt 2)
  Eigen::Matrix<double, 20, 20> HTVH_calib = decltype(HTVH_calib)::Zero();
  Eigen::Matrix<double, 20, 1>  HTVr_calib = decltype(HTVr_calib)::Zero();
  // §3 Eq. paper: pose↔extrinsic coupling ΔC_k = Σ w·J_pose·J_ext,kᵀ (6×6)
  M6d DC1 = M6d::Zero();
  M6d DC2 = M6d::Zero();
  std::vector<V3> plane_pts;
  std::vector<V3> edge_pts;
};


void SuperLIO::Observe(){
  size_t ptsize = ds_undistort_->size();

  static std::vector<float> _lengths;
  static std::vector<float> _weights;
  static std::vector<bool>  _is_lidar2;
  points_body_v3_.resize(ptsize);
  _lengths.resize(ptsize);
  _weights.resize(ptsize);
  _is_lidar2.resize(ptsize);

  // Fix A: dynamic resize — no more 20000-element limit
  abcd_vec_.resize(ptsize);
  line_dir_vec_.resize(ptsize);
  line_center_vec_.resize(ptsize);
  effect_knn_idxs_.resize(ptsize);
  effect_mask_.resize(ptsize);
  effect_knn_mask_.resize(ptsize);
  edge_mask_.resize(ptsize);

  effect_knn_num_ = ptsize;
  std::iota(effect_knn_idxs_.begin(), effect_knn_idxs_.end(), 0);

  for(size_t i = 0; i < ptsize; ++i){
    const auto& point_body_pcl = ds_undistort_->points[i];
    points_body_v3_[i] = V3(point_body_pcl.x, point_body_pcl.y, point_body_pcl.z);
    _lengths[i] = points_body_v3_[i].norm();
    // per-point weight: lidar2 points have negative intensity tag
    _is_lidar2[i] = (point_body_pcl.intensity < 0);
    _weights[i] = _is_lidar2[i] ? static_cast<float>(g_lidar2_weight) : 1.0f;

    // Intensity-based weighting: retroreflective poles produce much stronger
    // returns → more reliable features → higher weight.
    float abs_intensity = std::abs(point_body_pcl.intensity);
    if (abs_intensity > g_intensity_thresh) {
      // Linear ramp from 1.0 at threshold to 1.0+g_intensity_weight at 255
      float extra = g_intensity_weight * (abs_intensity - g_intensity_thresh)
                    / (255.0f - g_intensity_thresh);
      _weights[i] *= (1.0f + extra);
    }
  }

  // After map reset: skip IEKF to let map repopulate (empty map → no KNN)
  if (skip_observe_frames_ > 0) {
    skip_observe_frames_--;
    return;
  }

  // Phase 2: match ds points to full cloud for per-point motion info (time-offset Jacobian)
  if (kf_->isCalibEnabled() && !full_motion_info_.empty()) {
    pcl::KdTreeFLANN<PointType> kdtree;
    kdtree.setInputCloud(scan_undistort_full_);
    ds_motion_info_.resize(ptsize);
    std::vector<int> nn_idx(1);
    std::vector<float> nn_dist(1);
    for (size_t i = 0; i < ptsize; ++i) {
      kdtree.nearestKSearch(ds_undistort_->points[i], 1, nn_idx, nn_dist);
      ds_motion_info_[i] = full_motion_info_[nn_idx[0]];
    }
  }

  // Phase 1: global feature length for Jacobian normalisation.
  double s0 = 1.0;
  {
    std::vector<float> ranges;
    ranges.reserve(ptsize);
    for (size_t i = 0; i < ptsize; ++i)
      ranges.push_back(points_body_v3_[i].norm());
    if (!ranges.empty()) {
      std::sort(ranges.begin(), ranges.end());
      s0 = std::max(1.0, static_cast<double>(ranges[ranges.size() / 2]));
    }
  }

  kf_->setCalibS0(s0);

  ivox_->reset_max_group();
  int iter_num = 0;

  kf_->UpdateObserve([&, this, s0](const ESKF::KFState &kf_state,
                                     M6 &HTVH_pose, V6 &HTVr_pose,
                                     ESKF::OBS_MAT &HTVH_calib, ESKF::OBS_VEC &HTVr_calib) {
    const SE3 pose = kf_state.pose;
    const bool need_converge = kf_state.need_converge;
    const M3d R_transpose = (pose.R_.transpose()).cast<double>();
    const bool do_calib = kf_->isCalibEnabled();
    const double inv_s0 = 1.0 / s0;  // normalisation factor

    // Phase 1: use CURRENT calib estimate for Jacobian.
    // Deskewing now also uses kf_->getExtrinsic1/2() → self-consistent.
    const SE3 T_ext1_est = kf_state.T_L1_IMU;
    const SE3 T_ext2_est = kf_state.T_L2_IMU;
    M3d R_ext1_T = T_ext1_est.R_.transpose().cast<double>();
    M3d R_ext2_T = T_ext2_est.R_.transpose().cast<double>();
    V3d t_ext1 = T_ext1_est.t_.cast<double>();
    V3d t_ext2 = T_ext2_est.t_.cast<double>();

    tbb::enumerable_thread_specific<ThreadACC> tls_acc;

    tbb::parallel_for(
      tbb::blocked_range<size_t>(0, effect_knn_num_),
      [&](const tbb::blocked_range<size_t>& r) {
        KNNHeapType top_K;
        auto& local_acc = tls_acc.local();
        for (size_t r_s = r.begin(); r_s < r.end(); ++r_s) {
          int idx = effect_knn_idxs_[r_s];
          V3& point_body = points_body_v3_[idx];
          V3 point_world = pose * point_body;

          if(!need_converge){
            top_K.reset();
            ivox_->getTopK(point_world, top_K);
            if(top_K.count < 4){
              effect_mask_[idx] = false;
              effect_knn_mask_[idx] = false;
              edge_mask_[idx] = false;
              continue;
            }
            effect_knn_mask_[idx] = true;

            // Feature extraction: retroreflective poles (high intensity) are
            // cylindrical → line features. Try line first for them, plane first
            // for normal points. This gives longitudinal constraints in corridors.
            bool is_high_intensity =
                (std::abs(ds_undistort_->points[idx].intensity) > g_intensity_thresh);

            if (is_high_intensity && g_enable_edge_feature) {
              // High-intensity: try line first (cylinder axis)
              edge_mask_[idx] = fit_line_from_points(top_K.count, top_K.points_,
                                                     line_dir_vec_[idx], line_center_vec_[idx]);
              if (edge_mask_[idx]) {
                V3& dir = line_dir_vec_[idx];
                V3& center = line_center_vec_[idx];
                V3 scan_diff = point_world - center;
                scalar scan_dist = (scan_diff - dir * dir.dot(scan_diff)).norm();
                if (scan_dist > g_edge_line_dist_thresh)
                  edge_mask_[idx] = false;
              }
              if (!edge_mask_[idx]) {
                // Line failed → fall back to plane
                effect_mask_[idx] = calc_plane_coeff(top_K.count, top_K.points_, abcd_vec_[idx]);
              } else {
                effect_mask_[idx] = false;
              }
            } else {
              // Normal point: try plane first
              effect_mask_[idx] = calc_plane_coeff(top_K.count, top_K.points_, abcd_vec_[idx]);
              if (!effect_mask_[idx] && g_enable_edge_feature) {
                edge_mask_[idx] = fit_line_from_points(top_K.count, top_K.points_,
                                                       line_dir_vec_[idx], line_center_vec_[idx]);
                if (edge_mask_[idx]) {
                  V3& dir = line_dir_vec_[idx];
                  V3& center = line_center_vec_[idx];
                  V3 scan_diff = point_world - center;
                  scalar scan_dist = (scan_diff - dir * dir.dot(scan_diff)).norm();
                  if (scan_dist > g_edge_line_dist_thresh)
                    edge_mask_[idx] = false;
                }
              } else {
                edge_mask_[idx] = false;
              }
            }
          }

          if(!effect_mask_[idx] && !edge_mask_[idx]) continue;

          // --- point-to-plane residual ---
          if(effect_mask_[idx]){
            auto& abcd = abcd_vec_[idx];
            scalar error;
            effect_mask_[idx] = compute_error(abcd, point_world, _lengths[idx], error);
            if(effect_mask_[idx]){
              V3d normvec(abcd[0], abcd[1], abcd[2]);
              V3d nb = R_transpose * normvec;
              V3d point_body_d = point_body.cast<double>();
              V6d J_pose;
              J_pose.head<3>() = point_body_d.cross(nb);
              J_pose.tail<3>() = normvec;

              double w = 1000.0 * _weights[idx];
              local_acc.HTVH += J_pose * w * J_pose.transpose();
              local_acc.HTVr -= J_pose * w * error;
              local_acc.plane_pts.push_back(point_world);

              // ── Phase 1: calibration Jacobian (point-to-plane) ──
              if (do_calib) {
                bool from_lidar2 = _is_lidar2[idx];
                const M3d& R_ext_T = from_lidar2 ? R_ext2_T : R_ext1_T;
                const V3d& t_ext = from_lidar2 ? t_ext2 : t_ext1;

                // p_raw = T_nom^{-1} * p_body (LiDAR frame, using nominal extrinsic)
                V3d p_raw = R_ext_T * (point_body_d - t_ext);
                V3d nb_lidar = R_ext_T * nb;

                // SE(3) right perturbation Jacobians (about nominal extrinsic)
                V3d J_ext_rot  = p_raw.cross(nb_lidar) * inv_s0;  // ∂r/∂δθ = -(n×p)ᵀ = (p×n)ᵀ
                V3d J_ext_trans = nb_lidar;

                // ΔC_k accumulation: pose↔extrinsic coupling (paper §3)
                V6d J_ext_full;
                J_ext_full.head<3>() = J_ext_rot;
                J_ext_full.tail<3>() = J_ext_trans;
                if (from_lidar2) {
                  local_acc.DC2.noalias() += J_pose * (w * J_ext_full.transpose());
                } else {
                  local_acc.DC1.noalias() += J_pose * (w * J_ext_full.transpose());
                }

                // 20D layout: pose(6), ext1(6), ext2(6), dt(2)
                // Ext block: 6:12 for LiDAR1, 12:18 for LiDAR2
                int ext_offset = from_lidar2 ? 12 : 6;

                // Build 20D Jacobian
                Eigen::Matrix<double, 20, 1> J_calib;
                J_calib.setZero();
                J_calib.segment<6>(0)  = J_pose.cast<double>();
                J_calib.segment<3>(ext_offset)     = J_ext_rot;
                J_calib.segment<3>(ext_offset + 3) = J_ext_trans;
                // Phase 2: time-offset Jacobian (spatiotemporal calibration)
                double J_dt = 0.0;
                if (!ds_motion_info_.empty()) {
                  V3d v_body = ds_motion_info_[idx].v_body.cast<double>();
                  J_dt = normvec.dot(pose.R_.cast<double>() * v_body) * inv_s0;
                }
                int dt_offset = from_lidar2 ? 19 : 18;
                J_calib(dt_offset) = J_dt;

                local_acc.HTVH_calib += J_calib * w * J_calib.transpose();
                local_acc.HTVr_calib -= J_calib * w * static_cast<double>(error);
              }
            }
          }

          // --- point-to-line (edge) residual ---
          if(edge_mask_[idx]){
            V3& line_dir = line_dir_vec_[idx];
            V3& line_center = line_center_vec_[idx];

            V3 diff = point_world - line_center;
            scalar proj = line_dir.dot(diff);
            V3 perp = diff - line_dir * proj;
            scalar error = perp.norm();

            if(error > 1e-6f){
              V3 n_r = perp / error;

              V3d n_r_d = n_r.cast<double>();
              V3d nb = R_transpose * n_r_d;
              V3d point_body_d = point_body.cast<double>();
              V6d J_pose;
              J_pose.head<3>() = point_body_d.cross(nb);
              J_pose.tail<3>() = n_r_d;

              double w = g_edge_weight * _weights[idx];
              // Extra boost for retroreflective pole edge features
              if (std::abs(ds_undistort_->points[idx].intensity) > g_intensity_thresh)
                w *= g_high_intensity_edge_boost;
              local_acc.HTVH += J_pose * w * J_pose.transpose();
              local_acc.HTVr -= J_pose * w * error;
              local_acc.edge_pts.push_back(point_world);

              // ── Phase 1: calibration Jacobian (point-to-line) ──
              if (do_calib) {
                bool from_lidar2 = _is_lidar2[idx];
                const M3d& R_ext_T = from_lidar2 ? R_ext2_T : R_ext1_T;
                const V3d& t_ext = from_lidar2 ? t_ext2 : t_ext1;

                V3d p_raw = R_ext_T * (point_body_d - t_ext);
                V3d nb_lidar = R_ext_T * nb;

                V3d J_ext_rot  = nb_lidar.cross(p_raw) * inv_s0;
                V3d J_ext_trans = nb_lidar;

                // ΔC_k accumulation: pose↔extrinsic coupling (paper §3)
                V6d J_ext_full;
                J_ext_full.head<3>() = J_ext_rot;
                J_ext_full.tail<3>() = J_ext_trans;
                if (from_lidar2) {
                  local_acc.DC2.noalias() += J_pose * (w * J_ext_full.transpose());
                } else {
                  local_acc.DC1.noalias() += J_pose * (w * J_ext_full.transpose());
                }

                int ext_offset = from_lidar2 ? 12 : 6;
                Eigen::Matrix<double, 20, 1> J_calib;
                J_calib.setZero();
                J_calib.segment<6>(0)  = J_pose.cast<double>();
                J_calib.segment<3>(ext_offset)     = J_ext_rot;
                J_calib.segment<3>(ext_offset + 3) = J_ext_trans;

                // Phase 2: time-offset Jacobian (point-to-line)
                double J_dt = 0.0;
                if (!ds_motion_info_.empty()) {
                  V3d v_body = ds_motion_info_[idx].v_body.cast<double>();
                  J_dt = n_r_d.dot(pose.R_.cast<double>() * v_body) * inv_s0;
                }
                int dt_offset_line = from_lidar2 ? 19 : 18;
                J_calib(dt_offset_line) = J_dt;

                local_acc.HTVH_calib += J_calib * w * J_calib.transpose();
                local_acc.HTVr_calib -= J_calib * w * static_cast<double>(error);
              }
            } else {
              edge_mask_[idx] = false;
            }
          }
        }
    });

    M6d sum_HTVH = M6d::Zero();
    V6d sum_HTVr = V6d::Zero();
    M6d sum_DC1 = M6d::Zero();
    M6d sum_DC2 = M6d::Zero();
    Eigen::Matrix<double, 20, 20> sum_HTVH_calib = decltype(sum_HTVH_calib)::Zero();
    Eigen::Matrix<double, 20, 1>  sum_HTVr_calib = decltype(sum_HTVr_calib)::Zero();
    static CloudPtr plane_feat(new PointCloudType());
    static CloudPtr edge_feat(new PointCloudType());
    plane_feat->clear();
    edge_feat->clear();
    for(const auto& local_acc : tls_acc){
      sum_HTVH += local_acc.HTVH;
      sum_HTVr += local_acc.HTVr;
      sum_DC1 += local_acc.DC1;
      sum_DC2 += local_acc.DC2;
      sum_HTVH_calib += local_acc.HTVH_calib;
      sum_HTVr_calib += local_acc.HTVr_calib;
      for (auto& p : local_acc.plane_pts) {
        PointType pt; pt.x = p.x(); pt.y = p.y(); pt.z = p.z();
        plane_feat->push_back(pt);
      }
      for (auto& p : local_acc.edge_pts) {
        PointType pt; pt.x = p.x(); pt.y = p.y(); pt.z = p.z();
        edge_feat->push_back(pt);
      }
    }
    HTVH_pose = sum_HTVH.cast<scalar>();
    HTVr_pose = sum_HTVr.cast<scalar>();
    HTVH_calib = sum_HTVH_calib.cast<scalar>();
    HTVr_calib = sum_HTVr_calib.cast<scalar>();

    // ── Paper §3: per-frame H_pose + ΔC dump for Eq.(20) prediction validation ──
    // Columns: frame, H_pose(36), ΔC1(36), ΔC2(36) — row-major. Once per frame (iter 0).
    if (g_decouple_verify && iter_num == 0) {
      static std::ofstream pdc("/home/creepert/Super-LIO/src/super_lio/logs/pose_dc.csv",
                               std::ios::app);
      pdc << frame_num_;
      for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c) pdc << "," << sum_HTVH(r, c);
      for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c) pdc << "," << sum_DC1(r, c);
      for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c) pdc << "," << sum_DC2(r, c);
      pdc << "\n";
    }

    // publish feature points for visualization
    if (!plane_feat->empty())
      data_wrapper_->pub_plane_features(plane_feat);
    if (!edge_feat->empty())
      data_wrapper_->pub_edge_features(edge_feat);

    if(need_converge) return;

    int _effect_knn_num = 0;
    for(size_t i = 0; i < effect_knn_num_; ++i){
      int idx = effect_knn_idxs_[i];
      if(!effect_knn_mask_[idx]) continue;
      effect_knn_idxs_[_effect_knn_num] = idx;
      _effect_knn_num++;
    }

    effect_knn_num_ = _effect_knn_num;

    // ── Per-frame feature diagnostics (verbose only) ──
    if (g_verbose_logging) {
      static int feat_diag_count = 0;
      if (++feat_diag_count <= 5 || feat_diag_count % 200 == 0) {
        int edge_count = 0;
        for (size_t i = 0; i < ptsize; ++i)
          if (edge_mask_[i]) edge_count++;
        int plane_count = 0;
        for (size_t i = 0; i < ptsize; ++i)
          if (effect_mask_[i]) plane_count++;

        LOG(INFO) << "[FeatDiag#" << feat_diag_count << "]"
                  << " pts=" << ptsize
                  << " knn_valid=" << effect_knn_num_
                  << " plane=" << plane_count
                  << " edge=" << edge_count
                  << " |HTVr|=" << sum_HTVr.norm();
      }
    }

    iter_num++;
  });

  frame_num_++;
}


void SuperLIO::UpdateMap() {
  const size_t ptsize = ds_undistort_->size();
  if (ptsize == 0) return;
  
  last_pose_ = kf_->GetSE3();
  points_world_v3_.resize(ptsize);
  
  const auto R = last_pose_.R_;
  const auto t = last_pose_.t_;
  
  for (size_t i = 0; i < ptsize; ++i) {
    const auto& pt = points_body_v3_[i];
    points_world_v3_[i] = R * pt + t;
  }
  
  ivox_->insert(points_world_v3_);

}


void SuperLIO::Output(){
  auto state = kf_->GetNavState();
  data_wrapper_->pub_odom(state);  

  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  transformation.block<3, 3>(0, 0) = state.R.R_.cast<float>();
  transformation.block<3, 1>(0, 3) = state.p.cast<float>();

  CloudPtr world_pc(new PointCloudType());
  
  if(g_visual_map){
    static int count = -1;
    count++;
    if(count % g_pub_step != 0){
      return;
    }
    count = 0;
    if(g_visual_dense){
      pcl::transformPointCloud(*scan_undistort_full_, *world_pc, transformation);
      data_wrapper_->pub_cloud_world(world_pc, state.timestamp);
    }else{
      pcl::transformPointCloud(*ds_undistort_, *world_pc, transformation);
      data_wrapper_->pub_cloud_world(world_pc, state.timestamp);
    }
  }

  // ── Phase 1: calibration status log ──
  if (kf_->isCalibEnabled()) {
    static int calib_log_count = 0;
    int interval = (calib_log_count < 10) ? 1 : 200;
    if (++calib_log_count % interval == 0) {
      SE3 est1 = kf_->getExtrinsic1();
      SE3 est2 = kf_->getExtrinsic2();
      V3 rpy1 = SO3(est1.R_).log_vee() * 180.0 / M_PI;
      V3 rpy2 = SO3(est2.R_).log_vee() * 180.0 / M_PI;
      LOG(INFO) << CYAN << " ---> [Calib#" << calib_log_count
                << "] ext1: t=[" << est1.t_.transpose()
                << "] rpy=[" << rpy1.transpose() << "]°"
                << " | ext2: t=[" << est2.t_.transpose()
                << "] rpy=[" << rpy2.transpose() << "]°"
                << RESET;
    }
  }
}

void SuperLIO::printTimeRecord(){
  if(!g_time_eva) return;
  time_record_.PrintAll();
}

} // namespace END.