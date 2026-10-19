
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

// Displacement the platform must cover before the route direction is latched (manual
// grade reference). Large enough that no stationary jitter can reach it, small enough
// that the first straight stretch defines the direction.
static constexpr double kRouteLatchDist = 3.0;

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
  // Stationarity test for gravity / accelerometer-bias initialisation — the same
  // criterion as the relocalisation path (super_lio_reloc.cpp, "Fix 1").
  // Averaging motion samples biases the gravity direction; the resulting vertical
  // specific-force error is unobservable in SLAM (the map is built from the
  // estimate itself) and integrates into a severe z sag.
  constexpr double kStaticAccTol = 1.5;    // | ‖a‖ − g |  [m/s²]
  constexpr double kStaticGyrTol = 0.15;   // ‖ω‖          [rad/s]
  constexpr int    kMinStatic    = 100;    // stationary samples required
  constexpr int    kMaxTotal     = 400;    // hard deadline: init must never hang

  static int static_cout = 0;              // samples that passed the stationary test
  static int total_cout  = 0;              // every sample seen (deadline + fallback)
  static V3 mean_gyro = V3::Zero();
  static V3 mean_acce = V3::Zero();
  static V3 raw_mean_gyro = V3::Zero();
  static V3 raw_mean_acce = V3::Zero();

  for(auto& imu: measures_.imu){
    total_cout++;
    raw_mean_gyro += (imu.gyr - raw_mean_gyro) / total_cout;
    raw_mean_acce += (imu.acc - raw_mean_acce) / total_cout;

    if(std::abs(imu.acc.norm() - g_gravity_norm) < kStaticAccTol &&
       imu.gyr.norm() < kStaticGyrTol){
      static_cout++;
      mean_gyro += (imu.gyr - mean_gyro) / static_cout;
      mean_acce += (imu.acc - mean_acce) / static_cout;
    }
  }

  if(static_cout < kMinStatic){
    if(total_cout < kMaxTotal){
      return false;
    }
    // Deadline reached: proceed rather than hang, but report the shortfall.
    if(static_cout > 0){
      LOG(WARNING) << YELLOW
                   << " ---> [SuperLIO] Only " << static_cout << "/" << kMinStatic
                   << " stationary IMU samples (" << total_cout << " total) — gravity/bias"
                   << " init is unreliable. Keep the platform still at startup!" << RESET;
    }else{
      LOG(WARNING) << YELLOW
                   << " ---> [SuperLIO] No stationary IMU sample in " << total_cout
                   << " samples — falling back to the raw mean. Gravity/bias init is"
                   << " unreliable. Keep the platform still at startup!" << RESET;
      mean_gyro = raw_mean_gyro;
      mean_acce = raw_mean_acce;
    }
  }else{
    LOG(INFO) << GREEN << " ---> [SuperLIO] Gravity init from " << static_cout
              << "/" << total_cout << " stationary IMU samples." << RESET;
  }

  if(mean_acce.norm() < 1e-3){
    return false;   // nothing usable yet, keep accumulating
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

  // The ESKF applies the measurement as  acc = imu_scale * acc_raw - ba  (ESKF.cpp,
  // Predict / Propagation_Undistort).  imu_scale = g / ‖ā‖ already normalises the
  // static magnitude to g, i.e. it absorbs the radial part of the accelerometer
  // bias.  Subtracting mean_acce * (1 - imu_scale) on top of that double-counts the
  // bias and leaves the propagated specific force with a residual of exactly -ba —
  // a constant vertical error that is unobservable in SLAM and integrates into the
  // z sag.  The bias actually consistent with the scaled measurement is this residual.
  V3 estimated_ba = imu_scale * mean_acce - mean_acce.normalized() * g_gravity_norm;

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

  // (C1) 初始化阶段仍要求 lidar1 帧：lidar2-only 组不入图，等 lidar1 恢复
  //（避免空指针 pc==nullptr，也避免用单雷达完成建图初始化的半初始化状态）
  if (!measures_.has_lidar1 || !measures_.lidar.pc) {
    if (frame_num_ <= 3) {
      LOG(WARNING) << YELLOW << " ---> [Dual-LiDAR] map_init skipped: no lidar1 frame in group" << RESET;
    }
    return false;
  }

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
  std::string output_map_name = ((g_map_name.size() && g_map_name[0] == '/') ? g_map_name : (g_save_map_dir + "/" + g_map_name));

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
    std::string map_name = ((g_map_name.size() && g_map_name[0] == '/') ? g_map_name : (g_save_map_dir + "/" + g_map_name));
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
  const auto seed_state = kf_->GetDynamicState();   // ★ 2026-10-19 Plan A: 播种态快照(用于守卫回滚)
  const auto seed_sys   = kf_->GetSysState();       //   对应 SetX() 需要的类型
  propagate_states_.emplace_back(seed_state);
  // (C1) 本组的时间基准：有 lidar1 用它，否则用 lidar2（lidar2-only 组）
  kf_->SetObsTime(measures_.has_lidar1 ? measures_.lidar.end_time : measures_.lidar2.end_time);
  for (auto &imu : measures_.imu) {
    kf_->Predict(imu);
    propagate_states_.emplace_back(kf_->GetDynamicState());
  }
  // ★ 2026-10-19 Plan A —— 单帧位移守卫。
  //   背景: init 阻塞 ~35 s, 期间 last_obs_time_/时间记账停在 init 前; init 后第一帧的
  //   IMU 传播会把状态甩出数米(实测 7.1–7.9 m), 依此做 deskew 则点云被打散、knn_valid=0,
  //   整段退化为纯航位推算。本机器人一帧(<=0.2 s)物理上不可能位移 0.5 m, 故超限即判为
  //   时间记账不连续: 回滚滤波器状态到播种态、丢弃该帧 IMU 块, 并打 WARNING。
  {
    const double dp = (propagate_states_.back().p - seed_state.p).norm();
    static long guard_hits = 0; static long guard_frames = 0; ++guard_frames;
    if (dp > 0.5) {
      ++guard_hits;
      if (guard_hits <= 5 || guard_hits % 100 == 0)
        LOG(WARNING) << "[PropGuard] frame propagation displaced " << dp
                     << " m in one frame -> discard IMU block, restore seed "
                     << "(hit " << guard_hits << "/" << guard_frames << ")";
      kf_->SetX(seed_sys);
      propagate_states_.clear();
      propagate_states_.emplace_back(seed_state);
      kf_->SetObsTime(measures_.has_lidar1 ? measures_.lidar.end_time : measures_.lidar2.end_time);
    }
  }
  // per-frame reset of the deskew-fallback accounting (reported next to [FeatDiag])
  {  // P0-4 step1: report the PREVIOUS call population split, then reset
    static long pu_calls = 0;
    ++pu_calls;
    if (pu_calls <= 4 || pu_calls % 200 == 0)
      LOG(INFO) << "[PU] call=" << pu_calls
                << " prev below=" << stale_lo_n1_.load(std::memory_order_relaxed)
                << " inside=" << stale_in_n1_.load(std::memory_order_relaxed)
                << " above=" << stale_n1_.load(std::memory_order_relaxed)
                << " | imu_states=" << measures_.imu.size();
  }
  stale_n1_.store(0, std::memory_order_relaxed);
  stale_in_n1_.store(0, std::memory_order_relaxed);
  stale_lo_n1_.store(0, std::memory_order_relaxed);
  stale_n2_.store(0, std::memory_order_relaxed);
  stale_dt1_.store(0, std::memory_order_relaxed);
  stale_dt2_.store(0, std::memory_order_relaxed);

  const SE3 T_end = kf_->GetSE3();
  const M3  R_inv = T_end.R_.transpose();
  const V3  T_end_t = T_end.t_;

  // P0-5 positive control: measure the displacement the deskew actually applies,
  // so "deskew off changes nothing" can be told apart from "the switch did not
  // take effect".  Declared at FUNCTION scope: the log that reads them sits after
  // the LiDAR-1 block closes, which is what broke the previous build.
  std::atomic<double> dsk_sum{0.0};
  std::atomic<long>   dsk_n{0};

  // ── LiDAR 1 deskew ──────────────────────────────────────────
  {
    // Phase 1: use current calib estimate (not fixed nominal) for self-consistency
    const SE3 T_ext1 = kf_->getExtrinsic1();
    const M3 TLI_R = T_ext1.R_;
    const V3 TLI_t = T_ext1.t_;
    const double start_time = measures_.lidar.start_time;
    auto& raw_pc = measures_.lidar.pc;

    // (C1) lidar2-only 组：raw_pc == nullptr ⇒ ptsize=0，本块整体空转（后续按实际 size 合并 lidar2）
    std::size_t ptsize = raw_pc ? raw_pc->points.size() : 0;
    if (raw_pc && kf_->GetTime() > 1791880426.60 && kf_->GetTime() < 1791880427.40) {
      const double kf6 = kf_->GetTime();
      const double lt6 = measures_.lidar.end_time;
      FILE* ft = std::fopen("/tmp/rev106_frames.csv", "a");
      if (ft) { std::fprintf(ft, "%.6f,%.6f,%zu\n", kf6, lt6, raw_pc->points.size()); std::fclose(ft); }
      char nm[128];
      std::snprintf(nm, sizeof(nm), "/tmp/rev106_raw_%.3f.csv", kf6);
      FILE* f = std::fopen(nm, "w");
      if (f) {
        char b[192];
        for (std::size_t i = 0; i < raw_pc->points.size(); ++i) {
          const auto& p = raw_pc->points[i];
          int k = std::snprintf(b, sizeof(b), "%zu,%.6f,%.6f,%.6f,%.4f", i, p.x, p.y, p.z, p.intensity);
          b[k++] = 10; std::fwrite(b, 1, static_cast<std::size_t>(k), f);
        }
        std::fclose(f);
      }
    }
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
          stale_n1_.fetch_add(1, std::memory_order_relaxed);
          stale_dt1_.fetch_add(static_cast<long>((query_time - propagate_states_.back().time) * 1e6),
                               std::memory_order_relaxed);
          continue;
        }
        // P0-4: query_time < front <=> the bracket search finds nothing and
        // match_iter silently stays at begin(): the uncounted FRONT miss.
        {
          const double tf = propagate_states_.front().time;
          const double tb = propagate_states_.back().time;
          if (query_time < tf) {
            stale_lo_n1_.fetch_add(1, std::memory_order_relaxed);
            // (batch4-item1a) This MUST live inside the `query_time < tf` branch.
            // It used to sit one level out (mis-indented), so it counted EVERY
            // point and reported 47,565,970 -- "half the points fall off the front
            // of the IMU window" -- which was a pure placement artefact and has
            // been retracted.  `below` is 0 in every call that was ever measured.
            stale_lo_total_.fetch_add(1, std::memory_order_relaxed);
          } else if (query_time <= tb) {
            stale_in_n1_.fetch_add(1, std::memory_order_relaxed);
          }
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
        // P0-5: with deskew_mode=1 every point keeps its own frame -- i.e. NO
        // motion compensation at all.  Setting R_i to the frame-end rotation and
        // t_ei to zero makes R_inv*(R_end*(TLI_R*raw+TLI_t)) collapse to exactly
        // (TLI_R*raw + TLI_t), the untouched point in the IMU body frame.
        if (g_deskew_mode != 0) { R_i = T_end.R_; t_ei.setZero(); }
        V3 eigen_point = R_inv * (R_i * (TLI_R * raw + TLI_t) + t_ei);
        {
          const V3 raw_imu = TLI_R * raw + TLI_t;
          dsk_sum.fetch_add((eigen_point - raw_imu).norm(), std::memory_order_relaxed);
          dsk_n.fetch_add(1, std::memory_order_relaxed);
        }
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

  {  // P0-5 one-shot: what the deskew actually moved, per mode
    static int dsk_log = 0;
    if (dsk_log < 4) {
      ++dsk_log;
      const long dn = dsk_n.load(std::memory_order_relaxed);
      LOG(INFO) << "[Deskew] mode=" << g_deskew_mode << " n=" << dn
                << " mean|deskew displacement| = "
                << (dn ? dsk_sum.load(std::memory_order_relaxed) / dn * 1e3 : 0.0)
                << " mm";
    }
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
          stale_n2_.fetch_add(1, std::memory_order_relaxed);
          stale_dt2_.fetch_add(static_cast<long>((query_time - propagate_states_.back().time) * 1e6),
                               std::memory_order_relaxed);
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
  if (kf_->GetTime() > 1791880426.60 && kf_->GetTime() < 1791880427.40) {
    const double kf6 = kf_->GetTime();
    char nm[128];
    std::snprintf(nm, sizeof(nm), "/tmp/rev106_ds_%.3f.csv", kf6);
    FILE* f = std::fopen(nm, "w");
    if (f) {
      char b[192];
      for (std::size_t i = 0; i < ds_undistort_->size(); ++i) {
        const auto& p = ds_undistort_->points[i];
        int k = std::snprintf(b, sizeof(b), "%zu,%.6f,%.6f,%.6f,%.4f", i, p.x, p.y, p.z, p.intensity);
        b[k++] = 10; std::fwrite(b, 1, static_cast<std::size_t>(k), f);
      }
      std::fclose(f);
    }
    std::vector<float> mv; ivox_->getMap(mv);
    std::snprintf(nm, sizeof(nm), "/tmp/rev106_map_%.3f.csv", kf6);
    FILE* fm = std::fopen(nm, "w");
    if (fm) {
      char b[128];
      for (std::size_t i = 0; i + 2 < mv.size(); i += 3) {
        int k = std::snprintf(b, sizeof(b), "%.6f,%.6f,%.6f", mv[i], mv[i+1], mv[i+2]);
        b[k++] = 10; std::fwrite(b, 1, static_cast<std::size_t>(k), fm);
      }
      std::fclose(fm);
    }
    FILE* fd = std::fopen("/tmp/rev106_dscount.csv", "a");
    if (fd) { std::fprintf(fd, "%.6f,%zu,%zu\n", kf6, ds_undistort_->size(), mv.size()/3); std::fclose(fd); }
  }
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
  // ── Structural-regularity diagnostics (per frame, iteration 0 only) ──
  // "The constraint had no effect" and "the constraint never bit" are
  // indistinguishable without these: n_* prove planes were actually snapped, and
  // sum_dtheta_* say how many degrees the snap was trying to correct.
  int    n_snap_wall = 0;
  double sum_dtheta_wall = 0.0;     // radians
  // Raw plane-fit outcome and classification, before any snapping. Tells apart
  // "no ground plane is ever fitted" from "ground planes exist but are not snapped".
  int    n_fit_ok = 0;
  int    n_fit_fail = 0;
  int    n_cls_ground = 0;
  int    n_cls_wall = 0;
  int    n_cls_mid = 0;
};


// Robust plane fit of the ground from the CURRENT scan, in the body (IMU) frame.
// The map's own ground planes are unusable for the structural prior: they were built
// by the drifting estimate, so they carry the very tilt we want to measure, and
// re-orienting them while keeping the plane through the same map points leaves the
// residual (and hence the attitude leverage) almost unchanged. Fitting the scan
// directly gives an observation of the terrain that is independent of the map.
bool SuperLIO::fit_scan_ground(V3& n_body, int& n_inliers) const {
  n_inliers = 0;
  if (!ds_undistort_) return false;
  const std::size_t n = ds_undistort_->size();
  if (n < 200) return false;

  static std::vector<V3d> pts;   // static: avoids a per-frame allocation
  pts.clear();
  pts.reserve(n / 4);
  for (std::size_t i = 0; i < n; ++i) {
    const auto& p = ds_undistort_->points[i];
    if (p.z < -0.2f && p.z > -3.0f)   // below the sensor, i.e. the ground band
      pts.emplace_back(static_cast<double>(p.x), static_cast<double>(p.y),
                       static_cast<double>(p.z));
  }
  if (pts.size() < 100) return false;

  V3d nrm(0.0, 0.0, 1.0);
  std::vector<char> keep(pts.size(), 1);
  V3d c = V3d::Zero();
  for (int it = 0; it < 3; ++it) {
    c.setZero();
    int m = 0;
    for (std::size_t i = 0; i < pts.size(); ++i)
      if (keep[i]) { c += pts[i]; ++m; }
    if (m < 50) return false;
    c /= static_cast<double>(m);
    M3d C = M3d::Zero();
    for (std::size_t i = 0; i < pts.size(); ++i)
      if (keep[i]) { const V3d d = pts[i] - c; C.noalias() += d * d.transpose(); }
    Eigen::SelfAdjointEigenSolver<M3d> es(C);
    nrm = es.eigenvectors().col(0);
    if (nrm.z() < 0.0) nrm = -nrm;
    int kept = 0;
    for (std::size_t i = 0; i < pts.size(); ++i) {
      const bool in = std::abs((pts[i] - c).dot(nrm)) < 0.15;
      keep[i] = in ? 1 : 0;
      kept += in ? 1 : 0;
    }
    n_inliers = kept;
  }
  // A ground plane must be roughly horizontal in the body frame; reject anything
  // else (e.g. a wall face caught by the z-band) so a bogus reference never reaches
  // the pitch channel.
  if (nrm.z() < 0.7) return false;
  n_body = nrm.cast<scalar>();
  return n_inliers >= 100;
}


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

  // ── Horizontal direction of travel (world frame), for the structural ground prior ──
  // The terrain's grade is only defined relative to the direction of travel: the
  // ground channel removes the component of the floor normal along this direction
  // (the longitudinal grade) and leaves the cross-slope alone. Direction is taken
  // from the ESKF velocity and latched, because it is ill-defined at low speed.
  {
    const V3 vel = kf_->GetNavState().v;
    const double v_h = std::hypot(vel.x(), vel.y());
    if (v_h > 0.15) {
      travel_dir_w_ = V3(vel.x() / v_h, vel.y() / v_h, 0.0);
      travel_dir_valid_ = true;
    }
  }
  const bool have_travel_dir = travel_dir_valid_;
  const V3d t_w = travel_dir_w_.cast<double>();
  const V3d p_w(-t_w.y(), t_w.x(), 0.0);   // horizontal perpendicular
  // Reference longitudinal grade (tan of the angle). Captured terrain normal wins;
  // otherwise fall back to the configured site parameter.
  double grade_ref = std::tan(g_structure_ground_grade_deg * M_PI / 180.0);
  if (ground_ref_done_ && ground_ref_count_ > 0) {
    V3d g_ref = ground_ref_w_.cast<double>();
    if (g_ref[2] < 0) g_ref = -g_ref;
    if (g_ref[2] > 1e-3) grade_ref = -(g_ref.dot(t_w)) / g_ref[2];
  } else if (have_travel_dir) {
    // Manual reference: the configured scalar is defined along the ROUTE direction, so it
    // flips with the direction of travel just as the projected captured normal does.
    // Without the flip the residual r = s − grade_ref (s = longitudinal grade along the
    // CURRENT direction of travel) demands the same signed slope after a 180° turn, i.e. a
    // climb in both directions: on 5#GJD (377 m out, 0.566°) that biased the return leg by
    // 2×0.566° and the terminal z by ~7 m, while the out leg matched the truth to 0.02°.
    // The route direction comes from the DISPLACEMENT once it exceeds kRouteLatchDist —
    // latching the instantaneous velocity instead mis-signs the WHOLE run when the first
    // frames past init jitter over the 0.15 m/s gate (measured: z profile mirrored).
    const V3d p_now = kf_->GetNavState().p.cast<double>();
    if (!travel_dir0_valid_) {
      if (!travel_dir0_pending_) {
        travel_dir0_p0_ = kf_->GetNavState().p;
        travel_dir0_pending_ = true;
      }
      const V3d d = p_now - travel_dir0_p0_.cast<double>();
      const double dn = std::hypot(d.x(), d.y());
      if (dn > kRouteLatchDist) {
        travel_dir0_w_ = V3(d.x() / dn, d.y() / dn, 0.0);
        travel_dir0_valid_ = true;
        LOG(INFO) << GREEN << " ---> [Structure] Route direction latched from " << dn
                  << " m of travel; manual grade reference flips at the turn-around."
                  << RESET;
      }
    }
    if (travel_dir0_valid_ && t_w.dot(travel_dir0_w_.cast<double>()) < 0.0) {
      grade_ref = -grade_ref;
    }
  }
  const bool have_grade_ref = have_travel_dir && (ground_ref_done_ || !g_structure_ground_grade_auto);

  // Fit the ground from the CURRENT scan (body frame) for the pitch channel. Done
  // once per frame, outside the IEKF loop: it depends only on the scan, not on the
  // pose, so it is a genuine map-independent observation of the terrain.
  scan_ground_valid_ = g_structure_ground_enabled &&
                       fit_scan_ground(scan_ground_n_b_, scan_ground_inliers_);
  double grade_resid_tan = 0.0;   // diagnostics: tan(grade error) fed to the filter

  // One-shot confirmation that the ground channel actually engages (it stays silent
  // until the direction of travel is known, i.e. until the platform starts moving).
  static bool grade_logged = false;
  if (g_structure_ground_enabled && have_grade_ref && !grade_logged) {
    LOG(INFO) << GREEN << " ---> [Structure] Ground channel active: reference"
              << " longitudinal grade = " << std::atan(grade_ref) * 180.0 / M_PI
              << " deg; cross-slope preserved, roll untouched." << RESET;
    grade_logged = true;
  }

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

            // ── Plane-fit outcome + raw classification (iteration 0 only) ──
            // Counted BEFORE any snapping, so |n_z| is the map plane's own tilt.
            if (iter_num == 0) {
              const double _lim = g_structure_max_angle_deg * M_PI / 180.0;
              if (effect_mask_[idx]) {
                local_acc.n_fit_ok++;
                const double az = std::abs(abcd_vec_[idx][2]);
                if (az > std::cos(_lim))      local_acc.n_cls_ground++;
                else if (az < std::sin(_lim)) local_acc.n_cls_wall++;
                else                          local_acc.n_cls_mid++;
              } else {
                local_acc.n_fit_fail++;
              }
            }

            // ── Wall channel: vertical planes constrain ROLL (yaw left free) ──
            // The ground channel deliberately does NOT use this mechanism. A map plane
            // whose normal is re-oriented but which still passes through the same map
            // points yields a residual of only |p_w − c|·sin(dtheta) — measured at
            // 0.17 m × sin(8.4°) = 0.024 m, i.e. ~0.14° of attitude leverage over a
            // 10 m lever arm. That was measured to be a near no-op, so the pitch
            // channel is a dedicated scan-based residual instead (added after the pose
            // block is accumulated, below).
            if (g_structure_wall_enabled && effect_mask_[idx]) {
              auto& abcd = abcd_vec_[idx];
              const double lim = g_structure_max_angle_deg * M_PI / 180.0;
              if (std::abs(abcd[2]) < std::sin(lim)) {
                const V3d n_orig(abcd[0], abcd[1], abcd[2]);
                const double h = std::hypot(abcd[0], abcd[1]);
                const V3d n_reg(abcd[0] / h, abcd[1] / h, 0.0);
                // Keep the plane through the same inlier centroid: only the
                // orientation is regularised, not the surface position.
                V3d c = V3d::Zero();
                for (int j = 0; j < top_K.count; ++j) c += top_K.points_[j].cast<double>();
                c /= static_cast<double>(top_K.count);
                abcd[0] = n_reg[0];
                abcd[1] = n_reg[1];
                abcd[2] = n_reg[2];
                abcd[3] = -n_reg.dot(c);
                if (iter_num == 0) {
                  local_acc.n_snap_wall++;
                  local_acc.sum_dtheta_wall +=
                      std::acos(std::min(1.0, std::abs(n_orig.dot(n_reg))));
                }
              }
            }
          }

          if(!effect_mask_[idx] && !edge_mask_[idx]) continue;

          // --- point-to-plane residual ---
          if(effect_mask_[idx]){
            auto& abcd = abcd_vec_[idx];
            scalar error;
            const bool ok6 = compute_error(abcd, point_world, _lengths[idx], error);
            if (kf_->GetTime() > 1791880426.60 && kf_->GetTime() < 1791880427.40) {
              static FILE* f6 = std::fopen("/tmp/rev106_assoc.csv", "a");
              if (f6) {
                char b[512];
                int k = std::snprintf(b, sizeof(b), "%.3f,%d,%.6f,%d,%.6f,%.6f,%.6f",
                                      kf_->GetTime(), iter_num, _lengths[idx], ok6 ? 1 : 0,
                                      point_world[0], point_world[1], point_world[2]);
                for (int q = 0; q < top_K.count && k < 460; ++q)
                  k += std::snprintf(b + k, sizeof(b) - k, ",%.6f,%.6f,%.6f",
                                     top_K.points_[q][0], top_K.points_[q][1], top_K.points_[q][2]);
                k += std::snprintf(b + k, sizeof(b) - k, ",%d", static_cast<int>(top_K.count));
                b[k++] = 10; std::fwrite(b, 1, static_cast<std::size_t>(k), f6);
              }
            }
            effect_mask_[idx] = ok6;
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
              } else if (g_pose_dc_dump) {
                // ΔC 仅在导出打开、且在线标定关闭时补算：位姿-外参耦合是纯几何量，
                // 不依赖标定状态是否在线估计（关标定对照组同样需要 ΔC 做式(20) 验证）。
                bool from_lidar2 = _is_lidar2[idx];
                const M3d& R_ext_T = from_lidar2 ? R_ext2_T : R_ext1_T;
                const V3d& t_ext = from_lidar2 ? t_ext2 : t_ext1;
                V3d p_raw = R_ext_T * (point_body_d - t_ext);
                V3d nb_lidar = R_ext_T * nb;
                V6d J_ext_full;
                J_ext_full.head<3>() = p_raw.cross(nb_lidar) * inv_s0;
                J_ext_full.tail<3>() = nb_lidar;
                if (from_lidar2) {
                  local_acc.DC2.noalias() += J_pose * (w * J_ext_full.transpose());
                } else {
                  local_acc.DC1.noalias() += J_pose * (w * J_ext_full.transpose());
                }
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
              } else if (g_pose_dc_dump) {
                // 同平面分支：在线标定关闭时补算 ΔC（点-线残差沿用本分支的叉积次序）。
                bool from_lidar2 = _is_lidar2[idx];
                const M3d& R_ext_T = from_lidar2 ? R_ext2_T : R_ext1_T;
                const V3d& t_ext = from_lidar2 ? t_ext2 : t_ext1;
                V3d p_raw = R_ext_T * (point_body_d - t_ext);
                V3d nb_lidar = R_ext_T * nb;
                V6d J_ext_full;
                J_ext_full.head<3>() = nb_lidar.cross(p_raw) * inv_s0;
                J_ext_full.tail<3>() = nb_lidar;
                if (from_lidar2) {
                  local_acc.DC2.noalias() += J_pose * (w * J_ext_full.transpose());
                } else {
                  local_acc.DC1.noalias() += J_pose * (w * J_ext_full.transpose());
                }
              }
            } else {
              edge_mask_[idx] = false;
            }
          }
        }
    });

    // ── Capture the terrain normal during the initial stationary segment ──
    // The ground is NOT necessarily horizontal, so we must not assume it is: we
    // record the terrain's own normal once, while the gravity-aligned initialisation
    // is still trustworthy, and only ever enforce its component along the direction
    // of travel (the longitudinal grade) afterwards. Taken from the map planes that
    // were just fitted, so it costs nothing extra. Never blocks: if the platform is
    // already moving, or no ground plane is seen, it gives up and falls back to the
    // configured lio.structure.ground_grade_deg.
    if (g_structure_ground_enabled && g_structure_ground_grade_auto && !ground_ref_done_) {
      const V3 vel = kf_->GetNavState().v;
      const bool still = (std::hypot(vel.x(), vel.y()) < 0.10);
      ground_ref_frames_++;
      if (still) {
        const double cos_lim = std::cos(g_structure_max_angle_deg * M_PI / 180.0);
        for (size_t k = 0; k < effect_knn_num_; ++k) {
          const int idx = effect_knn_idxs_[k];
          if (!effect_mask_[idx]) continue;
          const auto& a = abcd_vec_[idx];
          if (std::abs(a[2]) < cos_lim) continue;   // ground / ceiling only
          V3d n(a[0], a[1], a[2]);
          if (n[2] < 0.0) n = -n;
          ground_ref_w_ += (n.cast<scalar>() - ground_ref_w_) /
                           static_cast<scalar>(ground_ref_count_ + 1);
          ground_ref_count_++;
        }
      }
      if (ground_ref_count_ >= 2000 || ground_ref_frames_ >= 40) {
        if (ground_ref_count_ > 0) {
          ground_ref_w_.normalize();
          const double tilt = std::acos(std::min(
              1.0, static_cast<double>(ground_ref_w_.z()))) * 180.0 / M_PI;
          LOG(INFO) << GREEN << " ---> [Structure] Terrain normal captured from "
                    << ground_ref_count_ << " plane samples over " << ground_ref_frames_
                    << " frames: tilt from vertical = " << tilt
                    << " deg. Only the longitudinal grade is held to this reference;"
                    << " the cross-slope is left untouched." << RESET;
        } else {
          LOG(WARNING) << YELLOW << " ---> [Structure] No ground plane seen during the"
                       << " initial stationary segment — falling back to"
                       << " lio.structure.ground_grade_deg." << RESET;
        }
        ground_ref_done_ = true;
      }
    }

    M6d sum_HTVH = M6d::Zero();
    V6d sum_HTVr = V6d::Zero();
    M6d sum_DC1 = M6d::Zero();
    M6d sum_DC2 = M6d::Zero();
    Eigen::Matrix<double, 20, 20> sum_HTVH_calib = decltype(sum_HTVH_calib)::Zero();
    Eigen::Matrix<double, 20, 1>  sum_HTVr_calib = decltype(sum_HTVr_calib)::Zero();
    int    n_snap_wall = 0;
    double sum_dtheta_wall = 0.0;
    int    n_fit_ok = 0, n_fit_fail = 0, n_cls_ground = 0, n_cls_wall = 0, n_cls_mid = 0;
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
      n_snap_wall += local_acc.n_snap_wall;
      sum_dtheta_wall += local_acc.sum_dtheta_wall;
      n_fit_ok += local_acc.n_fit_ok;
      n_fit_fail += local_acc.n_fit_fail;
      n_cls_ground += local_acc.n_cls_ground;
      n_cls_wall += local_acc.n_cls_wall;
      n_cls_mid += local_acc.n_cls_mid;
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

    // ── Scan-based ground prior: the PITCH channel, with full leverage ──
    // Constrains the CURRENT scan's ground normal — not the map's — so the Jacobian
    // is O(1) and the lever arm is the whole ground patch, instead of the ~0.17 m
    // 5-neighbour patch that made the map-normal snap a near no-op. Only the
    // component along the direction of travel (the longitudinal grade) is
    // constrained; the real cross-slope is left free, so roll is untouched.
    if (g_structure_ground_enabled && scan_ground_valid_ && have_grade_ref) {
      const M3d Rw = pose.R_.cast<double>();
      const V3d n_w = Rw * scan_ground_n_b_.cast<double>();
      const double cz = n_w.z();
      if (std::abs(cz) > 0.3) {
        const double a = n_w.dot(t_w);
        const double s = -a / cz;         // tan(longitudinal grade) of the current scan
        const double r = s - grade_ref;   // residual, tan units (≈ rad for small angles)
        // ds/dn_w = -(cz*t - a*z)/cz²   ;   dn_w/d(dtheta) = -[n_w]x R
        const V3d u = (cz * t_w - a * V3d(0.0, 0.0, 1.0)) / (cz * cz);
        const V3d J = Rw.transpose() * (u.cross(n_w));  // body-frame rotation Jacobian
        if (J.dot(J) > 1e-12) {
          const double w = g_structure_ground_weight;
          HTVH_pose.block<3, 3>(0, 0) += (w * J * J.transpose()).cast<scalar>();
          HTVr_pose.head<3>() -= (w * J * r).cast<scalar>();
          if (iter_num == 0) grade_resid_tan = r;
        }
      }
    }

    // ── Structural-regularity diagnostics ──────────────────────────────────
    // Answers "did the constraint bite, and by how much?" — the question a null
    // z-drift result cannot answer on its own. Written per frame (iter 0) to
    // logs/structure.csv, plus a periodic INFO summary.
    //   grade_resid_deg  = the pitch residual the scan-based prior actually fed the
    //                      filter (tan of the longitudinal-grade error, in degrees).
    //                      This is the number that must be non-trivial for the pitch
    //                      channel to have any chance of overriding the drifting map.
    //   ground_inliers   = ground points behind the scan fit (0 ⇒ channel idle).
    //   dtheta_wall      = how far the wall snap moved the map normals (roll channel).
    if (g_structure_wall_enabled || g_structure_ground_enabled) {
      const double dth_w = (n_snap_wall > 0)
          ? sum_dtheta_wall / n_snap_wall * 180.0 / M_PI : 0.0;
      const double grade_deg = grade_resid_tan * 180.0 / M_PI;
      if (iter_num == 0) {
        static std::ofstream st("/home/creepert/Super-LIO/src/super_lio/logs/structure.csv",
                                std::ios::app);
        st << frame_num_ << "," << n_snap_wall << "," << dth_w << ","
           << n_fit_ok << "," << n_fit_fail << ","
           << n_cls_ground << "," << n_cls_wall << "," << n_cls_mid << ","
           << (scan_ground_valid_ ? scan_ground_inliers_ : 0) << ","
           << grade_deg << "," << (scan_ground_valid_ ? 1 : 0) << "\n";
        static int st_count = 0;
        if (++st_count <= 3 || st_count % 200 == 0) {
          LOG(INFO) << "[Structure#" << st_count << "]"
                    << " fit_ok=" << n_fit_ok << " fit_fail=" << n_fit_fail
                    << " cls g/w/mid=" << n_cls_ground << "/" << n_cls_wall
                    << "/" << n_cls_mid
                    << " | snap_wall=" << n_snap_wall << " dtheta_wall=" << dth_w << "deg"
                    << " | ground_inliers=" << (scan_ground_valid_ ? scan_ground_inliers_ : 0)
                    << " grade_resid=" << grade_deg << "deg"
                    << (scan_ground_valid_ ? "" : " (ground channel IDLE)");
        }
      }
    }

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

    // ── Paper §3 (extended): H_pose + ΔC1/ΔC2 + timestamps + current relative extrinsic ──
    // Columns: frame, scan_time, H_pose(36), ΔC1(36), ΔC2(36), rel_rpy(3), rel_xyz(3) — row-major.
    // rel_* is T̂_L1⁻¹·T̂_L2 (rotation as angle-axis vector, translation in L1 frame); the analysis
    // script subtracts the converged (physical) value to obtain ξ_Δ for the Eq.(20) projection.
    if (g_pose_dc_dump && iter_num == 0) {
      const SE3 T1 = kf_->getExtrinsic1();
      const SE3 T2 = kf_->getExtrinsic2();
      M3d R1T = T1.R_.transpose().cast<double>();
      M3d R_rel = R1T * T2.R_.cast<double>();
      V3d t_rel = R1T * (T2.t_.cast<double>() - T1.t_.cast<double>());
      Eigen::AngleAxisd aa(R_rel);

      static std::ofstream pdc2("/home/creepert/Super-LIO/src/super_lio/logs/pose_dc_v2.csv",
                                std::ios::app);
      // 时间戳用最新 IMU 采样时间：lidar.end_time 在重定位模式下不逐帧更新（恒为常量）
      pdc2 << frame_num_ << ","
           << (measures_.imu.empty() ? 0.0 : measures_.imu.back().secs);
      for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c) pdc2 << "," << sum_HTVH(r, c);
      for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c) pdc2 << "," << sum_DC1(r, c);
      for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c) pdc2 << "," << sum_DC2(r, c);
      V3d rvec = aa.axis() * aa.angle();
      for (int i = 0; i < 3; ++i) pdc2 << "," << rvec(i);
      for (int i = 0; i < 3; ++i) pdc2 << "," << t_rel(i);
      pdc2 << "\n";
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
        {   // deskew-fallback accounting: a spike ⇒ the IMU bundle ended before the scan
          const long n1 = stale_n1_.load(std::memory_order_relaxed);
          const long n2 = stale_n2_.load(std::memory_order_relaxed);
          LOG(INFO) << "[Stale#" << feat_diag_count << "]"
                    << " L1 n=" << n1
                    << " dt=" << (n1 ? double(stale_dt1_.load(std::memory_order_relaxed)) / n1 / 1e6 : 0.0)
                    << "s | LO n=" << stale_lo_n1_.load(std::memory_order_relaxed)
                    << " (total " << stale_lo_total_.load(std::memory_order_relaxed) << ")"
                    << "s | L2 n=" << n2
                    << " dt=" << (n2 ? double(stale_dt2_.load(std::memory_order_relaxed)) / n2 / 1e6 : 0.0)
                    << "s";
        }
      }
    }

    // ── M20 per-iteration CSV tap (2026-10-14) ── ungated, so Sum(dRz) per
    //    frame is exact and the [Bias#] %2 print factor is not needed.
    {
      static FILE* m20_csv = nullptr;
      static long m20_rows = 0;
      if (!m20_csv) {
        m20_csv = std::fopen("/tmp/m20_trace.csv", "w");
        if (m20_csv) {
          std::fprintf(m20_csv,
              "t_data,frame,iter,dRz,wz,bgz,pts,knn_valid,plane,edge,htv,cond,dxlong,htv_y,pinvh_yy\n");
          std::fflush(m20_csv);
        } else {
          LOG(ERROR) << "[M20CSV] cannot open /tmp/m20_trace.csv";
        }
      }
      if (m20_csv) {
        int ec = 0, pc = 0;
        for (size_t i = 0; i < ptsize; ++i) if (edge_mask_[i]) ec++;
        for (size_t i = 0; i < ptsize; ++i) if (effect_mask_[i]) pc++;
        std::fprintf(m20_csv,
            "%.6f,%d,%d,%.10g,%.10g,%.10g,%zu,%d,%d,%d,%.8g,%.8g,%.10g,%.8g,%.8g\n",
            measures_.lidar.end_time, static_cast<int>(frame_num_), iter_num,
            kf_->dbg_dR_z, kf_->dbg_w_z(), kf_->dbg_bg_z(),
            ptsize, static_cast<int>(effect_knn_num_), pc, ec,
            sum_HTVr.norm(), kf_->dbg_cond, kf_->dbg_dx_long,
            sum_HTVr(2), kf_->dbg_PinvH_yy);
        if ((++m20_rows % 500) == 0) std::fflush(m20_csv);
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

  // ── Phase 1: calibration status log (before early return) ──
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

  // ── Accelerometer / gyroscope bias (before early return) ──
  // Print frequency is per processed frame, not per visual-map publish: below the
  // g_visual_map gate this would only advance every g_pub_step frames, which made
  // the logged index look like a frame number while being 1/g_pub_step of it.
  {
    static int bias_log_count = 0;
    int interval = (bias_log_count < 10) ? 1 : 200;
    if (++bias_log_count % interval == 0) {
      auto st = kf_->GetSysState();
      LOG(INFO) << YELLOW << " ---> [Bias#" << bias_log_count
                << "] ba=[" << st.ba.transpose() << "] |ba|=" << st.ba.norm()
                << " bg=[" << st.bg.transpose() << "] |bg|=" << st.bg.norm()
                << RESET;
    }
  }

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

}

void SuperLIO::printTimeRecord(){
  if(!g_time_eva) return;
  time_record_.PrintAll();
}

} // namespace END.