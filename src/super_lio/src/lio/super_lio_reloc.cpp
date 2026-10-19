
#include "lio/super_lio_reloc.h"

#include <sys/resource.h>
#include <fstream>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>


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


void SuperLIOReLoc::init(){
  ivox_.reset(new OctVoxMapType(OctVoxMapType::Options{g_ivox_resolution, g_ivox_capacity}));
  kf_.reset(new ESKF());
  data_wrapper_->setESKF(kf_);
  
  scan_undistort_full_.reset(new PointCloudType());
  scan_undistort_full_2_.reset(new PointCloudType());
  ds_undistort_.reset(new PointCloudType());
  world_pc_.reset(new PointCloudType());
  ds_world_.reset(new PointCloudType());
  point_map_.reset(new PointCloudType());
  init_obs_data_.reset(new PointCloudType());
  init_obs_data2_.reset(new PointCloudType());
  
  points_world_v3_.reserve(21000);
  voxel_grid_fliter_.setLeafSize(g_voxel_fliter_size);

  // set degeneracy options
  {
    ESKF::DegeneracyOptions degen_opts;
    degen_opts.enabled = g_degeneracy_enabled;
    degen_opts.max_trans_stddev = g_degeneracy_max_stddev;
    degen_opts.htvh_cond_thresh = g_degeneracy_htvh_cond_thresh;
    kf_->setDegeneracyOptions(degen_opts);
  }

  kf_->setCalibEnabled(g_enable_online_calib);

  // Relocation uses a prior map as the sole reference — clearing it is fatal.
  allow_map_reset_ = false;

  if (g_verbose_logging) {
    LOG(INFO) << GREEN << " ---> [SuperLIO]: initialized. "
              << "P_lb=" << (g_p_lower_bound_enabled ? "ON" : "OFF")
              << " σ=" << g_p_lower_bound_trans
              << " calib=" << (g_enable_online_calib ? "ON" : "OFF")
              << " degen=" << (g_degeneracy_enabled ? "ON" : "OFF")
              << RESET;
  }

  auto start_time = std::chrono::high_resolution_clock::now();
  SuperLIOReLoc::map_init();
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  LOG(INFO) << GREEN << " ---> [SuperLIO]: Map init success. Time: " << duration.count() << " ms." << RESET;

  state_fn_ = &SuperLIOReLoc::stateWaitKFInit;
}


bool SuperLIOReLoc::map_init(){
  static bool pcd_loaded = false;
  if(pcd_loaded) return true;

  std::string map_name = ((g_map_name.size() && g_map_name[0] == '/') ? g_map_name : (g_save_map_dir + "/" + g_map_name));
  if(pcl::io::loadPCDFile<PointType>(map_name, *point_map_) == -1){
    LOG(ERROR) << RED << " ---> Load map failed. File: " << map_name << RESET;
    return false;
  }

  std::vector<int> useless_indices;
  pcl::removeNaNFromPointCloud(*point_map_, *point_map_, useless_indices);

  VV3 point_map_v3;
  point_map_v3.reserve(point_map_->size());
  for(const auto& point: *point_map_){
    V3 pt(point.x, point.y, point.z);
    point_map_v3.push_back(pt);
  }

  ivox_->insert(point_map_v3);

  // compute map z-bounds for sanity checking during relocation
  {
    map_z_min_ = std::numeric_limits<float>::max();
    map_z_max_ = std::numeric_limits<float>::lowest();
    for (const auto& pt : *point_map_) {
      if (pt.z < map_z_min_) map_z_min_ = pt.z;
      if (pt.z > map_z_max_) map_z_max_ = pt.z;
    }
    LOG(INFO) << GREEN << " ---> Map Z bounds: [" << map_z_min_ << ", " << map_z_max_ << "]" << RESET;
  }

  LOG(INFO) << GREEN << " ---> Load map success. File: " << map_name << RESET;
  LOG(INFO) << GREEN << " ---> Map size: " << point_map_->size() << RESET;
  ivox_->printInfo();

  pcd_loaded = true;

  data_wrapper_->set_global_map(point_map_);
  data_wrapper_->set_initial_data(re_init_pose_, flg_get_init_guess_);
  return true;
}


bool SuperLIOReLoc::kf_init(){
  const int need_init_frames = g_reloc_init_frames;
  static int imu_cout = 0;
  static int stationary_imu_cout = 0;
  static int init_frame_count = 0;
  static V3 mean_gyro = V3::Zero();
  static V3 mean_acce = V3::Zero();

  /// get init guess from ROS topic.
  if(flg_get_init_guess_){
    imu_cout = 0;
    stationary_imu_cout = 0;
    init_frame_count = 0;
    init_obs_data_->clear();
    init_obs_data2_->clear();
    mean_gyro = V3::Zero();
    mean_acce = V3::Zero();
    flg_get_init_guess_ = false;
    return false;
  }

  // ── Fix 3: accumulate lidar1 and lidar2 points separately ──
  CloudPtr lidar1_cloud = CloudPtr(new PointCloudType());
  for(std::size_t i = 0; i < measures_.lidar.pc->size(); i++){
    auto p = measures_.lidar.pc->points[i];
    PointType point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point.intensity = p.intensity;
    lidar1_cloud->points.push_back(point);
  }

  CloudPtr lidar2_cloud = CloudPtr(new PointCloudType());
  if (measures_.has_lidar2 && measures_.lidar2.pc && !measures_.lidar2.pc->empty()) {
    for(std::size_t i = 0; i < measures_.lidar2.pc->size(); i++){
      auto p = measures_.lidar2.pc->points[i];
      PointType point;
      point.x = p.x;
      point.y = p.y;
      point.z = p.z;
      point.intensity = p.intensity;
      lidar2_cloud->points.push_back(point);
    }
  }

  if(init_frame_count < need_init_frames){
    *init_obs_data_ += *lidar1_cloud;
    if (!lidar2_cloud->empty())
      *init_obs_data2_ += *lidar2_cloud;
  }
  init_frame_count++;

  // ── Fix 1: only use stationary IMU data for gravity estimation ──
  // A measurement is "stationary" if accel magnitude ≈ gravity AND gyro is small.
  // This prevents motion acceleration from corrupting the gravity direction estimate.
  int total_imu = 0;
  for(auto& imu: measures_.imu){
    total_imu++;
    double acc_norm = imu.acc.norm();
    double gyr_norm = imu.gyr.norm();
    bool is_stationary = (std::abs(acc_norm - g_gravity_norm) < 1.5) && (gyr_norm < 0.15);
    if (is_stationary) {
      imu_cout++;
      mean_gyro += (imu.gyr - mean_gyro) / imu_cout;
      mean_acce += (imu.acc - mean_acce) / imu_cout;
    }
    stationary_imu_cout += (is_stationary ? 1 : 0);
  }

  if(imu_cout < 20){
    return false;
  }

  if(init_frame_count < need_init_frames){
    return false;
  }

  // ── Fix 4: warn if robot was moving during accumulation ──
  {
    double ratio = (total_imu > 0) ? static_cast<double>(stationary_imu_cout) / total_imu : 0.0;
    double mean_gyro_norm = mean_gyro.norm();
    if (ratio < 0.5 || mean_gyro_norm > 0.1) {
      LOG(WARNING) << YELLOW
                   << " ---> [Reloc] Robot may have been moving during init! "
                   << "stationary_imu=" << (ratio * 100) << "% "
                   << "mean_gyro=" << mean_gyro_norm << " rad/s "
                   << "| Gravity estimate may be unreliable — keep robot still!"
                   << RESET;
    }
  }

  LOG(INFO) << YELLOW << " ---> INIT start... obs_data size: " << init_obs_data_->size()
            << " obs_data2 size: " << init_obs_data2_->size()
            << " target size: " << point_map_->size() << RESET;

  V3 gravity = - mean_acce * g_gravity_norm / mean_acce.norm();
  V3 ref_gravity(0, 0, - g_gravity_norm);
  M3 init_rot = Quat::FromTwoVectors(gravity, ref_gravity).toRotationMatrix();
  V3 n = init_rot.col(0);
  double yaw = atan2(n(1), n(0));

  M3 R_yaw_inv = Eigen::AngleAxis<scalar>(-yaw, V3::UnitZ()).toRotationMatrix();
  M3 rot = R_yaw_inv * init_rot;

  M3 init_guess_R_ = re_init_pose_.R_ * rot;
  V3 init_guess_t_ = re_init_pose_.t_;
  M4 init_guess_T = M4::Identity();
  init_guess_T.block<3, 3>(0, 0) = init_guess_R_;
  init_guess_T.block<3, 1>(0, 3) = init_guess_t_;


  // ── Fix 3: transform each lidar with its own extrinsic, then merge in IMU frame ──
  pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_src(new pcl::PointCloud<pcl::PointXYZI>());
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp1(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*init_obs_data_, *tmp1, g_lidar_imu.matrix().cast<float>());
    *tmp_src = *tmp1;

    if (g_dual_lidar_enable && init_obs_data2_ && !init_obs_data2_->empty()) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr tmp2(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::transformPointCloud(*init_obs_data2_, *tmp2, g_lidar_imu2.matrix().cast<float>());
      *tmp_src += *tmp2;
      LOG(INFO) << CYAN << " ---> Merged lidar1(" << tmp1->size()
                << ") + lidar2(" << tmp2->size() << ") in IMU frame." << RESET;
    }
  }

  // ---- publish initial observation for rviz visualization ----
  {
    M4 lidar_to_world = init_guess_T;  // tmp_src is already in IMU frame, init_guess_T maps IMU→World
    data_wrapper_->pub_init_obs(tmp_src, lidar_to_world.matrix().cast<float>());
    LOG(INFO) << CYAN << " ---> Published init observation to /lio/init_obs (" << tmp_src->size() << " pts)." << RESET;
  }

  // ---- helper: downsample point cloud ----
  auto downsample = [](const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, double leaf) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>());
    if (leaf > 0.01 && !cloud->empty()) {
      pcl::VoxelGrid<pcl::PointXYZI> vg;
      vg.setInputCloud(cloud);
      vg.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf), static_cast<float>(leaf));
      vg.filter(*out);
    } else {
      *out = *cloud;
    }
    return out;
  };

  // ---- helper: make initial guess with yaw offset and optional z offset ----
  auto make_guess_T = [&](double yaw_offset_deg, double z_offset = 0.0) -> M4 {
    M4 T = init_guess_T;
    if (std::abs(yaw_offset_deg) > 0.01) {
      M3 R_yaw = Eigen::AngleAxis<scalar>(yaw_offset_deg * M_PI / 180.0, V3::UnitZ()).toRotationMatrix();
      T.block<3, 3>(0, 0) = init_guess_R_ * R_yaw;
    }
    T(2, 3) += z_offset;
    return T;
  };

  // ---- helper: run single NDT ----
  auto run_ndt = [](pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>& ndt,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr& out,
                     const Eigen::Matrix4f& init_guess, int max_iter, float resolution,
                     const pcl::PointCloud<pcl::PointXYZI>::Ptr& target,
                     const pcl::PointCloud<pcl::PointXYZI>::Ptr& source) -> bool {
    ndt.setTransformationEpsilon(1e-5);
    ndt.setEuclideanFitnessEpsilon(1e-4);
    ndt.setMaximumIterations(max_iter);
    ndt.setResolution(resolution);
    ndt.setInputTarget(target);
    ndt.setInputSource(source);
    ndt.align(*out, init_guess);
    return ndt.hasConverged();
  };

  // ---- prepare multi-resolution target maps ----
  auto fine_target   = downsample(point_map_, g_reloc_target_ds_size);
  auto medium_target = downsample(point_map_, g_reloc_medium_ds_size);
  auto coarse_target = downsample(point_map_, g_reloc_coarse_ds_size);

  LOG(INFO) << CYAN << " ---> Multi-res targets: coarse=" << coarse_target->size()
            << " (leaf=" << g_reloc_coarse_ds_size << "m)"
            << " medium=" << medium_target->size()
            << " (leaf=" << g_reloc_medium_ds_size << "m)"
            << " fine=" << fine_target->size()
            << " (leaf=" << g_reloc_target_ds_size << "m)" << RESET;

  // ---- build reusable registration objects ----
  pcl::PointCloud<pcl::PointXYZI>::Ptr unused_result(new pcl::PointCloud<pcl::PointXYZI>());

  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> coarse_ndt;
  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> medium_ndt;
  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> fine_ndt;

  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setMaxCorrespondenceDistance(g_reloc_icp_max_corr);
  icp.setMaximumIterations(g_reloc_icp_max_iter);
  icp.setTransformationEpsilon(1e-4);
  icp.setEuclideanFitnessEpsilon(1e-4);
  icp.setRANSACIterations(0);
  icp.setInputTarget(fine_target);
  icp.setInputSource(tmp_src);

  // ---- check ICP result quality (z-bounds + overlap ratio) ----
  auto verify_result = [&](const Eigen::Matrix4f& T) -> bool {
    // z-bounds sanity check: result must be within map z range (±2m margin)
    float result_z = T(2, 3);
    if (result_z < map_z_min_ - 2.0f || result_z > map_z_max_ + 2.0f) {
      return false;
    }
    // overlap ratio check (if enabled)
    if (g_reloc_min_overlap_ratio > 0.0) {
      pcl::PointCloud<pcl::PointXYZI> aligned;
      pcl::transformPointCloud(*tmp_src, aligned, T);
      pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
      kdtree.setInputCloud(fine_target);
      int inliers = 0;
      std::vector<int> idx(1);
      std::vector<float> dist(1);
      const float max_dist = static_cast<float>(g_reloc_icp_max_corr);
      for (const auto& pt : aligned) {
        if (kdtree.nearestKSearch(pt, 1, idx, dist) > 0 && dist[0] < max_dist) {
          inliers++;
        }
      }
      double overlap = static_cast<double>(inliers) / aligned.size();
      if (overlap < g_reloc_min_overlap_ratio) {
        return false;
      }
    }
    return true;
  };

  // ---- registration cascade for a single (yaw, z) candidate ----
  auto try_registration = [&](double yaw_offset_deg, double z_offset = 0.0) -> std::pair<bool, M4> {
    Eigen::Matrix4f guess = make_guess_T(yaw_offset_deg, z_offset).matrix().cast<float>();

    if (g_reloc_enable_multi_res) {
      // Stage 1: Coarse NDT (wide capture basin, fast)
      if (!run_ndt(coarse_ndt, unused_result, guess, 25,
                   static_cast<float>(g_reloc_coarse_ndt_resolution),
                   coarse_target, tmp_src))
      {
        return {false, M4::Identity()};
      }
      double coarse_score = coarse_ndt.getFitnessScore();
      guess = coarse_ndt.getFinalTransformation();

      // Stage 2: Medium NDT
      if (!run_ndt(medium_ndt, unused_result, guess, 25,
                   static_cast<float>(g_reloc_medium_ndt_resolution),
                   medium_target, tmp_src))
      {
        return {false, M4::Identity()};
      }
      double medium_score = medium_ndt.getFitnessScore();
      guess = medium_ndt.getFinalTransformation();

      // Stage 3: Fine ICP
      icp.align(*unused_result, guess);
      bool icp_ok = icp.hasConverged();
      double icp_score = icp.getFitnessScore();

      LOG(INFO) << " ---> yaw=" << yaw_offset_deg << "°"
                << " z_off=" << z_offset
                << " | coarse=" << coarse_score
                << " medium=" << medium_score
                << " | ICP: ok=" << icp_ok << " score=" << icp_score;

      if (icp_ok && icp_score < g_reloc_fitness_thresh) {
        M4 result = icp.getFinalTransformation().cast<scalar>();
        if (verify_result(icp.getFinalTransformation())) {
          return {true, result};
        }
      }
      return {false, M4::Identity()};

    } else {
      // Single-resolution fallback
      float fine_res = static_cast<float>(g_reloc_ndt_resolution);
      if (!run_ndt(fine_ndt, unused_result, guess, 25, fine_res, fine_target, tmp_src)) {
        return {false, M4::Identity()};
      }
      icp.align(*unused_result, fine_ndt.getFinalTransformation());
      bool icp_ok = icp.hasConverged();
      double icp_score = icp.getFitnessScore();

      LOG(INFO) << " ---> yaw=" << yaw_offset_deg << "°"
                << " z_off=" << z_offset
                << " | NDT+ICP: ok=" << icp_ok << " score=" << icp_score;

      if (icp_ok && icp_score < g_reloc_fitness_thresh) {
        M4 result = icp.getFinalTransformation().cast<scalar>();
        if (verify_result(icp.getFinalTransformation())) {
          return {true, result};
        }
      }
      return {false, M4::Identity()};
    }
  };

  // ---- first attempt with original yaw ----
  auto [success, best_T] = try_registration(0.0);
  double best_icp_score = success ? icp.getFitnessScore() : std::numeric_limits<double>::max();

  // ---- multi-yaw + z-offset fallback ----
  if (!success && g_reloc_multi_yaw_search) {
    LOG(WARNING) << YELLOW << " ---> First attempt failed, trying multi-yaw + z-offset search..."
                 << " z_range=±" << g_reloc_z_search_range
                 << " z_step=" << g_reloc_z_search_step << RESET;

    // collect z offsets to try (0, ±step, ±2*step, ...)
    std::vector<double> z_offsets = {0.0};
    for (double z = g_reloc_z_search_step; z <= g_reloc_z_search_range + 1e-6;
         z += g_reloc_z_search_step) {
      z_offsets.push_back(z);
      z_offsets.push_back(-z);
    }

    for (double z_off : z_offsets) {
      for (int i = 1; i <= g_reloc_yaw_search_count; ++i) {
        for (double sign : {1.0, -1.0}) {
          double yaw_off = sign * i * g_reloc_yaw_step_deg;
          auto [ok, T] = try_registration(yaw_off, z_off);
          if (ok) {
            double score = icp.getFitnessScore();
            LOG(INFO) << GREEN << " ---> Found: yaw=" << yaw_off
                      << "° z_off=" << z_off
                      << " ICP score=" << score << RESET;
            if (score < best_icp_score) {
              best_icp_score = score;
              best_T = T;
              success = true;
            }
          }
        }
      }
      if (success) break;  // stop at first z level that works
    }
  }

  if (!success)
  {
    /// reset init state.
    imu_cout = 0;
    init_frame_count = 0;
    init_obs_data_->clear();
    mean_gyro = V3::Zero();
    mean_acce = V3::Zero();
    LOG(INFO) << RED << " ---> Global ICP Converged Fail! BestFitnessScore: " << best_icp_score << RESET;
    return false;
  } else{
    init_guess_T = best_T;
    LOG(INFO) << GREEN << " ---> Global ICP Converged Succeed! FitnessScore: " << best_icp_score << RESET;
  }

  LOG(INFO) << GREEN << "\n" << init_guess_T << RESET;

  ESKF::Options options;
  options.gyro_var_ = g_imu_ng;
  options.acce_var_ = g_imu_na;
  options.bias_gyro_var_ = g_imu_nbg;
  options.bias_acce_var_ = g_imu_nba;
  options.num_iterations_ = g_kf_max_iterations;
  options.quit_eps_ = g_kf_quit_eps;

  float imu_scale = g_gravity_norm / mean_acce.norm();

  // See the note in SuperLIO::kf_init(): imu_scale already absorbs the radial part
  // of the accelerometer bias, so subtracting mean_acce * (1 - imu_scale) as well
  // double-counts it and injects a constant vertical specific-force error.
  V3 estimated_ba = imu_scale * mean_acce - mean_acce.normalized() * g_gravity_norm;

  kf_->SetInitialConditions(options, mean_gyro, estimated_ba, imu_scale, ref_gravity);
  auto state = kf_->GetSysState();
  /// The horizontal initial state of the imu in the robot coordinate system.
  state.R = SO3(init_guess_T.block<3, 3>(0, 0));
  state.p = init_guess_T.block<3, 1>(0, 3);
  state.bg = mean_gyro;
  state.ba = estimated_ba;
  // ★ 2026-10-19: 必须显式清零速度。上面 GetSysState() 拿的是 init 阻塞期间 IMU 回调
  //   已经 Predict 过的状态, 其 v 是未初始化/垃圾值(~几十 m/s); 不清零则 KF init 后的
  //   第一次传播就把位姿甩出地图(knn_valid=0, 位移发散)。init 门槛已保证机器人静止。
  state.v = V3::Zero();
  state.timestamp = measures_.imu.back().secs;
  kf_->SetX(state);
  sys_init_pose_ = kf_->GetSE3();

  // {
  //   point_map_->clear();
  //   point_map_.reset(new PointCloudType());
  //   init_obs_data_->clear();
  //   init_obs_data_ = nullptr;
  //   data_wrapper_->set_initial_data(re_init_pose_, flg_get_init_guess_, true);
  // }

  {
    point_map_->clear();
    point_map_.reset(new PointCloudType());
    init_obs_data_->clear();
    init_obs_data_ = nullptr;
    data_wrapper_->set_initial_data(re_init_pose_, flg_get_init_guess_, true);
    data_wrapper_->spinSensorOnce();
    data_wrapper_->spinSensorOnce();
    data_wrapper_->spinSensorOnce();
    data_wrapper_->clear();        //// 清理掉 old 数据.
  }



  return true;
}


void SuperLIOReLoc::UpdateMap() {
  // In relocation mode, the prior map should remain static.
  // Only update when explicitly enabled (e.g. for dynamic environments).
  if (!g_update_map) return;

  static int __update_delay = 100;
  if (__update_delay > 0) {
    __update_delay--;
    std::cout << "Update map Delay: " << 100 - __update_delay << " %" << std::endl;
    return;
  }

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


void SuperLIOReLoc::Output() {
  auto state = kf_->GetNavState();
  data_wrapper_->pub_odom(state);

  // ── Phase 1: calibration status log (before early return) ──
  if (kf_->isCalibEnabled()) {
    static int calib_log_count = 0;
    int interval = (calib_log_count < 10) ? 1 : 100;
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

    // ── Paper figures: per-frame CSV dump (extrinsics + P structure) ──
    {
      SE3 est1 = kf_->getExtrinsic1();
      SE3 est2 = kf_->getExtrinsic2();
      V3 rpy1 = SO3(est1.R_).log_vee() * 180.0 / M_PI;
      V3 rpy2 = SO3(est2.R_).log_vee() * 180.0 / M_PI;
      static std::ofstream csv("/home/creepert/Super-LIO/src/super_lio/logs/calib_curve.csv", std::ios::app);
      csv << calib_log_count
          << "," << est1.t_.transpose() << "," << rpy1.transpose()
          << "," << est2.t_.transpose() << "," << rpy2.transpose()
          << "," << (rpy1 - rpy2).transpose() << "\n";

      // P structure: per-frame ||P_nc||_F (cross-block) + full 32×32 snapshot
      static std::ofstream pcsv("/home/creepert/Super-LIO/src/super_lio/logs/p_diag.csv", std::ios::app);
      const auto P = kf_->GetCov();
      double pnc = P.block(0, 18, 18, 14).norm() + P.block(18, 0, 14, 18).norm();
      pcsv << calib_log_count << "," << pnc << "\n";
      if (calib_log_count == 3000 || calib_log_count == 6000) {
        std::ofstream pfull("/home/creepert/Super-LIO/src/super_lio/logs/p_matrix_" +
                            std::to_string(calib_log_count) + ".csv", std::ios::trunc);
        for (int r = 0; r < P.rows(); ++r) {
          for (int c = 0; c < P.cols(); ++c) {
            pfull << P(r, c) << (c + 1 == P.cols() ? "\n" : ",");
          }
        }
      }
    }
  }

  // ── Accelerometer / gyroscope bias ──
  // Outside the calibration gate on purpose — see the note in SuperLIO::stateProcess().
  {
    static int bias_log_count = 0;
    int interval = (bias_log_count < 10) ? 1 : 100;
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



} // namespace END.