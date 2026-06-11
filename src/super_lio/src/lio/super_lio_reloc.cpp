
#include "lio/super_lio_reloc.h"

#include <sys/resource.h>
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
  ds_undistort_.reset(new PointCloudType());
  world_pc_.reset(new PointCloudType());
  ds_world_.reset(new PointCloudType());
  point_map_.reset(new PointCloudType());
  init_obs_data_.reset(new PointCloudType());
  
  points_world_v3_.reserve(21000);
  abcd_vec_.resize(20000);
  effect_knn_idxs_.resize(20000);
  voxel_grid_fliter_.setLeafSize(g_voxel_fliter_size);

  LOG(INFO) << GREEN << " ---> [SuperLIO]: initialized." << RESET;

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

  std::string map_name = g_save_map_dir + "/" + g_map_name;
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

  LOG(INFO) << GREEN << " ---> Load map success. File: " << map_name << RESET;
  LOG(INFO) << GREEN << " ---> Map size: " << point_map_->size() << RESET;
  ivox_->printInfo();

  pcd_loaded = true;

  data_wrapper_->set_global_map(point_map_);
  data_wrapper_->set_initial_data(re_init_pose_, flg_get_init_guess_);
  return true;
}


bool SuperLIOReLoc::kf_init(){
  const int need_init_frames = 10;
  static int imu_cout = 0;
  static int init_frame_count = 0;
  static V3 mean_gyro = V3::Zero();
  static V3 mean_acce = V3::Zero();

  /// get init guess from ROS topic.
  if(flg_get_init_guess_){
    imu_cout = 0;
    init_frame_count = 0;
    init_obs_data_->clear();
    mean_gyro = V3::Zero();
    mean_acce = V3::Zero();
    flg_get_init_guess_ = false;
    return false;
  }

  CloudPtr point_cloud_pcl = CloudPtr(new PointCloudType());
  for(std::size_t i = 0; i < measures_.lidar.pc->size(); i++){
    auto p = measures_.lidar.pc->points[i];
    PointType point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point.intensity = p.intensity;
    point_cloud_pcl->points.push_back(point);
  }

  if(init_frame_count < need_init_frames){
    *init_obs_data_ += *point_cloud_pcl;
  }
  init_frame_count++;

  for(auto& imu: measures_.imu){
    imu_cout ++;
    mean_gyro += (imu.gyr - mean_gyro) / imu_cout;
    mean_acce += (imu.acc - mean_acce) / imu_cout;
  }

  if(imu_cout < 20){
    return false;
  }

  if(init_frame_count < need_init_frames){
    return false;
  }

  LOG(INFO) << YELLOW << " ---> INIT start... obs_data size: " << init_obs_data_->size() << " target size: " << point_map_->size() << RESET;

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


  pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_src(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud(*init_obs_data_, *tmp_src, g_lidar_imu.matrix().cast<float>());

  // ---- Plan A: publish initial observation for rviz visualization ----
  // LiDAR -> World = init_guess_T * g_lidar_imu (IMU pose × LiDAR-to-IMU extrinsic)
  M4 lidar_to_world = init_guess_T * g_lidar_imu.matrix();
  data_wrapper_->pub_init_obs(init_obs_data_, lidar_to_world.matrix().cast<float>());
  LOG(INFO) << CYAN << " ---> Published init observation to /lio/init_obs for visual debugging." << RESET;

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

  // ---- helper: make initial guess with yaw offset ----
  auto make_guess_T = [&](double yaw_offset_deg) -> M4 {
    M4 T = init_guess_T;
    if (std::abs(yaw_offset_deg) > 0.01) {
      M3 R_yaw = Eigen::AngleAxis<scalar>(yaw_offset_deg * M_PI / 180.0, V3::UnitZ()).toRotationMatrix();
      T.block<3, 3>(0, 0) = init_guess_R_ * R_yaw;
    }
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

  // ---- registration cascade for a single yaw candidate ----
  auto try_registration = [&](double yaw_offset_deg) -> std::pair<bool, M4> {
    Eigen::Matrix4f guess = make_guess_T(yaw_offset_deg).matrix().cast<float>();

    if (g_reloc_enable_multi_res) {
      // Stage 1: Coarse NDT (wide capture basin, fast)
      if (!run_ndt(coarse_ndt, unused_result, guess, 25,
                   static_cast<float>(g_reloc_coarse_ndt_resolution),
                   coarse_target, tmp_src))
      {
        LOG(INFO) << CYAN << " ---> yaw=" << yaw_offset_deg << "° | COARSE NDT: FAILED" << RESET;
        return {false, M4::Identity()};
      }
      double coarse_score = coarse_ndt.getFitnessScore();
      guess = coarse_ndt.getFinalTransformation();

      // Stage 2: Medium NDT
      if (!run_ndt(medium_ndt, unused_result, guess, 25,
                   static_cast<float>(g_reloc_medium_ndt_resolution),
                   medium_target, tmp_src))
      {
        LOG(INFO) << CYAN << " ---> yaw=" << yaw_offset_deg << "° | coarse=" << coarse_score
                  << " | MEDIUM NDT: FAILED" << RESET;
        return {false, M4::Identity()};
      }
      double medium_score = medium_ndt.getFitnessScore();
      guess = medium_ndt.getFinalTransformation();

      // Stage 3: Fine ICP
      icp.align(*unused_result, guess);
      bool icp_ok = icp.hasConverged();
      double icp_score = icp.getFitnessScore();

      LOG(INFO) << " ---> yaw=" << yaw_offset_deg << "°"
                << " | coarse=" << coarse_score
                << " medium=" << medium_score
                << " | ICP: ok=" << icp_ok << " score=" << icp_score << RESET;

      if (icp_ok && icp_score < g_reloc_fitness_thresh) {
        return {true, icp.getFinalTransformation().cast<scalar>()};
      }
      return {false, M4::Identity()};

    } else {
      // Single-resolution fallback (original behavior)
      float fine_res = static_cast<float>(g_reloc_ndt_resolution);
      if (!run_ndt(fine_ndt, unused_result, guess, 25, fine_res, fine_target, tmp_src)) {
        LOG(INFO) << CYAN << " ---> yaw=" << yaw_offset_deg << "° | NDT: FAILED" << RESET;
        return {false, M4::Identity()};
      }
      double ndt_score = fine_ndt.getFitnessScore();
      icp.align(*unused_result, fine_ndt.getFinalTransformation());
      bool icp_ok = icp.hasConverged();
      double icp_score = icp.getFitnessScore();

      LOG(INFO) << CYAN << " ---> yaw=" << yaw_offset_deg << "°"
                << " | NDT score=" << ndt_score
                << " | ICP: ok=" << icp_ok << " score=" << icp_score << RESET;

      if (icp_ok && icp_score < g_reloc_fitness_thresh) {
        return {true, icp.getFinalTransformation().cast<scalar>()};
      }
      return {false, M4::Identity()};
    }
  };

  // ---- first attempt with original yaw ----
  auto [success, best_T] = try_registration(0.0);
  double best_icp_score = success ? icp.getFitnessScore() : std::numeric_limits<double>::max();

  // ---- multi-yaw fallback ----
  if (!success && g_reloc_multi_yaw_search) {
    LOG(WARNING) << YELLOW << " ---> First attempt failed, trying multi-yaw search..." << RESET;

    for (int i = 1; i <= g_reloc_yaw_search_count; ++i) {
      for (double sign : {1.0, -1.0}) {
        double yaw_off = sign * i * g_reloc_yaw_step_deg;
        auto [ok, T] = try_registration(yaw_off);
        if (ok) {
          double score = icp.getFitnessScore();
          LOG(INFO) << GREEN << " ---> Found candidate: yaw_offset=" << yaw_off
                    << "° ICP score=" << score << RESET;
          if (score < best_icp_score) {
            best_icp_score = score;
            best_T = T;
            success = true;
          }
        }
      }
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
  kf_->SetInitialConditions(options, mean_gyro, V3::Zero(), imu_scale, ref_gravity);
  auto state = kf_->GetSysState();
  /// The horizontal initial state of the imu in the robot coordinate system.
  state.R = SO3(init_guess_T.block<3, 3>(0, 0));
  state.p = init_guess_T.block<3, 1>(0, 3);
  state.timestamp = measures_.imu.back().secs;
  kf_->SetX(state);
  sys_init_pose_ = kf_->GetSE3();

  {
    point_map_->clear();
    point_map_.reset(new PointCloudType());
    init_obs_data_->clear();
    init_obs_data_ = nullptr;
    data_wrapper_->set_initial_data(re_init_pose_, flg_get_init_guess_, true);
  }

  return true;
}


void SuperLIOReLoc::UpdateMap() {
  if(g_update_map){
    static int __update_delay = 100;
    if(__update_delay > 0){
      __update_delay--;
      std::cout << "Update map Delay: " << 100 - __update_delay << " %" << std::endl;
      return;
    }
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