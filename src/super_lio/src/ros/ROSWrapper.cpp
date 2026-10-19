
#include <cstdio>
#include "ros/ROSWrapper.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <iomanip>
#include <pcl/common/transforms.h>


using namespace BASIC;

namespace LI2Sup{

// Reporting period of the [Input] accounting line (seconds of header time).
static constexpr double IN_LOG_SEC = 100.0;

void LoadParamFromRos(rclcpp::Node& node)
{
  node.declare_parameter<bool>("lio.map.save_map", false);
  node.declare_parameter<bool>("lio.sensor.use_true_time_window", false);
  node.get_parameter("lio.sensor.use_true_time_window", g_use_true_time_window);
  node.declare_parameter<int>("lio.sensor.deskew_mode", 0);   // P0-5
  node.get_parameter("lio.sensor.deskew_mode", g_deskew_mode);
  node.declare_parameter<double>("lio.sensor.drop_long_frames_ms", 0.0);
  node.get_parameter("lio.sensor.drop_long_frames_ms", g_drop_long_frames_ms);
  node.get_parameter("lio.map.save_map", g_save_map);

  LOG(INFO) << GREEN << " ---> [Param] map/save_map: "
            << (g_save_map ? "true" : "false") << RESET;

  // (A) 雷达订阅 QoS。发布端 offered profile 实测 = RELIABLE(1)/depth 100|256（5#GJD bag
  // metadata，两个雷达 topic 都是），而节点原为 best_effort ⇒ 对该 reader 链路降级、丢包静默：
  // 实测 scan1 仅 9.05 Hz，而同轮 recorder 因按 offered 匹配为 RELIABLE 收满 10.00 Hz。
  // 默认 false = 保持原行为 ⇒ 开/关由配置切换，臂参数化不需要重编（判据卡 M3）。
  node.declare_parameter<bool>("lio.ros.lidar_qos_reliable", false);
  node.get_parameter("lio.ros.lidar_qos_reliable", g_lidar_qos_reliable);
  LOG(INFO) << GREEN << " ---> [Param] ros/lidar_qos_reliable: "
            << (g_lidar_qos_reliable ? "true (RELIABLE)" : "false (best_effort)") << RESET;

  // (C1) lidar2-only 量测组：lidar1 断流时让第二雷达继续参与解算。默认 false = 原门控。
  node.declare_parameter<bool>("lio.ros.lidar2_only_update", false);
  node.get_parameter("lio.ros.lidar2_only_update", g_lidar2_only_enable);
  // (C1) 断流门控阈值：仅当 lidar1 已缺席超过该秒数，才允许 lidar2-only 组（避免正常段翻倍出解）
  node.declare_parameter<double>("lio.ros.lidar2_only_gap", 0.3);
  node.get_parameter("lio.ros.lidar2_only_gap", g_lidar2_only_gap);
  LOG(INFO) << GREEN << " ---> [Param] ros/lidar2_only_update: "
            << (g_lidar2_only_enable ? "true (lidar2-only groups allowed)" : "false (lidar1 gates every update)")
            << (g_lidar2_only_enable ? (" | outage_gap=" + std::to_string(g_lidar2_only_gap) + "s") : "")
            << RESET;

  node.declare_parameter<bool>("lio.eva.timer", false);
  node.get_parameter("lio.eva.timer", g_time_eva);

  node.declare_parameter<bool>("lio.eva.verbose_logging", false);
  node.get_parameter("lio.eva.verbose_logging", g_verbose_logging);

  node.declare_parameter<bool>("lio.eva.decouple_verify", false);
  node.get_parameter("lio.eva.decouple_verify", g_decouple_verify);

  node.declare_parameter<bool>("lio.eva.pose_dc_dump", false);
  node.get_parameter("lio.eva.pose_dc_dump", g_pose_dc_dump);

  node.declare_parameter<bool>("lio.map.if_filter", false);
  node.get_parameter("lio.map.if_filter", g_if_filter);

  node.declare_parameter<std::string>("lio.map.save_map_dir", "");
  node.get_parameter("lio.map.save_map_dir", g_save_map_dir);
  // 2026-10-19: 允许 save_map_dir 直接写成绝对路径 (yaml 里显式给出);
  // 相对路径行为不变 (仍按编译期 ROOT 展开)。
  if (g_save_map_dir.empty() || g_save_map_dir.front() != '/')
    g_save_map_dir = g_root_dir + g_save_map_dir;

  node.declare_parameter<std::string>("lio.map.map_name", "default");
  node.get_parameter("lio.map.map_name", g_map_name);

  node.declare_parameter<double>("lio.map.ds_size", 0.5);
  node.get_parameter("lio.map.ds_size", g_map_ds_size);

  node.declare_parameter<int>("lio.map.save_interval", 1);
  node.get_parameter("lio.map.save_interval", g_pcd_save_interval);

  node.declare_parameter<std::string>("lio.ros.lidar_topic", "/lidar");
  node.get_parameter("lio.ros.lidar_topic", g_lidar_topic);

  node.declare_parameter<std::string>("lio.ros.imu_topic", "/imu");
  node.get_parameter("lio.ros.imu_topic", g_imu_topic);

  node.declare_parameter<int>("lio.sensor.lidar_type", 0);
  node.get_parameter("lio.sensor.lidar_type", g_lidar_type);

  double temp_range_dis;
  node.declare_parameter<double>("lio.sensor.blind", 0.0);
  node.get_parameter("lio.sensor.blind", temp_range_dis);
  g_blind2 = temp_range_dis * temp_range_dis;

  node.declare_parameter<double>("lio.sensor.maxrange", 100.0);
  node.get_parameter("lio.sensor.maxrange", temp_range_dis);
  g_maxrange2 = temp_range_dis * temp_range_dis;

  node.declare_parameter<int>("lio.sensor.filter_rate", 1);
  node.get_parameter("lio.sensor.filter_rate", g_filter_rate);

  node.declare_parameter<double>("lio.sensor.intensity_thresh", 200.0);
  node.get_parameter("lio.sensor.intensity_thresh", g_intensity_thresh);
  node.declare_parameter<double>("lio.sensor.intensity_weight", 2.0);
  node.get_parameter("lio.sensor.intensity_weight", g_intensity_weight);

  node.declare_parameter<bool>("lio.sensor.enable_downsample", false);
  node.get_parameter("lio.sensor.enable_downsample", g_enable_downsample);

  node.declare_parameter<double>("lio.sensor.voxel_fliter_size", 0.2);
  node.get_parameter("lio.sensor.voxel_fliter_size", g_voxel_fliter_size);

  if (g_voxel_fliter_size < 0.3) {
    LOG(WARNING) << YELLOW
                 << " ---> voxel_fliter_size=" << g_voxel_fliter_size
                 << " < 0.3m: excessive points may cause HTVH condition "
                 << "degeneracy and memory pressure. Recommend ≥ 0.3m."
                 << RESET;
  }

  node.declare_parameter<double>("lio.sensor.time_offset", 0.0);
  node.get_parameter("lio.sensor.time_offset", g_lidar1_time_offset);

  node.declare_parameter<double>("lio.sensor.scan_period", 0.1);
  node.get_parameter("lio.sensor.scan_period", g_lidar1_scan_period);

  // ================= second LiDAR (dual-LiDAR fusion) =================
  node.declare_parameter<bool>("lio.lidar2.enable", false);
  node.get_parameter("lio.lidar2.enable", g_dual_lidar_enable);

  node.declare_parameter<int>("lio.lidar2.lidar_type", 1);
  node.get_parameter("lio.lidar2.lidar_type", g_lidar_type2);

  node.declare_parameter<std::string>("lio.lidar2.lidar_topic", "/livox/lidar");
  node.get_parameter("lio.lidar2.lidar_topic", g_lidar_topic2);

  double temp_range_dis2;
  node.declare_parameter<double>("lio.lidar2.blind", 0.3);
  node.get_parameter("lio.lidar2.blind", temp_range_dis2);
  g_blind2_2 = temp_range_dis2 * temp_range_dis2;

  node.declare_parameter<double>("lio.lidar2.maxrange", 100.0);
  node.get_parameter("lio.lidar2.maxrange", temp_range_dis2);
  g_maxrange2_2 = temp_range_dis2 * temp_range_dis2;

  node.declare_parameter<int>("lio.lidar2.filter_rate", 3);
  node.get_parameter("lio.lidar2.filter_rate", g_filter_rate_2);

  node.declare_parameter<double>("lio.lidar2.weight", 1.0);
  node.get_parameter("lio.lidar2.weight", g_lidar2_weight);

  node.declare_parameter<double>("lio.lidar2.time_offset", 0.0);
  node.get_parameter("lio.lidar2.time_offset", g_lidar2_time_offset);
  node.declare_parameter<bool>("lio.lidar2.use_dynamic_offset", g_lidar2_use_dynamic_offset);
  node.get_parameter("lio.lidar2.use_dynamic_offset", g_lidar2_use_dynamic_offset);

  node.declare_parameter<double>("lio.lidar2.scan_period", 0.1);
  node.get_parameter("lio.lidar2.scan_period", g_lidar2_scan_period);

  std::vector<double> extrinsic_lidar2_imu;
  node.declare_parameter<std::vector<double>>(
      "lio.lidar2.extrinsic_imu", std::vector<double>(12, 0.0));
  node.get_parameter("lio.lidar2.extrinsic_imu", extrinsic_lidar2_imu);
  {
    V3 t2(extrinsic_lidar2_imu[0], extrinsic_lidar2_imu[1], extrinsic_lidar2_imu[2]);
    std::vector<scalar> r2_data(9);
    for (int i = 0; i < 9; ++i) r2_data[i] = static_cast<scalar>(extrinsic_lidar2_imu[3 + i]);
    M3 R2(r2_data.data());
    g_lidar_imu2 = SE3(R2, t2);
  }

  node.declare_parameter<double>("lio.sensor.gravity_norm", 9.81);
  node.get_parameter("lio.sensor.gravity_norm", g_gravity_norm);

  node.declare_parameter<int>("lio.sensor.imu_type", 0);
  node.get_parameter("lio.sensor.imu_type", g_imu_type);

  node.declare_parameter<double>("lio.sensor.imu_na", 0.0);
  node.get_parameter("lio.sensor.imu_na", g_imu_na);

  node.declare_parameter<double>("lio.sensor.imu_ng", 0.0);
  node.get_parameter("lio.sensor.imu_ng", g_imu_ng);

  node.declare_parameter<double>("lio.sensor.imu_nba", 0.0);
  node.get_parameter("lio.sensor.imu_nba", g_imu_nba);

  node.declare_parameter<double>("lio.sensor.imu_nbg", 0.0);
  node.get_parameter("lio.sensor.imu_nbg", g_imu_nbg);

  // ================= extrinsic =================
  std::vector<double> extrinsic_lidar_imu;
  node.declare_parameter<std::vector<double>>(
      "lio.extrinsic.lidar_imu", std::vector<double>(12, 0.0));
  node.get_parameter("lio.extrinsic.lidar_imu", extrinsic_lidar_imu);

  V3 __t(extrinsic_lidar_imu[0],
         extrinsic_lidar_imu[1],
         extrinsic_lidar_imu[2]);
  std::vector<scalar> r_data(9);
  for (int i = 0; i < 9; ++i) {
    r_data[i] = static_cast<scalar>(extrinsic_lidar_imu[3 + i]);
  }
  M3 __R(r_data.data());
  g_lidar_imu = SE3(__R, __t);

  std::vector<double> extrinsic_odom_robo;
  node.declare_parameter<std::vector<double>>(
      "lio.extrinsic.odom_robo", std::vector<double>(6, 0.0));
  node.get_parameter("lio.extrinsic.odom_robo", extrinsic_odom_robo);

  __t = V3(extrinsic_odom_robo[0],
           extrinsic_odom_robo[1],
           extrinsic_odom_robo[2]);

  auto temp_R =
      Eigen::AngleAxisd(extrinsic_odom_robo[5] * M_PI / 180.0,
                          Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(extrinsic_odom_robo[4] * M_PI / 180.0,
                          Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(extrinsic_odom_robo[3] * M_PI / 180.0,
                          Eigen::Vector3d::UnitX());

  g_odom_robo.R_ = temp_R.cast<scalar>();
  g_odom_robo.R_ = g_odom_robo.R_.transpose().eval();
  g_odom_robo = SE3(g_odom_robo.R_, __t);

  auto temp_R_yaw =
      Eigen::AngleAxisd(extrinsic_odom_robo[5] * M_PI / 180.0,
                        Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  g_lidar_robo_yaw = temp_R_yaw.cast<scalar>();

  // ================= hash map =================
  node.declare_parameter<int>("lio.hash_map.hash_capacity", 1000000);
  node.get_parameter("lio.hash_map.hash_capacity", g_ivox_capacity);

  node.declare_parameter<double>("lio.hash_map.vox_resolution", 0.5);
  node.get_parameter("lio.hash_map.vox_resolution", g_ivox_resolution);

  // kf
  node.declare_parameter<int>("lio.kf.kf_type", 0);
  node.get_parameter("lio.kf.kf_type", g_kf_type);

  node.declare_parameter<int>("lio.kf.kf_max_iterations", 0);
  node.get_parameter("lio.kf.kf_max_iterations", g_kf_max_iterations);

  node.declare_parameter<bool>("lio.kf.kf_align_gravity", false);
  node.get_parameter("lio.kf.kf_align_gravity", g_kf_align_gravity);

  node.declare_parameter<double>("lio.kf.kf_quit_eps", 0.0);
  node.get_parameter("lio.kf.kf_quit_eps", g_kf_quit_eps);

  // submaps
  node.declare_parameter<double>("lio.submap.submap_resolution", 0.0);
  node.get_parameter("lio.submap.submap_resolution", g_submap_resolution);

  node.declare_parameter<int>("lio.submap.submap_capacity", 0);
  node.get_parameter("lio.submap.submap_capacity", g_submap_capacity);

  // visual
  node.declare_parameter<bool>("lio.output.robot", false);
  node.get_parameter("lio.output.robot", g_2_robot);

  node.declare_parameter<bool>("lio.output.planner", false);
  node.get_parameter("lio.output.planner", g_planner_enable);

  node.declare_parameter<bool>("lio.output.plan_env_world", false);
  node.get_parameter("lio.output.plan_env_world", g_2_plan_env_world);

  node.declare_parameter<bool>("lio.output.plan_env_body", false);
  node.get_parameter("lio.output.plan_env_body", g_2_plan_env_body);

  node.declare_parameter<bool>("lio.output.ml_map", false);
  node.get_parameter("lio.output.ml_map", g_2_ml_map);

  node.declare_parameter<bool>("lio.output.map", false);
  node.get_parameter("lio.output.map", g_visual_map);

  node.declare_parameter<bool>("lio.output.dense", false);
  node.get_parameter("lio.output.dense", g_visual_dense);

  node.declare_parameter<int>("lio.output.pub_step", 0);
  node.get_parameter("lio.output.pub_step", g_pub_step);

  node.declare_parameter<bool>("lio.output.nav2_tf", false);
  node.get_parameter("lio.output.nav2_tf", g_nav2_tf);

  // ================= relocation =================
  node.declare_parameter<bool>("lio.relocation.update_map", false);
  node.get_parameter("lio.relocation.update_map", g_update_map);

  std::vector<double> init_pose;
  node.declare_parameter<std::vector<double>>(
      "lio.relocation.init_pose", std::vector<double>(6, 0.0));
  node.get_parameter("lio.relocation.init_pose", init_pose);

  g_init_px    = init_pose[0];
  g_init_py    = init_pose[1];
  g_init_pz    = init_pose[2];
  g_init_roll  = init_pose[3];
  g_init_pitch = init_pose[4];
  g_init_yaw   = init_pose[5];

  // relocation registration parameters
  node.declare_parameter<double>("lio.relocation.target_ds_size", 0.5);
  node.get_parameter("lio.relocation.target_ds_size", g_reloc_target_ds_size);

  node.declare_parameter<double>("lio.relocation.ndt_resolution", 1.0);
  node.get_parameter("lio.relocation.ndt_resolution", g_reloc_ndt_resolution);

  node.declare_parameter<double>("lio.relocation.icp_max_corr", 8.0);
  node.get_parameter("lio.relocation.icp_max_corr", g_reloc_icp_max_corr);

  node.declare_parameter<int>("lio.relocation.icp_max_iter", 40);
  node.get_parameter("lio.relocation.icp_max_iter", g_reloc_icp_max_iter);

  node.declare_parameter<double>("lio.relocation.fitness_thresh", 1.5);
  node.get_parameter("lio.relocation.fitness_thresh", g_reloc_fitness_thresh);

  node.declare_parameter<bool>("lio.relocation.multi_yaw_search", false);
  node.get_parameter("lio.relocation.multi_yaw_search", g_reloc_multi_yaw_search);

  node.declare_parameter<double>("lio.relocation.yaw_step_deg", 30.0);
  node.get_parameter("lio.relocation.yaw_step_deg", g_reloc_yaw_step_deg);

  node.declare_parameter<int>("lio.relocation.yaw_search_count", 6);
  node.get_parameter("lio.relocation.yaw_search_count", g_reloc_yaw_search_count);

  // multi-resolution NDT parameters
  node.declare_parameter<bool>("lio.relocation.enable_multi_res", true);
  node.get_parameter("lio.relocation.enable_multi_res", g_reloc_enable_multi_res);

  node.declare_parameter<double>("lio.relocation.coarse_ds_size", 1.5);
  node.get_parameter("lio.relocation.coarse_ds_size", g_reloc_coarse_ds_size);

  node.declare_parameter<double>("lio.relocation.coarse_ndt_resolution", 5.0);
  node.get_parameter("lio.relocation.coarse_ndt_resolution", g_reloc_coarse_ndt_resolution);

  node.declare_parameter<double>("lio.relocation.medium_ds_size", 0.8);
  node.get_parameter("lio.relocation.medium_ds_size", g_reloc_medium_ds_size);

  node.declare_parameter<double>("lio.relocation.medium_ndt_resolution", 2.0);
  node.get_parameter("lio.relocation.medium_ndt_resolution", g_reloc_medium_ndt_resolution);

  // ================= online calibration (Phase 1) =================
  node.declare_parameter<bool>("lio.calib.enable_online_calib", true);
  node.get_parameter("lio.calib.enable_online_calib", g_enable_online_calib);

  node.declare_parameter<double>("lio.calib.ext_rot_rw", 1.7e-4);
  node.get_parameter("lio.calib.ext_rot_rw", g_calib_ext_rot_rw);

  node.declare_parameter<double>("lio.calib.ext_trans_rw", 1.0e-4);
  node.get_parameter("lio.calib.ext_trans_rw", g_calib_ext_trans_rw);

  node.declare_parameter<double>("lio.calib.dt_rw", 1.0e-6);
  node.get_parameter("lio.calib.dt_rw", g_calib_dt_rw);

  // ================= edge features (point-to-line) =================
  node.declare_parameter<bool>("lio.feature.enable_edge", true);
  node.get_parameter("lio.feature.enable_edge", g_enable_edge_feature);

  node.declare_parameter<double>("lio.feature.edge_weight", 500.0);
  node.get_parameter("lio.feature.edge_weight", g_edge_weight);

  node.declare_parameter<double>("lio.feature.edge_line_dist_thresh", 0.15);
  node.get_parameter("lio.feature.edge_line_dist_thresh", g_edge_line_dist_thresh);

  node.declare_parameter<double>("lio.feature.edge_linearity_ratio", 3.0);
  node.get_parameter("lio.feature.edge_linearity_ratio", g_edge_linearity_ratio);

  node.declare_parameter<double>("lio.feature.high_intensity_edge_boost", 3.0);
  node.get_parameter("lio.feature.high_intensity_edge_boost", g_high_intensity_edge_boost);

  // ================= P lower bound (covariance clamp) =================
  node.declare_parameter<bool>("lio.kf.p_lower_bound.enabled", false);
  node.get_parameter("lio.kf.p_lower_bound.enabled", g_p_lower_bound_enabled);

  node.declare_parameter<double>("lio.kf.p_lower_bound.trans_sigma", 0.05);
  node.get_parameter("lio.kf.p_lower_bound.trans_sigma", g_p_lower_bound_trans);

  // ================= degeneracy handling =================
  node.declare_parameter<bool>("lio.degeneracy.enabled", true);
  node.get_parameter("lio.degeneracy.enabled", g_degeneracy_enabled);

  node.declare_parameter<double>("lio.degeneracy.max_stddev", 0.5);
  node.get_parameter("lio.degeneracy.max_stddev", g_degeneracy_max_stddev);

  node.declare_parameter<double>("lio.degeneracy.htvh_cond_thresh", 100.0);
  node.get_parameter("lio.degeneracy.htvh_cond_thresh", g_degeneracy_htvh_cond_thresh);

  // ================= structural regularity =================
  // Two independent channels: walls constrain roll, the ground's longitudinal
  // component constrains pitch. The ground channel needs a terrain prior (the
  // ground is not necessarily horizontal), hence it is opt-in and parameterised.
  node.declare_parameter<bool>("lio.structure.wall_enabled", false);
  node.get_parameter("lio.structure.wall_enabled", g_structure_wall_enabled);

  node.declare_parameter<bool>("lio.structure.ground_enabled", false);
  node.get_parameter("lio.structure.ground_enabled", g_structure_ground_enabled);

  node.declare_parameter<bool>("lio.structure.ground_grade_auto", true);
  node.get_parameter("lio.structure.ground_grade_auto", g_structure_ground_grade_auto);

  node.declare_parameter<double>("lio.structure.ground_grade_deg", 0.0);
  node.get_parameter("lio.structure.ground_grade_deg", g_structure_ground_grade_deg);

  node.declare_parameter<double>("lio.structure.ground_weight", 1.0e6);
  node.get_parameter("lio.structure.ground_weight", g_structure_ground_weight);

  node.declare_parameter<double>("lio.structure.max_angle_deg", 20.0);
  node.get_parameter("lio.structure.max_angle_deg", g_structure_max_angle_deg);

  // ================= relocation robustness =================
  node.declare_parameter<int>("lio.relocation.init_frames", 5);
  node.get_parameter("lio.relocation.init_frames", g_reloc_init_frames);

  node.declare_parameter<double>("lio.relocation.z_search_range", 2.0);
  node.get_parameter("lio.relocation.z_search_range", g_reloc_z_search_range);

  node.declare_parameter<double>("lio.relocation.z_search_step", 1.0);
  node.get_parameter("lio.relocation.z_search_step", g_reloc_z_search_step);

  node.declare_parameter<double>("lio.relocation.min_overlap_ratio", 0.0);
  node.get_parameter("lio.relocation.min_overlap_ratio", g_reloc_min_overlap_ratio);

  // ================= ROS frame ids (configurable) =================
  node.declare_parameter<std::string>("lio.frame.world", g_frame_world);
  node.get_parameter("lio.frame.world", g_frame_world);
  node.declare_parameter<std::string>("lio.frame.imu", g_frame_imu);
  node.get_parameter("lio.frame.imu", g_frame_imu);
  node.declare_parameter<std::string>("lio.frame.map", g_frame_map);
  node.get_parameter("lio.frame.map", g_frame_map);
  node.declare_parameter<std::string>("lio.frame.odom", g_frame_odom);
  node.get_parameter("lio.frame.odom", g_frame_odom);
  node.declare_parameter<std::string>("lio.frame.base", g_frame_base);
  node.get_parameter("lio.frame.base", g_frame_base);
  LOG(INFO) << GREEN << " ---> [Param] frame ids: world='" << g_frame_world
            << "' imu='" << g_frame_imu << "' map='" << g_frame_map
            << "' odom='" << g_frame_odom << "' base='" << g_frame_base
            << "'" << RESET;

  LOG(INFO) << GREEN << " ---> [Params]: Load from ROS2 parameter server."
            << RESET;
}


std::string lidarTypeToString(int type) {
  if (type <= 0 || type >= static_cast<int>(LID_TYPE_NAMES.size())) return "UNKNOWN";
  return LID_TYPE_NAMES[type];
}


inline bool validPoint(double x, double y, double z)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
    return false;

  double d2 = x * x + y * y + z * z;
  return (d2 > g_blind2 && d2 < g_maxrange2);
}


bool ROSWrapper::validPoint2(double x, double y, double z)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
    return false;
  double d2 = x * x + y * y + z * z;
  return (d2 > g_blind2_2 && d2 < g_maxrange2_2);
}


// ── Dynamic time offset estimation ──────────────────────────────
// Implementations at end of file (after stampToSec definition)


inline double stampToSec(const builtin_interfaces::msg::Time& t)
{
  return static_cast<double>(t.sec) +
         static_cast<double>(t.nanosec) * 1e-9;
}


inline builtin_interfaces::msg::Time toRosTime(double t_sec)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(std::floor(t_sec));
  t.nanosec = static_cast<uint32_t>((t_sec - t.sec) * 1e9);
  return t;
}


ROSWrapper::ROSWrapper(const rclcpp::NodeOptions& options)
: rclcpp::Node("super_lio", options)
{
  LoadParamFromRos(*this);
  LOG(INFO) << GREEN << " ---> Using Lidar type: "
            << lidarTypeToString(g_lidar_type) << RESET;

  msg2uav_.header.frame_id = g_frame_world;
  path_.header.frame_id = g_frame_world;

  setupIO();
}


void ROSWrapper::setupIO(){
  //// input ======================================
  cb_sensor_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions sub_opt;
  sub_opt.callback_group = cb_sensor_;

  auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(500))
                 .best_effort()
                 .durability_volatile();

  // (A) 见 LoadParamFromRos：发布端是 RELIABLE，best_effort 订阅在丢包时静默丢帧。
  // depth 仍 20（不引入第二个变量）；imu 不动（500 Hz、maxgap 仅 0.004–0.006 s，无丢帧）。
  auto lidar_qos = g_lidar_qos_reliable
                   ? rclcpp::QoS(rclcpp::KeepLast(20)).reliable().durability_volatile()
                   : rclcpp::QoS(rclcpp::KeepLast(20)).best_effort().durability_volatile();

  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      g_imu_topic,
      imu_qos,
      std::bind(&ROSWrapper::imuHandler, this, std::placeholders::_1),
      sub_opt);

  if (g_lidar_type == LID_TYPE::LIVOX) {
    LOG(ERROR) << " ---> [Livox] CustomMsg support has been removed from this build; "
               << "please select a PointCloud2 lidar type.";
  } else {
    sub_lidar_std_ =
        this->create_subscription<sensor_msgs::msg::PointCloud2>(
            g_lidar_topic,
            lidar_qos,
            std::bind(&ROSWrapper::stdMsgHandler, this, std::placeholders::_1),
            sub_opt);
  }

  // second LiDAR subscription (dual-LiDAR fusion)
  if (g_dual_lidar_enable) {
    if (g_lidar_type2 == LID_TYPE::LIVOX) {
      LOG(ERROR) << " ---> [Livox] CustomMsg support has been removed from this build; "
                 << "please select a PointCloud2 lidar type for lidar2.";
    } else {
      sub_lidar2_std_ =
          this->create_subscription<sensor_msgs::msg::PointCloud2>(
              g_lidar_topic2,
              lidar_qos,
              std::bind(&ROSWrapper::stdMsgHandler2, this, std::placeholders::_1),
              sub_opt);
    }
    LOG(INFO) << GREEN << " ---> [Dual-LiDAR] lidar2 subscribed: topic="
              << g_lidar_topic2 << " type=" << g_lidar_type2 << RESET;
  }

  /// output ======================================
  pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/lio/odom", 100);

  pub_imu_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/lio/imu/odom", 10);

  pub_robo_odom_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/lio/robo/odom", 10);

  pub_path_ = this->create_publisher<nav_msgs::msg::Path>(
      "/lio/path", 10);

  pub_cloud_world_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lio/cloud_world", 10);

  tf_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(this);
}


// Input-side accounting, one line per IN_LOG_SEC of header time. Reports the message
// count and, more importantly, the EFFECTIVE rate (1/mean inter-arrival) and the largest
// gap: those two reveal silent drops without needing to know the nominal rate. Called
// from the scan handlers, so the line keeps coming even if the IMU stream dies.
void ROSWrapper::logInputAccounting(double t){
  if (in_log_t_ < 0.0) { in_log_t_ = t; return; }
  const double dt = t - in_log_t_;
  if (dt < IN_LOG_SEC) return;
  const auto rate_of = [](const InStat& s) {
    return (s.n > 1 && s.sum_gap > 1e-9) ? (s.n - 1) / s.sum_gap : 0.0;
  };
  LOG(INFO) << " ---> [Input] " << std::fixed << std::setprecision(1) << dt << "s: "
            << "imu n=" << in_imu_.n << " rate=" << std::setprecision(1) << rate_of(in_imu_)
            << "Hz maxgap=" << std::setprecision(3) << in_imu_.max_gap << "s"
            << " | scan1 n=" << in_scan1_.n << " rate=" << std::setprecision(1) << rate_of(in_scan1_)
            << "Hz maxgap=" << std::setprecision(3) << in_scan1_.max_gap << "s"
            << " | scan2 n=" << in_scan2_.n << " rate=" << std::setprecision(1) << rate_of(in_scan2_)
            << "Hz maxgap=" << std::setprecision(3) << in_scan2_.max_gap << "s"
            << " | lag(相对全程首帧,s) imu " << std::setprecision(3) << in_imu_.lag.mean()
            << "[" << in_imu_.lag.mn << "," << in_imu_.lag.mx << "]"
            << " 1 " << in_scan1_.lag.mean() << "[" << in_scan1_.lag.mn << "," << in_scan1_.lag.mx << "]"
            << " 2 " << in_scan2_.lag.mean() << "[" << in_scan2_.lag.mn << "," << in_scan2_.lag.mx << "]";
  in_log_t_ = t;
  for (InStat* s : {&in_imu_, &in_scan1_, &in_scan2_}) {   // keep `last` so the boundary gap survives
    s->n = 0;
    s->sum_gap = 0.0;
    s->max_gap = 0.0;
    s->lag.clear_window();                                 // 全程 ref 保留 ⇒ 跨窗积压可见（M5）
  }
}

void ROSWrapper::imuHandler(const sensor_msgs::msg::Imu::SharedPtr msg){
  in_imu_.add(stampToSec(msg->header.stamp));
  IMUData data;
  data.secs = stampToSec(msg->header.stamp);
  data.acc  = V3(msg->linear_acceleration.x,
                 msg->linear_acceleration.y,
                 msg->linear_acceleration.z);
  data.gyr  = V3(msg->angular_velocity.x,
                 msg->angular_velocity.y,
                 msg->angular_velocity.z);

  if (data.secs < last_timestamp_imu_) {
    LOG(WARNING) << "imu loop back, clear buffer";
    imu_buffer_.clear();
    imu_buffer_.push_back(data);
    last_timestamp_imu_ = data.secs;
    // eskf_->Reset();   // todo:
    return;
  }

  imu_buffer_.push_back(data);
  last_timestamp_imu_ = data.secs;

  DynamicState imu_state, robo_state;
  if(eskf_->Predict(data, imu_state, robo_state)){
    nav_msgs::msg::Odometry odom_imu, odom_robo;

    {
      odom_imu.pose.pose.position.x = imu_state.p(0);
      odom_imu.pose.pose.position.y = imu_state.p(1);
      odom_imu.pose.pose.position.z = imu_state.p(2);

      Quat q(imu_state.R);
      q.normalize();

      odom_imu.pose.pose.orientation.x = q.x();
      odom_imu.pose.pose.orientation.y = q.y();
      odom_imu.pose.pose.orientation.z = q.z();
      odom_imu.pose.pose.orientation.w = q.w();

      odom_imu.twist.twist.linear.x = imu_state.v(0);
      odom_imu.twist.twist.linear.y = imu_state.v(1);
      odom_imu.twist.twist.linear.z = imu_state.v(2);

      odom_imu.twist.twist.angular.x = imu_state.w(0);
      odom_imu.twist.twist.angular.y = imu_state.w(1);
      odom_imu.twist.twist.angular.z = imu_state.w(2);
    }

    {
      odom_robo.pose.pose.position.x = robo_state.p(0);
      odom_robo.pose.pose.position.y = robo_state.p(1);
      odom_robo.pose.pose.position.z = robo_state.p(2);

      Quat q(robo_state.R);
      q.normalize();

      odom_robo.pose.pose.orientation.x = q.x();
      odom_robo.pose.pose.orientation.y = q.y();
      odom_robo.pose.pose.orientation.z = q.z();
      odom_robo.pose.pose.orientation.w = q.w();
    }

    odom_imu.header.stamp = msg->header.stamp;
    odom_robo.header.stamp = msg->header.stamp;
    odom_imu.header.frame_id = g_frame_world;
    odom_robo.header.frame_id = g_frame_world;
    pub_imu_odom_->publish(odom_imu);
    pub_robo_odom_->publish(odom_robo);
  }
}


void ROSWrapper::stdMsgHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
  if(msg->data.size() < 10) return;
  in_scan1_.add(stampToSec(msg->header.stamp));
  logInputAccounting(stampToSec(msg->header.stamp));
  t_last_lidar1_wall_ = stampToSec(this->now());   // (C1) 断流门控基准
  updateLidar1Clock(stampToSec(msg->header.stamp));
  LidarData lidar_data;
  lidar_data.pc.reset(new pcl::PointCloud<LI2Sup::PointXTZIT>());

  double offset_time = 0.0;
  double dis = 0.0;

  switch (g_lidar_type) {

  case LID_TYPE::HESAI16:
  {
    pcl::PointCloud<hesai_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    const double time_begin = pl_orig.points[0].timestamp;
    lidar_data.start_time = time_begin;
    // P0-4: TRUE min/max over the raw sweep.  EVERYTHING that used points[0] as
    // the reference has now bitten us three times (entry 21 shift, entry 22
    // model, entry 33 the detector itself), because points[0] can be a whole
    // sweep late -- measured up to 99.98 ms.
    double raw_tmin = 1e18, raw_tmax = -1e18;
    for (const auto& q : pl_orig.points) {
      if (q.timestamp < raw_tmin) raw_tmin = q.timestamp;
      if (q.timestamp > raw_tmax) raw_tmax = q.timestamp;
    }
    const double raw_span_ms = (raw_tmax - raw_tmin) * 1e3;
    const double lead_ms     = (time_begin - raw_tmin) * 1e3;   // how late points[0] is

    if (g_drop_long_frames_ms > 0.0 && raw_span_ms > g_drop_long_frames_ms) {
      static int drop_n = 0;
      if (++drop_n <= 5 || drop_n % 20 == 0)
        LOG(WARNING) << "[DropLong] true span " << raw_span_ms << " ms (lead "
                     << lead_ms << " ms) > " << g_drop_long_frames_ms
                     << " -> dropped (n=" << drop_n << ")";
      return;
    }

    if (g_use_true_time_window) {
      // widen ONLY the window; per-point query_time is unaffected (entry 22).
      static int win_n = 0;
      if (++win_n <= 5 || win_n % 200 == 0)
        LOG(INFO) << "[TrueWin] lead " << lead_ms << " ms, span " << raw_span_ms
                  << " ms -> start_time moved back by " << lead_ms << " ms (n="
                  << win_n << ")";
      lidar_data.start_time = raw_tmin;
    }
    // ── P0-4 入口记账 (entry accounting) ─────────────────────────────────
    // "About half the points never reach the estimator" was previously INFERRED:
    // (inside + above) ~ 47-50k vs 95.5k/frame, plus the `filter_rate: 2` line.
    // Measure it per frame instead -- the ROS message's own declared count
    // (msg->width*msg->height) vs THIS loop's length (pl_orig.size(), i.e. what
    // fromROSMsg actually decoded) vs what survives, with every drop attributed
    // to the intensity condition or to validPoint.
    const std::size_t msg_n = static_cast<std::size_t>(msg->width) * msg->height;
    const std::size_t fr = (g_filter_rate > 0) ? static_cast<std::size_t>(g_filter_rate)
                                               : std::size_t(1);   // guard i % 0
    std::size_t n_even = 0, n_odd_hi = 0, n_odd_lo = 0, n_invalid = 0;
for(std::size_t i = 0; i < pl_orig.size(); i++)
    {
      auto& pt = pl_orig.points[i];
      const bool on_stride = (i % fr == 0);
      if (!on_stride && pt.intensity < g_intensity_thresh) { ++n_odd_lo; continue; }
      if (!validPoint(pt.x, pt.y, pt.z)) { ++n_invalid; continue; }
      if (on_stride) ++n_even; else ++n_odd_hi;
      offset_time = pt.timestamp - time_begin;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, pt.intensity, offset_time);
    }
    lidar_data.end_time = time_begin + offset_time;
    {
      // rev102: per-frame raw counts (msg vs loop) so the burst frame is visible.
      static FILE* f102 = std::fopen("/tmp/rev102_entry.csv", "a");
      if (f102) {
        char b102[256];
        int n102 = std::snprintf(b102, sizeof(b102), "%.6f,%ld,%zu,%zu,%zu,%zu",
                                 lidar_data.end_time, static_cast<long>(msg_n),
                                 pl_orig.size(), n_even, n_odd_hi, n_odd_lo);
        b102[n102++] = 10;
        std::fwrite(b102, 1, static_cast<std::size_t>(n102), f102);
      }
    }
    {
      static long entry_frames = 0, s_msg = 0, s_loop = 0, s_even = 0, s_oddhi = 0,
                  s_oddlo = 0, s_inv = 0;
      ++entry_frames;
      s_msg += static_cast<long>(msg_n);
      s_loop += static_cast<long>(pl_orig.size());
      s_even += static_cast<long>(n_even);
      s_oddhi += static_cast<long>(n_odd_hi);
      s_oddlo += static_cast<long>(n_odd_lo);
      s_inv += static_cast<long>(n_invalid);
      if (entry_frames <= 2 || entry_frames % 100 == 0) {
        const double F = static_cast<double>(entry_frames);
        const double kept = static_cast<double>(s_even + s_oddhi);
        char ebuf[400];
        std::snprintf(ebuf, sizeof(ebuf),
                      "[Entry] frames=%ld avg msg=%.0f loop=%.0f kept=%.0f (%.1f%% of loop)"
                      " | even=%.0f odd_hi=%.0f odd_lo_DROP=%.0f invalid_DROP=%.0f"
                      " | sum(loop-msg)=%ld fr=%zu",
                      entry_frames, static_cast<double>(s_msg) / F,
                      static_cast<double>(s_loop) / F, kept / F,
                      s_loop > 0 ? 100.0 * kept / static_cast<double>(s_loop) : 0.0,
                      static_cast<double>(s_even) / F, static_cast<double>(s_oddhi) / F,
                      static_cast<double>(s_oddlo) / F, static_cast<double>(s_inv) / F,
                      s_loop - s_msg, fr);
        LOG(INFO) << GREEN << " ---> " << ebuf << RESET;
      }
    }
    {
      static bool hesai_diag_logged = false;
      if (!hesai_diag_logged && !lidar_data.pc->empty()) {
        double omin = 1e18, omax = -1e18;
        for (std::size_t i = 0; i < lidar_data.pc->size(); i++) {
          const double o = lidar_data.pc->points[i].offset_time;
          if (o < omin) omin = o;
          if (o > omax) omax = o;
        }
        char dbuf[320];
        std::snprintf(dbuf, sizeof(dbuf),
                      "[HESAI16] DIAG n_raw=%zu n_kept=%zu t_first=%.6f t_last_raw=%.6f "
                      "off_min=%.6f off_max=%.6f (expect 0 .. ~0.1s; all-zero => deskew DEAD)",
                      pl_orig.size(), lidar_data.pc->size(), time_begin,
                      pl_orig.points[pl_orig.size() - 1].timestamp, omin, omax);
        LOG(INFO) << GREEN << " ---> " << dbuf << RESET;
        hesai_diag_logged = true;
      }
    }
    break;
  }
  case LID_TYPE::VEL_NCLT:
  {
    pcl::PointCloud<NCLT::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    lidar_data.start_time = stampToSec(msg->header.stamp);
    
    for(std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate){
      auto& pt = pl_orig.points[i];
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      offset_time = pt.time * 1e-6;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, 1.0, offset_time);
    }
    lidar_data.end_time = lidar_data.start_time + offset_time;
    break;
  }
  case LID_TYPE::VELO16:
  case LID_TYPE::VELO32:
  {
    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    lidar_data.start_time = stampToSec(msg->header.stamp);

    for(std::size_t i = 0; i < pl_orig.size(); i++){
      auto& pt = pl_orig.points[i];
      if(i % g_filter_rate != 0 && pt.intensity < g_intensity_thresh) continue;
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, pt.intensity, pt.time);
    }
    lidar_data.end_time = lidar_data.start_time + lidar_data.pc->points.back().offset_time;
    break;
  }
  case OUSTER:
  {
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    lidar_data.start_time = stampToSec(msg->header.stamp);

    for(std::size_t i = 0; i < pl_orig.size(); i++){
      auto& pt = pl_orig.points[i];
      if(i % g_filter_rate != 0 && pt.intensity < g_intensity_thresh) continue;
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
      offset_time = pt.t * 1e-9;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, pt.intensity, offset_time);
    }
    lidar_data.end_time = lidar_data.start_time + offset_time;
    break;
  }
  case LID_TYPE::LIVOX_PC2:
  {
    pcl::PointCloud<pcl::PointXYZI> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    if (pl_orig.empty()) break;

    static bool first_log = true;
    if (first_log) {
      LOG(INFO) << GREEN << " ---> [LIVOX_PC2] Receiving: "
                << pl_orig.size() << " pts/msg" << RESET;
      first_log = false;
    }

    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate + 1);
    const double header_time = stampToSec(msg->header.stamp);
    lidar_data.start_time = header_time;

    if (g_lidar1_scan_period <= 1e-3) {
      // no per-point timestamps and deskew disabled: treat the whole frame as one instant
      for (std::size_t i = 0; i < pl_orig.size(); i++) {
        auto& pt = pl_orig.points[i];
        if(i % g_filter_rate != 0 && pt.intensity < g_intensity_thresh) continue;
        if (!validPoint(pt.x, pt.y, pt.z)) continue;
        lidar_data.pc->emplace_back(pt.x, pt.y, pt.z, pt.intensity, 0.0);
      }
      if (lidar_data.pc->empty()) break;
      lidar_data.end_time = header_time;
      break;
    }

    // ── azimuth-based deskew ────────────────────────────────────────
    // The cloud has no per-point timestamp (x,y,z,intensity only), but a
    // 360-deg sweep is monotonic in azimuth: unwrap atan2(y,x) to recover
    // the acquisition order, then normalise it to g_lidar1_scan_period.
    {
      std::vector<float>  kx, ky, kz, ki;
      std::vector<double> az_un;
      const std::size_t cap = pl_orig.size() / g_filter_rate + 1;
      kx.reserve(cap); ky.reserve(cap); kz.reserve(cap); ki.reserve(cap); az_un.reserve(cap);

      double az_acc = 0.0, az_prev = 0.0;
      bool az_first = true;
      for (std::size_t i = 0; i < pl_orig.size(); i++) {
        auto& pt = pl_orig.points[i];
        if(i % g_filter_rate != 0 && pt.intensity < g_intensity_thresh) continue;
        if (!validPoint(pt.x, pt.y, pt.z)) continue;
        const double a = std::atan2(static_cast<double>(pt.y), static_cast<double>(pt.x));
        if (az_first) { az_acc = a; az_prev = a; az_first = false; }
        else {
          double d = a - az_prev;
          while (d >  M_PI) d -= 2.0 * M_PI;
          while (d < -M_PI) d += 2.0 * M_PI;
          az_acc += d;
          az_prev = a;
        }
        kx.push_back(pt.x); ky.push_back(pt.y); kz.push_back(pt.z); ki.push_back(pt.intensity);
        az_un.push_back(az_acc);
      }
      if (kx.empty()) break;

      const double span  = az_un.back() - az_un.front();
      const double scale = (std::fabs(span) > 1e-6) ? (g_lidar1_scan_period / span) : 0.0;
      for (std::size_t j = 0; j < kx.size(); j++) {
        const double off = (az_un[j] - az_un.front()) * scale;
        lidar_data.pc->emplace_back(kx[j], ky[j], kz[j], ki[j], off);
      }

      static bool deskew_logged1 = false;
      if (!deskew_logged1) {
        LOG(INFO) << GREEN << " ---> [LIVOX_PC2] azimuth-based deskew ON: scan_period="
                  << g_lidar1_scan_period << "s, azimuth span="
                  << span * 180.0 / M_PI << " deg, pts=" << lidar_data.pc->size() << RESET;
        deskew_logged1 = true;
      }
    }

    lidar_data.end_time = header_time + g_lidar1_scan_period;
    break;
  }
  default:
    return;
  }

  lidar_data.start_time += g_lidar1_time_offset;
  lidar_data.end_time   += g_lidar1_time_offset;
  lidar_buffer_.push_back(lidar_data);
}


// ── Second LiDAR handlers (dual-LiDAR) ──────────────────────────

void ROSWrapper::stdMsgHandler2(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
  if(msg->data.size() < 10) return;
  in_scan2_.add(stampToSec(msg->header.stamp));
  updateLidar2Clock(stampToSec(msg->header.stamp));

  LidarData lidar_data;
  lidar_data.pc.reset(new pcl::PointCloud<LI2Sup::PointXTZIT>());

  double offset_time = 0.0;

  switch (g_lidar_type2) {

  case LID_TYPE::HESAI16:
  {
    pcl::PointCloud<hesai_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate_2 + 1);
    const double time_begin = pl_orig.points[0].timestamp;
    lidar_data.start_time = time_begin;
    for(std::size_t i = 0; i < pl_orig.size(); i++)
    {
      auto& pt = pl_orig.points[i];
      if(i % g_filter_rate_2 != 0 && pt.intensity < g_intensity_thresh) continue;
      if (!validPoint2(pt.x, pt.y, pt.z)) continue;
      offset_time = pt.timestamp - time_begin;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, pt.intensity, offset_time);
    }
    lidar_data.end_time = time_begin + offset_time;
    break;
  }
  case LID_TYPE::VEL_NCLT:
  {
    pcl::PointCloud<NCLT::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate_2 + 1);
    lidar_data.start_time = stampToSec(msg->header.stamp);
    for(std::size_t i = 0; i < pl_orig.size(); i += g_filter_rate_2){
      auto& pt = pl_orig.points[i];
      if (!validPoint2(pt.x, pt.y, pt.z)) continue;
      offset_time = pt.time * 1e-6;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, 1.0, offset_time);
    }
    lidar_data.end_time = lidar_data.start_time + offset_time;
    break;
  }
  case LID_TYPE::VELO16:
  case LID_TYPE::VELO32:
  {
    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate_2 + 1);
    lidar_data.start_time = stampToSec(msg->header.stamp);
    for(std::size_t i = 0; i < pl_orig.size(); i++){
      auto& pt = pl_orig.points[i];
      if(i % g_filter_rate_2 != 0 && pt.intensity < g_intensity_thresh) continue;
      if (!validPoint2(pt.x, pt.y, pt.z)) continue;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, pt.intensity, pt.time);
    }
    lidar_data.end_time = lidar_data.start_time + lidar_data.pc->points.back().offset_time;
    break;
  }
  case OUSTER:
  {
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate_2 + 1);
    lidar_data.start_time = stampToSec(msg->header.stamp);
    for(std::size_t i = 0; i < pl_orig.size(); i++){
      auto& pt = pl_orig.points[i];
      if(i % g_filter_rate_2 != 0 && pt.intensity < g_intensity_thresh) continue;
      if (!validPoint2(pt.x, pt.y, pt.z)) continue;
      offset_time = pt.t * 1e-9;
      lidar_data.pc->emplace_back(
          pt.x, pt.y, pt.z, pt.intensity, offset_time);
    }
    lidar_data.end_time = lidar_data.start_time + offset_time;
    break;
  }
  case LID_TYPE::LIVOX_PC2:
  {
    // Use generic PointXYZI — x,y,z,intensity are standard fields
    // that every PointCloud2 has. Custom fields (tag,line,timestamp)
    // may have driver-specific names that don't match our PCL struct.
    pcl::PointCloud<pcl::PointXYZI> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    if (pl_orig.empty()) {
      static bool empty_logged = false;
      if (!empty_logged) {
        LOG(WARNING) << YELLOW << " ---> [LIVOX_PC2 lidar2] Deserialization produced empty cloud!"
                     << " msg fields: " << pcl::getFieldsList(*msg)
                     << " data size: " << msg->data.size() << RESET;
        empty_logged = true;
      }
      break;
    }

    static bool first_log2 = true;
    if (first_log2) {
      LOG(INFO) << GREEN << " ---> [LIVOX_PC2 lidar2] Receiving: "
                << pl_orig.size() << " pts/msg" << RESET;
      first_log2 = false;
    }

    lidar_data.pc->reserve(pl_orig.size() / g_filter_rate_2 + 1);
    const double header_time = stampToSec(msg->header.stamp);
    lidar_data.start_time = header_time;

    if (g_lidar2_scan_period <= 1e-3) {
      for (std::size_t i = 0; i < pl_orig.size(); i++) {
        auto& pt = pl_orig.points[i];
        if(i % g_filter_rate_2 != 0 && pt.intensity < g_intensity_thresh) continue;
        if (!validPoint2(pt.x, pt.y, pt.z)) continue;
        lidar_data.pc->emplace_back(pt.x, pt.y, pt.z, pt.intensity, 0.0);
      }
      if (lidar_data.pc->empty()) break;
      lidar_data.end_time = header_time;
      break;
    }

    // ── azimuth-based deskew (see stdMsgHandler for the rationale) ──
    {
      std::vector<float>  kx, ky, kz, ki;
      std::vector<double> az_un;
      const std::size_t cap = pl_orig.size() / g_filter_rate_2 + 1;
      kx.reserve(cap); ky.reserve(cap); kz.reserve(cap); ki.reserve(cap); az_un.reserve(cap);

      double az_acc = 0.0, az_prev = 0.0;
      bool az_first = true;
      for (std::size_t i = 0; i < pl_orig.size(); i++) {
        auto& pt = pl_orig.points[i];
        if(i % g_filter_rate_2 != 0 && pt.intensity < g_intensity_thresh) continue;
        if (!validPoint2(pt.x, pt.y, pt.z)) continue;
        const double a = std::atan2(static_cast<double>(pt.y), static_cast<double>(pt.x));
        if (az_first) { az_acc = a; az_prev = a; az_first = false; }
        else {
          double d = a - az_prev;
          while (d >  M_PI) d -= 2.0 * M_PI;
          while (d < -M_PI) d += 2.0 * M_PI;
          az_acc += d;
          az_prev = a;
        }
        kx.push_back(pt.x); ky.push_back(pt.y); kz.push_back(pt.z); ki.push_back(pt.intensity);
        az_un.push_back(az_acc);
      }
      if (kx.empty()) break;

      const double span  = az_un.back() - az_un.front();
      const double scale = (std::fabs(span) > 1e-6) ? (g_lidar2_scan_period / span) : 0.0;
      for (std::size_t j = 0; j < kx.size(); j++) {
        const double off = (az_un[j] - az_un.front()) * scale;
        lidar_data.pc->emplace_back(kx[j], ky[j], kz[j], ki[j], off);
      }

      static bool deskew_logged2 = false;
      if (!deskew_logged2) {
        LOG(INFO) << GREEN << " ---> [LIVOX_PC2 lidar2] azimuth-based deskew ON: scan_period="
                  << g_lidar2_scan_period << "s, azimuth span="
                  << span * 180.0 / M_PI << " deg, pts=" << lidar_data.pc->size() << RESET;
        deskew_logged2 = true;
      }
    }

    lidar_data.end_time = header_time + g_lidar2_scan_period;
    break;
  }
  default:
    return;
  }

  // apply clock offset correction (dynamic if initialized, static as fallback)
  // ── M20 双雷达修正 (2026-10-12) ─────────────────────────────────
  // lidar1 的时间 = header + g_lidar1_time_offset(跨域位移: 本机时钟域 -> IMU/机器人时钟域)。
  // 而 lidar2_dynamic_offset_ = lidar1_clock_ - lidar2_clock_ 只把 lidar2 对齐到 lidar1 的
  // *header* 域;两台雷达同源(同一个 rslidar_sdk、同一台主机)时它 ~= 0,不含那个跨域位移
  // => 只加 dynamic 会让 lidar2 与 lidar1 相差 g_lidar1_time_offset,
  //    sync_measure 的双雷达时间窗(start_time/end_time 相交)永远配不上。
  // 故此处补上同一个常数。对应 yaml: lio.lidar2.time_offset 应填 0.0(两雷达同源,静态差为 0)。
  // true 分支 = 改造前原式(逐字符未变), 故 B2(键缺省取 true) 行为不受影响。
  // false 分支 = 与 lidar1 同规则, 供两雷达共享同一时钟的平台(M20)使用。
  double offset;
  if (g_lidar2_use_dynamic_offset) {
    offset = (lidar1_clock_valid_ && lidar2_clock_valid_)
                 ? lidar2_dynamic_offset_ : g_lidar2_time_offset;
  } else {
    offset = g_lidar2_time_offset + g_lidar1_time_offset;
  }
  lidar_data.start_time += offset;
  lidar_data.end_time   += offset;
  lidar_buffer_2_.push_back(lidar_data);
}


bool ROSWrapper::sync_measure(MeasureGroup& meas){
  // (C1) lidar2-only 组：lidar1 断流时不再整段丢弃第二雷达。
  // 开关为 false 时（默认）本行条件与原来的 `lidar_buffer_.empty() || imu_buffer_.empty()` **等价**，
  // 且下方 else 分支不可达 ⇒ OFF 路径行为逐字节不变。
  const bool have1 = !lidar_buffer_.empty();
  const bool have2 = g_dual_lidar_enable && !lidar_buffer_2_.empty();
  // (C1) **断流门控**：以 **墙钟** 计（lidar1 消息的到达间隔）——不依赖雷达/IMU 的时基约定，
  // 避免数据时间轴上的隐性偏移把判据恒置为假（实测踩坑：数据时间版本恒假 ⇒ 断流也不放行）。
  // ⚠ 回放速率 ≠ 1.0 时该阈值按墙钟折算。缺此门控则正常段（两雷达相位错开）也命中 ⇒ 帧率翻倍（10.0 → 19.3 Hz）。
  const double now_s = stampToSec(this->now());
  const bool lidar1_outage = g_lidar2_only_enable && have2 && t_last_lidar1_wall_ > 0.0 &&
                             (now_s - t_last_lidar1_wall_) > g_lidar2_only_gap;
  if ((!have1 && !lidar1_outage) || imu_buffer_.empty()) {
    return false;
  }
  // (C1) 诊断（每 5 s 一条）：开关开时打印三条件实测值，便于"门控为何不触发"的定位
  if (g_lidar2_only_enable && now_s - dbg_c1_t_ > 5.0) {
    dbg_c1_t_ = now_s;
    LOG(INFO) << YELLOW << " ---> [C1Dbg] have1=" << have1 << " have2=" << have2
              << " dt_arrival=" << (t_last_lidar1_wall_ > 0.0 ? now_s - t_last_lidar1_wall_ : -1.0)
              << " dt_datatime=" << (have2 && last_timestamp_lidar_ > 0.0
                                     ? lidar_buffer_2_.front().end_time - last_timestamp_lidar_ : -1.0)
              << " gap=" << g_lidar2_only_gap << " last1=" << last_timestamp_lidar_ << RESET;
  }

  if (!lidar_pushed_) {
    if (have1) {
      meas.lidar = lidar_buffer_.front();
      meas.has_lidar1 = true;
      meas.has_lidar2 = false;
    } else {
      // 无 lidar1：本组以 lidar2 的时间跨度为基准；meas.lidar 留空，
      // 下游一律按 has_lidar1 跳过 lidar1 环节（deskew / 建图初始 / SetObsTime）。
      meas.lidar = LidarData();
      meas.has_lidar1 = false;
      meas.lidar2 = lidar_buffer_2_.front();
      meas.has_lidar2 = true;
      lidar_buffer_2_.pop_front();
    }
    lidar_pushed_ = true;
  }

  const double grp_end = meas.has_lidar1 ? meas.lidar.end_time : meas.lidar2.end_time;

  if (meas.has_lidar1) {
    if(last_timestamp_lidar_ > meas.lidar.end_time){
      lidar_buffer_.pop_front();
      lidar_pushed_ = false;
      return false;
    }

    // ── second LiDAR: retry until matched or IMU ready ──
    // lidar2 data may arrive after lidar1; keep trying on each poll cycle.
    if (g_dual_lidar_enable && !meas.has_lidar2 && !lidar_buffer_2_.empty()) {
      double lidar1_start = meas.lidar.start_time;
      double lidar1_end   = meas.lidar.end_time;
      while (!lidar_buffer_2_.empty() && lidar_buffer_2_.front().end_time < lidar1_start) {
        lidar_buffer_2_.pop_front();
      }
      if (!lidar_buffer_2_.empty() && lidar_buffer_2_.front().start_time <= lidar1_end) {
        meas.lidar2 = lidar_buffer_2_.front();
        meas.has_lidar2 = true;
        lidar_buffer_2_.pop_front();
      }
    }
  }

  if (last_timestamp_imu_ < grp_end) {
    return false;
  }

  double imu_time = imu_buffer_.front().secs;
  meas.imu.clear();
  while ((!imu_buffer_.empty()) && (imu_time < grp_end)) {
    imu_time = imu_buffer_.front().secs;
    if (imu_time > grp_end) break;
    meas.imu.push_back(imu_buffer_.front());
    imu_buffer_.pop_front();
  }

  if (meas.has_lidar1) {
    last_timestamp_lidar_ = meas.lidar.end_time;
    lidar_buffer_.pop_front();
  }
  lidar_pushed_ = false;
  return true;
}


void ROSWrapper::pub_odom(const NavState& state){
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = g_frame_world;

  odom.header.stamp = toRosTime(state.timestamp);
  odom.pose.pose.position.x = state.p[0];
  odom.pose.pose.position.y = state.p[1];
  odom.pose.pose.position.z = state.p[2];

  V4 temp_q = state.R.coeffs();
  odom.pose.pose.orientation.x = temp_q[0];
  odom.pose.pose.orientation.y = temp_q[1];
  odom.pose.pose.orientation.z = temp_q[2];
  odom.pose.pose.orientation.w = temp_q[3];

  odom.twist.twist.linear.x = state.v[0];
  odom.twist.twist.linear.y = state.v[1];
  odom.twist.twist.linear.z = state.v[2];

  pub_odom_->publish(odom);    // imu frame -> lidar frequency

  V3 robo_position = state.R.R_ * ( - g_odom_robo.R_ * g_odom_robo.t_) + state.p;

  if(g_2_robot){
    static auto pub_msg2uav_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/mavros/vision_pose/pose", 10);
    M3 robo_rotation = state.R.R_ * g_odom_robo.R_;
    msg2uav_.header.stamp = odom.header.stamp;
    msg2uav_.pose.position.x = robo_position[0];
    msg2uav_.pose.position.y = robo_position[1];
    msg2uav_.pose.position.z = robo_position[2];
    Quat robo_quat(robo_rotation);
    msg2uav_.pose.orientation.w = robo_quat.w();
    msg2uav_.pose.orientation.x = robo_quat.x();
    msg2uav_.pose.orientation.y = robo_quat.y();
    msg2uav_.pose.orientation.z = robo_quat.z();
    pub_msg2uav_->publish(msg2uav_);
  }

  if((last_path_point_ - robo_position).norm() > 0.1)
  {
    path_.header.stamp = odom.header.stamp;
    geometry_msgs::msg::PoseStamped point;
    point.pose = odom.pose.pose;
    path_.poses.push_back(point);
    pub_path_->publish(path_);
    last_path_point_ = robo_position;
  }

  geometry_msgs::msg::TransformStamped tf_msg;

  tf_msg.header.stamp = odom.header.stamp;
  tf_msg.header.frame_id = g_frame_world;
  tf_msg.child_frame_id = g_frame_imu;

  tf_msg.transform.translation.x = state.p[0];
  tf_msg.transform.translation.y = state.p[1];
  tf_msg.transform.translation.z = state.p[2];

  tf_msg.transform.rotation.x = temp_q.x();
  tf_msg.transform.rotation.y = temp_q.y();
  tf_msg.transform.rotation.z = temp_q.z();
  tf_msg.transform.rotation.w = temp_q.w();

  tf_broadcaster_->sendTransform(tf_msg);

  // ── Nav2 TF: map → odom → base_link ──
  if (g_nav2_tf) {
    geometry_msgs::msg::TransformStamped map_odom_tf;
    map_odom_tf.header.stamp = odom.header.stamp;
    map_odom_tf.header.frame_id = g_frame_map;
    map_odom_tf.child_frame_id = g_frame_odom;
    map_odom_tf.transform.translation.x = 0.0;
    map_odom_tf.transform.translation.y = 0.0;
    map_odom_tf.transform.translation.z = 0.0;
    map_odom_tf.transform.rotation.x = 0.0;
    map_odom_tf.transform.rotation.y = 0.0;
    map_odom_tf.transform.rotation.z = 0.0;
    map_odom_tf.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(map_odom_tf);

    geometry_msgs::msg::TransformStamped odom_base_tf;
    odom_base_tf.header.stamp = odom.header.stamp;
    odom_base_tf.header.frame_id = g_frame_odom;
    odom_base_tf.child_frame_id = g_frame_base;
    odom_base_tf.transform.translation.x = robo_position[0];
    odom_base_tf.transform.translation.y = robo_position[1];
    odom_base_tf.transform.translation.z = robo_position[2];
    M3 robo_rot = state.R.R_ * g_odom_robo.R_;
    Quat robo_q(robo_rot);
    odom_base_tf.transform.rotation.x = robo_q.x();
    odom_base_tf.transform.rotation.y = robo_q.y();
    odom_base_tf.transform.rotation.z = robo_q.z();
    odom_base_tf.transform.rotation.w = robo_q.w();
    tf_broadcaster_->sendTransform(odom_base_tf);
  }

}


void ROSWrapper::pub_cloud_world(const CloudPtr& pc, double time){
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*pc, cloud);
  cloud.header.frame_id = g_frame_world;
  cloud.header.stamp = toRosTime(time);
  pub_cloud_world_->publish(cloud);
}


void ROSWrapper::pub_init_obs(const BASIC::CloudPtr& obs, const Eigen::Matrix4f& pose){
  static auto pub_init_obs_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lio/init_obs", 10);

  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud(*obs, *transformed, pose);

  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*transformed, cloud);
  cloud.header.frame_id = g_frame_world;
  cloud.header.stamp = this->now();
  pub_init_obs_->publish(cloud);
}


void ROSWrapper::pub_plane_features(const BASIC::CloudPtr& pc){
  static auto pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lio/plane_features", 10);
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*pc, cloud);
  cloud.header.frame_id = g_frame_world;
  cloud.header.stamp = this->now();
  pub_->publish(cloud);
}


void ROSWrapper::pub_edge_features(const BASIC::CloudPtr& pc){
  static auto pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lio/edge_features", 10);
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*pc, cloud);
  cloud.header.frame_id = g_frame_world;
  cloud.header.stamp = this->now();
  pub_->publish(cloud);
}


void ROSWrapper::pub_cloud2planner(const CloudPtr& pc, double time){
  static auto pub_cloud2robot_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lio/robo/cloud_world", 10);
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*pc, cloud);
  cloud.header.frame_id = g_frame_world;
  cloud.header.stamp = toRosTime(time);
  pub_cloud2robot_->publish(cloud);
}


void ROSWrapper::pub_cloud_body_pose(const CloudPtr& pc, 
  const NavState& state)
{
  static auto pub_cloud_body_pose_ =
    this->create_publisher<super_lio::msg::CloudPose>(
        "/lio/body/cloud_pose", 10);
  super_lio::msg::CloudPose cloud_pose;
  pcl::toROSMsg(*pc, cloud_pose.cloud);
  cloud_pose.cloud.header.stamp = toRosTime(state.timestamp); 
  cloud_pose.pose.position.x = state.p[0];
  cloud_pose.pose.position.y = state.p[1];
  cloud_pose.pose.position.z = state.p[2];
  V4 temp_q = state.R.coeffs();
  cloud_pose.pose.orientation.x = temp_q[0];
  cloud_pose.pose.orientation.y = temp_q[1];
  cloud_pose.pose.orientation.z = temp_q[2];
  cloud_pose.pose.orientation.w = temp_q[3];

  pub_cloud_body_pose_->publish(cloud_pose);
}


void ROSWrapper::pub_cloud_world_pose(const CloudPtr& pc, 
   const NavState& state)
{
  static auto pub_cloud_world_pose_ =
    this->create_publisher<super_lio::msg::CloudPose>(
        "/lio/world/cloud_pose", 10);
  super_lio::msg::CloudPose cloud_pose;
  pcl::toROSMsg(*pc, cloud_pose.cloud);
  cloud_pose.cloud.header.stamp = toRosTime(state.timestamp);  
  cloud_pose.pose.position.x = state.p[0];
  cloud_pose.pose.position.y = state.p[1];
  cloud_pose.pose.position.z = state.p[2];
  V4 temp_q = state.R.coeffs();
  cloud_pose.pose.orientation.x = temp_q[0];
  cloud_pose.pose.orientation.y = temp_q[1];
  cloud_pose.pose.orientation.z = temp_q[2];
  cloud_pose.pose.orientation.w = temp_q[3];
  pub_cloud_world_pose_->publish(cloud_pose);
}


void ROSWrapper::pub_processing_time(double time, 
  double current_time, double mean_time, double std_time)
{
  static auto pub_processing_time_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/lio/processing_time", 10);
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = toRosTime(time);
  msg.pose.position.x = current_time;
  msg.pose.position.y = mean_time;
  msg.pose.position.z = std_time;
  pub_processing_time_->publish(msg);
}


void ROSWrapper::set_global_map(const BASIC::CloudPtr& global_map){
  pcl::toROSMsg(*global_map, global_map_msg_);
  global_map_msg_.header.frame_id = g_frame_world;

  static auto global_map_pub =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/lio/global_map", 10);

  static auto global_map_timer =
    this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        static int count = -1;
        static int publish_interval = 1;

        count++;
        if (count % publish_interval != 0) {
          return;
        }

        count = 0;
        publish_interval++;
        if (publish_interval > 10) {
          publish_interval = 10;
        }
        global_map_msg_.header.stamp = this->now();
        global_map_pub->publish(global_map_msg_);
      });
}


void ROSWrapper::set_initial_data(BASIC::SE3& init_pose, bool& flg_get_init_guess, bool flg_finish_init)
{
  static auto init_pose_sub =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 1,
        [this, &init_pose, &flg_get_init_guess](
          const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) 
        {
          V3 init_translation;
          init_translation << msg->pose.pose.position.x,
                              msg->pose.pose.position.y,
                              msg->pose.pose.position.z;

          double x = msg->pose.pose.orientation.x;
          double y = msg->pose.pose.orientation.y;
          double z = msg->pose.pose.orientation.z;
          double w = msg->pose.pose.orientation.w;

          Quat init_rotation(w, x, y, z);

          init_pose = BASIC::SE3(SO3(init_rotation.toRotationMatrix()), init_translation);

          flg_get_init_guess = true;

          auto euler = init_rotation.toRotationMatrix().eulerAngles(0, 1, 2);
          LOG(INFO) << YELLOW
                  << " ---> GET Initial guess: pos=("
                  << init_translation.transpose()
                  << ") rpy=(" << euler.transpose() << ")"
                  << RESET;
        });

  if (flg_finish_init) {
    init_pose_sub.reset();
  }
}


// ── Dynamic time offset estimation (implementations) ────────────
// clock = header_stamp - system_receive_time
// offset = clock1 - clock2 (add to lidar2 to align with lidar1 reference)

void ROSWrapper::updateLidar1Clock(double hdr_stamp) {
  double sys = stampToSec(this->now());
  lidar1_clock_ = hdr_stamp - sys;
  lidar1_clock_valid_ = true;

  if (lidar2_clock_valid_) {
    double raw = lidar1_clock_ - lidar2_clock_;
    double alpha = 0.02;
    lidar2_dynamic_offset_ = alpha * raw + (1.0 - alpha) * lidar2_dynamic_offset_;
    static bool first_update = true;
    if (first_update) {
      lidar2_dynamic_offset_ = raw;
      LOG(INFO) << GREEN << " ---> [DynOffset] Initialized: clock1=" << lidar1_clock_
                << " clock2=" << lidar2_clock_
                << " offset=" << lidar2_dynamic_offset_ << RESET;
      first_update = false;
    }
  }
}

void ROSWrapper::updateLidar2Clock(double hdr_stamp) {
  double sys = stampToSec(this->now());
  lidar2_clock_ = hdr_stamp - sys;
  lidar2_clock_valid_ = true;

  if (lidar1_clock_valid_) {
    double raw = lidar1_clock_ - lidar2_clock_;
    double alpha = 0.02;
    lidar2_dynamic_offset_ = alpha * raw + (1.0 - alpha) * lidar2_dynamic_offset_;
    static bool first_update2 = true;
    if (first_update2) {
      lidar2_dynamic_offset_ = raw;
      LOG(INFO) << GREEN << " ---> [DynOffset] Initialized: clock1=" << lidar1_clock_
                << " clock2=" << lidar2_clock_
                << " offset=" << lidar2_dynamic_offset_ << RESET;
      first_update2 = false;
    }
  }
}


} // namespace END.