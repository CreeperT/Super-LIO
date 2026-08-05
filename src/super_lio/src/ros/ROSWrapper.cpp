
#include "ros/ROSWrapper.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <iomanip>
#include <pcl/common/transforms.h>


using namespace BASIC;

namespace LI2Sup{

void LoadParamFromRos(rclcpp::Node& node)
{
  node.declare_parameter<bool>("lio.map.save_map", false);
  node.get_parameter("lio.map.save_map", g_save_map);

  LOG(INFO) << GREEN << " ---> [Param] map/save_map: "
            << (g_save_map ? "true" : "false") << RESET;

  node.declare_parameter<bool>("lio.eva.timer", false);
  node.get_parameter("lio.eva.timer", g_time_eva);

  node.declare_parameter<bool>("lio.eva.verbose_logging", false);
  node.get_parameter("lio.eva.verbose_logging", g_verbose_logging);

  node.declare_parameter<bool>("lio.eva.decouple_verify", false);
  node.get_parameter("lio.eva.decouple_verify", g_decouple_verify);

  node.declare_parameter<bool>("lio.map.if_filter", false);
  node.get_parameter("lio.map.if_filter", g_if_filter);

  node.declare_parameter<std::string>("lio.map.save_map_dir", "");
  node.get_parameter("lio.map.save_map_dir", g_save_map_dir);
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

  // ================= relocation robustness =================
  node.declare_parameter<int>("lio.relocation.init_frames", 5);
  node.get_parameter("lio.relocation.init_frames", g_reloc_init_frames);

  node.declare_parameter<double>("lio.relocation.z_search_range", 2.0);
  node.get_parameter("lio.relocation.z_search_range", g_reloc_z_search_range);

  node.declare_parameter<double>("lio.relocation.z_search_step", 1.0);
  node.get_parameter("lio.relocation.z_search_step", g_reloc_z_search_step);

  node.declare_parameter<double>("lio.relocation.min_overlap_ratio", 0.0);
  node.get_parameter("lio.relocation.min_overlap_ratio", g_reloc_min_overlap_ratio);

  LOG(INFO) << GREEN << " ---> [Params]: Load from ROS2 parameter server."
            << RESET;
}


void livox2pcl(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg, CloudPtr& point_cloud){
  point_cloud->clear();
  CloudPtr cloud_full(new PointCloudType());
  int plsize = msg->point_num;
  cloud_full->resize(plsize);
  point_cloud->reserve(plsize);
  std::vector<bool> is_valid_pt(plsize, false);
  std::vector<std::size_t> index(plsize - 1);
  std::iota(std::begin(index), std::end(index), 1);

  std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const uint &i) {
    if((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)
    {
      // if (i % g_filter_rate == 0) 
      {
        cloud_full->at(i).x = msg->points[i].x;
        cloud_full->at(i).y = msg->points[i].y;
        cloud_full->at(i).z = msg->points[i].z;
        cloud_full->at(i).intensity = msg->points[i].reflectivity;

        if ((abs(cloud_full->at(i).x - cloud_full->at(i - 1).x) > 1e-7) ||
            (abs(cloud_full->at(i).y - cloud_full->at(i - 1).y) > 1e-7) ||
            (abs(cloud_full->at(i).z - cloud_full->at(i - 1).z) > 1e-7))
        {
          double normal_dis = cloud_full->at(i).x * cloud_full->at(i).x + 
                              cloud_full->at(i).y * cloud_full->at(i).y +
                              cloud_full->at(i).z * cloud_full->at(i).z;
          if(normal_dis > g_blind2 and normal_dis < g_maxrange2){
            is_valid_pt[i] = true;
          }
        }
      }
    }
  });

  for (int i = 1; i < plsize; i++) {
    if (is_valid_pt[i]) {
      point_cloud->points.push_back(cloud_full->at(i));
    }
  }
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

  msg2uav_.header.frame_id = "world";
  path_.header.frame_id = "world";

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

  auto lidar_qos = rclcpp::QoS(rclcpp::KeepLast(20))
                   .best_effort()
                   .durability_volatile();

  sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      g_imu_topic,
      imu_qos,
      std::bind(&ROSWrapper::imuHandler, this, std::placeholders::_1),
      sub_opt);

  if (g_lidar_type == LID_TYPE::LIVOX) {
    sub_lidar_ =
        this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            g_lidar_topic,
            lidar_qos,
            std::bind(&ROSWrapper::livoxHandler, this, std::placeholders::_1),
            sub_opt);
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
      sub_lidar2_ =
          this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
              g_lidar_topic2,
              lidar_qos,
              std::bind(&ROSWrapper::livoxHandler2, this, std::placeholders::_1),
              sub_opt);
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


void ROSWrapper::imuHandler(const sensor_msgs::msg::Imu::SharedPtr msg){
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
    odom_imu.header.frame_id = "world";
    odom_robo.header.frame_id = "world";
    pub_imu_odom_->publish(odom_imu);
    pub_robo_odom_->publish(odom_robo);
  }
}


void ROSWrapper::livoxHandler(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg){
  if(msg->point_num < 10) return;
  updateLidar1Clock(stampToSec(msg->header.stamp));
  LidarData lidar_data;
  std::size_t ptsize = msg->point_num;
  lidar_data.pc.reset(new pcl::PointCloud<LI2Sup::PointXTZIT>());
  lidar_data.pc->reserve(ptsize / g_filter_rate + 1);

  double offset_time = 0.0;
  for(std::size_t _i = 0; _i < ptsize; _i++){
    auto& pt = msg->points[_i];
    if(_i % g_filter_rate != 0 && pt.reflectivity < g_intensity_thresh) continue;
    auto tag = pt.tag & 0x30;
    if (tag == 0x10 || tag == 0x00){
      auto dis = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
      if(dis > g_blind2 && dis < g_maxrange2){
        offset_time = pt.offset_time * 1e-9;
        lidar_data.pc->emplace_back(pt.x, pt.y, pt.z, pt.reflectivity, offset_time);
      }
    }
  }
  lidar_data.start_time = stampToSec(msg->header.stamp) + g_lidar1_time_offset;
  lidar_data.end_time   = lidar_data.start_time + offset_time;
  lidar_buffer_.push_back(lidar_data);
}


void ROSWrapper::stdMsgHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
  if(msg->data.size() < 10) return;
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
    for(std::size_t i = 0; i < pl_orig.size(); i++)
    {
      auto& pt = pl_orig.points[i];
      if(i % g_filter_rate != 0 && pt.intensity < g_intensity_thresh) continue;
      if (!validPoint(pt.x, pt.y, pt.z)) continue;
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
  default:
    return;
  }

  lidar_data.start_time += g_lidar1_time_offset;
  lidar_data.end_time   += g_lidar1_time_offset;
  lidar_buffer_.push_back(lidar_data);
}


// ── Second LiDAR handlers (dual-LiDAR) ──────────────────────────

void ROSWrapper::livoxHandler2(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg){
  if(msg->point_num < 10) return;
  updateLidar2Clock(stampToSec(msg->header.stamp));
  LidarData lidar_data;
  std::size_t ptsize = msg->point_num;
  lidar_data.pc.reset(new pcl::PointCloud<LI2Sup::PointXTZIT>());
  lidar_data.pc->reserve(ptsize / g_filter_rate_2 + 1);

  double offset_time = 0.0;
  for(std::size_t _i = 0; _i < ptsize; _i++){
    auto& pt = msg->points[_i];
    if(_i % g_filter_rate_2 != 0 && pt.reflectivity < g_intensity_thresh) continue;
    auto tag = pt.tag & 0x30;
    if (tag == 0x10 || tag == 0x00){
      auto dis = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
      if(dis > g_blind2_2 && dis < g_maxrange2_2){
        offset_time = pt.offset_time * 1e-9;
        lidar_data.pc->emplace_back(pt.x, pt.y, pt.z, pt.reflectivity, offset_time);
      }
    }
  }
  double offset = (lidar1_clock_valid_ && lidar2_clock_valid_)
                      ? lidar2_dynamic_offset_ : g_lidar2_time_offset;
  lidar_data.start_time = stampToSec(msg->header.stamp) + offset;
  lidar_data.end_time   = lidar_data.start_time + offset_time;
  lidar_buffer_2_.push_back(lidar_data);
}


void ROSWrapper::stdMsgHandler2(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
  if(msg->data.size() < 10) return;
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

    for (std::size_t i = 0; i < pl_orig.size(); i++) {
      auto& pt = pl_orig.points[i];
      if(i % g_filter_rate_2 != 0 && pt.intensity < g_intensity_thresh) continue;
      if (!validPoint2(pt.x, pt.y, pt.z)) continue;
      // Mid-360 scan duration is short (~100ms); use 0 offset_time
      lidar_data.pc->emplace_back(pt.x, pt.y, pt.z, pt.intensity, 0.0);
    }
    if (lidar_data.pc->empty()) break;
    lidar_data.end_time = header_time;
    break;
  }
  default:
    return;
  }

  // apply clock offset correction (dynamic if initialized, static as fallback)
  double offset = (lidar1_clock_valid_ && lidar2_clock_valid_)
                      ? lidar2_dynamic_offset_ : g_lidar2_time_offset;
  lidar_data.start_time += offset;
  lidar_data.end_time   += offset;
  lidar_buffer_2_.push_back(lidar_data);
}


bool ROSWrapper::sync_measure(MeasureGroup& meas){
  if (lidar_buffer_.empty() || imu_buffer_.empty()) {
    return false;
  }

  if (!lidar_pushed_) {
    meas.lidar = lidar_buffer_.front();
    meas.has_lidar2 = false;
    lidar_pushed_ = true;
  }

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

  if (last_timestamp_imu_ < meas.lidar.end_time) {
    return false;
  }

  double imu_time = imu_buffer_.front().secs;
  meas.imu.clear();
  while ((!imu_buffer_.empty()) && (imu_time < meas.lidar.end_time)) {
    imu_time = imu_buffer_.front().secs;
    if (imu_time > meas.lidar.end_time) break;
    meas.imu.push_back(imu_buffer_.front());
    imu_buffer_.pop_front();
  }

  last_timestamp_lidar_ = meas.lidar.end_time;
  lidar_buffer_.pop_front();
  lidar_pushed_ = false;
  return true;
}


void ROSWrapper::pub_odom(const NavState& state){
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "world";

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
  tf_msg.header.frame_id = "world";
  tf_msg.child_frame_id = "dog_imu_link";

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
    map_odom_tf.header.frame_id = "map";
    map_odom_tf.child_frame_id = "odom";
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
    odom_base_tf.header.frame_id = "odom";
    odom_base_tf.child_frame_id = "base_link";
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
  cloud.header.frame_id = "world";
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
  cloud.header.frame_id = "world";
  cloud.header.stamp = this->now();
  pub_init_obs_->publish(cloud);
}


void ROSWrapper::pub_plane_features(const BASIC::CloudPtr& pc){
  static auto pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lio/plane_features", 10);
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*pc, cloud);
  cloud.header.frame_id = "world";
  cloud.header.stamp = this->now();
  pub_->publish(cloud);
}


void ROSWrapper::pub_edge_features(const BASIC::CloudPtr& pc){
  static auto pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lio/edge_features", 10);
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*pc, cloud);
  cloud.header.frame_id = "world";
  cloud.header.stamp = this->now();
  pub_->publish(cloud);
}


void ROSWrapper::pub_cloud2planner(const CloudPtr& pc, double time){
  static auto pub_cloud2robot_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lio/robo/cloud_world", 10);
  sensor_msgs::msg::PointCloud2 cloud;
  pcl::toROSMsg(*pc, cloud);
  cloud.header.frame_id = "world";
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
  global_map_msg_.header.frame_id = "world";

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